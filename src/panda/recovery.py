"""
recovery.py

Recovery strategies stepped alongside a perturbed episode (run_libero_eval_perturb.py). At every step the episode loop
calls `before_step` (which may activate the strategy), drops the queued action chunk if `flush` is set, queries the
policy with `instruction(...)` unless `override(...)` returns an action, and reports the executed step to `observe`.
A strategy activates when `triggered` fires; the default is an oracle trigger `delay` steps after the perturbation's
injection ends. A strategy with an online detector (core_detect) instead activates from `detect`, called right after
each policy query (the queried chunk is then not executed). Steps are the same 1-based counts as in perturbations.py.
"""

import copy
import json
import os
import time
from collections import deque

import mujoco
import numpy as np
from robosuite.controllers import controller_factory
from scipy.spatial.transform import Rotation as Rot


class Recovery:
    """Base class. Subclasses implement `activate` and, as needed, `instruction`, `override` and `observe`."""

    def __init__(self, env, perturb, delay):
        self.env, self.perturb, self.delay = env.env, perturb, delay
        self.trigger_steps, self.flush = [], False
        self.handoff = []  # actions (raw policy outputs) to queue for the policy when control returns (with flush)
        # Set by the episode loop: query(obs) -> raw action chunk from the policy for an observation dict (None when
        # replaying recorded actions), process_action(raw) -> env action; recorded: a replayed episode's record();
        # task: the instruction; latent() -> the action head's latent of the latest policy query; clean_dir /
        # detector_dir: this policy's clean rollouts (collect_clean_trajectories.py, mire) and detector
        # (calibrate_detector.py, core_detect)
        self.query, self.process_action, self.recorded = None, None, None
        self.task, self.latent = None, None
        self.clean_dir, self.detector_dir = None, None

    @property
    def sim(self):  # env.reset() may rebuild the simulator, so never cache it
        return self.env.sim

    def reset(self, obs):
        """Called after the wait steps, right before the policy starts acting."""

    def triggered(self, t):
        """Oracle trigger: once, after `delay` steps have passed since the injection ended."""
        end = self.perturb.end_step
        return not self.trigger_steps and end is not None and t == end + self.delay + 1

    def before_step(self, t, obs):
        if self.triggered(t):
            self.trigger_steps.append(t)
            self.activate(t, obs)

    def activate(self, t, obs):
        pass

    def detect(self, t, obs):
        """Called right after the policy was queried at step t (not when replaying); True if the strategy activated
        on it, in which case the queried chunk is not executed."""
        return False

    def instruction(self, t, task_description):
        """Instruction to query the policy with at step t."""
        return task_description

    def override(self, t, obs):
        """Action (env action space, np.ndarray) to execute instead of the policy's at step t, or None."""
        return None

    def observe(self, t, obs, action):
        """Called after env.step at step t with the executed action."""

    def record(self):
        return {"trigger_steps": self.trigger_steps}


class Vanilla(Recovery):
    """B0: no recovery; the frozen policy simply continues. The trigger step is still logged for comparison."""


class Replan(Recovery):
    """B1 (failed-state re-plan): drop the rest of the queued action chunk so the policy is re-queried from the
    perturbed state with the same instruction."""

    def activate(self, t, obs):
        self.flush = True


class JointOverride:
    """Temporarily stands in for the robot's OSC controller: tracks `goal_qpos` with robosuite's JOINT_POSITION
    controller while keeping the OSC's control_dim / limits, so env actions keep their size (arm part ignored)."""

    def __init__(self, robot, kp):
        keep = ("robot_name", "sim", "eef_name", "eef_rot_offset", "joint_indexes", "actuator_range", "policy_freq",
                "ndim")
        cfg = {k: robot.controller_config[k] for k in keep}
        cfg.update({"input_max": 1, "input_min": -1, "output_max": 0.05, "output_min": -0.05, "kp": kp,
                    "damping_ratio": 1, "impedance_mode": "fixed", "kp_limits": [0, 300],
                    "damping_ratio_limits": [0, 10], "qpos_limits": None, "interpolation": None, "ramp_ratio": 0.2})
        self.jp = controller_factory("JOINT_POSITION", cfg)
        self.control_dim, self.control_limits = robot.controller.control_dim, robot.controller.control_limits

    def set_goal(self, action):
        pass

    def run_controller(self):
        return self.jp.run_controller()

    def __getattr__(self, name):
        return getattr(self.jp, name)


class JointSpaceRecovery(Recovery):
    """Shared machinery for recoveries that move the arm in joint space: temporarily swap the robot's OSC for a
    JOINT_POSITION controller (JointOverride), restore it with a forced state refresh (the OSC's cached EEF pose is
    stale after the swap) and drop the queued chunk, and log contacts made while in control."""

    JOINT_KP = 150

    def reset(self, obs):
        self.phase, self.info, self.contacts = None, {}, {}

    def _joint_qpos(self):
        return self.sim.data.qpos[self.env.robots[0]._ref_joint_pos_indexes].copy()

    def _gripper_state(self):
        """Gripper command matching its current state (1 closed, -1 open). The gripper integrates the commanded
        direction (robosuite Panda: 0.25 per step), so a single flipped policy command does not change this sign."""
        return float(np.sign(self.env.robots[0].gripper.current_action[-1]) or -1.0)  # length 1 before the first step

    def _touching_gripper(self):
        """Objects in contact with any gripper geom (finger or pad)."""
        e, m, d = self.env, self.sim.model._model, self.sim.data._data
        grip = {m.geom(g).id for g in e.robots[0].gripper.contact_geoms}
        touched = {c.geom2 if c.geom1 in grip else c.geom1 for c in d.contact[: d.ncon]
                   if c.geom1 in grip or c.geom2 in grip}
        return [o for o, obj in e.objects_dict.items() if touched & {m.geom(g).id for g in obj.contact_geoms}]

    def _held_objects(self):
        """Objects touching the closed gripper (carried along: their contacts are not collisions)."""
        return self._touching_gripper() if self._gripper_state() > 0 else []

    REST_STEPS = 5  # Steps an object must be off the gripper before its height counts as its rest height

    def _object_z(self, obj):
        return self.sim.data.body_xpos[self.env.obj_body_id[obj]][2]

    def _init_rest(self, objects):
        self.z_rest = {o: self._object_z(o) for o in objects}
        self.off_gripper = {o: self.REST_STEPS for o in objects}

    def _track_rest(self):
        """Update the rest heights (height once off the gripper for REST_STEPS steps, so a contact flickering
        while an object is carried does not reset it mid-air). Returns the objects touching the gripper."""
        touching = self._touching_gripper()
        for o in self.z_rest:
            self.off_gripper[o] = 0 if o in touching else self.off_gripper[o] + 1
            if self.off_gripper[o] >= self.REST_STEPS:
                self.z_rest[o] = self._object_z(o)
        return touching

    def _lifted(self, obj, height):
        return self._object_z(obj) > self.z_rest[obj] + height

    def _swap_to_joint(self):
        robot = self.env.robots[0]
        self.osc = robot.controller
        self.override_ctrl = robot.controller = JointOverride(robot, self.JOINT_KP)

    def _finish(self):
        self.env.robots[0].controller = self.osc
        self.osc.update(force=True)  # The OSC did not run during the swap, so its cached EEF pose is stale
        self.osc.reset_goal()
        self.phase, self.flush = None, True

    def _record_contacts(self):
        """Deepest penetration (mm) per non-robot body touched by the robot (arm or gripper) during the recovery."""
        m, d = self.sim.model._model, self.sim.data._data
        root = m.body_rootid[m.body("robot0_base").id]
        for c in d.contact[: d.ncon]:
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            r1, r2 = m.body_rootid[b1] == root, m.body_rootid[b2] == root
            if r1 != r2:
                name = m.body(b2 if r1 else b1).name
                self.contacts[name] = max(self.contacts.get(name, 0.0), round(-1000 * float(c.dist), 2))

    def record(self):
        return {**super().record(), **self.info, "contacts": self.contacts,
                "interrupted": self.phase is not None}


class Rehome(JointSpaceRecovery):
    """Return the arm to its initial configuration, then re-query the policy. The arm first lifts with the OSC
    (constant speed, wrist rotation held) to clear the objects, then interpolates in joint space to the initial joint
    angles with a temporary JOINT_POSITION controller, so twisted post-impact configurations also unwind. A held
    object (touching the closed gripper and more than HOLD_LIFT above its rest height, see _track_rest) stays
    held (gripper kept closed); otherwise the gripper is opened. Robosuite's _check_grasp (both finger pads in contact)
    is not used: it drops in and out while an object is carried, e.g. by a mug handle. On arrival (or timeout) the OSC
    is restored and the queued chunk is dropped. All rehome steps count toward the episode step limit."""

    LIFT_HEIGHT, LIFT_SPEED, LIFT_MAX, LIFT_TOL = 0.10, 0.05, 40, 0.01  # m above current, m/step, steps, m
    ORI_SPEED, POS_MAX, ORI_MAX = 0.1, 0.05, 0.5  # rad/step; OSC_POSE output limits per unit action
    JOINT_SPEED, JOINT_MAX, JOINT_TOL_DEG = 0.05, 120, 1.0  # rad/step, steps, deg
    HOLD_LIFT = 0.01  # m

    def reset(self, obs):
        super().reset(obs)
        self.q0 = self._joint_qpos()
        self.z0 = obs["robot0_eef_pos"][2]
        self._init_rest(self.env.objects_dict)

    def activate(self, t, obs):
        held = [o for o in self._held_objects() if self._lifted(o, self.HOLD_LIFT)]
        self.grip = 1.0 if held else -1.0
        p = obs["robot0_eef_pos"]
        self.lift_goal = np.array([p[0], p[1], max(self.z0, p[2] + self.LIFT_HEIGHT)])
        self.hold_rot = Rot.from_quat(obs["robot0_eef_quat"])
        self.phase, self.n = "lift", 0
        self.info = {"holding": self.grip > 0, "held": held, "lift_steps": None, "joint_steps": None, "arrived": False}

    def override(self, t, obs):
        if self.phase == "lift":
            dp = self.lift_goal - obs["robot0_eef_pos"]
            if np.linalg.norm(dp) < self.LIFT_TOL or self.n >= self.LIFT_MAX:
                self.info["lift_steps"] = self.n
                self._start_joint()
            else:
                self.n += 1
                dr = (self.hold_rot * Rot.from_quat(obs["robot0_eef_quat"]).inv()).as_rotvec()
                a_p = dp / max(np.linalg.norm(dp) / self.LIFT_SPEED, 1.0) / self.POS_MAX
                a_r = dr / max(np.linalg.norm(dr) / self.ORI_SPEED, 1.0) / self.ORI_MAX
                return np.concatenate([a_p, a_r, [self.grip]])
        if self.phase == "joint":
            self.n += 1
            self.override_ctrl.jp.goal_qpos = self.q_start + (self.q0 - self.q_start) * min(self.n / self.n_interp, 1.0)
            return np.array([0.0] * 6 + [self.grip])
        return None

    def observe(self, t, obs, action):
        self._track_rest()
        if self.phase is None:
            return
        self._record_contacts()
        if self.phase == "joint":
            err = np.degrees(np.abs(self._joint_qpos() - self.q0)).max()
            arrived = self.n >= self.n_interp and err < self.JOINT_TOL_DEG
            if arrived or self.n >= self.JOINT_MAX:
                self.info.update(joint_steps=self.n, arrived=bool(arrived), joint_err_deg=round(float(err), 2))
                self._finish()

    def _start_joint(self):
        self._swap_to_joint()
        self.q_start = self._joint_qpos()
        self.n_interp = max(1, int(np.ceil(np.abs(self.q0 - self.q_start).max() / self.JOINT_SPEED)))
        self.phase, self.n = "joint", 0


class Retreat(JointSpaceRecovery):
    """Simple Retreat (arXiv 2610.06921): take over for a fixed HOLD_STEPS and drive the arm by linear joint-space
    interpolation back to its configuration BACK_STEPS steps earlier (the first step's if the episode is shorter),
    gripper included: the gripper is driven to its open / closed state at that step. The interpolation spans at least
    INTERP_STEPS (one chunk in the paper) and respects a per-joint step limit; once there the pose is held until
    HOLD_STEPS have passed, then control returns to the policy (no arrival check, no lift, no planner)."""

    BACK_STEPS, HOLD_STEPS, INTERP_STEPS, JOINT_SPEED = 120, 40, 20, 0.1  # steps, steps, steps, rad/step

    def reset(self, obs):
        super().reset(obs)
        # History index i = state after step i (0 = after the wait steps)
        self.q_hist, self.grip_hist = [self._joint_qpos()], [self._gripper_state()]

    def activate(self, t, obs):
        i = max(0, (t - 1) - self.BACK_STEPS)  # The current state is the one after step t - 1
        self.q_target, self.grip = self.q_hist[i], self.grip_hist[i]
        self._swap_to_joint()
        self.q_start = self._joint_qpos()
        self.n_interp = max(self.INTERP_STEPS,
                            int(np.ceil(np.abs(self.q_target - self.q_start).max() / self.JOINT_SPEED)))
        self.phase, self.n = "retreat", 0
        self.info = {"target_step": i, "clamped": (t - 1) - self.BACK_STEPS < 0, "gripper": self.grip,
                     "held": self._held_objects()}

    def override(self, t, obs):
        if self.phase != "retreat":
            return None
        self.n += 1
        frac = min(self.n / self.n_interp, 1.0)
        self.override_ctrl.jp.goal_qpos = self.q_start + (self.q_target - self.q_start) * frac
        return np.array([0.0] * 6 + [self.grip])

    def observe(self, t, obs, action):
        self.q_hist.append(self._joint_qpos())
        self.grip_hist.append(self._gripper_state())
        if self.phase != "retreat":
            return
        self._record_contacts()
        if self.n >= self.HOLD_STEPS:
            err = np.degrees(np.abs(self._joint_qpos() - self.q_target)).max()
            self.info.update(joint_err_deg=round(float(err), 2), arrived=bool(err < 1.0))
            self._finish()


class SafeLoop(JointSpaceRecovery):
    """SafeLoop-style rollback (arXiv 2609.26313, LIBERO release config) with an oracle in place of the learned risk
    predictor and decider: every RECORD_INTERVAL steps before the perturbation fires, the arm joint configuration is
    stored as a safe waypoint in a queue of the last QUEUE_LEN waypoints (none while or after it fires). At the
    trigger the arm rolls back to the oldest queued waypoint (the one RECORD_INTERVAL steps before the latest when
    both exist, else the only one), standing in for the release config's minimum waypoint age; with an empty queue
    (trigger before the first save) nothing is done. The rollback follows the straight joint-space path unless it
    collides (SafeLoop's LinearJointPlanner check), in which case nothing is done either. It is executed physically
    (as on SafeLoop's hardware, not its kinematic sim writes) by joint-space interpolation at JOINT_SPEED; the gripper
    is held in its current state; control returns on arrival (or after MAX_STEPS)."""

    RECORD_INTERVAL, QUEUE_LEN = 20, 2  # steps, waypoints
    JOINT_SPEED, MAX_STEPS, ARRIVE_TOL_DEG = 0.08, 120, 1.0  # rad/step (L2 over the 7 joints), steps, deg
    GRIPPER_KEYWORDS = ("gripper", "finger", "pad", "tip", "hand", "palm", "eef")

    def reset(self, obs):
        super().reset(obs)
        self.waypoints = deque(maxlen=self.QUEUE_LEN)  # (step, joint angles), oldest first

    def observe(self, t, obs, action):
        if self.perturb.fired_step is None and t % self.RECORD_INTERVAL == 0:
            self.waypoints.append((t, self._joint_qpos()))
        if self.phase != "rollback":
            return
        self._record_contacts()
        err = np.degrees(np.abs(self._joint_qpos() - self.q_target)).max()
        arrived = self.n >= self.n_interp and err < self.ARRIVE_TOL_DEG
        if arrived or self.n >= self.MAX_STEPS:
            self.info.update(steps=self.n, arrived=bool(arrived), joint_err_deg=round(float(err), 2))
            self._finish()

    def activate(self, t, obs):
        self.info = {"rollback": False, "queued": [s for s, _ in self.waypoints], "held": self._held_objects()}
        if not self.waypoints:
            self.info["skipped"] = "no_waypoint"
            return
        step, self.q_target = self.waypoints[0]
        self.q_start = self._joint_qpos()
        self.info.update(target_step=step, age=(t - 1) - step)
        path_steps = max(1, int(np.ceil(np.linalg.norm(self.q_target - self.q_start) / self.JOINT_SPEED)))
        hit = self._path_collision(path_steps)
        if hit:
            self.info.update(skipped="path_collision", collision=hit)
            return
        self._swap_to_joint()
        self.n_interp, self.phase, self.n = path_steps, "rollback", 0
        self.grip = self._gripper_state()
        self.info.update(rollback=True, gripper=self.grip)

    def override(self, t, obs):
        if self.phase != "rollback":
            return None
        self.n += 1
        frac = min(self.n / self.n_interp, 1.0)
        self.override_ctrl.jp.goal_qpos = self.q_start + (self.q_target - self.q_start) * frac
        return np.array([0.0] * 6 + [self.grip])

    def _path_collision(self, path_steps):
        """SafeLoop's path check, run kinematically on the path points after the start: any contact of an arm geom
        (robot0_* bodies other than the hand) with anything, the arm itself included, is a collision; gripper
        contacts are ignored. Runs on a copy of the simulator data, so the episode itself is untouched (writing and
        restoring qpos would still change the solver warm start and the derived quantities). Returns the first
        colliding body pair and path point, or None."""
        m, d = self.sim.model._model, self.sim.data._data
        robot = self.env.robots[0]
        arm = set()
        for g in range(m.ngeom):
            body = m.body(m.geom_bodyid[g]).name
            if "robot0" in body and not any(k in (body + m.geom(g).name).lower() for k in self.GRIPPER_KEYWORDS):
                arm.add(g)
        scratch = copy.copy(d)  # MjData copy (mj_copyData)
        for k in range(1, path_steps + 1):
            scratch.qpos[robot._ref_joint_pos_indexes] = self.q_start + (self.q_target - self.q_start) * (k / path_steps)
            scratch.qvel[robot._ref_joint_vel_indexes] = 0
            mujoco.mj_forward(m, scratch)
            for c in scratch.contact[: scratch.ncon]:
                if c.geom1 in arm or c.geom2 in arm:
                    return [m.body(m.geom_bodyid[c.geom1]).name, m.body(m.geom_bodyid[c.geom2]).name, k]
        return None


class RewindIL(JointSpaceRecovery):
    """Rewind-IL-style checkpoint respawn (arXiv 2604.16683, RA-L 2026) with oracle detection and oracle subgoal
    recognition: Rewind-IL recognises VLM-chosen subgoal checkpoints ("just after a grasp closes", "sub-goal
    transition") by policy-feature similarity and, on failure, returns to its own joint configuration from when it
    passed the latest one. Here the checkpoints are simulator conditions read from the BDDL goal, tracked throughout
    the episode, each debounced (it switches on / off only after STABLE_STEPS consecutive steps):
      - grasp:<obj>: a goal object (first argument of an in/on goal) touches the gripper and is more than LIFT_HEIGHT
        above its rest height (see _track_rest);
      - goal:<predicate>: a goal predicate holds (for in/on, with the object no longer touching the gripper).
    A checkpoint (joint angles, gripper state) is recorded each time a condition switches on (the latest kept);
    conditions on at the start (e.g. a stove already on) only count after switching off and on again. At the trigger
    the arm returns to the most recent checkpoint whose condition is still on and that was recorded more than
    PEAK_GAP steps earlier (Rewind-IL's peak gap: a slot is a respawn target only once its similarity has not
    improved for that many steps, i.e. once the robot has moved on from it), by joint-space interpolation at
    JOINT_SPEED (no path check: Rewind-IL assumes a free workspace), and control returns on arrival (or after
    MAX_STEPS). With none, nothing is done: the start pose is not a checkpoint (Rewind-IL's Algorithm 1 rewinds only
    when a passed checkpoint exists and otherwise executes the policy's action; it contrasts itself with fixed-pose
    resets)."""

    STABLE_STEPS, LIFT_HEIGHT = 5, 0.01  # steps, m
    PEAK_GAP = 30  # steps; Rewind-IL's peak gap: a checkpoint is a target only once more than this many steps old
    JOINT_SPEED, MAX_STEPS, ARRIVE_TOL_DEG = 0.08, 120, 1.0  # rad/step (L2 over the 7 joints), steps, deg

    def reset(self, obs):
        super().reset(obs)
        e = self.env
        self.goals = {f"goal:{g[0]}({','.join(g[1:])})": g for g in e.parsed_problem["goal_state"]}
        self.objects = list(dict.fromkeys(g[1] for g in self.goals.values()
                                          if g[0] in ("in", "on") and g[1] in e.objects_dict))
        self._init_rest(self.objects)
        self.checkpoints, self.log = {}, []  # key -> (step, joint angles, gripper state); log of [step, key]
        self.state = self._conditions()  # Debounced condition states
        self.count = {k: 0 for k in self.state}

    def _conditions(self):
        touching = self._track_rest()
        conds = {f"grasp:{o}": o in touching and self._lifted(o, self.LIFT_HEIGHT) for o in self.objects}
        for key, g in self.goals.items():
            conds[key] = bool(self.env._eval_predicate(g)) and g[1] not in touching
        return conds

    def observe(self, t, obs, action):
        for key, c in self._conditions().items():
            if c == self.state[key]:
                self.count[key] = 0
                continue
            self.count[key] += 1
            if self.count[key] >= self.STABLE_STEPS:
                self.state[key], self.count[key] = c, 0
                if c:
                    self.checkpoints[key] = (t, self._joint_qpos(), self._gripper_state())
                    self.log.append([t, key])
        if self.phase != "rollback":
            return
        self._record_contacts()
        err = np.degrees(np.abs(self._joint_qpos() - self.q_target)).max()
        arrived = self.n >= self.n_interp and err < self.ARRIVE_TOL_DEG
        if arrived or self.n >= self.MAX_STEPS:
            self.info.update(steps=self.n, arrived=bool(arrived), joint_err_deg=round(float(err), 2))
            self._finish()

    def activate(self, t, obs):
        valid = sorted((step, key) for key, (step, _, _) in self.checkpoints.items()
                       if self.state[key] and (t - 1) - step > self.PEAK_GAP)
        self.info = {"rollback": False, "valid": [key for _, key in valid], "held": self._held_objects()}
        if not valid:
            self.info["skipped"] = "no_valid_checkpoint"
            return
        step, key = valid[-1]
        _, self.q_target, self.grip = self.checkpoints[key]
        self.q_start = self._joint_qpos()
        self._swap_to_joint()
        self.n_interp = max(1, int(np.ceil(np.linalg.norm(self.q_target - self.q_start) / self.JOINT_SPEED)))
        self.phase, self.n = "rollback", 0
        self.info.update(rollback=True, target=key, target_step=step, age=(t - 1) - step, gripper=self.grip)

    def override(self, t, obs):
        if self.phase != "rollback":
            return None
        self.n += 1
        frac = min(self.n / self.n_interp, 1.0)
        self.override_ctrl.jp.goal_qpos = self.q_start + (self.q_target - self.q_start) * frac
        return np.array([0.0] * 6 + [self.grip])

    def record(self):
        return {**super().record(), "checkpoint_log": self.log}


class VoLo(JointSpaceRecovery):
    """VoLo-style grasp tool (arXiv 2606.07723, CoRL 2026) with oracle detection, routing and perception. VoLo's VLM
    orchestrator calls grasp(target) = segmentation -> GraspGen -> multi-start IK -> joint trajectory, and then returns
    control to the VLA. Here, at the trigger, the tool is chosen from the simulator state: grasp the perturbed target
    if it is not in the gripper, its goal is not met, and it was disturbed (it was held before the perturbation, or
    moved more than MOVED_DIST, or rotated more than TIPPED_DEG since: tipped or spun in place, either is off the
    demonstrations), else continue (the policy goes on; also when no grasp candidate is feasible). The grasp (grasping.plan_grasp: pre-generated GraspGen candidates with
    conf >= 0.8 and opening <= 7.8 cm, best confidence first, VoLo's IK, collision checks) is executed in joint space:
    to the pre-grasp at MOVE_SPEED with the gripper opening, straight approach, CLOSE_STEPS of closing, straight lift;
    then control returns to the policy (one attempt; a failed grasp is handed back as well, as in VoLo). VoLo's place,
    rewrite and replan tools are not used: the policy places the object, with the original instruction."""

    MOVED_DIST, TIPPED_DEG, HOLD_LIFT = 0.02, 30.0, 0.01  # m, deg, m (held = touching the closed gripper and lifted)
    # ARRIVE_TOL_DEG: 1 deg left the hand 0.5-0.8 cm short of the grasp pose; 0.1 deg reaches it within 1 mm
    MOVE_SPEED, ARRIVE_TOL_DEG, FINGER_OPEN = 0.08, 0.1, 0.03  # rad/step (L2), deg, m (each finger joint)
    MOVE_MAX, APPROACH_MAX, CLOSE_STEPS, LIFT_MAX = 120, 40, 10, 40  # steps

    def reset(self, obs):
        super().reset(obs)
        self._init_rest(self.env.objects_dict)
        self.target = self.perturb.target
        self.goal = next((g for g in self.env.parsed_problem["goal_state"] if len(g) > 1 and g[1] == self.target),
                         None)
        self.held_last, self.pose_last, self.was_held, self.pose0 = False, self._target_pose(), None, None

    def _target_pose(self):
        b = self.env.obj_body_id[self.target]
        return self.sim.data.body_xpos[b].copy(), self.sim.data.body_xquat[b].copy()

    def _held(self):
        return self.target in self._held_objects() and self._lifted(self.target, self.HOLD_LIFT)

    def _fingers(self):
        g = self.env.robots[0].gripper
        return [self.sim.data.qpos[self.sim.model._model.jnt_qposadr[self.sim.model._model.joint(j).id]]
                for j in g.joints]

    def observe(self, t, obs, action):
        self._track_rest()
        if self.perturb.fired_step is None:  # Keep the state just before the perturbation
            self.held_last, self.pose_last = self._held(), self._target_pose()
        elif self.pose0 is None:
            self.was_held, self.pose0 = self.held_last, self.pose_last
        if self.phase is None:
            return
        self._record_contacts()
        q = self._joint_qpos()
        err = lambda q_goal: np.degrees(np.abs(q - q_goal)).max()
        if self.phase == "move":
            opened = min(abs(f) for f in self._fingers()) >= self.FINGER_OPEN
            if (self.n >= self.n_interp and err(self.plan["q_pre"]) < self.ARRIVE_TOL_DEG and opened) \
                    or self.n >= self.MOVE_MAX:
                self._next("approach", timeout=self.n >= self.MOVE_MAX)
        elif self.phase == "approach":
            if (self.n >= len(self.plan["approach"]) and err(self.plan["approach"][-1]) < self.ARRIVE_TOL_DEG) \
                    or self.n >= self.APPROACH_MAX:
                self._next("close", timeout=self.n >= self.APPROACH_MAX)
        elif self.phase == "close":
            if self.n >= self.CLOSE_STEPS:
                self._next("lift")
        elif self.phase == "lift":
            if (self.n >= len(self.plan["lift"]) and err(self.plan["lift"][-1]) < self.ARRIVE_TOL_DEG) \
                    or self.n >= self.LIFT_MAX:
                self.info["steps"]["lift"] = self.n
                self.info["timeouts"] += ["lift"] if self.n >= self.LIFT_MAX else []
                self.info["grasped"] = bool(self._held())
                self._finish()

    def _next(self, phase, timeout=False):
        self.info["steps"][self.phase] = self.n
        if timeout:
            self.info["timeouts"].append(self.phase)
        self.phase, self.n = phase, 0

    def activate(self, t, obs):
        import time

        import src.panda.grasping as grasping

        pos0, quat0 = self.pose0 if self.pose0 is not None else self.pose_last
        pos, quat = self._target_pose()
        moved = float(np.linalg.norm(pos - pos0))
        tilt = float(np.degrees((Rot.from_quat(quat[[1, 2, 3, 0]]) * Rot.from_quat(quat0[[1, 2, 3, 0]]).inv())
                                .magnitude()))
        held_now = self._held()
        goal_done = bool(self.env._eval_predicate(self.goal)) if self.goal else False
        disturbed = bool(self.was_held) or moved > self.MOVED_DIST or tilt > self.TIPPED_DEG
        self.info = {"tool": "continue", "was_held": bool(self.was_held), "moved_cm": round(100 * moved, 1),
                     "tilt_deg": round(tilt, 1), "held_now": bool(held_now), "goal_done": goal_done}
        if held_now or goal_done or not disturbed:
            return
        t0 = time.time()
        self.plan, stats = grasping.plan_grasp(self.env, self.target)
        self.info.update(plan_stats=stats, plan_seconds=round(time.time() - t0, 2))
        if self.plan is None:
            self.info["skipped"] = "no_feasible_grasp"
            return
        self._swap_to_joint()
        self.q_start = self._joint_qpos()
        self.n_interp = max(1, int(np.ceil(np.linalg.norm(self.plan["q_pre"] - self.q_start) / self.MOVE_SPEED)))
        self.phase, self.n = "move", 0
        self.info.update(tool="grasp", grasp_index=self.plan["index"], grasp_conf=round(self.plan["conf"], 3),
                         approach_axis=np.round(self.plan["approach_axis"], 3).tolist(), steps={}, timeouts=[],
                         grasped=False)

    def override(self, t, obs):
        if self.phase is None:
            return None
        self.n += 1
        jp = self.override_ctrl.jp
        if self.phase == "move":
            frac = min(self.n / self.n_interp, 1.0)
            jp.goal_qpos = self.q_start + (self.plan["q_pre"] - self.q_start) * frac
            grip = -1.0
        elif self.phase == "approach":
            jp.goal_qpos = self.plan["approach"][min(self.n, len(self.plan["approach"])) - 1]
            grip = -1.0
        elif self.phase == "close":
            jp.goal_qpos = self.plan["approach"][-1]
            grip = 1.0
        else:  # lift
            jp.goal_qpos = self.plan["lift"][min(self.n, len(self.plan["lift"])) - 1]
            grip = 1.0
        return np.array([0.0] * 6 + [grip])


class Imagination:
    """A second MjSim on the live model in which the policy's imagined continuation runs, so the episode's simulator is
    never touched: the robot is driven as in env.step (a copy of the live OSC controller bound to this sim, with the
    live controller's null-space posture, robosuite's Panda gripper logic, the same physics substeps), and the policy's
    observation is rendered with this sim's own context (same cameras, resolution, visible geom groups and image
    convention as the live observations). One per process; the render context is freed after each rollout."""

    _sim = None

    def __init__(self, env, render=True):
        from robosuite.utils.binding_utils import MjRenderContextOffscreen, MjSim

        model = env.sim.model._model
        Imagination.release(env)  # a fresh sim per rollout: identical start, recording or replaying
        Imagination._sim = MjSim(model)
        if render:
            MjRenderContextOffscreen(Imagination._sim, device_id=env.render_gpu_device_id)  # registers itself
            Imagination._sim._render_context_offscreen.vopt.geomgroup[:] = \
                env.sim._render_context_offscreen.vopt.geomgroup
            env.sim._render_context_offscreen.gl_ctx.make_current()  # creating a context makes it current
        self.env, self.sim, self.m, self.d = env, Imagination._sim, model, Imagination._sim.data._data
        robot = env.robots[0]
        self.robot, self.site, self.gripper_qadr = robot, robot.eef_site_id, robot._ref_gripper_joint_pos_indexes
        self.eef_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, robot.robot_model.eef_name)
        self.arm_act, self.torque_limits = robot._ref_joint_actuator_indexes, robot.torque_limits
        self.grip_act = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in robot.gripper.actuators]
        rng = model.actuator_ctrlrange[self.grip_act]
        self.grip_bias, self.grip_weight = 0.5 * (rng[:, 1] + rng[:, 0]), 0.5 * (rng[:, 1] - rng[:, 0])
        self.substeps = int(round(env.control_timestep / env.model_timestep))

    @staticmethod
    def release(env):
        """Free the sim and its render context, and make the live context current again: robosuite renders with
        whatever EGL context is current."""
        if Imagination._sim is not None:
            rendering = Imagination._sim._render_context_offscreen is not None
            if rendering:
                # MuJoCo's GL objects are deleted in whatever context is current, and the two contexts number their
                # objects alike: make this one current first, or the live context's buffers and textures get deleted.
                # No other reference to the context may survive free(), or its deletion runs later, in the live one.
                Imagination._sim._render_context_offscreen.gl_ctx.make_current()
            Imagination._sim.free()
            Imagination._sim = None
            if rendering:
                import OpenGL.EGL as EGL

                EGL.eglBindAPI(EGL.EGL_OPENGL_API)  # freeing the context released the thread (API back to OpenGL ES)
        env.sim._render_context_offscreen.gl_ctx.make_current()

    def reset(self, qpos, qvel, grip_cmd):
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[:], self.d.qvel[:] = qpos, qvel
        mujoco.mj_forward(self.m, self.d)
        cfg = dict(self.robot.controller_config, sim=self.sim)
        self.osc = controller_factory(cfg["type"], cfg)
        self.osc.update_initial_joints(np.array(self.robot.controller.initial_joint))  # same null-space posture
        self.osc.update(force=True)
        self.osc.reset_goal()
        self.grip_cmd = np.broadcast_to(np.asarray(grip_cmd, dtype=float), (2,)).copy()

    def step(self, action):
        """One env step with the env action (as robosuite: controller goal on the first substep, torques and gripper
        control every substep)."""
        a = np.asarray(action, dtype=float)
        for i in range(self.substeps):
            mujoco.mj_forward(self.m, self.d)
            if i == 0:
                self.osc.set_goal(a[:-1])
            torques = np.clip(self.osc.run_controller(), *self.torque_limits)
            self.grip_cmd = np.clip(self.grip_cmd + np.array([-1.0, 1.0]) * 0.01 * np.sign(a[-1]), -1.0, 1.0)
            self.d.ctrl[self.grip_act] = self.grip_bias + self.grip_weight * self.grip_cmd
            self.d.ctrl[self.arm_act] = torques
            mujoco.mj_step(self.m, self.d)
        mujoco.mj_forward(self.m, self.d)

    def arm_q(self):
        return self.d.qpos[self.robot._ref_joint_pos_indexes].copy()

    def tcp(self):
        return self.d.site_xpos[self.site].copy()

    def observe(self):
        """Observation dict (the keys the policy reads) of the current imagined state."""
        self.sim._render_context_offscreen.gl_ctx.make_current()  # robosuite renders with the current context
        try:
            obs = {f"{cam}_image": self.sim.render(camera_name=cam, width=w, height=h)
                   for cam, h, w in zip(self.env.camera_names, self.env.camera_heights, self.env.camera_widths)}
        finally:
            self.env.sim._render_context_offscreen.gl_ctx.make_current()
        quat = self.d.xquat[self.eef_body]
        obs.update(robot0_eef_pos=self.tcp(), robot0_eef_quat=quat[[1, 2, 3, 0]].copy(),
                   robot0_gripper_qpos=self.d.qpos[self.gripper_qadr].copy())
        return obs


class Core(JointSpaceRecovery):
    """CoRe (arXiv 2608.14822) with oracle detection, executed physically. CoRe takes as recovery anchor the latest
    in-distribution frame with the gripper clear of every movable object, imagines the frozen policy's continuation
    from it (rolling the policy out on synthesized observations of the anchor's scene with the arm advanced by forward
    kinematics, until the TCP comes within d_prox of an object), takes the latest frame before that as the rejoin state
    c*, returns the objects that must be restored to their c* poses by motion-planned pick-and-place (verified, one
    retry, an intervention request when unrecoverable), moves the arm to c*'s configuration and hands back to the
    policy with the unexecuted remainder of the imagined chunk.
    Here: the anchor is the latest frame before the perturbation fired with the TCP at least CLEARANCE from every movable
    object's bounding box (the start of the episode if none). The continuation is imagined on a second simulator
    (Imagination) started from the anchor state, the robot driven by a copy of its own controller (CoRe integrates the
    arm by forward kinematics) and the observations rendered there (CoRe synthesizes them from RGB-D voxel stores),
    for at most ROLLOUT_TICKS queries. Only the target is restored (the perturbations displace only the
    target), when it is off its c* pose (beyond DISP_XY / DISP_Z / DISP_ROT): pick (grasping.plan_pick_place: the
    pre-generated GraspGen candidates, VoLo's IK, collision checks with the carried object; moves to the pre-grasp via
    the via height if a straight joint line collides, CoRe's three-segment transport at it, grasping.via_height),
    carry, set down, retreat,
    verified to VERIFY_POS with one retry; if no pick-and-place is feasible, the intervention request is recorded and
    the policy goes on from c*. The arm then moves to c* (joint interpolation, gripper at its c* command)."""

    CLEARANCE = 0.08  # m, TCP to object bounding box (CoRe LIBERO: anchor clearance and rollout d_prox)
    DISP_XY, DISP_Z, DISP_ROT = 0.03, 0.005, 0.30  # m, m, rad: displaced versus the rejoin pose (CoRe Table A4)
    VERIFY_POS, RETRIES = 0.03, 1  # m; retries of the restoration (CoRe: verified, one retry)
    ROLLOUT_TICKS = 15  # policy queries of the imagined continuation (CoRe rollout cap)
    MOVE_SPEED, ARRIVE_TOL_DEG, REJOIN_TOL_DEG, SETTLE_MAX = 0.08, 0.1, 1.0, 40  # rad/step, deg, deg, steps
    FINGER_OPEN, CLOSE_STEPS, OPEN_STEPS = 0.03, 10, 10  # m (each finger), steps, steps
    _CUBE = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], dtype=float)

    def reset(self, obs):
        super().reset(obs)
        e, m, d = self.env, self.sim.model._model, self.sim.data._data
        self.target = self.perturb.target
        self.z0 = float(obs["robot0_eef_pos"][2])  # TCP height at the start: the via height's floor (rehome's lift)
        self.joints = {o: obj.joints[0] for o, obj in e.objects_dict.items() if obj.joints}
        self.boxes = {}  # body-frame bounding box of the collision geoms (centre, half extents)
        for o in self.joints:
            b = e.obj_body_id[o]
            R_b, p_b = d.xmat[b].reshape(3, 3), d.xpos[b]
            corners = []
            for name in e.objects_dict[o].contact_geoms:
                g = m.geom(name).id
                local = m.geom_aabb[g][:3] + m.geom_aabb[g][3:] * self._CUBE
                corners.append((local @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g] - p_b) @ R_b)
            corners = np.concatenate(corners)
            self.boxes[o] = (0.5 * (corners.max(0) + corners.min(0)), 0.5 * (corners.max(0) - corners.min(0)))
        self.frames = []  # the trace: the state after step frames[i]["t"] (0 = after the wait steps)
        self._trace(0, obs)

    def _trace(self, t, obs):
        """Record the state after step t (until the trigger: later frames cannot be anchors)."""
        if not self.trigger_steps:
            self.frames.append(self._frame(t, obs))

    def _frame(self, t, obs):
        return {"t": t, "qpos": self.sim.data._data.qpos.copy(), "qvel": self.sim.data._data.qvel.copy(),
                "grip_cmd": np.array(self.env.robots[0].gripper.current_action, dtype=float),
                "tcp": np.array(obs["robot0_eef_pos"], dtype=float), "grip": self._gripper_state(),
                "poses": {o: self.sim.data.get_joint_qpos(j).copy() for o, j in self.joints.items()}}

    @staticmethod
    def _rot(pose):
        return Rot.from_quat(pose[[4, 5, 6, 3]])  # free-joint qpos: x y z qw qx qy qz

    def _clearance(self, tcp, poses):
        """Distance from the TCP to the nearest movable object's bounding box."""
        dist = np.inf
        for o, (c, h) in self.boxes.items():
            local = self._rot(poses[o]).inv().apply(tcp - poses[o][:3])
            dist = min(dist, float(np.linalg.norm(np.maximum(np.abs(local - c) - h, 0.0))))
        return dist

    def _deviation(self, o, ref):
        cur = self.sim.data.get_joint_qpos(self.joints[o])
        return (float(np.linalg.norm(cur[:2] - ref[:2])), float(abs(cur[2] - ref[2])),
                float((self._rot(cur) * self._rot(ref).inv()).magnitude()))

    # --- counterfactual continuation ------------------------------------------------------------------------------
    def _imagine(self, anchor):
        """Roll the policy out from the anchor in the imagined scene; returns the rejoin state."""
        recorded = None if self.query else self._recorded_imagined().get("chunks", [])
        im = Imagination(self.env, render=self.query is not None)
        try:
            im.reset(anchor["qpos"], anchor["qvel"], anchor["grip_cmd"])
            q, fingers, grip = im.arm_q(), im.d.qpos[im.gripper_qadr].copy(), anchor["grip"]
            out = {"chunks": [], "steps": 0, "end": "cap", "handoff": []}
            for tick in range(self.ROLLOUT_TICKS):
                if self.query:
                    chunk = [np.asarray(a) for a in self.query(im.observe())]
                elif tick < len(recorded):  # replay: the recorded policy outputs
                    chunk = [np.asarray(a, dtype=self._recorded_imagined()["dtype"]) for a in recorded[tick]]
                else:
                    break
                out["dtype"] = str(chunk[0].dtype)
                out["chunks"].append([[float(x) for x in a] for a in chunk])
                for j, raw in enumerate(chunk):
                    a = np.asarray(self.process_action(raw), dtype=float)
                    im.step(a)
                    if self._clearance(im.tcp(), anchor["poses"]) < self.CLEARANCE:
                        out.update(end="proximity", handoff=chunk[j:])
                        return {**out, "q": q, "fingers": fingers, "grip": grip}
                    q, fingers, grip = im.arm_q(), im.d.qpos[im.gripper_qadr].copy(), float(np.sign(a[-1]) or grip)
                    out["steps"] += 1
            return {**out, "q": q, "fingers": fingers, "grip": grip}
        finally:
            Imagination.release(self.env)

    def _recorded_imagined(self):
        """Replay: the recorded imagined rollout of the current recovery."""
        return (self.recorded or {}).get("imagined", {})

    # --- physical restoration -------------------------------------------------------------------------------------
    def _restoration_phases(self, plan):
        P = [] if plan["held"] else ([{"name": "via", "path": plan["via"], "grip": -1.0}] if plan.get("via") else []) + [
            {"name": "move", "to": plan["q_pre"], "grip": -1.0, "need_open": True},
            {"name": "approach", "path": plan["approach"], "grip": -1.0},
            {"name": "close", "hold": self.CLOSE_STEPS, "grip": 1.0},
        ]
        return P + [{"name": "lift", "path": plan["lift"], "grip": 1.0},
                    {"name": "check_held"},  # a missed grasp is not carried around: straight to the verification
                    {"name": "transport", "path": plan["transport"], "grip": 1.0},
                    {"name": "descend", "path": plan["descend"], "grip": 1.0},
                    {"name": "release", "hold": self.OPEN_STEPS, "grip": -1.0},
                    {"name": "retreat", "path": plan["retreat"], "grip": -1.0},
                    {"name": "verify"}]

    def _plan_restoration(self):
        """Phases restoring the target (or none, with the intervention request recorded)."""
        import src.panda.grasping as grasping

        held = self.target in self._held_objects()
        t0 = time.time()
        plan, stats = grasping.plan_pick_place(self.env, self.target, self.goal, held=held, z0=self.z0)
        self.info["attempts"].append({"held": held, "plan_stats": stats, "plan_seconds": round(time.time() - t0, 2),
                                      "grasp_index": None if plan is None else plan.get("index")})
        if plan is not None:
            return self._restoration_phases(plan)
        if held:  # the current hold cannot set it down: let go, then pick it up like a dropped object
            return [{"name": "drop", "hold": self.OPEN_STEPS + self.SETTLE_MAX // 4, "grip": -1.0}, {"name": "replan"}]
        self.info["intervention_request"] = "no_feasible_pick_place"
        return []

    def _may_restore(self):
        """Whether the displaced target may be restored now (always; core_detect caps it per episode)."""
        return True

    # --- control --------------------------------------------------------------------------------------------------
    def _anchor(self):
        """The recovery anchor and whether it is the fallback: the latest frame before the perturbation fired with the
        TCP at least CLEARANCE from every movable object (the episode start if none)."""
        ok = [f for f in self.frames[: self.perturb.fired_step]
              if self._clearance(f["tcp"], f["poses"]) >= self.CLEARANCE]
        return (ok[-1], False) if ok else (self.frames[0], True)

    def activate(self, t, obs):
        anchor, fallback = self._anchor()
        t0 = time.time()
        self.rejoin = self._imagine(anchor)
        self.goal = anchor["poses"][self.target]  # the scene is static in the imagined continuation
        self.q_target = self.rejoin["q"]
        dev = self._deviation(self.target, self.goal)
        displaced = dev[0] > self.DISP_XY or dev[1] > self.DISP_Z or dev[2] > self.DISP_ROT
        self.info = {"anchor_step": anchor["t"], "back_steps": (t - 1) - anchor["t"],
                     "clearance_cm": round(100 * self._clearance(anchor["tcp"], anchor["poses"]), 1),
                     "fallback": fallback,
                     "imagined": {key: self.rejoin[key] for key in ("chunks", "steps", "end", "dtype")
                                  if key in self.rejoin},
                     "imagine_seconds": round(time.time() - t0, 2), "handoff_len": len(self.rejoin["handoff"]),
                     "gripper": self.rejoin["grip"], "held": self._held_objects(),
                     "target_dev": [round(100 * dev[0], 1), round(100 * dev[1], 1), round(dev[2], 2)],
                     "displaced": bool(displaced), "attempts": [], "placements": 0, "missed_grasps": 0,
                     "placed_cm": None, "restored": False,
                     "intervention_request": None, "steps": {}, "timeouts": []}
        self._swap_to_joint()
        self.phases = self._plan_restoration() if displaced and self._may_restore() else []
        self.phases.append({"name": "rejoin", "to": self.q_target, "grip": self.rejoin["grip"],
                            "tol": self.REJOIN_TOL_DEG})
        self._next_phase()

    def _next_phase(self):
        while self.phases:
            ph = self.phases.pop(0)
            if ph["name"] == "check_held":
                if self.target not in self._held_objects():
                    self.info["missed_grasps"] += 1
                    while self.phases[0]["name"] != "verify":
                        self.phases.pop(0)
                continue
            if ph["name"] == "verify":
                err = float(np.linalg.norm(self.sim.data.get_joint_qpos(self.joints[self.target])[:3] - self.goal[:3]))
                self.info["placements"] += 1
                self.info["placed_cm"] = round(100 * err, 1)
                self.info["restored"] = err <= self.VERIFY_POS
                if not self.info["restored"] and self.info["placements"] <= self.RETRIES:
                    self.phases = self._plan_restoration() + self.phases
                continue
            if ph["name"] == "replan":
                self.phases = self._plan_restoration() + self.phases
                continue
            if "to" in ph:  # a move from wherever the arm is now
                import src.panda.grasping as grasping

                ph["path"] = grasping.path(self._joint_qpos(), ph["to"], self.MOVE_SPEED)
            self.ph, self.phase, self.n = ph, ph["name"], 0
            return
        self.phase = None
        self.handoff = list(self.rejoin["handoff"])
        self._finish()

    def override(self, t, obs):
        if self.phase is None:
            return None
        self.n += 1
        ph = self.ph
        if "path" in ph:
            self.override_ctrl.jp.goal_qpos = ph["path"][min(self.n, len(ph["path"])) - 1]
        elif self.override_ctrl.jp.goal_qpos is None:  # a hold first (letting go of a held target): stay put
            self.override_ctrl.jp.goal_qpos = self._joint_qpos()
        return np.array([0.0] * 6 + [ph["grip"]])

    def observe(self, t, obs, action):
        if self.phase is None:
            self._trace(t, obs)
            return
        self._record_contacts()
        ph = self.ph
        if "hold" in ph:
            done, timeout = self.n >= ph["hold"], False
        else:
            err = np.degrees(np.abs(self._joint_qpos() - ph["path"][-1])).max()
            fingers = self.sim.data.qpos[self.env.robots[0]._ref_gripper_joint_pos_indexes]
            opened = not ph.get("need_open") or min(abs(fingers)) >= self.FINGER_OPEN
            timeout = self.n >= len(ph["path"]) + self.SETTLE_MAX
            done = (self.n >= len(ph["path"]) and err < ph.get("tol", self.ARRIVE_TOL_DEG) and opened) or timeout
        if done:
            self.info["steps"][ph["name"]] = self.info["steps"].get(ph["name"], 0) + self.n
            if timeout:
                self.info["timeouts"].append(ph["name"])
            if ph["name"] == "rejoin":
                self.info.update(arrived=not timeout, joint_err_deg=round(float(err), 2))
            self._next_phase()


class LatentDetector:
    """CoRe's online OOD score for one task (Eq. 2): the Mahalanobis distance of the z-standardized action-head latent
    to the task's successful clean rollouts, with the ridge-regularized covariance, and the threshold tau (a quantile
    of the scores of held-out successful rollouts). Built by calibrate_detector.py; one per process (the precision
    matrix is 64 MB)."""

    _loaded = {}

    def __init__(self, path):
        z = np.load(path)
        self.mu, self.sigma, self.zbar = z["mu"], z["sigma"], z["zbar"]
        self.prec, self.tau = z["prec"], float(z["tau"])

    @classmethod
    def for_task(cls, directory, task):
        index = json.load(open(os.path.join(directory, "index.json")))
        path = os.path.join(directory, index[task])
        if path not in cls._loaded:
            cls._loaded = {path: cls(path)}
        return cls._loaded[path]

    def score(self, h):
        d = ((np.asarray(h, dtype=np.float64) - self.mu) / self.sigma - self.zbar).astype(np.float32)
        return float(np.sqrt(max(float(d @ (self.prec @ d)), 0.0)))


class CoreDetect(Core):
    """core with CoRe's own detection instead of the oracle trigger (CoRe Algorithm A1, Table A4). At every policy
    query the action head's latent (the last hidden state before the output layer, mean-pooled over the chunk; OFT's
    head is deterministic, so one pass instead of CoRe's K = 8 sampled ones) is scored against the task's clean
    successful rollouts (LatentDetector); a recovery fires, instead of executing the queried chunk, when the score
    exceeds tau for N_CONSEC consecutive queries or PLATEAU_FRAC * tau for PLATEAU_N, at least REFRACTORY queries after
    the previous one. Each recovery is core's: the anchor is the latest in-distribution frame of the trace with the TCP
    at least CLEARANCE from every movable object, the trace is then truncated to it (later recoveries chain from this
    one, CoRe), and the target is restored at most RESTORE_CAP times per episode (CoRe's restore cap; counted when a
    restoration is started). In-distribution, as in core, means before the perturbation; frames after a recovery's
    handoff are in-distribution again unless the perturbation (still) acted at or after that recovery's trigger.
    Replays take the detections and the imagined rollouts from the record."""

    N_CONSEC, PLATEAU_N, PLATEAU_FRAC, REFRACTORY, RESTORE_CAP = 3, 5, 0.9, 16, 1  # queries, queries, -, queries, -

    def reset(self, obs):
        self.realigned = 0  # trigger step of the latest recovery: perturbations ending before it are undone
        super().reset(obs)
        self.detector = LatentDetector.for_task(self.detector_dir, self.task) if self.query else None
        self.scores, self.recoveries, self.restorations = [], [], 0
        self.above, self.near, self.since = 0, 0, None  # consecutive queries over tau / 0.9 tau; queries since trigger

    def _trace(self, t, obs):
        f = self._frame(t, obs)
        end = self.perturb.end_step
        f["in_dist"] = self.perturb.fired_step is None or (end is not None and end < self.realigned)
        self.frames.append(f)

    def _anchor(self):
        """The latest in-distribution frame with the TCP clear of every movable object (the episode start if none);
        the trace is truncated to it."""
        ok = [i for i, f in enumerate(self.frames)
              if f["in_dist"] and self._clearance(f["tcp"], f["poses"]) >= self.CLEARANCE]
        i = ok[-1] if ok else 0
        self.frames = self.frames[: i + 1]
        return self.frames[i], not ok

    def _may_restore(self):
        self.info["capped"] = self.restorations >= self.RESTORE_CAP
        self.restorations += not self.info["capped"]
        return not self.info["capped"]

    def _recorded_imagined(self):
        return self.recorded["recoveries"][len(self.trigger_steps) - 1]["imagined"]

    def triggered(self, t):
        """Replay: the recorded detections (live, `detect` decides)."""
        return self.query is None and t in (self.recorded or {}).get("trigger_steps", [])

    def detect(self, t, obs):
        s = self.detector.score(self.latent())
        self.scores.append([t, round(s, 3)])
        self.above = self.above + 1 if s > self.detector.tau else 0
        self.near = self.near + 1 if s > self.PLATEAU_FRAC * self.detector.tau else 0
        self.since = None if self.since is None else self.since + 1
        if (self.since is None or self.since >= self.REFRACTORY) and \
                (self.above >= self.N_CONSEC or self.near >= self.PLATEAU_N):
            self.trigger_steps.append(t)
            self.above, self.near, self.since = 0, 0, 0
            self.activate(t, obs)
            return True
        return False

    def activate(self, t, obs):
        self.realigned = t
        super().activate(t, obs)
        self.recoveries.append(self.info)

    def record(self):
        return {"trigger_steps": self.trigger_steps, "tau": None if self.detector is None else self.detector.tau,
                "scores": self.scores, "recoveries": self.recoveries, "contacts": self.contacts,
                "interrupted": self.phase is not None}


class CleanRollouts:
    """The clean rollouts of collect_clean_trajectories.py, one per initial state; per process, the current task's."""

    _index, _task = None, (None, None)

    @classmethod
    def match(cls, directory, task, env):
        """The task's clean rollout from this episode's initial state: the one whose objects start where they are
        now (called after the wait steps). Returns (rollout, largest object position difference in m)."""
        import glob
        import re

        if cls._index is None:
            cls._index = {}
            for p in sorted(glob.glob(os.path.join(directory, "task*_init0.npz"))):
                cls._index[str(np.load(p)["task"])] = int(re.search(r"task(\d+)_init", p).group(1))
        if cls._task[0] != task:
            starts = []
            for p in sorted(glob.glob(os.path.join(directory, f"task{cls._index[task]}_init*.npz"))):
                z = np.load(p)
                starts.append((p, list(z["bodies"]), z["body_pos"][0]))
            cls._task = (task, starts)
        d = env.sim.data
        best = None
        for p, bodies, pos0 in cls._task[1]:
            diff = max(float(np.linalg.norm(pos0[i] - d.body_xpos[env.obj_body_id[b]]))
                       for i, b in enumerate(bodies) if b in env.objects_dict)
            if best is None or diff < best[1]:
                best = (p, diff)
        z = np.load(best[0])
        return {key: z[key] for key in z.files} | {"path": best[0]}, best[1]


class Mire(JointSpaceRecovery):
    """MIRE (ours): minimal physical intervention in the current (perturbed) world and an early, validated handoff,
    instead of restoring the world (core) or rewinding (retreat, safeloop, rewind_il). Detection, perception and
    goals are oracles, as for the baselines (the goal positions come from the task's clean rollout from the same
    initial state, collect_clean_trajectories.py, relative to the BDDL goal's body). At the trigger the perturbed
    target's state routes the recovery (volo's rules):
      - held: carry it towards its goal;
      - disturbed (it was held and is not any more, or moved more than MOVED_DIST, or turned more than TIPPED_DEG
        since the perturbation, up to its symmetry: grasping.SYMMETRY) and not held: nothing if its goal is met, else
        grasp it (below) and carry it; nothing either if no grasp is feasible (given up);
      - untouched: nothing if the arm is on its clean path (TCP within ARM_DEV of the clean rollout's at the same
        step), else resume the clean rollout's current subgoal: for a pick-and-place, grasp its object (below)
        unless held, and carry it; for another subgoal (button, drawer, door), follow its clean joint path with a DMP.
    Deferred: when the target is disturbed while a subgoal that comes before its own in the clean rollout is still to
    be done (its BDDL goal condition false now, true in the clean rollout before the target's), MIRE leaves the target
    alone (resuming that subgoal if the arm was pushed off its path) and recovers it once that subgoal is done and the
    TCP comes within DEFER_DIST (horizontally) of the target or of where it stood before the perturbation, or
    DEFER_WAIT steps after that subgoal was done.
    Grasping: the policy's own grasp of the object in the clean rollout (object frame), turned by the object's
    symmetries, or a GraspGen candidate within NEAR_DEG and NEAR_DIST of it, the hand's world rotation closest to the
    policy's first (grasping.plan_own); if none is feasible, the object is set down in its pose before the clean grasp
    (orientation and height; where it stood, where it lies, or SET_DOWN_STEP aside) with a GraspGen grasp and taken
    from there with the own grasp (grasping.plan_set_down); else a GraspGen grasp (best confidence first, as volo) is
    used as it is. A held object is set down the same way (with its current hold) and taken again when its hold is
    more than HELD_DEG off the own grasp or its carry is infeasible.
    MIRE never finishes a subgoal itself: it goes at most to the clean rollout's arm pose HANDOFF_BEFORE steps before
    the release of the object (or the subgoal's end), and the policy places it. Carrying: a joint-space DMP
    (dmp.JointDMP) reproducing the clean rollout's carry of the object, from the current joints to that pose (the hand
    as the policy's there, moved with the goal's body; else its joints as they were), rolled out beforehand and
    checked for collisions with the held object (touching the goal's own body up to GOAL_CONTACT_TOL); while it
    collides a repulsive coupling term is switched on with growing gains (grasping.dmp_transport).
    Handoff: once the object is held for its goal (lifting, carrying; or along another subgoal's path; never before a
    grasp nor while setting it down), every CHECK_EVERY steps of the intervention the policy is queried on the live
    observation (shadow mode); it agrees when the mean translation of its first CHECK_ACTIONS actions is within COS_MIN
    (cosine) of the TCP motion planned for the next CHECK_ACTIONS steps (forward kinematics), all their gripper
    commands equal the current one, the hand is within WRIST_DEG of the clean rollout's hand at the matching row and
    the TCP within HANDOFF_DIST of the clean rollout's path of that subgoal (the policy takes over only in a state of
    its own). After N_AGREE consecutive agreements control returns
    to the policy with that chunk; otherwise at the pose HANDOFF_BEFORE steps before the goal, or right away (holding
    the object) if that pose cannot be reached or the grasp missed (after GRASP_RETRIES retries)."""

    MOVED_DIST, TIPPED_DEG, HOLD_LIFT = 0.02, 30.0, 0.01  # m, deg, m (volo)
    ARM_DEV, CARRY_MIN, CARRY_RISE = 0.08, 0.05, 0.01  # m (CoRe's delta); m, m: carried in the clean rollout
    GOAL_LAG = 10  # steps: a goal condition coming true this soon after a subgoal's end belongs to it
    MOVE_SPEED, ARRIVE_TOL_DEG, FOLLOW_TOL_DEG, FINGER_OPEN = 0.08, 0.1, 1.0, 0.03  # rad/step, deg, deg, m (volo)
    SETTLE_MAX, CLOSE_STEPS, GRASP_RETRIES = 40, 10, 1  # steps, steps, -
    CHECK_EVERY, CHECK_ACTIONS, N_AGREE, COS_MIN, MIN_MOTION = 4, 4, 3, 0.8, 0.002  # steps, actions, checks, -, m
    HANDOFF_BEFORE, MIN_DEMO = 40, 10  # steps before the clean release / subgoal end where control returns; rows
    CHECK_PHASES = ("lift", "transport", "follow")  # the object held (or another subgoal): never before a grasp
    NEAR_DEG, NEAR_DIST, HELD_DEG = 10.0, 0.02, 30.0  # deg, m: a grasp this close to the policy's own counts as its
    #                                                    own; deg: a held object's hold this far off it is retaken
    WRIST_DEG, HANDOFF_DIST = 30.0, 0.05  # deg: hand vs clean at handoff; m: TCP from the clean path at handoff
    MAX_VARIANTS, MAX_NEAR, MAX_PICKS, OPEN_STEPS = 12, 24, 4, 10  # tried: own-grasp variants, near GraspGen
    #                                                                  candidates; set-down picks kept; release steps
    SET_DOWN_STEP = 0.10  # m: set-down spots besides where it stood and where it lies: this far aside along +-x, +-y
    DEFER_DIST, DEFER_WAIT = 0.08, 80  # m (horizontal, to the target now or where it stood), steps: deferred recovery
    OWN_CONTACT_MARGIN = 0.002  # m: the own grasp may touch the object this much deeper than the policy's did

    def reset(self, obs):
        super().reset(obs)
        self._init_rest(self.env.objects_dict)
        self.target = self.perturb.target
        self.z0 = float(obs["robot0_eef_pos"][2])  # TCP height at the start: the via height's floor (rehome's lift)
        self.clean, self.match_err = CleanRollouts.match(self.clean_dir, self.task, self.env)
        self.units = self._units()
        self.held_last, self.pose_last, self.was_held, self.pose0 = False, self._pose(self.target), None, None
        self.checks, self.agreed, self.k, self.retries = [], 0, 0, 0
        self.info, self.deferred = {}, None

    # --- clean rollout ----------------------------------------------------------------------------------------------
    def _units(self):
        """Subgoals of the clean rollout: each closed-gripper interval [close, end) (steps) with what preceded it
        since the previous release ([start, close)); a pick-and-place of the object it carried, if one moved more
        than CARRY_MIN and rose, else another subgoal (button, drawer, door) if a BDDL goal condition on an
        articulated object came true in it (up to GOAL_LAG steps after), else a missed grasp, folded into the next
        subgoal's approach; plus the stretch from the last release to success if such a condition came true in it."""
        C = self.clean
        grip = np.sign(C["action"][:, 6])  # grip[s - 1]: gripper command at step s
        n, pos, bodies = len(grip), C["body_pos"], list(C["bodies"])
        units, prev = [], 0
        for tc in [s for s in range(1, n + 1) if grip[s - 1] > 0 and (s == 1 or grip[s - 2] <= 0)]:
            end = next((s for s in range(tc + 1, n + 1) if grip[s - 1] <= 0), n)
            carried, best = None, self.CARRY_MIN
            for i, b in enumerate(bodies):
                if b not in self.env.objects_dict:
                    continue
                moved = float(np.linalg.norm(pos[end - 1, i] - pos[tc - 1, i]))
                if moved > best and pos[tc - 1: end, i, 2].max() - pos[tc - 1, i, 2] > self.CARRY_RISE:
                    carried, best = b, moved
            units.append({"kind": "pnp" if carried else "other", "obj": carried, "start": prev, "close": tc,
                          "end": end})
            prev = end
        success = int(C["success_step"])
        if success > prev + 1:
            units.append({"kind": "other", "obj": None, "start": prev, "close": None, "end": success})
        # goals[row, i]: BDDL goal condition i true after step row; only conditions on articulated objects (not
        # placements: in / on) make a subgoal of a closed-gripper interval without carrying
        goals = C["goals"][:, [str(g).split()[0] not in ("in", "on") for g in C["goal_state"]]]
        kept, start = [], None
        for u in units:
            hi = min(u["end"] + self.GOAL_LAG, len(goals) - 1)
            if u["kind"] == "other" and not (goals[hi] & ~goals[u["start"]]).any():
                start = u["start"] if start is None else start  # a missed grasp: part of the next approach
                continue
            if start is not None:
                u["start"], start = start, None
            kept.append(u)
        return kept

    def _clean_pose(self, row, obj):
        i = list(self.clean["bodies"]).index(obj)
        return self._T(self.clean["body_pos"][row, i], self.clean["body_quat"][row, i])

    @staticmethod
    def _T(pos, quat_wxyz):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = Rot.from_quat(np.asarray(quat_wxyz)[[1, 2, 3, 0]]).as_matrix(), pos
        return T

    def _live_pose(self, obj):
        b = self.env.obj_body_id[obj]
        return self._T(self.sim.data.body_xpos[b], self.sim.data.body_xquat[b])

    def _pose(self, obj):
        b = self.env.obj_body_id[obj]
        return self.sim.data.body_xpos[b].copy(), self.sim.data.body_xquat[b].copy()

    def _held(self, obj):
        return obj in self._held_objects() and self._lifted(obj, self.HOLD_LIFT)

    def _hand(self):
        """World pose of the hand (GraspGen's gripper frame: the grip site moved back along its z)."""
        import src.panda.grasping as grasping

        site = self.env.robots[0].eef_site_id
        T = grasping.pose_matrix(self.sim.data.site_xpos[site], self.sim.data.site_xmat[site])
        T[:3, 3] -= grasping.GRIP_SITE_OFFSET * T[:3, 2]
        return T

    # --- routing --------------------------------------------------------------------------------------------------
    def activate(self, t, obs):
        self.info = {}
        self._route(t, obs, may_defer=True)

    def _pending(self):
        """Indices of the BDDL goal conditions that come true in the clean rollout before the target's own (and are
        not true at its start) but are false now: the subgoals still to be done before the target's turn."""
        goals, states = self.clean["goals"], self.env.parsed_problem["goal_state"]
        first = [int(np.argmax(goals[:, j])) if goals[:, j].any() else None for j in range(goals.shape[1])]
        own = [first[j] for j, g in enumerate(states) if self.target in g[1:] and first[j] is not None]
        if not own:
            return []
        return [j for j, g in enumerate(states) if self.target not in g[1:] and first[j] is not None
                and not goals[0, j] and first[j] < min(own) and not self.env._eval_predicate(g)]

    def _route(self, t, obs, may_defer):
        """Route the recovery on the target's state now (at the trigger, or when a deferred recovery fires)."""
        import src.panda.grasping as grasping

        self.k, self.agreed = 0, 0  # this intervention's steps (a deferred one starts anew)
        pos0, quat0 = self.pose0 if self.pose0 is not None else self.pose_last
        pos, quat = self._pose(self.target)
        moved = float(np.linalg.norm(pos - pos0))
        R_now, R0 = Rot.from_quat(quat[[1, 2, 3, 0]]).as_matrix(), Rot.from_quat(quat0[[1, 2, 3, 0]]).as_matrix()
        tilt = grasping.turn_deg(self.target, R_now, R0)  # up to the object's symmetry
        goal = next((g for g in self.env.parsed_problem["goal_state"] if len(g) > 1 and g[1] == self.target), None)
        goal_done = bool(self.env._eval_predicate(goal)) if goal else False
        held_now, disturbed = self._held(self.target), bool(self.was_held) or moved > self.MOVED_DIST \
            or tilt > self.TIPPED_DEG
        row = min(t - 1, len(self.clean["tcp"]) - 1)
        tcp = self.sim.data.site_xpos[self.env.robots[0].eef_site_id]
        arm_dev = float(np.linalg.norm(tcp - self.clean["tcp"][row]))
        self.fk = grasping.ArmKinematics(self.env, self.target)
        state = {"was_held": bool(self.was_held), "moved_cm": round(100 * moved, 1), "tilt_deg": round(tilt, 1),
                 "turn_raw_deg": round(float(grasping.rot_deg(R_now, R0)), 1), "held_now": bool(held_now),
                 "goal_done": goal_done, "arm_dev_cm": round(100 * arm_dev, 1)}
        self.info.update({"clean": os.path.basename(self.clean["path"]), "match_cm": round(100 * self.match_err, 2),
                          **state, "route": None, "steps": {}, "timeouts": [], "plans": [], "grasps": [],
                          "handoff": None, "start": t})
        pending = self._pending() if may_defer and not held_now and disturbed and not goal_done else []
        if pending:  # not its turn yet: the policy does the earlier subgoal first (back on its path if pushed off)
            states = self.env.parsed_problem["goal_state"]
            self.deferred = {"pending": [" ".join(states[j]) for j in pending], "at_trigger": state, "done": None,
                             "fire": None}
            self._pending_idx = pending
            if arm_dev > self.ARM_DEV:
                self._resume(row)
                self.deferred["resumed"] = self.info["route"]
            self.info["route"] = "deferred"
            return
        if held_now:  # carried as held, unless its hold is far off the policy's own: set down and taken again
            self.info["route"] = "carry"
            own = self._own_grasp(self.target, row)
            if own is not None:
                G_now = np.linalg.inv(self._live_pose(self.target)) @ self._hand()
                self.info["held_dev_deg"] = round(float(grasping.grasp_deg(
                    self.target, G_now[None], own["G_b"], flip=grasping.flip_equivalent(self.target, own["G_b"]))[0]), 1)
            if not (self.info.get("held_dev_deg", 0.0) > self.HELD_DEG and self._set_down_held(self.target, row,
                                                                                               "turned_in_hand")):
                self._carry(self.target, row, fresh=False, route="carry")
        elif disturbed:
            if goal_done:
                self.info["route"] = "none:goal_done"
            else:
                self._grasp(self.target, row, route="grasp")
        elif arm_dev <= self.ARM_DEV:
            self.info["route"] = "none:on_path"
        else:
            self._resume(row)

    def _resume(self, row):
        """The clean rollout's current subgoal (the arm was pushed off its path)."""
        u = next((u for u in self.units if u["start"] <= row < u["end"]), None)
        self.info["unit"] = u
        if u is None:
            self.info["route"] = "none:no_subgoal"
        elif u["kind"] == "pnp":
            if self._held(u["obj"]):
                self._carry(u["obj"], row, fresh=False, route="resume_carry")
            else:
                self._grasp(u["obj"], row, route="resume_grasp")
        else:
            self._follow(u, row)

    # --- plans ----------------------------------------------------------------------------------------------------
    def _own_grasp(self, obj, row):
        """The policy's grasp of obj in the clean rollout, for the subgoal at `row` (else its last pick-and-place of
        it): the body-frame grasp, the hand's world rotation, the row before closing, the object's pose there."""
        us = [u for u in self.units if u["kind"] == "pnp" and u["obj"] == obj]
        if not us:
            return None
        u = next((u for u in us if u["end"] > row), us[-1])
        r = u["close"] - 1
        import src.panda.grasping as grasping

        T_hand, T_obj = self._clean_hand(r), self._clean_pose(r, obj)
        # its open fingers may touch the object as deep as the policy's own did there (collision meshes overlap)
        allow = max(grasping.TARGET_CONTACT_TOL,
                    grasping.own_contact(self.env, obj, self.clean["q"][r], T_obj) + self.OWN_CONTACT_MARGIN)
        return {"G_b": np.linalg.inv(T_obj) @ T_hand, "R": T_hand[:3, :3], "row": r, "T_obj": T_obj,
                "release": u["end"] - 1, "allow": allow}

    def _set_down_spots(self, obj, own):
        xy = self._live_pose(obj)[:2, 3]
        return [("original", own["T_obj"][:2, 3]), ("current", xy)] + [
            ("aside", xy + self.SET_DOWN_STEP * np.array(d)) for d in ((1, 0), (-1, 0), (0, 1), (0, -1))]

    def _set_down_phases(self, obj, row, plan):
        """Set down (not for the goal yet: no shadow checks, phase names outside CHECK_PHASES), then the own grasp."""
        return [{"name": "lift_aside", "path": plan["lift"], "grip": 1.0},
                {"name": "check_aside", "obj": obj, "row": row},
                {"name": "carry_aside", "path": plan["transport"], "grip": 1.0},
                {"name": "descend", "path": plan["descend"], "grip": 1.0},
                {"name": "release", "hold": self.OPEN_STEPS, "grip": -1.0},
                {"name": "retreat", "path": plan["retreat"], "grip": -1.0},
                {"name": "regrasp", "obj": obj, "row": row, "T_goal": plan["T_goal"]}]

    def _set_down_held(self, obj, row, reason):
        """Set the held object down in its clean pose (current hold) and take it again; False if infeasible."""
        import src.panda.grasping as grasping

        t0 = time.time()
        own = self._own_grasp(obj, row)
        self.info["set_down_tried"] = True
        if own is None:
            return False
        T_up = own["T_obj"].copy()
        plan, stats, _ = grasping.plan_set_down(self.env, obj, own["G_b"], own["R"], T_up,
                                                self._set_down_spots(obj, own), self.MAX_VARIANTS, self.MAX_PICKS,
                                                held=True, allow=own["allow"], z0=self.z0, max_step=self.MOVE_SPEED)
        self.info["plans"].append({"grasp": "set_down", "held": True, "reason": reason, "ok": plan is not None,
                                   "stats": stats, "seconds": round(time.time() - t0, 2)})
        if plan is None:
            return False
        self.info["plans"][-1].update(spot=plan["spot"], own_kind=plan["own_kind"])
        self.info["grasps"].append("set_down")
        self.info.update(obj=obj, grasp_source="set_down")
        self._start(self._set_down_phases(obj, row, plan))
        return True

    def _grasp(self, obj, row, route, set_down=True):
        """Grasp obj: the policy's own grasp; else set it down upright and take it with that (unless `set_down` is
        False: right after a set-down); else a GraspGen grasp. Then lift and carry it."""
        import src.panda.grasping as grasping

        t0 = time.time()
        if self.info["route"] is None:
            self.info["route"] = route
        own, plan, kind, scanned, first = self._own_grasp(obj, row), None, None, False, None
        if own is not None:
            plan, stats = grasping.plan_own(self.env, obj, own["G_b"], own["R"], self.NEAR_DEG, self.NEAR_DIST,
                                            self.MAX_VARIANTS, self.MAX_NEAR, allow=own["allow"], z0=self.z0)
            self.info["plans"].append({"grasp": "own", "ok": plan is not None, "stats": stats})
            if plan is not None:
                kind = plan["kind"]
                self.info["plans"][-1].update(kind=kind, world_deg=plan["world_deg"], index=plan["index"])
            elif set_down:
                self.info["set_down_tried"] = True
                plan, stats, first = grasping.plan_set_down(self.env, obj, own["G_b"], own["R"], own["T_obj"].copy(),
                                                            self._set_down_spots(obj, own), self.MAX_VARIANTS,
                                                            self.MAX_PICKS, allow=own["allow"], z0=self.z0,
                                                            max_step=self.MOVE_SPEED)
                self.info["plans"].append({"grasp": "set_down", "ok": plan is not None, "stats": stats})
                scanned = True  # GraspGen's candidates were scanned in confidence order: `first` is its best grasp
                if plan is not None:
                    kind = "set_down"
                    self.info["plans"][-1].update({k: plan[k] for k in ("index", "spot", "own_kind")})
        if plan is None and scanned:
            plan = first
            self.info["plans"].append({"grasp": "graspgen", "ok": plan is not None, "from_set_down": True,
                                       "index": None if plan is None else plan["index"]})
            kind = "foreign" if plan is not None else None
        elif plan is None:
            plan, stats = grasping.plan_grasp(self.env, obj)
            self.info["plans"].append({"grasp": "graspgen", "ok": plan is not None, "stats": stats,
                                       "index": None if plan is None else plan["index"]})
            kind = "foreign" if plan is not None else None
        self.info["plans"][-1]["seconds"] = round(time.time() - t0, 2)
        if plan is None:
            if self.phase is None and not self.info["steps"]:
                self.info["route"] = route + ":no_feasible_grasp"  # given up: the policy goes on, untouched
                return
            return self._handoff("missed_grasp")
        self.info["grasps"].append(kind)
        self.info.update(obj=obj, grasp_source=kind)
        phases = ([{"name": "via", "path": plan["via"], "grip": -1.0}] if plan.get("via") else []) + [
            {"name": "move", "to": plan["q_pre"], "grip": -1.0, "need_open": True},
            {"name": "approach", "path": plan["approach"], "grip": -1.0},
            {"name": "close", "hold": self.CLOSE_STEPS, "grip": 1.0}]
        if kind == "set_down":
            phases += self._set_down_phases(obj, row, plan)
        else:  # the guard's references while lifting: the policy's hand at its grasp, its path of this subgoal
            ref = None if own is None else [own["R"]] * len(plan["lift"])
            rows = None if own is None else (own["row"], own["release"])
            phases += [{"name": "lift", "path": plan["lift"], "grip": 1.0, "ref_R": ref, "path_rows": rows},
                       {"name": "check_held", "obj": obj, "row": row},
                       {"name": "plan_carry", "obj": obj, "row": row, "fresh": True}]
        self._start(phases)

    def _clean_hand(self, row):
        import src.panda.grasping as grasping

        T = grasping.pose_matrix(self.clean["tcp"][row], self.clean["tcp_R"][row])
        T[:3, 3] -= grasping.GRIP_SITE_OFFSET * T[:3, 2]
        return T

    def _carry(self, obj, row, fresh, route):
        if self.info["route"] is None:
            self.info["route"] = route
        self.info["obj"] = obj
        self._start([{"name": "plan_carry", "obj": obj, "row": row, "fresh": fresh}])

    def _reference(self, obj, row):
        """The body the object's goal refers to (BDDL: the body whose name prefixes the goal region), else the body
        nearest to the object at the clean release row."""
        bodies = [b for b in self.clean["bodies"] if b != obj]
        goal = next((g for g in self.env.parsed_problem["goal_state"] if len(g) > 2 and g[1] == obj), None)
        named = [b for b in bodies if goal and goal[2].startswith(b)]
        if named:
            return max(named, key=len)
        i = list(self.clean["bodies"]).index(obj)
        dist = {b: np.linalg.norm(self.clean["body_pos"][row, list(self.clean["bodies"]).index(b)]
                                  - self.clean["body_pos"][row, i]) for b in bodies}
        return min(dist, key=dist.get)

    def _plan_carry(self, obj, row, fresh):
        """Transport (joint DMP) of the held object to the clean rollout's arm pose HANDOFF_BEFORE steps before its
        release of the object (the hand as the policy's there, moved with the goal's body; else its joints as they
        were), where control returns at the latest; None if unreachable."""
        import src.panda.grasping as grasping

        t0 = time.time()
        u = ([u for u in self.units if u["kind"] == "pnp" and u["obj"] == obj] or [None])[-1]  # its final release
        rec = {"carry": obj, "ok": False, "tried": []}
        self.info["plans"].append(rec)
        if u is None:
            rec["reason"] = "no_clean_carry"
            return None
        release = u["end"] - 1  # the row before the clean rollout's release command
        target = max(u["close"], release - self.HANDOFF_BEFORE)  # still holding it there in the clean rollout
        ref = self._reference(obj, release)
        T_ref = self._live_pose(ref) @ np.linalg.inv(self._clean_pose(target, ref))
        G_b = np.linalg.inv(self._live_pose(obj)) @ self._hand()
        carried = lambda T_hand: T_hand @ np.linalg.inv(G_b)
        kin = grasping.AvoidKinematics(self.env, obj)
        q0 = kin.q0
        m = self.sim.model._model
        root = m.body_rootid[self.env.obj_body_id[ref]]
        kin.soft_geoms = frozenset(g for g in range(m.ngeom) if m.body_rootid[m.geom_bodyid[g]] == root)
        if fresh:  # taken again: the clean carry from its row whose TCP is horizontally nearest the hand (the progress
            # already made), not from the clean grasp, which the DMP (keeping the demonstration's shape) would retrace
            tcp = self.sim.data.site_xpos[self.env.robots[0].eef_site_id]
            rows = np.arange(u["close"] - 1, target + 1)
            start = int(rows[np.argmin(np.linalg.norm(self.clean["tcp"][rows, :2] - tcp[:2], axis=1))])
        else:
            start = max(u["close"] - 1, row)  # the rest of the clean carry when mid-carry
            if target - start < self.MIN_DEMO:  # past (or nearly past) the target row: the clean carry up to it
                start = u["close"] - 1
        demo = self.clean["q"][start: target + 1]
        rec.update(ref=ref, release_row=int(release), target_row=int(target), demo_rows=[int(start), int(target)])
        goals = []
        q_hand = kin.ik(T_ref @ self._clean_hand(target) @ kin.site_off, self.clean["q"][target])
        if q_hand is not None:
            goals.append(("clean_hand", q_hand))
        goals.append(("clean_joints", np.array(self.clean["q"][target], dtype=float)))
        try:
            for name, q_goal in goals:
                if kin.carry_collides(q_goal, carried):
                    rec["tried"].append([name, "goal_collision"])
                    continue
                stats = {}
                path = grasping.dmp_transport(kin, demo, q0, q_goal, carried, self.MOVE_SPEED, stats)
                if path is not None:
                    rec.update(ok=True, goal_pose=name, stats=stats, seconds=round(time.time() - t0, 2))
                    # the guard's reference along the way: the clean hand at the matching demonstration row (moved
                    # with the goal's body when the goal pose is)
                    R_T = T_ref[:3, :3] if name == "clean_hand" else np.eye(3)
                    rows = [start + int(round((i + 1) * (target - start) / len(path))) for i in range(len(path))]
                    return {"transport": path, "ref_R": [R_T @ self.clean["tcp_R"][r] for r in rows],
                            "path_rows": (start, release)}
                rec["tried"].append([name, "transport_collides"])
        finally:
            kin.soft_geoms = frozenset()
        rec.update(reason="goal_unreachable", seconds=round(time.time() - t0, 2))
        return None

    def _follow(self, u, row):
        """Another subgoal: its clean joint path from this step on, reproduced by a DMP from the current joints, up to
        HANDOFF_BEFORE steps before the subgoal's end (back to that pose if the clean rollout was already past it)."""
        import src.panda.grasping as grasping
        from src.dmp import JointDMP, limited_rollout

        self.info["route"] = "resume_follow"
        target = max(u["start"], u["end"] - 1 - self.HANDOFF_BEFORE)
        q_now, q_goal = self._joint_qpos(), np.array(self.clean["q"][target], dtype=float)
        ahead = target - row >= self.MIN_DEMO
        demo = self.clean["q"][row: target + 1] if ahead else [q_now] + grasping.path(q_now, q_goal, self.MOVE_SPEED)
        lo, hi = self.fk.lo, self.fk.hi
        qs, _ = limited_rollout(JointDMP(np.asarray(demo)), q_now, q_goal, self.MOVE_SPEED, None, lo, hi)
        if qs is None:
            self.info["route"] = "none:follow_too_fast"
            return
        if ahead:  # the clean rollout's gripper commands along its path (action[r] = the command at step r + 1)
            rows = [min(row + int(round((i + 1) * (target - row) / len(qs))), target) for i in range(len(qs))]
            grips = [float(np.sign(self.clean["action"][r][6]) or -1.0) for r in rows]
        else:
            rows = [target] * len(qs)
            grips = [self._gripper_state()] * len(qs)
        self.info["follow_rows"] = [int(row), int(target)]
        self._start([{"name": "follow", "path": qs, "grips": grips, "tol": self.FOLLOW_TOL_DEG,
                      "ref_R": [self.clean["tcp_R"][r] for r in rows], "path_rows": (min(row, target), target)},
                     {"name": "done", "reason": "pre_goal"}])

    # --- execution --------------------------------------------------------------------------------------------------
    def _start(self, phases):
        if not isinstance(self.env.robots[0].controller, JointOverride):  # not yet (a grasp retry keeps it)
            self._swap_to_joint()
        self.phases = phases
        self._next_phase()

    def _next_phase(self):
        import src.panda.grasping as grasping

        while self.phases:
            ph = self.phases.pop(0)
            if ph["name"] in ("check_held", "check_aside"):
                if not self._held(ph["obj"]):
                    self.info["missed_grasps"] = self.info.get("missed_grasps", 0) + 1
                    if self.retries < self.GRASP_RETRIES:
                        self.retries += 1
                        self.phases = []
                        return self._grasp(ph["obj"], ph["row"], route=self.info["route"])
                    return self._handoff("missed_grasp")
                continue
            if ph["name"] == "regrasp":  # set down: how it stands, then the own grasp (no second set-down)
                T_now, T_goal = self._live_pose(ph["obj"]), ph["T_goal"]
                self.info["set_down"] = {
                    "off_cm": round(100 * float(np.linalg.norm(T_now[:3, 3] - T_goal[:3, 3])), 1),
                    "turn_deg": round(grasping.turn_deg(ph["obj"], T_now[:3, :3], T_goal[:3, :3]), 1)}
                self.phases = []
                return self._grasp(ph["obj"], ph["row"], route=self.info["route"], set_down=False)
            if ph["name"] == "plan_carry":
                plan = self._plan_carry(ph["obj"], ph["row"], ph["fresh"])
                if plan is None:
                    if not ph["fresh"] and not self.info.get("set_down_tried") and \
                            self._set_down_held(ph["obj"], ph["row"], "carry_infeasible"):
                        return  # held since the trigger: set down and taken again, then carried
                    return self._handoff("goal_unreachable")
                self.phases = [{"name": "transport", "path": plan["transport"], "grip": 1.0, "ref_R": plan["ref_R"],
                                "path_rows": plan["path_rows"]},
                               {"name": "done", "reason": "pre_goal"}] + self.phases
                continue
            if ph["name"] == "done":
                return self._handoff(ph["reason"])
            if "to" in ph:  # a move from wherever the arm is now
                ph["path"] = grasping.path(self._joint_qpos(), ph["to"], self.MOVE_SPEED)
            self.ph, self.phase, self.n = ph, ph["name"], 0
            return
        self._handoff("pre_goal")

    def _handoff(self, reason, chunk=None):
        self.info.update(handoff=reason, intervention_steps=self.k)
        self.handoff = list(chunk) if chunk else []
        self.phases = []
        self._finish()

    def before_step(self, t, obs):
        super().before_step(t, obs)
        d = self.deferred
        if d is not None and d["fire"] is None and self.phase is None:
            if d["done"] is None and all(self.env._eval_predicate(self.env.parsed_problem["goal_state"][j])
                                         for j in self._pending_idx):
                d["done"] = t
            if d["done"] is not None:
                tcp = self.sim.data.site_xpos[self.env.robots[0].eef_site_id][:2]
                dist = min(np.linalg.norm(tcp - self._pose(self.target)[0][:2]),
                           np.linalg.norm(tcp - (self.pose0 if self.pose0 is not None else self.pose_last)[0][:2]))
                if dist <= self.DEFER_DIST or t - d["done"] >= self.DEFER_WAIT:
                    d.update(fire=t, reason="near" if dist <= self.DEFER_DIST else "wait",
                             dist_cm=round(100 * float(dist), 1))
                    if d.get("resumed"):  # the subgoal resumed at the trigger
                        d["first"] = {k: self.info.get(k) for k in ("handoff", "intervention_steps", "steps")}
                    self.info["deferred"] = d
                    self._route(t, obs, may_defer=False)
        if self.phase in self.CHECK_PHASES and self.k % self.CHECK_EVERY == 0:
            self._check(t, obs)

    def _check(self, t, obs):
        """Shadow query: does the policy want to do what the intervention is about to do?"""
        import src.panda.grasping as grasping

        ph = self.ph
        rec = {"t": t, "phase": ph["name"], "k": self.k}
        self.checks.append(rec)
        ahead, _ = self.fk.site_pose(ph["path"][min(self.n + self.CHECK_ACTIONS, len(ph["path"])) - 1])
        planned = ahead - self.sim.data.site_xpos[self.env.robots[0].eef_site_id]
        grip = ph["grips"][min(self.n, len(ph["grips"]) - 1)] if "grips" in ph else ph["grip"]
        if np.linalg.norm(planned) < self.MIN_MOTION:
            rec["skipped"] = "no_motion"
            return
        if self.query:
            chunk = [np.asarray(a) for a in self.query(obs)]
        else:  # replay: the recorded policy outputs
            r = self.recorded["checks"][len(self.checks) - 1]
            chunk = [np.asarray(a, dtype=r["dtype"]) for a in r["chunk"]]
        acts = np.array([self.process_action(a) for a in chunk[: self.CHECK_ACTIONS]], dtype=float)
        v = acts[:, :3].mean(0)
        cos = float(v @ planned / (np.linalg.norm(v) * np.linalg.norm(planned) + 1e-12))
        grip_ok = bool(all(np.sign(a[6]) == np.sign(grip) for a in acts))
        wrist = None  # the hand versus the clean rollout's at the matching row
        if ph.get("ref_R") is not None:
            R_hand = self.sim.data.site_xmat[self.env.robots[0].eef_site_id].reshape(3, 3)
            wrist = float(grasping.rot_deg(R_hand, ph["ref_R"][min(max(self.n, 1), len(ph["ref_R"])) - 1]))
        wrist_ok = wrist is None or wrist <= self.WRIST_DEG
        path = None  # the TCP versus the clean rollout's path of this subgoal
        if ph.get("path_rows") is not None:
            lo, hi = ph["path_rows"]
            tcp = self.sim.data.site_xpos[self.env.robots[0].eef_site_id]
            path = float(np.linalg.norm(self.clean["tcp"][lo: hi + 1] - tcp, axis=1).min())
        path_ok = path is None or path <= self.HANDOFF_DIST
        agree = cos >= self.COS_MIN and grip_ok and wrist_ok and path_ok
        self.agreed = self.agreed + 1 if agree else 0
        rec.update(cos=round(cos, 3), grip_ok=grip_ok, wrist_deg=None if wrist is None else round(wrist, 1),
                   wrist_ok=wrist_ok, path_cm=None if path is None else round(100 * path, 2), path_ok=path_ok,
                   agree=agree, grip=grip, planned=planned.tolist(),
                   dtype=str(chunk[0].dtype), chunk=[[float(x) for x in a] for a in chunk])
        if self.agreed >= self.N_AGREE:
            self._handoff("agree", chunk)

    def override(self, t, obs):
        if self.phase is None:
            return None
        self.n += 1
        self.k += 1
        ph = self.ph
        if "path" in ph:
            self.override_ctrl.jp.goal_qpos = ph["path"][min(self.n, len(ph["path"])) - 1]
        elif self.override_ctrl.jp.goal_qpos is None:
            self.override_ctrl.jp.goal_qpos = self._joint_qpos()
        grip = ph["grips"][min(self.n, len(ph["grips"])) - 1] if "grips" in ph else ph["grip"]
        return np.array([0.0] * 6 + [grip])

    def observe(self, t, obs, action):
        self._track_rest()
        if self.perturb.fired_step is None:  # keep the target's state just before the perturbation
            self.held_last, self.pose_last = self._held(self.target), self._pose(self.target)
        elif self.pose0 is None:
            self.was_held, self.pose0 = self.held_last, self.pose_last
        if self.phase is None:
            return
        self._record_contacts()
        ph = self.ph
        if "hold" in ph:
            done, timeout = self.n >= ph["hold"], False
        else:
            err = np.degrees(np.abs(self._joint_qpos() - ph["path"][-1])).max()
            fingers = self.sim.data.qpos[self.env.robots[0]._ref_gripper_joint_pos_indexes]
            opened = not ph.get("need_open") or min(abs(fingers)) >= self.FINGER_OPEN
            timeout = self.n >= len(ph["path"]) + self.SETTLE_MAX
            done = (self.n >= len(ph["path"]) and err < ph.get("tol", self.ARRIVE_TOL_DEG) and opened) or timeout
        if done:
            self.info["steps"][ph["name"]] = self.info["steps"].get(ph["name"], 0) + self.n
            if timeout:
                self.info["timeouts"].append(ph["name"])
            self._next_phase()

    def record(self):
        return {"trigger_steps": self.trigger_steps, **self.info, "deferred": self.deferred, "checks": self.checks,
                "contacts": self.contacts, "interrupted": self.phase is not None}


RECOVERIES = {
    "vanilla": Vanilla,
    "replan": Replan,
    "rehome": Rehome,
    "retreat": Retreat,
    "safeloop": SafeLoop,
    "rewind_il": RewindIL,
    "volo": VoLo,
    "core": Core,
    "core_detect": CoreDetect,
    "mire": Mire,
}
