"""
perturbations.py

CoRe-style online perturbations for LIBERO-Long (arm collision, grasp failure, object displace, object tip).
A perturbation runs alongside the policy: `before_step` may change the action or the simulator state before
`env.step`, and `after_step` tracks events and validity after it. Steps are 1-based counts after the wait steps.
"""

import mujoco
import numpy as np

GUARD_DIST = 0.06  # Pre-contact guard: fire immediately once the gripper is this close to the target surface (m)
REACH_RADIUS = 0.75  # Objects must stay within this planar radius of the robot base (m)


class Perturbation:
    """Base class. Subclasses set `self.fired_step` when injecting and `self.invalid` (a reason string) if invalid.
    The injection lasts INJECT_STEPS steps from `fired_step` (1 = applied within a single step)."""

    INJECT_STEPS = 1

    def __init__(self, env, target, phases, timing, seed):
        self.env, self.target = env.env, target
        self.phases, self.timing = phases, timing
        self.rng = np.random.default_rng(seed)
        self.fired_step, self.invalid, self.params = None, None, {}

    @property
    def sim(self):  # env.reset() may rebuild the simulator, so never cache it
        return self.env.sim

    @property
    def end_step(self):
        """Last step of the injection (None until fired)."""
        return None if self.fired_step is None else self.fired_step + self.INJECT_STEPS - 1

    def reset(self):
        """Called after the wait steps, right before the policy starts acting."""

    def before_step(self, t, action):
        return action

    def after_step(self, t, action):
        pass

    # Simulator helpers
    def _geoms(self, names):
        return [self.sim.model._model.geom(n).id for n in names]

    def surface_distance(self, distmax=GUARD_DIST):
        """Shortest distance between the gripper collision geoms and the target collision geoms (capped at distmax)."""
        m, d, fromto = self.sim.model._model, self.sim.data._data, np.zeros(6)
        grip = self._geoms(self.env.robots[0].gripper.contact_geoms)
        obj = self._geoms(self.env.objects_dict[self.target].contact_geoms)
        return min(mujoco.mj_geomDistance(m, d, a, b, distmax, fromto) for a in grip for b in obj)

    def contact_geoms(self, obj, penetrating=False):
        """Geoms currently in contact with `obj` (only penetrating contacts if requested)."""
        d, own = self.sim.data._data, set(self._geoms(self.env.objects_dict[obj].contact_geoms))
        out = set()
        for c in d.contact[: d.ncon]:
            if (c.geom1 in own) != (c.geom2 in own) and (not penetrating or c.dist < 0):
                out.add(c.geom2 if c.geom1 in own else c.geom1)
        return out

    def in_reach(self, obj):
        base = self.sim.data.body_xpos[self.sim.model._model.body("robot0_base").id]
        return np.linalg.norm(self.sim.data.body_xpos[self.env.obj_body_id[obj]][:2] - base[:2]) <= REACH_RADIUS

    def object_poses(self):
        return {o: (self.sim.data.body_xpos[i].copy(), self.sim.data.body_xquat[i].copy())
                for o, i in self.env.obj_body_id.items() if o in self.env.objects_dict}

    def fixture_joints(self):
        """Positions of the articulated fixture joints (drawers: slide, stove button / microwave door: hinge)."""
        m, out = self.sim.model._model, {}
        roots = {m.body_rootid[self.env.obj_body_id[f]] for f in self.env.fixtures_dict}
        for j in range(m.njnt):
            if m.body_rootid[m.jnt_bodyid[j]] in roots and m.jnt_type[j] in (mujoco.mjtJoint.mjJNT_SLIDE,
                                                                             mujoco.mjtJoint.mjJNT_HINGE):
                out[m.joint(j).name] = (int(m.jnt_type[j]), float(self.sim.data.qpos[m.jnt_qposadr[j]]))
        return out

    def moved_objects(self, poses, max_dist, max_rot_deg, exclude=()):
        """Objects whose planar displacement or rotation since `poses` exceeds the limits."""
        now = self.object_poses()
        return [o for o, (p, q) in poses.items() if o not in exclude and (
            np.linalg.norm(now[o][0][:2] - p[:2]) > max_dist or quat_angle_deg(now[o][1], q) > max_rot_deg)]


def quat_angle_deg(q1, q2):
    return float(np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(q1, q2)))))))


class PreContactPerturbation(Perturbation):
    """Fires at `timing` x pre-grasp phase, or immediately once the gripper is within GUARD_DIST of the target."""

    def __init__(self, *args):
        super().__init__(*args)
        self.tick = max(1, round(self.timing * self.phases["pre_grasp"]))

    def should_fire(self, t):
        if self.fired_step is not None or (t < self.tick and self.surface_distance() >= GUARD_DIST):
            return False
        # Target already in the gripper: defer to the next tick
        return not self.env._check_grasp(gripper=self.env.robots[0].gripper,
                                         object_geoms=self.env.objects_dict[self.target])


class ObjectDisplace(PreContactPerturbation):
    """G5: relocate the target 12 cm along a random planar direction during the pre-grasp phase."""

    DIST, MAX_TRIES = 0.12, 64

    def before_step(self, t, action):
        if not self.should_fire(t):
            return action

        joint = self.env.objects_dict[self.target].joints[0]
        qpos0, qvel0 = self.sim.data.get_joint_qpos(joint).copy(), self.sim.data.get_joint_qvel(joint).copy()
        self.sim.forward()
        support = self.contact_geoms(self.target)  # Whatever the target rests on counts as the table
        for i in range(self.MAX_TRIES):
            angle = self.rng.uniform(0, 2 * np.pi)
            qpos = qpos0.copy()
            qpos[:2] += self.DIST * np.array([np.cos(angle), np.sin(angle)])
            self.sim.data.set_joint_qpos(joint, qpos)
            self.sim.data.set_joint_qvel(joint, np.zeros(6))
            self.sim.forward()
            if self.in_reach(self.target) and not (self.contact_geoms(self.target, penetrating=True) - support):
                self.fired_step = t
                self.params = {"angle": float(angle), "tries": i + 1, "guard": t < self.tick}
                return action
        self.sim.data.set_joint_qpos(joint, qpos0)
        self.sim.data.set_joint_qvel(joint, qvel0)
        self.sim.forward()
        self.invalid = "displace_no_free_spot"
        return action


class ArmCollision(Perturbation):
    """R1: torque impulse on all seven arm joints for 5 steps (per-joint standard normal x 50), at
    `timing` x full episode. A draw is rejected if, within CHECK_STEPS steps of the impulse start (the arm keeps
    moving after the impulse), the basket drifts > 0.15 m or rotates > 20 deg, or an articulated fixture joint
    moves > 15 cm (drawer) / > 20 deg (hinge). CoRe checks every movable object; we let the arm knock loose objects
    and only reject disturbances of containers and fixtures. Up to 8 redraws, each simulated from a snapshot
    (holding the current action) and rolled back; the real injection runs with the policy's actions, so the same
    check is repeated on it and a violation makes the episode invalid. CoRe does not specify the check window."""

    MAGNITUDE, STEPS, CHECK_STEPS, MAX_DRAWS = 50.0, 5, 10, 8
    INJECT_STEPS = STEPS
    CHECKED_OBJECTS = ("basket_1",)  # Movable objects whose disturbance rejects a draw (fixtures are always checked)
    MAX_DRIFT, MAX_ROT_DEG, MAX_SLIDE, MAX_HINGE_DEG = 0.15, 20.0, 0.15, 20.0

    def __init__(self, *args):
        super().__init__(*args)
        self.tick = max(1, round(self.timing * self.phases["full"]))
        self.dofs, self.torque = self.env.robots[0]._ref_joint_vel_indexes, None

    def before_step(self, t, action):
        if self.fired_step is None and t >= self.tick:
            self.torque = self._draw(action)
            if self.torque is None:
                self.invalid = "collision_rejected"
                return action
            self.fired_step, self.snapshot = t, (self.object_poses(), self.fixture_joints())
        if self.fired_step is not None and t < self.fired_step + self.STEPS:
            self.sim.data.qfrc_applied[self.dofs] = self.torque
        return action

    def after_step(self, t, action):
        if self.fired_step is None or t >= self.fired_step + self.CHECK_STEPS:
            return
        if t == self.fired_step + self.STEPS - 1:
            self.sim.data.qfrc_applied[self.dofs] = 0.0
        if moved := self.disturbed(*self.snapshot):
            self.invalid, self.params["disturbed"] = "collision_disturbed", moved

    def disturbed(self, poses, joints):
        """Objects / fixture joints moved beyond the limits since the snapshot."""
        moved = [o for o in self.moved_objects(poses, self.MAX_DRIFT, self.MAX_ROT_DEG) if o in self.CHECKED_OBJECTS]
        for name, (jtype, q) in self.fixture_joints().items():
            limit = self.MAX_SLIDE if jtype == mujoco.mjtJoint.mjJNT_SLIDE else np.radians(self.MAX_HINGE_DEG)
            if abs(q - joints[name][1]) > limit:
                moved.append(name)
        return moved

    def _draw(self, action):
        state, snapshot = self.sim.get_state(), (self.object_poses(), self.fixture_joints())
        clock = (self.env.timestep, self.env.cur_time, self.env.done)
        gripper = self.env.robots[0].gripper
        grip_cmd = gripper.current_action.copy()  # Accumulated finger command; not part of the sim state
        for i in range(self.MAX_DRAWS):
            torque = self.MAGNITUDE * self.rng.standard_normal(len(self.dofs))  # Per joint; norm ~ 50 x sqrt(7)
            moved = []
            for k in range(self.CHECK_STEPS):
                self.sim.data.qfrc_applied[self.dofs] = torque if k < self.STEPS else 0.0
                self.env.step(action)
                moved = moved or self.disturbed(*snapshot)
            self.sim.data.qfrc_applied[self.dofs] = 0.0
            self.sim.set_state(state)
            self.sim.forward()
            self.env.timestep, self.env.cur_time, self.env.done = clock
            gripper.current_action = grip_cmd.copy()
            if not moved:
                self.params = {"torque": torque.round(3).tolist(), "norm": round(float(np.linalg.norm(torque)), 1),
                               "draws": i + 1}
                return torque
        return None


class GraspFailure(Perturbation):
    """T2: once the target lifts 1 cm with the gripper closed (carry anchor), open the gripper in place at
    anchor + `timing` x carry phase by overriding the gripper command for OPEN_STEPS steps. Invalid if, at the last
    forced-open step, a finger still penetrates the target deeper than PEN_TOL (resting contact alone, e.g. a finger
    left inside a mug, does not count)."""

    LIFT_HEIGHT, OPEN_STEPS, PEN_TOL = 0.01, 10, 0.002
    INJECT_STEPS = OPEN_STEPS

    def reset(self):
        self.spawn_z = self.sim.data.body_xpos[self.env.obj_body_id[self.target]][2]
        self.lift_step = self.tick = None

    def before_step(self, t, action):
        if self.fired_step is None and self.tick is not None and t >= self.tick:
            self.fired_step = t
        if self.fired_step is not None and t < self.fired_step + self.OPEN_STEPS:
            action = action.copy()
            action[-1] = -1.0  # -1 = open
        return action

    def after_step(self, t, action):
        if self.lift_step is None and action[-1] > 0 and \
                self.sim.data.body_xpos[self.env.obj_body_id[self.target]][2] > self.spawn_z + self.LIFT_HEIGHT:
            self.lift_step = t
            self.tick = t + max(1, round(self.timing * self.phases["carry"]))
            self.params = {"lift_step": t}
        if self.fired_step is not None and t == self.fired_step + self.OPEN_STEPS - 1:
            d, own = self.sim.data._data, set(self._geoms(self.env.objects_dict[self.target].contact_geoms))
            grip = set(self._geoms(self.env.robots[0].gripper.contact_geoms))
            depth = min([c.dist for c in d.contact[: d.ncon] if {c.geom1, c.geom2} & own and {c.geom1, c.geom2} & grip],
                        default=0.0)
            self.params["max_pen_mm"] = round(-1000 * depth, 2)
            if depth < -self.PEN_TOL:
                self.invalid = "gripper_penetrates"


class ObjectTip(PreContactPerturbation):
    """G4: torque wrench on the target about a horizontal axis for TORQUE_STEPS steps (CoRe), with magnitude
    attenuation x 0.05 N*m x sqrt(target mass / mean movable-object mass). CoRe leaves the axis unspecified and
    tunes the attenuation for four light objects visually; we fix both per object (TIP_TORQUES) from a physics
    sweep so the target tips into the intended pose without flying off. The axis is picked at random from the
    object's list. As in CoRe, the episode is invalid if a neighbor moves > 5 cm or rotates > 20 deg, or if the
    target leaves the reach radius, checked SETTLE steps after the torque ends."""

    BASE_TORQUE, TORQUE_STEPS, SETTLE, NEIGHBOR_DIST, NEIGHBOR_ROT_DEG = 0.05, 4, 20, 0.05, 20.0
    INJECT_STEPS = TORQUE_STEPS
    # (attenuation, [signed object-frame torque axes]); positive rotation about the axis by the right-hand rule
    X, Y, Z = np.eye(3)
    TIP_TORQUES = {
        "alphabet_soup": (0.3, [X, -X, Z, -Z]),       # Can (axis = y): lie on its side
        "cream_cheese": (0.25, [X, -X]),              # Stand on a long side (y axes overshoot to another face)
        "moka_pot": (0.7, [X]),                       # Handle (+y) up
        "porcelain_mug": (0.2, [Y]),                  # Handle (-x) up
        "white_yellow_mug": (0.3, [-Y]),              # Handle (+x) up
        "black_book": (0.4, [Y, -Y]),                 # Stands upright: fall onto the front / back cover
        "akita_black_bowl": (0.25, [Y]),              # Only toward the robot stays within reach; ends ~upside down
    }

    def before_step(self, t, action):
        body = self.env.obj_body_id[self.target]
        if self.should_fire(t):
            m, d = self.sim.model._model, self.sim.data._data
            atten, axes = self.TIP_TORQUES[self.target.rsplit("_", 1)[0]]  # e.g. moka_pot_2 -> moka_pot
            i = self.rng.integers(len(axes))
            axis = d.xmat[body].reshape(3, 3) @ axes[i]
            axis[2] = 0.0
            rel_mass = m.body_subtreemass[body] / np.mean(
                [m.body_subtreemass[self.env.obj_body_id[o]] for o in self.env.objects_dict])
            self.wrench = atten * self.BASE_TORQUE * np.sqrt(rel_mass) * axis / np.linalg.norm(axis)
            self.fired_step, self.poses, self.com0 = t, self.object_poses(), d.xipos[body].copy()
            self.params = {"axis": axes[i].astype(int).tolist(), "torque": round(float(np.linalg.norm(self.wrench)), 4),
                           "guard": t < self.tick}
        if self.fired_step is not None and t < self.fired_step + self.TORQUE_STEPS:
            self.sim.data.xfrc_applied[body, 3:] = self.wrench
        return action

    def after_step(self, t, action):
        if self.fired_step is None:
            return
        body = self.env.obj_body_id[self.target]
        if t == self.fired_step + self.TORQUE_STEPS - 1:
            self.sim.data.xfrc_applied[body, 3:] = 0.0
        if t != self.fired_step + self.TORQUE_STEPS - 1 + self.SETTLE:
            return
        pose = self.object_poses()[self.target]
        self.params["final_rot_deg"] = round(quat_angle_deg(pose[1], self.poses[self.target][1]), 1)
        com = self.sim.data._data.xipos[body]
        self.params["moved_cm"] = round(100 * float(np.linalg.norm(com[:2] - self.com0[:2])), 1)  # COM, planar
        if self.moved_objects(self.poses, self.NEIGHBOR_DIST, self.NEIGHBOR_ROT_DEG, exclude=(self.target,)):
            self.invalid = "tip_disturbed_neighbors"
        elif not self.in_reach(self.target):
            self.invalid = "out_of_reach"


PERTURBATIONS = {
    "arm_collision": ArmCollision,
    "grasp_failure": GraspFailure,
    "object_displace": ObjectDisplace,
    "object_tip": ObjectTip,
}
