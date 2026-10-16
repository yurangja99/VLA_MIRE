"""
grasping.py

Grasp primitive shared by the grasp-based recoveries in recovery.py (volo; later core, mire). The pre-generated GraspGen
candidates of the target object (outputs/grasps/<suite>, see generate_grasps.py) are mapped to its current pose, filtered
(confidence, required opening), visited in a ranking order (GraspGen confidence for volo), and the first one that
VoLo's multi-start damped-least-squares IK can reach without collisions is turned into a joint-space plan:
current pose -> pre-grasp (PREGRASP_DIST back along the approach axis) -> approach -> close -> lift. Only the key poses
(pre-grasp, grasp, lift) are solved by IK; approach and lift interpolate the joints between them (VoLo streams an
interpolated joint trajectory), checked to stay within LINE_TOL of the straight line.
All IK and collision checks run on copies of the simulator model and data (the copy's robot geoms get a CLEARANCE
contact margin), so the episode itself is never touched.

Frames: GraspGen's gripper frame (approach +z, fingers closing along x, origin at the hand base) coincides with
robosuite's PandaGripper `right_gripper` body; the OSC / IK point `grip_site` sits GRIP_SITE_OFFSET further along z.
The gripper is symmetric about its approach axis (GraspGen: symmetric_antipodal), so each grasp is also tried rotated
by pi about z, and the variant needing the smaller joint motion is used.
"""

import copy
import os
import re

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as Rot

import src.paths as P

GRASP_DIR = P.out("grasps", "libero_10")  # generate_grasps.py output
CONF_MIN, WIDTH_MAX = 0.8, 0.078  # GraspGen confidence; object span between the pads (robosuite Panda opens 8 cm)
GRIP_SITE_OFFSET = 0.097  # m, right_gripper -> grip_site along z
PREGRASP_DIST, APPROACH_STEP = 0.10, 0.01  # m back along the approach axis; m per approach waypoint
LIFT_HEIGHT, LIFT_STEP = 0.10, 0.01  # m straight up after closing; m per lift waypoint
TARGET_CONTACT_TOL = 0.005  # m: open fingers may touch the target this deep at the approach / grasp poses
CLEARANCE = 0.01  # m: required distance between the robot and anything but the target (joint tracking lags a bit)
PATH_STEP = 0.08  # rad (L2 over the 7 joints): resolution of the joint-path collision check
BRANCH_CAP = 0.5  # rad: VoLo's joint-branch consistency cap between consecutive key poses (pre-grasp, grasp, lift)
LINE_TOL = 0.01  # m: joint-interpolated approach / lift may leave the straight line by at most this much

# VoLo (arXiv 2606.07723, App. B.2) IK: damped least squares with null-space joint centering, multi-start
IK_DAMPING, IK_NULL_GAIN, IK_MAX_STEP = 5e-3, 0.5, 0.2  # lambda, alpha, rad
IK_POS_TOL, IK_ROT_TOL, IK_MAX_ITERS = 5e-4, 5e-3, 500  # m, rad, iterations
IK_FK_ROT_TOL_DEG, IK_RANDOM_SEEDS, IK_RANDOM_RANGE = 2.0, 3, 1.5  # deg, draws, rad around the joint mid-range
IK_SEED = 0  # RNG seed of the random IK starts (a fresh generator per solve: same state -> same solution)
IK_STALL_ITERS = 30  # stop a start early once its error has not dropped by 0.1% over this many iterations

SUPPORT_MARGIN = 0.02  # m, pre-filter tolerance below the object's lowest point
# Gripper hull in GraspGen's frame (hand base corners, finger roots and tips at full opening)
GRIPPER_HULL = np.array([[x, y, z] for x in (-0.053, 0.053) for y in (-0.01, 0.01) for z in (0.0, 0.0584, 0.1053)])
_FLIP_Z = np.diag([-1.0, -1.0, 1.0, 1.0])  # rotation by pi about the gripper's approach axis
_cache = {}


def object_kind(name):
    return re.sub(r"_\d+$", "", name)  # e.g. moka_pot_2 -> moka_pot


def load_candidates(name):
    """Grasp candidates of the object `name` (body frame) passing the confidence / opening filters, best first."""
    kind = object_kind(name)
    if kind not in _cache:
        z = np.load(os.path.join(GRASP_DIR, f"{kind}.npz"))
        keep = (z["conf"] >= CONF_MIN) & (np.nan_to_num(z["width"], nan=np.inf) <= WIDTH_MAX)
        _cache[kind] = (z["grasps"][keep].astype(np.float64), z["conf"][keep].astype(np.float64))
    return _cache[kind]


def load_points(name):
    kind = object_kind(name)
    key = kind + ":points"
    if key not in _cache:
        _cache[key] = np.load(os.path.join(GRASP_DIR, f"{kind}.npz"))["points"].astype(np.float64)
    return _cache[key]


def pose_matrix(pos, mat):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = np.asarray(mat).reshape(3, 3), pos
    return T


class ArmKinematics:
    """IK and collision checks for the robot arm on a scratch copy of the simulator data."""

    def __init__(self, env, target):
        self.env, self.robot = env, env.robots[0]
        self.live = env.sim.data._data
        self.m = copy.copy(env.sim.model._model)  # scratch MjModel: robot geoms get a CLEARANCE contact margin
        self.d = copy.copy(self.live)  # scratch MjData
        self.qadr = np.array(self.robot._ref_joint_pos_indexes)
        self.vadr = np.array(self.robot._ref_joint_vel_indexes)
        jids = [self.m.joint(n).id for n in self.robot.robot_joints]
        self.lo, self.hi = self.m.jnt_range[jids, 0], self.m.jnt_range[jids, 1]
        self.mid = 0.5 * (self.lo + self.hi)
        self.home = np.clip(np.array(self.robot.init_qpos, float), self.lo, self.hi)
        self.site = self.m.site(self.robot.controller.eef_name).id
        self.finger_qadr = [self.m.jnt_qposadr[self.m.joint(n).id] for n in self.robot.gripper.joints]
        root = self.m.body_rootid[self.m.body("robot0_base").id]
        self.robot_geom = self.m.body_rootid[self.m.geom_bodyid] == root
        self.m.geom_margin[self.robot_geom] = np.maximum(self.m.geom_margin[self.robot_geom], CLEARANCE)
        tgt = env.objects_dict[target]
        self.target_geoms = {self.m.geom(g).id for g in tgt.contact_geoms}
        self.gripper_geoms = {self.m.geom(g).id for g in self.robot.gripper.contact_geoms}
        self.q0 = self.live.qpos[self.qadr].copy()
        self.ignore = self._pairs(self.contacts(self.q0))  # contacts already there (e.g. the robot mount) never count
        # What the target rests on (its current non-robot contacts): approaching it closer than CLEARANCE is allowed
        self.support = {g for c in self.live.contact[: self.live.ncon] for g in (c.geom1, c.geom2)
                        if {c.geom1, c.geom2} & self.target_geoms and g not in self.target_geoms
                        and not self.robot_geom[g]}

    # --- kinematics -----------------------------------------------------------------------------------------------
    def _set(self, q, fingers_open=True):
        self.d.qpos[:] = self.live.qpos
        self.d.qvel[:] = 0
        self.d.qpos[self.qadr] = q
        if fingers_open:
            self.d.qpos[self.finger_qadr[0]], self.d.qpos[self.finger_qadr[1]] = 0.04, -0.04

    def site_pose(self, q):
        self._set(q)
        mujoco.mj_kinematics(self.m, self.d)
        return self.d.site_xpos[self.site].copy(), self.d.site_xmat[self.site].reshape(3, 3).copy()

    def _solve_from(self, target, seed):
        p_t, R_t = target[:3, 3], target[:3, :3]
        q = np.clip(seed, self.lo, self.hi)
        jacp, jacr = np.zeros((3, self.m.nv)), np.zeros((3, self.m.nv))
        lam2 = IK_DAMPING ** 2
        history = []
        for it in range(IK_MAX_ITERS):
            self._set(q)
            mujoco.mj_kinematics(self.m, self.d)
            mujoco.mj_comPos(self.m, self.d)
            p, R = self.d.site_xpos[self.site], self.d.site_xmat[self.site].reshape(3, 3)
            e = np.concatenate([p_t - p, Rot.from_matrix(R_t @ R.T).as_rotvec()])
            if np.linalg.norm(e[:3]) < IK_POS_TOL and np.linalg.norm(e[3:]) < IK_ROT_TOL:
                return q
            history.append(np.linalg.norm(e))
            if it >= IK_STALL_ITERS and history[-1] > 0.999 * history[-1 - IK_STALL_ITERS]:
                return None  # stalled: this start will not converge
            mujoco.mj_jacSite(self.m, self.d, jacp, jacr, self.site)
            J = np.vstack([jacp[:, self.vadr], jacr[:, self.vadr]])
            J_pinv = J.T @ np.linalg.inv(J @ J.T + lam2 * np.eye(6))
            dq = J_pinv @ e + (np.eye(7) - J_pinv @ J) @ (IK_NULL_GAIN * (self.mid - q))
            n = np.linalg.norm(dq)
            if n > IK_MAX_STEP:
                dq *= IK_MAX_STEP / n
            q = np.clip(q + dq, self.lo, self.hi)
        return None

    def ik(self, target, seed, cap=None):
        """VoLo's multi-start ladder: the caller's seed, +-pi on joint 7, the robot's home pose, then IK_RANDOM_SEEDS
        draws around the joint mid-range; the first converged solution whose FK rotation error is < 2 deg and, with
        `cap`, whose joints all stay within `cap` of the caller's seed (VoLo's joint-branch consistency cap)."""
        rng = np.random.default_rng(IK_SEED)
        flips = [seed + np.r_[np.zeros(6), s] for s in (np.pi, -np.pi)]
        starts = [seed] + [f for f in flips if self.lo[6] <= f[6] <= self.hi[6]] + [self.home]
        starts += [self.mid + rng.uniform(-IK_RANDOM_RANGE, IK_RANDOM_RANGE, 7) for _ in range(IK_RANDOM_SEEDS)]
        for s in starts:
            q = self._solve_from(target, s)
            if q is None:
                continue
            if cap is not None and np.abs(q - seed).max() > cap:
                continue
            _, R = self.site_pose(q)
            if np.degrees(np.linalg.norm(Rot.from_matrix(target[:3, :3] @ R.T).as_rotvec())) < IK_FK_ROT_TOL_DEG:
                return q
        return None

    def off_line(self, qs, p_from, p_to):
        """Largest distance of the grip site from the segment p_from -> p_to along the joint waypoints qs."""
        seg = p_to - p_from
        worst = 0.0
        for q in qs:
            p, _ = self.site_pose(q)
            s = np.clip((p - p_from) @ seg / max(seg @ seg, 1e-12), 0.0, 1.0)
            worst = max(worst, float(np.linalg.norm(p - (p_from + s * seg))))
        return worst

    # --- collisions -----------------------------------------------------------------------------------------------
    def contacts(self, q, fingers_open=True):
        """(geom1, geom2, depth) of every contact involving the robot at joint angles q."""
        self._set(q, fingers_open)
        mujoco.mj_forward(self.m, self.d)
        out = []
        for c in self.d.contact[: self.d.ncon]:
            if self.robot_geom[c.geom1] or self.robot_geom[c.geom2]:
                out.append((c.geom1, c.geom2, -float(c.dist)))
        return out

    @staticmethod
    def _pairs(contacts):
        return {(min(g1, g2), max(g1, g2)) for g1, g2, _ in contacts}

    def collides(self, q, allow_target=0.0, held=False):
        """True if the (open-handed) robot at q comes within CLEARANCE of anything new: other objects, fixtures,
        the table, itself. Gripper-target contacts are allowed up to `allow_target` deep; with `held` any robot-target
        contact is."""
        for g1, g2, depth in self.contacts(q):
            if (min(g1, g2), max(g1, g2)) in self.ignore:
                continue
            robot, other = (g1, g2) if self.robot_geom[g1] else (g2, g1)
            if other in self.target_geoms and (held or (robot in self.gripper_geoms and depth <= allow_target)):
                continue
            if other in self.support and depth <= 0:  # near the supporting surface, not into it
                continue
            return True
        return False

    def path_collides(self, q_from, q_to):
        n = max(1, int(np.ceil(np.linalg.norm(q_to - q_from) / PATH_STEP)))
        return any(self.collides(q_from + (q_to - q_from) * k / n) for k in range(1, n + 1))


def interpolate(q_from, q_to, n):
    """n joint waypoints from q_from (excluded) to q_to (included)."""
    return [q_from + (q_to - q_from) * k / n for k in range(1, n + 1)]


def path(q_from, q_to, step=PATH_STEP):
    """Joint waypoints from q_from (excluded) to q_to (included) at most `step` apart (L2 over the joints)."""
    return interpolate(q_from, q_to, max(1, int(np.ceil(np.linalg.norm(q_to - q_from) / step))))


# Pick-and-place (core): the target is carried to a goal pose and set down there
PLACE_LIFT = 0.10  # m: the pre-place pose is this far above the place pose (vertical descent onto the spot)
RELEASE_CLEARANCE = 0.005  # m: the object is let go this far above its goal pose
RETREAT_DIST = 0.10  # m: back along the approach axis after letting go
PENETRATION_TOL = 0.001  # m: a carried / placed target may sink this deep into something (resting contact)
# Via height (core, mire), as rehome lifts: the higher of the TCP height at the episode start and the current one
# plus VIA_LIFT. Moves to a pre-grasp pose that collide in a straight joint line go up to it first; core carries at it
# (CoRe: "approach, transport at a fixed carry height, and descent").
VIA_LIFT = 0.10  # m


def _qpos7(T):
    q = Rot.from_matrix(T[:3, :3]).as_quat()  # x y z w
    return np.concatenate([T[:3, 3], [q[3], q[0], q[1], q[2]]])


def pose_from_qpos(qpos7):
    """4x4 pose of a free-joint qpos (x y z qw qx qy qz)."""
    T = np.eye(4)
    T[:3, :3] = Rot.from_quat(np.asarray(qpos7)[[4, 5, 6, 3]]).as_matrix()
    T[:3, 3] = qpos7[:3]
    return T


class CarryKinematics(ArmKinematics):
    """ArmKinematics whose checks can move the target with the hand (held with a body-frame grasp) or put it at a
    given pose."""

    def __init__(self, env, target):
        super().__init__(env, target)
        joint = env.objects_dict[target].joints[0]
        self.obj_qadr = self.m.jnt_qposadr[self.m.joint(joint).id]
        self.site_off = np.eye(4)
        self.site_off[2, 3] = GRIP_SITE_OFFSET

    def _contacts_with(self, q, obj_pose):
        """All contacts with the arm at q and the target at obj_pose (4x4, or a callable of the hand pose)."""
        self._set(q)
        mujoco.mj_kinematics(self.m, self.d)
        if callable(obj_pose):
            T_site = pose_matrix(self.d.site_xpos[self.site], self.d.site_xmat[self.site])
            obj_pose = obj_pose(T_site @ np.linalg.inv(self.site_off))
        self.d.qpos[self.obj_qadr: self.obj_qadr + 7] = _qpos7(obj_pose)
        mujoco.mj_forward(self.m, self.d)
        return [(c.geom1, c.geom2, -float(c.dist)) for c in self.d.contact[: self.d.ncon]]

    def carry_collides(self, q, obj_pose, gripper_target=True, allow_target=0.0):
        """With the target at obj_pose: the robot near anything but the target (as in collides), a non-gripper robot
        part near the target, the gripper into the target deeper than allow_target (unless gripper_target, i.e.
        held), or the target penetrating anything but the robot."""
        for g1, g2, depth in self._contacts_with(q, obj_pose):
            r1, r2 = self.robot_geom[g1], self.robot_geom[g2]
            t1, t2 = g1 in self.target_geoms, g2 in self.target_geoms
            if not (r1 or r2):
                if (t1 or t2) and not (t1 and t2) and depth > PENETRATION_TOL:
                    return True  # the target into the table, a fixture or another object
                continue
            if (min(g1, g2), max(g1, g2)) in self.ignore:
                continue
            robot, other = (g1, g2) if r1 else (g2, g1)
            if other in self.target_geoms:
                if robot in self.gripper_geoms and (gripper_target or depth <= allow_target):
                    continue
                return True
            if other in self.support and depth <= 0:
                continue
            return True
        return False


def _fail(stats, reason):
    stats[reason] = stats.get(reason, 0) + 1
    return None


def via_height(kin, q, z0):
    """The via height (rehome's lift) from arm joints q: max(z0, TCP height at q + VIA_LIFT); None without z0."""
    return None if z0 is None else max(z0, float(kin.site_pose(q)[0][2]) + VIA_LIFT)


def _vertical(kin, q_from, z, step=LIFT_STEP):
    """Joint waypoints moving the TCP straight up / down from its pose at q_from to height z (orientation kept), one
    IK per `step` seeded with the previous waypoint; [] if already there, None if the IK fails."""
    p, R = kin.site_pose(q_from)
    n = int(np.ceil(abs(z - p[2]) / step))
    out, q = [], q_from
    for k in range(1, n + 1):
        q = kin.ik(pose_matrix(np.array([p[0], p[1], p[2] + (z - p[2]) * k / n]), R), q, cap=BRANCH_CAP)
        if q is None:
            return None
        out.append(q)
    return out


def _pick(kin, G, stats, check_path=True, allow=TARGET_CONTACT_TOL, via_z=None):
    """Pre-grasp, approach and lift for the world-frame grasp G of the target (lift checked with the target carried),
    and the joint path from kin.q0 to the pre-grasp unless not check_path: a straight joint line, else (with via_z)
    the TCP straight up to via_z first ("via", joint waypoints) then a straight joint line; the open fingers may touch
    the target `allow` deep at the approach and grasp poses. None (and the reason counted in stats) if infeasible."""
    T_grasp = G @ kin.site_off
    T_pre = T_grasp.copy()
    T_pre[:3, 3] -= PREGRASP_DIST * G[:3, 2]
    q_pre = kin.ik(T_pre, kin.q0)
    if q_pre is None:
        return _fail(stats, "ik_fail")
    if kin.collides(q_pre):
        return _fail(stats, "collision")
    q_grasp = kin.ik(T_grasp, q_pre, cap=BRANCH_CAP)
    if q_grasp is None or kin.collides(q_grasp, allow_target=allow):
        return _fail(stats, "collision")
    T_lift = T_grasp.copy()
    T_lift[2, 3] += LIFT_HEIGHT
    q_lift = kin.ik(T_lift, q_grasp, cap=BRANCH_CAP)
    if q_lift is None:
        return _fail(stats, "ik_fail")
    approach = interpolate(q_pre, q_grasp, int(np.ceil(PREGRASP_DIST / APPROACH_STEP)))
    lift = interpolate(q_grasp, q_lift, int(np.ceil(LIFT_HEIGHT / LIFT_STEP)))
    if (kin.off_line(approach, T_pre[:3, 3], T_grasp[:3, 3]) > LINE_TOL
            or kin.off_line(lift, T_grasp[:3, 3], T_lift[:3, 3]) > LINE_TOL):
        return _fail(stats, "not_straight")
    if any(kin.collides(q, allow_target=allow) for q in approach):
        return _fail(stats, "approach_fail")
    G_b = np.linalg.inv(kin.T_obj) @ G
    carried = lambda T_hand: T_hand @ np.linalg.inv(G_b)
    if any(kin.carry_collides(q, carried) for q in lift):
        return _fail(stats, "lift_fail")
    via = []
    if check_path and kin.path_collides(kin.q0, q_pre):
        if via_z is not None:
            if getattr(kin, "_via", (None,))[0] != via_z:  # the same for every candidate: once per kinematics
                up = _vertical(kin, kin.q0, via_z)
                kin._via = (via_z, None if up is None or any(kin.collides(q) for q in up) else up)
            via = kin._via[1]
        if not via or kin.path_collides(via[-1], q_pre):
            return _fail(stats, "path_collision")
        stats["via"] = stats.get("via", 0) + 1
    return {"q_pre": q_pre, "via": via, "approach": approach, "lift": lift}


def _place(kin, G_b, T_goal, q_from, stats, transport_fn=None, carry_z=None):
    """Transport from q_from (target held with body-frame grasp G_b) to above the goal, vertical descent, release
    RELEASE_CLEARANCE above the goal pose and retreat along the approach axis; None if infeasible. The transport is
    transport_fn(q_preplace) (joint waypoints, None if infeasible), else with carry_z CoRe's three segments (TCP
    straight up to carry_z, a joint line to above the pre-place pose, straight down to it), else the joint path to
    the pre-place pose."""
    G = T_goal @ G_b
    T_place = G @ kin.site_off
    T_place[2, 3] += RELEASE_CLEARANCE
    T_preplace = T_place.copy()
    T_preplace[2, 3] += PLACE_LIFT
    T_retreat = T_place.copy()
    T_retreat[:3, 3] -= RETREAT_DIST * G[:3, 2]
    carried = lambda T_hand: T_hand @ np.linalg.inv(G_b)
    q_preplace = kin.ik(T_preplace, q_from)
    if q_preplace is None:
        return _fail(stats, "place_ik_fail")
    q_place = kin.ik(T_place, q_preplace, cap=BRANCH_CAP)
    q_retreat = None if q_place is None else kin.ik(T_retreat, q_place, cap=BRANCH_CAP)
    if q_retreat is None:
        return _fail(stats, "place_ik_fail")
    descend = interpolate(q_preplace, q_place, int(np.ceil(PLACE_LIFT / APPROACH_STEP)))
    retreat = interpolate(q_place, q_retreat, int(np.ceil(RETREAT_DIST / APPROACH_STEP)))
    if (kin.off_line(descend, T_preplace[:3, 3], T_place[:3, 3]) > LINE_TOL
            or kin.off_line(retreat, T_place[:3, 3], T_retreat[:3, 3]) > LINE_TOL):
        return _fail(stats, "place_not_straight")
    if transport_fn is not None:
        transport = transport_fn(q_preplace)
    elif carry_z is not None:
        z = max(carry_z, float(kin.site_pose(q_from)[0][2]), T_preplace[2, 3])
        up = _vertical(kin, q_from, z)
        T_over = T_preplace.copy()
        T_over[2, 3] = z
        q_over = None if up is None else kin.ik(T_over, q_preplace)
        down = None if q_over is None else _vertical(kin, q_over, T_preplace[2, 3])
        transport = None if down is None else \
            up + path(up[-1] if up else q_from, q_over) + down + path(down[-1] if down else q_over, q_preplace)
    else:
        transport = path(q_from, q_preplace)
    if transport is None:
        return _fail(stats, "transport_fail")
    if any(kin.carry_collides(q, carried) for q in transport + descend):
        return _fail(stats, "place_collision")
    T_rest = T_goal.copy()  # after letting go the target is at its goal pose (dropped RELEASE_CLEARANCE)
    if any(kin.carry_collides(q, T_rest, gripper_target=False, allow_target=TARGET_CONTACT_TOL)
           for q in [q_place] + retreat):
        return _fail(stats, "retreat_collision")
    return {"transport": transport, "descend": descend, "retreat": retreat}


# mire: carrying with a joint-space DMP, a repulsive coupling term when its rollout collides
AVOID_DIST = 0.05  # m: obstacles closer than this to the robot or the held target push them away
GOAL_CONTACT_TOL = 0.005  # m: the robot and the held target may touch the goal's body this deep (resting contact; the
#                           policy's own releases in the clean rollouts press into a drawer's floor up to ~4.4 mm)
AVOID_GAINS = (3e-5, 1e-4, 3e-4)  # coupling gains tried in turn (joint acceleration per unit push, rad/step^2)
STATIC_BODIES = ("base", "link0", "mount", "pedestal", "controller_box", "torso")  # robot bodies that never move


class AvoidKinematics(CarryKinematics):
    """CarryKinematics with a repulsive push: robot or held-target points within AVOID_DIST of an obstacle are pushed
    away along the contact normal (MuJoCo contacts with an AVOID_DIST margin) by 1/d - 1/AVOID_DIST, mapped to the
    arm joints with the Jacobian transpose at the contact point (the held target moves with the hand)."""

    soft_geoms = frozenset()  # the goal's geoms: contacts with them count only beyond GOAL_CONTACT_TOL
    start_contacts = {}  # geom pair -> depth at the carry's start: tolerated while it gets no deeper (moving off it)

    def carry_collides(self, q, obj_pose, gripper_target=True, allow_target=0.0):
        if not self.soft_geoms and not self.start_contacts:
            return super().carry_collides(q, obj_pose, gripper_target, allow_target)
        saved = self._contacts_with

        def kept(c):
            if (c[0] in self.soft_geoms or c[1] in self.soft_geoms) and c[2] <= GOAL_CONTACT_TOL:
                return False
            start = self.start_contacts.get((min(c[0], c[1]), max(c[0], c[1])))
            return start is None or c[2] > start + PENETRATION_TOL

        self._contacts_with = lambda q_, pose: [c for c in saved(q_, pose) if kept(c)]
        try:
            return super().carry_collides(q, obj_pose, gripper_target, allow_target)
        finally:
            del self._contacts_with

    def __init__(self, env, target):
        super().__init__(env, target)
        self.m2 = copy.copy(self.m)
        self.m2.geom_margin[self.robot_geom] = AVOID_DIST
        for g in self.target_geoms:
            self.m2.geom_margin[g] = AVOID_DIST
        self.d2 = copy.copy(self.d)
        self.hand_body = self.m.site_bodyid[self.site]
        self.static = {b for b in range(self.m.nbody) if any(n in self.m.body(b).name for n in STATIC_BODIES)}
        self.jacp, self.jacr = np.zeros((3, self.m.nv)), np.zeros((3, self.m.nv))

    def repulsion(self, q, obj_pose):
        self.d2.qpos[:] = self.live.qpos
        self.d2.qvel[:] = 0
        self.d2.qpos[self.qadr] = q
        mujoco.mj_kinematics(self.m2, self.d2)
        T_site = pose_matrix(self.d2.site_xpos[self.site], self.d2.site_xmat[self.site])
        self.d2.qpos[self.obj_qadr: self.obj_qadr + 7] = _qpos7(obj_pose(T_site @ np.linalg.inv(self.site_off)))
        mujoco.mj_forward(self.m2, self.d2)
        push = np.zeros(7)
        for c in self.d2.contact[: self.d2.ncon]:
            mine1 = self.robot_geom[c.geom1] or c.geom1 in self.target_geoms
            mine2 = self.robot_geom[c.geom2] or c.geom2 in self.target_geoms
            if mine1 == mine2 or c.dist >= AVOID_DIST:
                continue  # robot / held target with itself, or two obstacles; or out of range
            mine = c.geom1 if mine1 else c.geom2
            if (c.geom1 if mine2 else c.geom2) in self.soft_geoms and (mine in self.gripper_geoms
                                                                      or mine in self.target_geoms):
                continue  # the hand and the held target are to reach the goal; the arm links still keep off it
            body = self.hand_body if mine in self.target_geoms else self.m2.geom_bodyid[mine]
            if body in self.static:
                continue
            normal = np.array(c.frame[:3]) * (-1.0 if mine1 else 1.0)  # MuJoCo's normal points from geom1 to geom2
            mujoco.mj_jac(self.m2, self.d2, self.jacp, self.jacr, np.array(c.pos), body)
            push += self.jacp[:, self.vadr].T @ ((1.0 / max(float(c.dist), 0.005) - 1.0 / AVOID_DIST) * normal)
        return push


def dmp_transport(kin, demo, q_from, q_to, carried, max_step, stats):
    """Joint waypoints from q_from to q_to reproducing the shape of `demo` (joint rows; a straight joint path if it
    is shorter than 5 rows) with a joint-space DMP, slowed down to at most max_step (L2) per step, clear of
    collisions with the held target (carried: hand pose -> target pose); a repulsive coupling term is added with
    the AVOID_GAINS in turn while the rollout collides; contacts the held target already has at q_from are tolerated
    while they get no deeper. Returns the waypoints (ending exactly at q_to) or None."""
    if isinstance(kin, AvoidKinematics):  # contacts the held target already has (e.g. lifted beside a carton)
        kin.start_contacts = {(min(g1, g2), max(g1, g2)): d for g1, g2, d in CarryKinematics._contacts_with(
            kin, q_from, carried) if (g1 in kin.target_geoms) != (g2 in kin.target_geoms)}
    try:
        return _dmp_transport(kin, demo, q_from, q_to, carried, max_step, stats)
    finally:
        if isinstance(kin, AvoidKinematics):
            kin.start_contacts = {}


def _dmp_transport(kin, demo, q_from, q_to, carried, max_step, stats):
    from src.dmp import JointDMP, limited_rollout

    # the straight joint line as a minimum-jerk demonstration (at rest at both ends, peak speed max_step)
    n = max(2, int(np.ceil(1.875 * np.linalg.norm(np.asarray(q_to) - q_from) / max_step)))
    r = np.linspace(0.0, 1.0, n + 1)[:, None]
    straight = list(q_from + (np.asarray(q_to) - q_from) * (10 * r ** 3 - 15 * r ** 4 + 6 * r ** 5))
    demos = [("straight", straight)]
    if demo is not None and len(demo) >= 5:
        demos.insert(0, ("demo", demo))
    for kind, rows in demos:  # the demonstration's shape; else a straight joint path (when the shape cannot be kept)
        dmp = JointDMP(np.asarray(rows))
        tau0 = max(dmp.tau, 2.0 * np.linalg.norm(np.asarray(q_to) - q_from) / max_step)  # a bell-shaped speed profile
        for gain in (0.0,) + AVOID_GAINS:
            coupling = None if not gain else \
                (lambda tau, gain=gain: lambda x, v: tau ** 2 * gain * kin.repulsion(x, carried))  # rad/step^2 units
            qs, tau = limited_rollout(dmp, q_from, q_to, max_step, coupling, kin.lo, kin.hi, tau=tau0)
            if qs is not None and not any(kin.carry_collides(q, carried) for q in qs):
                stats.update(dmp_shape=kind, dmp_gain=gain, dmp_steps=len(qs), dmp_tau=round(tau, 1))
                return qs
    stats["dmp_collides"] = stats.get("dmp_collides", 0) + 1
    return None


def plan_pick(env, target, G_world):
    """Pick (pre-grasp, approach, lift; as plan_pick_place) of `target` with the world-frame grasp G_world or its
    symmetric variant, whichever is feasible first; None if neither."""
    kin = CarryKinematics(env, target)
    body = env.obj_body_id[target]
    kin.T_obj = pose_matrix(env.sim.data._data.xpos[body], env.sim.data._data.xmat[body])
    stats = {}
    for G in (G_world, G_world @ _FLIP_Z):
        pick = _pick(kin, G, stats)
        if pick is not None:
            return {"grasp_world": G, **pick}, stats
    return None, stats


def plan_pick_place(env, target, goal_qpos, held=False, z0=None):
    """Plan to carry `target` to the free-joint pose goal_qpos: pick (pre-grasp, approach, close, lift) with the
    first candidate, in GraspGen confidence order and either symmetric variant, whose pick and place are both
    feasible, then place (transport, vertical descent, release, retreat). With `held` (the target is in the closed
    gripper) the current hold is used instead: a vertical lift, then the place. With z0 (the TCP height at the
    episode start), moves to the pre-grasp go via the via height if needed and the transport is CoRe's three segments
    at it (_pick, _place). Returns (plan, stats); plan None if infeasible."""
    kin = CarryKinematics(env, target)
    body = env.obj_body_id[target]
    kin.T_obj = pose_matrix(env.sim.data._data.xpos[body], env.sim.data._data.xmat[body])
    T_goal = pose_from_qpos(goal_qpos)
    stats = {"candidates": 0, "ik_fail": 0, "collision": 0}
    if held:
        T_site = pose_matrix(*kin.site_pose(kin.q0))
        G_b = np.linalg.inv(kin.T_obj) @ T_site @ np.linalg.inv(kin.site_off)
        T_lift = T_site.copy()
        T_lift[2, 3] += LIFT_HEIGHT
        q_lift = kin.ik(T_lift, kin.q0, cap=BRANCH_CAP)
        if q_lift is None:
            return None, {**stats, "held_lift_fail": 1}
        lift = interpolate(kin.q0, q_lift, int(np.ceil(LIFT_HEIGHT / LIFT_STEP)))
        if any(kin.carry_collides(q, lambda T: T @ np.linalg.inv(G_b)) for q in lift):
            return None, {**stats, "held_lift_fail": 1}
        place = _place(kin, G_b, T_goal, q_lift, stats, carry_z=via_height(kin, q_lift, z0))
        if place is None:
            return None, stats
        return {"held": True, "lift": lift, **place}, stats
    grasps, conf = load_candidates(target)
    stats["candidates"] = len(grasps)
    world = kin.T_obj @ grasps
    support_z = (load_points(target) @ kin.T_obj[:3, :3].T + kin.T_obj[:3, 3])[:, 2].min()
    goal_z = (load_points(target) @ T_goal[:3, :3].T + T_goal[:3, 3])[:, 2].min()
    hull = lambda W: np.einsum("nij,kj->nki", W[:, :3, :3], GRIPPER_HULL)[..., 2].min(1) + W[:, 2, 3]
    below = (hull(world) < support_z - SUPPORT_MARGIN) | (hull(T_goal @ grasps) < goal_z - SUPPORT_MARGIN)
    stats["below_support"] = int(below.sum())
    for i in range(len(grasps)):
        if below[i]:
            continue
        for G in (world[i], world[i] @ _FLIP_Z):  # the two symmetric variants, each with its own pick and place
            pick = _pick(kin, G, stats, via_z=via_height(kin, kin.q0, z0))
            if pick is None:
                continue
            G_b = np.linalg.inv(kin.T_obj) @ G
            place = _place(kin, G_b, T_goal, pick["lift"][-1], stats, carry_z=via_height(kin, pick["lift"][-1], z0))
            if place is None:
                continue
            stats["rank"] = i
            return {"held": False, "index": i, "conf": float(conf[i]), "grasp_world": G, **pick, **place}, stats
    return None, stats


def plan_grasp(env, target, order=None):
    """Plan a grasp of `target` from the current state. Candidates are visited best-first (GraspGen confidence, or
    `order(grasps_world, conf) -> indices`) and the first feasible one is returned as a dict with the joint waypoints
    of each phase, or None with the reason if none is."""
    grasps, conf = load_candidates(target)
    body = env.obj_body_id[target]
    T_obj = pose_matrix(env.sim.data._data.xpos[body], env.sim.data._data.xmat[body])
    kin = ArmKinematics(env, target)
    site_off = np.eye(4)
    site_off[2, 3] = GRIP_SITE_OFFSET
    stats = {"candidates": len(grasps), "ik_fail": 0, "collision": 0}
    if not len(grasps):
        return None, stats
    world = T_obj @ grasps
    idx = order(world, conf) if order else np.arange(len(grasps))
    # Cheap pre-filter before IK: drop grasps whose hand or fingers would reach more than SUPPORT_MARGIN below the
    # lowest point of the object (it rests there); the MuJoCo collision check below remains the actual test
    support_z = (load_points(target) @ T_obj[:3, :3].T + T_obj[:3, 3])[:, 2].min()
    hand = np.einsum("nij,kj->nki", world[:, :3, :3], GRIPPER_HULL) + world[:, None, :3, 3]
    below = hand[..., 2].min(1) < support_z - SUPPORT_MARGIN
    stats["below_support"] = int(below.sum())
    for i in idx:
        if below[i]:
            continue
        best = None
        for G in (world[i], world[i] @ _FLIP_Z):  # the two equivalent orientations of the symmetric gripper
            T_grasp = G @ site_off  # grip_site pose at the grasp
            approach = G[:3, 2]
            T_pre = T_grasp.copy()
            T_pre[:3, 3] -= PREGRASP_DIST * approach
            q_pre = kin.ik(T_pre, kin.q0)
            if q_pre is None:
                continue
            cost = np.linalg.norm(q_pre - kin.q0)
            if best is None or cost < best[0]:
                best = (cost, G, T_grasp, T_pre, q_pre)
        if best is None:
            stats["ik_fail"] += 1
            continue
        _, G, T_grasp, T_pre, q_pre = best
        # Cheapest checks first: the pre-grasp and the grasp pose, then the approach / lift waypoints (straight lines,
        # each seeded with the previous solution), finally the joint path from the current pose to the pre-grasp
        if kin.collides(q_pre):
            stats["collision"] += 1
            continue
        q_grasp = kin.ik(T_grasp, q_pre, cap=BRANCH_CAP)
        if q_grasp is None:
            stats["ik_fail"] += 1
            continue
        if kin.collides(q_grasp, allow_target=TARGET_CONTACT_TOL):
            stats["collision"] += 1
            continue
        T_lift = T_grasp.copy()
        T_lift[2, 3] += LIFT_HEIGHT
        q_lift = kin.ik(T_lift, q_grasp, cap=BRANCH_CAP)
        if q_lift is None:
            stats["ik_fail"] += 1
            continue
        # Approach and lift: joint-space interpolation between the key poses (as VoLo streams its joint trajectory),
        # which must stay close to the straight line and clear of everything
        approach_q = interpolate(q_pre, q_grasp, int(np.ceil(PREGRASP_DIST / APPROACH_STEP)))
        lift_q = interpolate(q_grasp, q_lift, int(np.ceil(LIFT_HEIGHT / LIFT_STEP)))
        if (kin.off_line(approach_q, T_pre[:3, 3], T_grasp[:3, 3]) > LINE_TOL
                or kin.off_line(lift_q, T_grasp[:3, 3], T_lift[:3, 3]) > LINE_TOL):
            stats["not_straight"] = stats.get("not_straight", 0) + 1
            continue
        if any(kin.collides(qa, allow_target=TARGET_CONTACT_TOL) for qa in approach_q):
            stats["approach_fail"] = stats.get("approach_fail", 0) + 1
            continue
        if any(kin.collides(ql, held=True) for ql in lift_q):
            stats["lift_fail"] = stats.get("lift_fail", 0) + 1
            continue
        if kin.path_collides(kin.q0, q_pre):
            stats["path_collision"] = stats.get("path_collision", 0) + 1
            continue
        stats["rank"] = int(np.where(idx == i)[0][0])
        return {"index": int(i), "conf": float(conf[i]), "grasp_world": G, "q_pre": q_pre, "approach": approach_q,
                "lift": lift_q, "approach_axis": G[:3, 2].tolist()}, stats
    return None, stats


# mire: the policy's own grasp (from the clean rollout) up to the object's symmetry; else set the object down upright
# with another grasp and take it with the own grasp from there
# Rotational symmetry of the targets: (body axes, equivalent turns about each; 0 = any angle). The GraspGen point cloud
# turned about such an axis lies on itself (mean nearest-point distance under 2.5% of the cloud's size, against
# 3.4-7.5% for the mugs and moka pots, whose handles break it): such a turn changes nothing for a pose or a grasp.
SYMMETRY = {"alphabet_soup": ("y", 0), "akita_black_bowl": ("z", 0), "cream_cheese": ("xyz", 2),
            "black_book": ("xyz", 2)}
SYMMETRY_STEP = 10.0  # deg: sampling of a continuous symmetry
SUPPORT_PROBE = 0.003  # m: a set-down pose rests on the table if the object lowered this much touches only a table


def symmetries(name):
    """Body-frame rotations (n x 3 x 3) leaving the object `name` unchanged, the identity first."""
    axes, n = SYMMETRY.get(object_kind(name), ("", 1))
    out = [np.eye(3)]
    for ax in axes:
        k = n or int(round(360.0 / SYMMETRY_STEP))
        out += [Rot.from_rotvec(2 * np.pi * j / k * np.eye(3)["xyz".index(ax)]).as_matrix() for j in range(1, k)]
    return np.array(out)


def rot_deg(Ra, Rb):
    """Angle (deg) of the rotation between Ra and Rb (3 x 3; broadcasts over leading axes)."""
    c = (np.einsum("...ij,...ij->...", Ra, Rb) - 1.0) / 2.0
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


def turn_deg(name, R_now, R_ref):
    """How far the object `name` has turned from orientation R_ref to R_now (deg), up to its symmetry."""
    return float(rot_deg(R_now, R_ref @ symmetries(name)).min())


FLIP_DEG, FLIP_DIST = 10.0, 0.02  # deg, m: the hand turned by pi about its approach axis holds the object the same way


def grasp_deg(name, g_b, G_b, flip=True):
    """Rotation (deg) between the body-frame grasps g_b (n x 4 x 4) and G_b of the object `name`, up to the object's
    symmetry and, with `flip`, the gripper's (turning the hand by pi about its approach axis)."""
    refs = symmetries(name) @ G_b[:3, :3]
    if flip:
        refs = np.concatenate([refs, refs @ _FLIP_Z[:3, :3]])
    return rot_deg(np.asarray(g_b)[:, None, :3, :3], refs).min(-1)


def near_grasps(name, g_b, G_b, max_deg, max_dist, flip=True):
    """Whether each body-frame grasp g_b (n x 4 x 4) is within max_deg (rotation) and max_dist (hand position) of G_b,
    for one and the same of G_b's variants under the object's symmetry (and, with `flip`, the gripper's: same
    position)."""
    S = symmetries(name)
    R_ref, p_ref = S @ G_b[:3, :3], G_b[:3, 3] @ S.transpose(0, 2, 1)  # each symmetric variant's hand rotation, origin
    g = np.asarray(g_b)
    rot = rot_deg(g[:, None, :3, :3], R_ref)
    if flip:
        rot = np.minimum(rot, rot_deg(g[:, None, :3, :3], R_ref @ _FLIP_Z[:3, :3]))
    dist = np.linalg.norm(g[:, None, :3, 3] - p_ref[None], axis=-1)
    return ((rot <= max_deg) & (dist <= max_dist)).any(1)


def flip_equivalent(name, G_b):
    """Whether the hand turned by pi about its approach axis holds the object `name` as G_b does (a symmetry of the
    object maps one onto the other). Otherwise the turned hand holds it turned by pi: carried to the policy's own hand
    pose, the object arrives turned (and the hand itself is turned by pi from the policy's when the object is not)."""
    return bool(near_grasps(name, (G_b @ _FLIP_Z)[None], G_b, FLIP_DEG, FLIP_DIST, flip=False)[0])


def _with_symmetry(name, G_b):
    """G_b turned by each of the object's symmetries; each also with the hand turned by pi about its approach axis if
    that holds the object the same way (flip_equivalent) (n x 4 x 4)."""
    S = np.tile(np.eye(4), (len(symmetries(name)), 1, 1))
    S[:, :3, :3] = symmetries(name)
    return np.concatenate([S @ G_b, S @ G_b @ _FLIP_Z]) if flip_equivalent(name, G_b) else S @ G_b


def _below_support(kin, target, world):
    """GraspGen's pre-filter (plan_grasp): world grasps reaching SUPPORT_MARGIN below the object's lowest point."""
    support_z = (load_points(target) @ kin.T_obj[:3, :3].T + kin.T_obj[:3, 3])[:, 2].min()
    hand = np.einsum("nij,kj->nki", world[:, :3, :3], GRIPPER_HULL) + world[:, None, :3, 3]
    return hand[..., 2].min(1) < support_z - SUPPORT_MARGIN


def _kin_now(env, target):
    kin = CarryKinematics(env, target)
    body = env.obj_body_id[target]
    kin.T_obj = pose_matrix(env.sim.data._data.xpos[body], env.sim.data._data.xmat[body])
    return kin


def plan_own(env, target, G_b, R_ref, near_deg, near_dist, max_variants, max_near, kin=None, check_path=True,
             allow=TARGET_CONTACT_TOL, z0=None):
    """Pick `target` with the policy's own grasp G_b (body frame, from the clean rollout): G_b itself, turned by the
    object's symmetries (and the gripper's), at most max_variants of them; then at most max_near GraspGen candidates
    (either gripper variant) within near_deg and near_dist of it (near_grasps). Each set is visited in order of the
    hand's world rotation from R_ref (the policy's hand at that grasp), the first feasible pick (as plan_pick)
    returned. `kin`: the kinematics of the checks (default: the current state); check_path, z0 (via height): as
    _pick; allow: how deep
    G_b's open fingers may touch the target (own_contact: as deep as the policy's own did; GraspGen's candidates keep
    TARGET_CONTACT_TOL). Returns (plan, stats); plan["kind"] is "clean" (G_b) or "near" (GraspGen), None if neither."""
    kin = kin or _kin_now(env, target)
    stats = {"variants": 0, "near": 0}
    sets = [("clean", kin.T_obj @ _with_symmetry(target, G_b), None, max_variants)]
    grasps, _ = load_candidates(target)
    if max_near:  # each candidate with the hand orientation that holds the object as G_b does (both if equivalent)
        both = flip_equivalent(target, G_b)
        direct = np.where(near_grasps(target, grasps, G_b, near_deg, near_dist, flip=both))[0]
        turned = np.where(near_grasps(target, grasps @ _FLIP_Z, G_b, near_deg, near_dist, flip=both))[0]
        world = np.concatenate([kin.T_obj @ grasps[direct], kin.T_obj @ grasps[turned] @ _FLIP_Z])
        if len(world):
            sets.append(("near", world, np.concatenate([direct, turned]), max_near))
    for kind, world, index, cap in sets:
        dev, below = rot_deg(world[:, :3, :3], R_ref), _below_support(kin, target, world)
        order = [i for i in np.argsort(dev, kind="stable") if not below[i]]
        for i in order[:cap]:
            stats["variants" if kind == "clean" else "near"] += 1
            pick = _pick(kin, world[i], stats, check_path, allow if kind == "clean" else TARGET_CONTACT_TOL,
                         via_height(kin, kin.q0, z0))
            if pick is not None:
                return {"kind": kind, "grasp_world": world[i], "world_deg": round(float(dev[i]), 1),
                        "index": None if index is None else int(index[i]), **pick}, stats
    return None, stats


def own_contact(env, target, q, T_obj):
    """How deep (m) the open fingers / hand at arm joints q touch the target at pose T_obj (e.g. the policy's own grasp
    in the clean rollout: its collision meshes may overlap where the policy's fingers are), 0 if they do not."""
    kin = CarryKinematics(env, target)
    depth = [d for g1, g2, d in kin._contacts_with(np.asarray(q, float), T_obj)
             if (g1 in kin.target_geoms) != (g2 in kin.target_geoms) and (kin.gripper_geoms & {g1, g2})]
    return max([0.0] + depth)


def _kin_set_down(kin_now, env, target, T_obj):
    """The kinematics of the current state with `target` at T_obj instead (resting there; the arm where it is)."""
    kin = CarryKinematics(env, target)
    kin.live = copy.copy(kin_now.live)
    kin.live.qpos[kin.obj_qadr: kin.obj_qadr + 7] = _qpos7(T_obj)
    kin.live.qvel[:] = 0
    kin.T_obj, kin.ignore = T_obj, kin_now.ignore
    T_low = T_obj.copy()
    T_low[2, 3] -= SUPPORT_PROBE
    touched = {g for g1, g2, depth in CarryKinematics._contacts_with(kin, kin.q0, T_low) for g in (g1, g2)
               if ((g1 in kin.target_geoms) != (g2 in kin.target_geoms)) and g not in kin.target_geoms
               and not kin.robot_geom[g]}
    kin.support = touched
    on_table = bool(touched) and all("table" in kin.m.body(kin.m.geom_bodyid[g]).name for g in touched)
    return kin, on_table


def plan_set_down(env, target, G_b, R_ref, T_up, spots, max_variants, max_picks, held=False, allow=TARGET_CONTACT_TOL,
                  z0=None, max_step=None):
    """Set `target` down in its pose before the policy's grasp in the clean rollout (T_up: its orientation, the
    rotation about the vertical included, and height) at one of `spots` ((name, xy), in order; resting on the table
    only, and allowing the policy's own grasp G_b there), then take it with G_b from there (plan_own with G_b's
    symmetric variants only, from the arm's pose after letting go). The target is picked with a GraspGen grasp,
    visited in confidence order: like core's restoration, every feasible pick is tried at the first usable spot until
    one also sets it down there; the first max_picks feasible picks are then tried at the other spots. With `held`,
    it is carried with the current hold after a vertical lift instead. Placed as core places (_place), but carried
    there along a joint-space DMP (dmp_transport: a straight joint demonstration, repulsive coupling while it
    collides; at most max_step per step). allow, z0 (via height of moves to a pre-grasp): as plan_own. Returns (plan, stats, first): plan None if nothing is feasible; first: the first feasible GraspGen pick
    (GraspGen's best feasible grasp, as plan_grasp; None with `held` or if there is none)."""
    kin = _kin_now(env, target)
    stats = {"candidates": 0, "ik_fail": 0, "collision": 0, "picks": 0, "poses": 0, "off_table": 0, "no_own": 0}
    grasps, conf = load_candidates(target)
    kin_av = AvoidKinematics(env, target)  # the DMP carry's collision checks and repulsion
    usable = []  # (name, pose, kinematics) of the spots on the table where the own grasp is feasible at all
    for spot, xy in spots:
        stats["poses"] += 1
        T = T_up.copy()
        T[:2, 3] = xy
        kin2, on_table = _kin_set_down(kin, env, target, T)
        if not on_table:
            stats["off_table"] += 1
        elif plan_own(env, target, G_b, R_ref, 0.0, 0.0, max_variants, 0, kin=kin2, check_path=False,
                      allow=allow)[0] is None:
            stats["no_own"] += 1  # not even without the move to it
        else:
            usable.append((spot, T, kin2))

    def set_down(i, G, pick, spot, T, kin2):
        G_hold = np.linalg.inv(kin.T_obj) @ G
        carried = lambda T_hand: T_hand @ np.linalg.inv(G_hold)
        q_from = pick["lift"][-1]
        place = _place(kin, G_hold, T, q_from, stats,
                       transport_fn=lambda q_pre: dmp_transport(kin_av, None, q_from, q_pre, carried, max_step, stats))
        if place is None:
            return None
        kin2.q0 = place["retreat"][-1]  # the own grasp from where the arm is after letting go
        kin2._via = (None,)
        own, _ = plan_own(env, target, G_b, R_ref, 0.0, 0.0, max_variants, 0, kin=kin2, allow=allow, z0=z0)
        if own is None:
            stats["no_own_after"] = stats.get("no_own_after", 0) + 1
            return None
        return {"index": None if i is None else int(i), "conf": None if i is None else float(conf[i]),
                "grasp_world": G, "spot": spot, "T_goal": T, "own_kind": own["kind"], **pick, **place}

    if held:
        T_site = pose_matrix(*kin.site_pose(kin.q0))
        G = T_site @ np.linalg.inv(kin.site_off)
        T_lift = T_site.copy()
        T_lift[2, 3] += LIFT_HEIGHT
        q_lift = kin.ik(T_lift, kin.q0, cap=BRANCH_CAP)
        lift = None if q_lift is None else interpolate(kin.q0, q_lift, int(np.ceil(LIFT_HEIGHT / LIFT_STEP)))
        G_b_now = np.linalg.inv(kin.T_obj) @ G
        if lift is None or any(kin.carry_collides(q, lambda T: T @ np.linalg.inv(G_b_now)) for q in lift):
            return None, stats, None
        stats["picks"] = 1
        for spot, T, kin2 in usable:
            plan = set_down(None, G, {"lift": lift}, spot, T, kin2)
            if plan is not None:
                return plan, stats, None
        return None, stats, None
    stats["candidates"] = len(grasps)
    world = kin.T_obj @ grasps
    below = _below_support(kin, target, world) if len(grasps) else []
    picks, first = [], None
    for i in range(len(grasps)):
        if below[i]:
            continue
        for G in (world[i], world[i] @ _FLIP_Z):
            pick = _pick(kin, G, stats, via_z=via_height(kin, kin.q0, z0))
            if pick is None:
                continue
            if first is None:
                first = {"index": int(i), "conf": float(conf[i]), "grasp_world": G, **pick}
            picks.append((i, G, pick))
            if usable:
                plan = set_down(i, G, pick, *usable[0])
                if plan is not None:
                    stats["picks"] = len(picks)
                    return plan, stats, first
            break
    stats["picks"] = len(picks)
    for spot, T, kin2 in usable[1:]:
        for i, G, pick in picks[:max_picks]:
            plan = set_down(i, G, pick, spot, T, kin2)
            if plan is not None:
                return plan, stats, first
    return None, stats, first
