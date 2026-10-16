"""
generate_grasps.py

Pre-generates GraspGen (Franka Panda) grasp candidates for every movable LIBERO object type of a task suite, in the
object's body frame, for the grasp primitives in recovery.py. The point cloud is sampled from the object's visual
meshes as compiled in the MuJoCo model (exact simulator geometry and scale), keeping only points on the outer surface
(some meshes model hidden inner surfaces, e.g. the pages inside the book); GraspGen's outlier removal is not used. Grasps follow GraspGen's gripper
convention: approach along +z, fingers closing along x, fingertips at z = 0.1053 m. At runtime the candidates are
mapped to the object's current pose and filtered (reachability, collisions); see recovery.py. Each file stores, sorted
by GraspGen confidence: grasps (N, 4, 4, body frame), conf (N,), width (N,; object span between the finger pads, to
compare with the robosuite Panda's 8 cm opening; NaN if nothing is between the pads) and the point cloud used.

    python src/panda/generate_grasps.py [--num_grasps 2000]
"""

import os
import re
import sys
import types
from dataclasses import dataclass

import draccus
import mujoco
import numpy as np
import torch
import trimesh
from scipy import ndimage
from libero.libero import benchmark, get_libero_path
from libero.libero.envs import OffScreenRenderEnv

import src.paths as P


@dataclass
class GraspGenConfig:
    # fmt: off
    task_suite_name: str = "libero_10"
    gripper_config: str = os.path.join(P.ROOT, "checkpoints", "GraspGenModels", "checkpoints", "graspgen_franka_panda.yml")
    out_dir: str = ""                             # "" = outputs/grasps/<suite>
    num_points: int = 2048                        # Object surface samples fed to GraspGen
    num_grasps: int = 2000                        # Candidates kept per object (generated in batches)
    batch: int = 500                              # Grasps generated per GraspGen call
    seed: int = 0
    # fmt: on


GRIPPER_DEPTH, GRIPPER_WIDTH = 0.10527314, 0.10537486  # GraspGen franka_panda: base to fingertips, opening


def grasp_occupancy(grasps, pc):
    """Object points (counts per grasp) in the gap between the open fingers and in the hand below the fingers."""
    local = np.einsum("nji,nkj->nki", grasps[:, :3, :3], pc[None] - grasps[:, None, :3, 3])  # gripper frame
    x, y, z = local[..., 0], local[..., 1], local[..., 2]
    gap = (np.abs(x) < GRIPPER_WIDTH / 2 - 0.005) & (np.abs(y) < 0.01) & (z > GRIPPER_DEPTH / 2) & (z < GRIPPER_DEPTH)
    hand = (np.abs(x) < GRIPPER_WIDTH / 2) & (np.abs(y) < 0.03) & (z > 0) & (z < GRIPPER_DEPTH / 2 - 0.005)
    return gap.sum(1), hand.sum(1)


def required_opening(grasps, pc, pad=(0.085, 0.105)):
    """Object span along the closing axis inside the finger-pad band (robosuite Panda pads: z = 0.085-0.101 m from
    the hand); NaN if nothing is between the pads. The robosuite gripper opens 8 cm."""
    local = np.einsum("nji,nkj->nki", grasps[:, :3, :3], pc[None] - grasps[:, None, :3, 3])
    x, y, z = local[..., 0], local[..., 1], local[..., 2]
    band = (np.abs(x) < GRIPPER_WIDTH / 2) & (np.abs(y) < 0.01) & (z > pad[0]) & (z < pad[1])
    span = np.where(band, x, -np.inf).max(1) - np.where(band, x, np.inf).min(1)
    return np.where(band.any(1), span, np.nan)


def subtree_geoms(m, body):
    bodies = {b for b in range(m.nbody) if _ancestor(m, b, body)}
    return [g for g in range(m.ngeom) if m.geom_bodyid[g] in bodies]


def collision_extent(m, d, geoms, body):
    """Body-frame extent of the collision geoms' bounding boxes (MuJoCo geom AABBs, corners transformed)."""
    R_b, p_b = d.xmat[body].reshape(3, 3), d.xpos[body]
    corners = []
    for g in geoms:
        c, h = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
        local = c + h * np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
        world = local @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        corners.append((world - p_b) @ R_b)
    corners = np.concatenate(corners) if corners else np.zeros((1, 3))
    return corners.max(0) - corners.min(0)


def subtree_mesh_geoms(m, body):
    """Mesh geoms of `body` and its descendants; the visual ones (contype = conaffinity = 0) if there are any."""
    bodies = {b for b in range(m.nbody) if _ancestor(m, b, body)}
    geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] in bodies and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
    visual = [g for g in geoms if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0]
    return visual or geoms


def _ancestor(m, b, body):
    while b > 0:
        if b == body:
            return True
        b = m.body_parentid[b]
    return False


def body_mesh(m, d, geoms, body):
    """The geoms' triangles as one trimesh in the body frame."""
    R_b, p_b = d.xmat[body].reshape(3, 3), d.xpos[body]
    V, F, off = [], [], 0
    for g in geoms:
        mid = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[mid]: m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
        f = m.mesh_face[m.mesh_faceadr[mid]: m.mesh_faceadr[mid] + m.mesh_facenum[mid]]
        world = v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
        V.append((world - p_b) @ R_b)
        F.append(f + off)
        off += len(v)
    return trimesh.Trimesh(np.concatenate(V), np.concatenate(F), process=False)


def surface_points(m, d, geoms, body, n, rng):
    """Area-weighted samples of the geoms' triangles, in the body frame."""
    mesh = body_mesh(m, d, geoms, body)
    return sample_triangles(mesh.triangles, n, rng)


def sample_triangles(tris, n, rng):
    area = 0.5 * np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1)
    idx = rng.choice(len(tris), n, p=area / area.sum())
    u, v = rng.random(n), rng.random(n)
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    t = tris[idx]
    return t[:, 0] + u[:, None] * (t[:, 1] - t[:, 0]) + v[:, None] * (t[:, 2] - t[:, 0])


def exterior_points(m, d, geoms, body, n, rng, pitch=0.003, batch=8192):
    """Like surface_points, but only points on the outer surface: the mesh is voxelised at `pitch`, the empty space
    connected to the outside is flood-filled, and a sample is kept if a voxel next to it is outside. Gaps narrower
    than the pitch stay sealed, so the pages modelled inside the book are dropped while a mug's open inside (and the
    page block's edges) remain. Returns the points and the share of samples that were exterior."""
    mesh = body_mesh(m, d, geoms, body)
    occ = mesh.voxelized(pitch).matrix
    occ = np.pad(occ, 2)
    filled = ndimage.binary_fill_holes(occ)  # occupied or enclosed
    outside = ndimage.binary_dilation(~filled, structure=np.ones((3, 3, 3), bool))  # empty-outside or next to it
    origin = mesh.voxelized(pitch).translation - 2 * pitch  # centre of voxel (0, 0, 0) after padding
    kept, tried = [], 0
    while sum(len(k) for k in kept) < n:
        pts = sample_triangles(mesh.triangles, batch, rng)
        idx = np.clip(np.round((pts - origin) / pitch).astype(int), 0, np.array(occ.shape) - 1)
        kept.append(pts[outside[idx[:, 0], idx[:, 1], idx[:, 2]]])
        tried += batch
    kept = np.concatenate(kept)
    return kept[rng.choice(len(kept), n, replace=False)], len(kept) / tried


def stub_pointnet2_ops():
    """GraspGen JIT-compiles its pointnet2_ops CUDA extension when the package is not installed, which fails here (the
    system nvcc 11.8 does not match torch cu121). The Franka models are PTv3 and never call these ops, so empty stand-in
    modules are enough, and GraspGen itself stays unmodified."""
    try:
        import pointnet2_ops  # noqa: F401
    except ImportError:
        stub = types.ModuleType("pointnet2_ops")
        stub._ext = types.ModuleType("pointnet2_ops._ext")
        sys.modules["pointnet2_ops"], sys.modules["pointnet2_ops._ext"] = stub, stub._ext


@draccus.wrap()
def generate(cfg: GraspGenConfig) -> None:
    stub_pointnet2_ops()
    cfg.out_dir = cfg.out_dir or P.out("grasps", cfg.task_suite_name)
    from grasp_gen.grasp_server import GraspGenSampler, load_grasp_cfg

    rng = np.random.default_rng(cfg.seed)
    torch.manual_seed(cfg.seed)
    sampler = GraspGenSampler(load_grasp_cfg(cfg.gripper_config))
    os.makedirs(cfg.out_dir, exist_ok=True)
    suite = benchmark.get_benchmark_dict()[cfg.task_suite_name]()
    done = set()
    for tid in range(suite.n_tasks):
        task = suite.get_task(tid)  # The LIBERO env as the eval builds it (openvla-oft libero_utils.get_libero_env)
        env = OffScreenRenderEnv(bddl_file_name=os.path.join(get_libero_path("bddl_files"), task.problem_folder, task.bddl_file),
                                 camera_heights=64, camera_widths=64)
        env.seed(0)
        env.reset()
        m, d = env.env.sim.model._model, env.env.sim.data._data
        for name in env.env.objects_dict:
            kind = re.sub(r"_\d+$", "", name)  # e.g. moka_pot_2 -> moka_pot
            if kind in done:
                continue
            body = env.env.obj_body_id[name]
            geoms = subtree_mesh_geoms(m, body)
            pc, exterior = exterior_points(m, d, geoms, body, cfg.num_points, rng)
            grasps, conf = [], []
            while sum(len(g) for g in grasps) < cfg.num_grasps:
                # No outlier removal: it is meant for noisy depth clouds and deletes thin parts (mug handles) and
                # most of a large object's sparse mesh samples
                g, c = GraspGenSampler.run_inference(pc, sampler, grasp_threshold=-1.0, num_grasps=cfg.batch,
                                                     topk_num_grasps=cfg.batch, min_grasps=1, max_tries=1,
                                                     remove_outliers=False)
                if len(g) == 0:
                    break
                grasps.append(g.cpu().numpy())
                conf.append(c.cpu().numpy())
            grasps, conf = np.concatenate(grasps)[: cfg.num_grasps], np.concatenate(conf)[: cfg.num_grasps]
            order = np.argsort(-conf)
            grasps, conf = grasps[order], conf[order]
            width = required_opening(grasps, pc)
            np.savez_compressed(os.path.join(cfg.out_dir, f"{kind}.npz"), grasps=grasps.astype(np.float32),
                                conf=conf.astype(np.float32), width=width.astype(np.float32),
                                points=pc.astype(np.float32), exterior_share=exterior, source_task=tid,
                                source_object=name,
                                geoms=[m.geom(g).name for g in geoms])
            approach_z = grasps[:, 2, 2]  # body-frame z of the approach axis
            between, palm = grasp_occupancy(grasps, pc)
            coll = [g for g in subtree_geoms(m, body) if m.geom_contype[g] or m.geom_conaffinity[g]]
            span_v, span_c = pc.max(0) - pc.min(0), collision_extent(m, d, coll, body)
            print(f"{'':22s} check: wider than the 8 cm opening {np.mean(width > 0.078):.0%} (top-200 "
                  f"{np.mean(width[:200] > 0.078):.0%}), nothing between the pads {np.mean(np.isnan(width)):.0%}", flush=True)
            print(f"{'':22s} check: object between the fingers {np.mean(between >= 5):.0%}, palm intersects object "
                  f"{np.mean(palm >= 5):.0%}; visual extent {np.round(100 * span_v, 1).tolist()} cm vs collision "
                  f"{np.round(100 * span_c, 1).tolist()} cm", flush=True)
            print(f"{kind:22s} task {tid} geoms {len(geoms):2d}  exterior {exterior:.0%}  grasps {len(grasps):4d}  "
                  f"conf {conf.mean():.2f}  "
                  f"approach from body-top {np.mean(approach_z < -0.7):.0%} / side {np.mean(abs(approach_z) <= 0.3):.0%} "
                  f"/ bottom {np.mean(approach_z > 0.3):.0%}", flush=True)
            done.add(kind)
        env.close()


if __name__ == "__main__":
    generate()
