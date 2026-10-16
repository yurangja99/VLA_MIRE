"""
replay_libero.py

Re-renders episodes of run_libero_eval_perturb.py from their saved policy actions, without the model. Each selected
JSONL record is replayed with the same perturbation (family, timing, seed) and recovery, the outcome is checked against
the record, and the video is written to the path the eval run would have used
(outputs/videos/<suite>/oft/<run_id>/task=...mp4) unless --out_dir is given. Records are selected with filters, e.g.

    python src/panda/oft/replay_libero.py --recovery rehome --family object_tip --task_ids 0-4 --success false
"""

import glob
import json
import os
from dataclasses import dataclass
from typing import Optional

import cv2
import draccus
import imageio
import numpy as np
import tqdm
from libero.libero import benchmark

from experiments.robot.libero.libero_utils import get_libero_env
from src.panda.perturbations import PERTURBATIONS
from src.panda.recovery import RECOVERIES
from src.panda.oft.run_libero_eval import out_dir, parse_task_ids, resolve_outputs
import src.paths as P
from src.panda.oft.run_libero_eval_perturb import load_calibration, run_perturbed_episode


@dataclass
class ReplayConfig:
    # fmt: off
    logs: str = ""                                    # JSONL file(s) to select from (glob; "" = outputs/logs/<suite>/oft/PERTURB-*.jsonl)
    # Filters (comma-separated lists; unset = any)
    run: Optional[str] = None                         # Substring of the run id (e.g. a date or a run_id_note)
    recovery: Optional[str] = None                    # e.g. "vanilla,rehome"
    family: Optional[str] = None                      # e.g. "grasp_failure,object_tip"
    timing: Optional[str] = None                      # e.g. "0.30,0.75"
    task_ids: Optional[str] = None                    # e.g. "0-4,8"
    init: Optional[str] = None                        # Initial state indices
    seed: Optional[str] = None                        # Perturbation seeds
    success: Optional[bool] = None                    # Only successful (true) / failed (false) episodes
    valid: Optional[bool] = None                      # Only valid (true) / invalid (false) episodes
    limit: Optional[int] = None                       # Replay at most this many episodes
    # Output
    out_dir: Optional[str] = None                     # Video directory (default: the eval run's outputs/videos/<suite>/oft/<run id>)
    overwrite: bool = False                           # Re-render videos that already exist
    wrist: bool = False                               # Put the wrist camera next to the third-person view
    annotate: bool = False                            # Overlay the step and the perturbation / recovery phases
    # Environment (must match the eval run)
    task_suite_name: str = "libero_10"
    calib_path: str = ""                              # "" = outputs/calibration/<suite>/oft/CALIB-<suite>.json
    model_family: str = "openvla"
    num_steps_wait: int = 10
    num_open_loop_steps: int = 8
    max_steps: int = 600
    late_steps: int = 0                               # As run_libero_eval_perturb (the recorded runs used 0)
    env_img_res: int = 256
    # fmt: on


def split(spec, cast=str):
    return None if spec is None else {cast(x) for x in str(spec).split(",") if x.strip()}


def select(cfg):
    recs = [json.loads(line) for path in sorted(glob.glob(cfg.logs)) for line in open(path)]
    recs = [r for r in recs if "actions" in r and "fixtures" in r]  # Older runs cannot be replayed
    tasks = None if cfg.task_ids is None else set(parse_task_ids(cfg.task_ids, 10 ** 6))
    rules = [
        (cfg.run, lambda r: cfg.run in r["run_id"]),
        (split(cfg.recovery), lambda r: r["recovery"] in split(cfg.recovery)),
        (split(cfg.family), lambda r: r["perturb_family"] in split(cfg.family)),
        (split(cfg.timing, float), lambda r: r["perturb_timing"] in split(cfg.timing, float)),
        (tasks, lambda r: r["task_id"] in tasks),
        (split(cfg.init, int), lambda r: r["init_state_idx"] in split(cfg.init, int)),
        (split(cfg.seed, int), lambda r: r["seed"] in split(cfg.seed, int)),
        (cfg.success, lambda r: r["success"] == cfg.success),
        (cfg.valid, lambda r: r["valid"] == cfg.valid),
    ]
    recs = [r for r in recs if all(spec is None or keep(r) for spec, keep in rules)]
    return recs[: cfg.limit] if cfg.limit else recs


class WristRecorder:
    """Env proxy that keeps the wrist image of every observation (perturbations / recoveries use env.env)."""

    def __init__(self, env):
        self._env, self.wrist = env, []

    def set_init_state(self, state):
        obs = self._env.set_init_state(state)
        self.wrist = [obs["robot0_eye_in_hand_image"][::-1, ::-1]]
        return obs

    def step(self, action):
        out = self._env.step(action)
        self.wrist.append(out[0]["robot0_eye_in_hand_image"][::-1, ::-1])
        return out

    def __getattr__(self, name):
        return getattr(self._env, name)


def phase_labels(n, perturb, recovery_info):
    """Per-frame label: frame t-1 is the observation the agent saw before step t."""
    labels = [""] * n
    if perturb.fired_step is not None:
        for t in range(perturb.fired_step, perturb.end_step + 1):
            labels[min(t, n) - 1] = "PERTURB"
    for t0 in recovery_info.get("trigger_steps", []):
        lift, joint = recovery_info.get("lift_steps"), recovery_info.get("joint_steps")
        spans = [("REHOME lift", lift), ("REHOME joint", joint)] if lift is not None else [("RECOVERY", 1)]
        t = t0
        for name, length in spans:
            for _ in range(length or 0):
                if t <= n:
                    labels[t - 1] = name
                t += 1
    return labels


@draccus.wrap()
def replay(cfg: ReplayConfig) -> None:
    resolve_outputs(cfg)
    cfg.logs = cfg.logs or os.path.join(out_dir("logs", cfg), "PERTURB-*.jsonl")
    recs = select(cfg)
    print(f"Replaying {len(recs)} episode(s)")
    suite = benchmark.get_benchmark_dict()[cfg.task_suite_name]()
    calib = load_calibration(cfg.calib_path)
    envs, mismatches = {}, 0
    for r in tqdm.tqdm(recs):
        name = (f"task={r['task_id']}--init={r['init_state_idx']}--seed={r['seed']}--valid={r['valid']}"
                f"--success={r['success']}.mp4")
        if cfg.out_dir:  # one folder for every run
            path = os.path.join(cfg.out_dir, f"{r['run_id']}--{name}")
        else:  # where the eval run would have saved it
            path = os.path.join(out_dir("videos", cfg), r["run_id"], name)
        if os.path.exists(path) and not cfg.overwrite:
            continue
        if r["task_id"] not in envs:
            envs[r["task_id"]] = get_libero_env(suite.get_task(r["task_id"]), cfg.model_family,
                                                resolution=cfg.env_img_res)
        env, task_description = envs[r["task_id"]]
        env = WristRecorder(env)
        perturb = PERTURBATIONS[r["perturb_family"]](
            env, r["target"], calib[r["task_id"]]["phases"], r["perturb_timing"], r["seed"])
        recovery = RECOVERIES[r["recovery"]](env, perturb, r["recovery_delay"])
        recovery.recorded = r["recovery_info"]  # e.g. core's imagined policy outputs, which need the model
        success_step, frames, _, _ = run_perturbed_episode(
            cfg, env, task_description, None, 224, None, None, None, None,
            suite.get_task_init_states(r["task_id"])[r["init_state_idx"]], perturb, recovery,
            replay_actions=np.load(P.find_actions(r["actions"], out_dir("actions", cfg))), replay_fixtures=r["fixtures"])

        got = {"valid": perturb.invalid is None, "invalid_reason": perturb.invalid,
               "fired_step": perturb.fired_step, "success_step": success_step}
        if any(got[k] != r[k] for k in got):
            mismatches += 1
            print(f"WARNING: replay differs from the record for {path}: "
                  f"{ {k: (r[k], got[k]) for k in got if got[k] != r[k]} }")

        if cfg.wrist or cfg.annotate:
            labels = phase_labels(len(frames), perturb, recovery.record())
            out = []
            for i, img in enumerate(frames):
                img = np.ascontiguousarray(img)
                if cfg.wrist:
                    img = np.ascontiguousarray(np.concatenate([img, env.wrist[cfg.num_steps_wait + i]], axis=1))
                if cfg.annotate:
                    cv2.putText(img, f"t={i + 1} {labels[i]}", (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (255, 255, 0), 1)
                out.append(img)
            frames = out
        os.makedirs(os.path.dirname(path), exist_ok=True)
        imageio.mimsave(path, frames, fps=30)
    print(f"Done: {len(recs)} selected, {mismatches} replay mismatch(es)")


if __name__ == "__main__":
    replay()
