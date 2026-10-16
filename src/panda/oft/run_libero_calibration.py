"""
run_libero_calibration.py

Runs clean (unperturbed) rollouts and records, for each initial state, which goal object the policy touches and lifts
first (target = first lifted) and the phase boundaries used to time perturbations (CoRe protocol): full episode,
pre-grasp phase, and carry phase.
All step values are 1-based step counts after the initial wait steps.
"""

import json
import os
from collections import Counter, deque
from dataclasses import dataclass

import draccus
import numpy as np
import tqdm
from libero.libero import benchmark

from experiments.robot.libero.libero_utils import get_libero_dummy_action, get_libero_env
from src.panda.oft.run_libero_eval import (
    GenerateConfig,
    initialize_model,
    log_message,
    parse_task_ids,
    prepare_observation,
    process_action,
    resolve_outputs,
    validate_config,
)
from experiments.robot.robot_utils import DATE_TIME, get_action, get_image_resize_size, set_seed_everywhere


MIN_CARRY_STEPS = 10  # Lift segments shorter than this are not counted as carrying


@dataclass
class CalibConfig(GenerateConfig):
    # fmt: off
    max_steps: int = 600                             # Episode step limit (excluding the initial wait steps)
    move_thresh: float = 0.005                       # Displacement (m) at which a goal object counts as moved
    lift_height: float = 0.01                        # Height (m) above its spawn at which a goal object counts as lifted
    calib_dir: str = ""                              # Calibration JSON ("" = outputs/calibration/<suite>/oft; log in local_log_dir)
    # fmt: on


def run_calib_episode(cfg, env, task_description, model, resize_size, processor, action_head, proprio_projector,
                      noisy_action_projector, initial_state):
    """Run one clean episode and record per-goal-object move step, first lift step, and lift-release segments."""
    env.reset()
    obs = env.set_init_state(initial_state)
    for _ in range(cfg.num_steps_wait):
        obs, _, _, _ = env.step(get_libero_dummy_action(cfg.model_family))

    sim, body_id = env.env.sim, env.env.obj_body_id
    goal_objects = [p[1] for p in env.env.parsed_problem["goal_state"] if len(p) > 1 and p[1] in env.env.objects_dict]
    spawn = {o: sim.data.body_xpos[body_id[o]].copy() for o in goal_objects}
    events = {o: {"move": None, "lift": None, "segments": []} for o in goal_objects}  # segments: [[lift, release]]
    down = {o: True for o in goal_objects}  # Back near spawn height since the last release (re-grasp allowed)

    action_queue = deque(maxlen=cfg.num_open_loop_steps)
    success_step = None
    for t in range(1, cfg.max_steps + 1):
        if len(action_queue) == 0:
            observation, _ = prepare_observation(obs, resize_size)
            action_queue.extend(get_action(
                cfg, model, observation, task_description, processor=processor, action_head=action_head,
                proprio_projector=proprio_projector, noisy_action_projector=noisy_action_projector, use_film=cfg.use_film,
            ))
        action = process_action(action_queue.popleft(), cfg.model_family)
        obs, _, done, _ = env.step(action.tolist())

        closed = action[-1] > 0  # +1 = close, -1 = open
        for o in goal_objects:
            pos, ev = sim.data.body_xpos[body_id[o]], events[o]
            raised, segs = pos[2] > spawn[o][2] + cfg.lift_height, events[o]["segments"]
            if ev["move"] is None and np.linalg.norm(pos - spawn[o]) > cfg.move_thresh:
                ev["move"] = t
            if segs and segs[-1][1] is None:  # Carrying: ends at the first open command
                if not closed:
                    segs[-1][1] = t
            elif closed and raised and down[o]:  # Lift-off only counts with the gripper closed
                segs.append([t, None])
                ev["lift"] = ev["lift"] or t
                down[o] = False
            if not raised:
                down[o] = True
        if done:
            success_step = t
            break

    def first(key):
        hit = [o for o in goal_objects if events[o][key] is not None]
        return min(hit, key=lambda o: events[o][key]) if hit else None

    first_moved, first_lifted = first("move"), first("lift")
    return {"success": success_step is not None, "success_step": success_step,
            "target": first_lifted or first_moved, "first_moved": first_moved, "first_lifted": first_lifted,
            "mismatch": first_lifted is not None and first_moved != first_lifted,
            "goal_objects": goal_objects, "events": events}


def compute_phases(records):
    """Phase lengths from successful rollouts (CoRe: minimum over successes)."""
    ok = [r for r in records if r["success"] and r["target"] is not None]
    # Carry = last lift segment of the target; ends at release, or at success if the task completes while holding.
    # Segments shorter than MIN_CARRY_STEPS (missed grasps, gripper-command flicker) are ignored.
    carry = []
    for r in ok:
        lens = [(end or r["success_step"]) - start for start, end in r["events"][r["target"]]["segments"]]
        if lens := [n for n in lens if n >= MIN_CARRY_STEPS]:
            carry.append(lens[-1])
    return {
        "num_success": len(ok),
        "target_counts": dict(Counter(r["target"] for r in ok)),
        "num_mismatch": sum(r["mismatch"] for r in ok),
        "full": min((r["success_step"] for r in ok), default=None),
        "pre_grasp": min((r["events"][r["target"]]["move"] for r in ok), default=None),
        "carry": min(carry, default=None),
    }


@draccus.wrap()
def calibrate_phases(cfg: CalibConfig) -> None:
    validate_config(cfg)
    resolve_outputs(cfg)
    set_seed_everywhere(cfg.seed)
    model, action_head, proprio_projector, noisy_action_projector, processor = initialize_model(cfg)
    resize_size = get_image_resize_size(cfg)
    task_suite = benchmark.get_benchmark_dict()[cfg.task_suite_name]()

    run_id = f"CALIB-{cfg.task_suite_name}-{DATE_TIME}" + (f"--{cfg.run_id_note}" if cfg.run_id_note else "")
    os.makedirs(cfg.local_log_dir, exist_ok=True)
    os.makedirs(cfg.calib_dir, exist_ok=True)
    out_path = os.path.join(cfg.calib_dir, run_id + ".json")
    log_file = open(os.path.join(cfg.local_log_dir, run_id + ".txt"), "w")

    results = {}
    for task_id in tqdm.tqdm(parse_task_ids(cfg.task_ids, task_suite.n_tasks)):
        env, task_description = get_libero_env(task_suite.get_task(task_id), cfg.model_family, resolution=cfg.env_img_res)
        initial_states = task_suite.get_task_init_states(task_id)
        records = []
        for idx in tqdm.tqdm(range(min(cfg.num_trials_per_task, len(initial_states)))):
            rec = run_calib_episode(cfg, env, task_description, model, resize_size, processor, action_head,
                                    proprio_projector, noisy_action_projector, initial_states[idx])
            records.append({"init_state_idx": idx, **rec})
            log_message(f"task {task_id} init {idx}: success={rec['success']} step={rec['success_step']} "
                        f"target={rec['target']} first_moved={rec['first_moved']} mismatch={rec['mismatch']} "
                        f"events={rec['events']}", log_file)

        phases = compute_phases(records)
        results[task_id] = {"task_description": task_description, "phases": phases, "episodes": records}
        log_message(f"task {task_id} phases: {phases}", log_file)
        with open(out_path, "w") as f:
            json.dump(results, f, indent=2)

    log_message(f"Saved phase calibration to {out_path}", log_file)
    log_file.close()


if __name__ == "__main__":
    calibrate_phases()
