"""
run_libero_eval_perturb.py

Evaluates a frozen policy on LIBERO under one CoRe-style perturbation condition (family x timing), optionally with a
recovery strategy from recovery.py (vanilla = none). The policy's per-step actions are saved so that any episode can
be replayed exactly (and rendered) later without the model; see replay_libero.py.
Per task, initial states k = 0, 1, ... are run with perturbation seed k (skipping states that failed in the clean
calibration); invalid episodes are discarded until `num_valid_per_task` valid ones are collected. If a full pass over
the states is not enough, another pass starts with seed k + seed_stride, then k + 2 * seed_stride, and so on.
"""

import json
import os
from collections import deque
from dataclasses import dataclass

import draccus
import imageio
import numpy as np
import tqdm
from libero.libero import benchmark

from experiments.robot.libero.libero_utils import get_libero_dummy_action, get_libero_env
from src.panda.perturbations import PERTURBATIONS
from src.panda.recovery import RECOVERIES
from src.panda.oft.run_libero_eval import (
    GenerateConfig,
    initialize_model,
    log_message,
    out_dir,
    parse_task_ids,
    prepare_observation,
    process_action,
    resolve_outputs,
    validate_config,
)
from experiments.robot.robot_utils import DATE_TIME, get_action, get_image_resize_size, set_seed_everywhere

import src.paths as P


@dataclass
class PerturbConfig(GenerateConfig):
    # fmt: off
    perturb_family: str = "object_displace"          # arm_collision | grasp_failure | object_displace | object_tip
    perturb_timing: float = 0.55                     # Fraction of the calibrated phase: 0.30 | 0.55 | 0.75
    calib_path: str = ""                             # Phase calibration ("" = outputs/calibration/<suite>/oft/CALIB-<suite>.json)
    num_valid_per_task: int = 20                     # Valid perturbed episodes to collect per task (CoRe: 20)
    seed_stride: int = 50                            # Seed offset added for each additional pass over the states
    max_steps: int = 600                             # Episode step limit (excluding the initial wait steps)
    late_steps: int = 0                              # An episode not successful by max_steps runs this many more steps,
    #                                                  only to record a late success (success still means by max_steps)
    recovery: str = "vanilla"                        # vanilla | replan | rehome | retreat | safeloop | rewind_il | volo | core | core_detect
    recovery_delay: int = 8                          # Steps after the injection ends before the recovery triggers
    actions_dir: str = ""                            # Per-episode policy actions, <actions_dir>/<run_id>/task*_init*_seed*.npy
    #                                                  ("" = outputs/actions/<suite>/oft)
    # fmt: on


LATENT = {}  # "h": the action head's latent of the latest policy query (tap_latent; set by the workers when parallel)


def tap_latent(action_head):
    """Keep the action head's last hidden state before its output layer, mean-pooled over the chunk, at every policy
    query (CoRe's detection feature: the action head's latent before motor decoding)."""

    def hook(module, inputs):
        LATENT["h"] = inputs[0].float().mean(dim=1)[0].cpu().numpy()

    return action_head.model.fc2.register_forward_pre_hook(hook)


def load_calibration(calib_path):
    with open(calib_path, "r") as f:
        return {int(k): v for k, v in json.load(f).items()}


def fixture_pose(env, pose=None):
    """Get (or, with `pose`, set) the model pose of every fixture. LIBERO re-samples fixture placements on each reset
    (a few cm), independently of the initial state and not controlled by env.seed, so replays restore them."""
    m, ids = env.env.sim.model._model, {f: env.env.obj_body_id[f] for f in env.env.fixtures_dict}
    if pose is None:
        return {f: m.body_pos[i].tolist() + m.body_quat[i].tolist() for f, i in ids.items()}
    for f, i in ids.items():
        m.body_pos[i], m.body_quat[i] = pose[f][:3], pose[f][3:]


def run_perturbed_episode(cfg, env, task_description, model, resize_size, processor, action_head, proprio_projector,
                          noisy_action_projector, initial_state, perturb, recovery, replay_actions=None,
                          replay_fixtures=None):
    """Run one episode with the perturbation and the recovery strategy stepped alongside the policy. Returns the
    success step, the frames, and the policy's action at every step (env action space; NaN where the recovery
    overrode it), and the fixture pose. With `replay_actions` (such an array) and `replay_fixtures`, those actions
    stand in for the policy, which is not queried, so the episode is reproduced exactly."""
    env.reset()
    # robosuite ends an episode at its horizon (LIBERO: 1000 steps, the wait steps included): room for the late steps
    env.env.horizon = max(env.env.horizon, cfg.num_steps_wait + cfg.max_steps + cfg.late_steps)
    if replay_fixtures is not None:
        fixture_pose(env, replay_fixtures)
    fixtures = fixture_pose(env)
    env.env.sim.data._data.qacc_warmstart[:] = 0  # Left over from the reset's settling; zero it for exact replays
    obs = env.set_init_state(initial_state)
    for _ in range(cfg.num_steps_wait):
        obs, _, _, _ = env.step(get_libero_dummy_action(cfg.model_family))
    perturb.reset()
    recovery.process_action = lambda raw: process_action(raw, cfg.model_family)
    recovery.task, recovery.latent = task_description, lambda: LATENT.get("h")
    recovery.clean_dir, recovery.detector_dir = out_dir("clean", cfg), out_dir("detector", cfg)  # This policy's data
    if replay_actions is None:  # policy queries on observations the recovery builds itself (core's imagined rollout)
        recovery.query = lambda raw_obs: get_action(
            cfg, model, prepare_observation(raw_obs, resize_size)[0], task_description, processor=processor,
            action_head=action_head, proprio_projector=proprio_projector,
            noisy_action_projector=noisy_action_projector, use_film=cfg.use_film)
    recovery.reset(obs)

    action_queue = deque(maxlen=cfg.num_open_loop_steps)
    success_step, replay_images, policy_actions = None, [], []
    for t in range(1, cfg.max_steps + cfg.late_steps + 1):
        observation, img = prepare_observation(obs, resize_size)
        replay_images.append(img)
        recovery.before_step(t, obs)
        if recovery.flush:
            action_queue.clear()
            action_queue.extend(recovery.handoff)  # e.g. the unexecuted rest of core's imagined chunk
            recovery.handoff, recovery.flush = [], False
        action = recovery.override(t, obs)
        if action is None and replay_actions is None and len(action_queue) == 0:
            action_queue.extend(get_action(
                cfg, model, observation, recovery.instruction(t, task_description), processor=processor,
                action_head=action_head, proprio_projector=proprio_projector,
                noisy_action_projector=noisy_action_projector, use_film=cfg.use_film,
            ))
            if recovery.detect(t, obs):  # an online detector fired on this query (core_detect): recover instead
                action = recovery.override(t, obs)
        if action is not None:
            policy_actions.append(np.full(7, np.nan))
        elif replay_actions is not None:
            if t > len(replay_actions) or np.isnan(replay_actions[t - 1]).any():
                break  # The replay diverged from the recorded episode
            action = replay_actions[t - 1].copy()
            policy_actions.append(action.copy())
        else:
            action = process_action(action_queue.popleft(), cfg.model_family)
            policy_actions.append(np.array(action, dtype=np.float64))  # float64: replay must be bit-exact
        action = perturb.before_step(t, action)
        if perturb.invalid:
            break
        obs, _, done, _ = env.step(action.tolist())
        perturb.after_step(t, action)
        recovery.observe(t, obs, action)
        if perturb.invalid:
            break
        if done:
            success_step = t
            break

    if perturb.invalid is None and perturb.fired_step is None:
        perturb.invalid = "not_fired"
    return success_step, replay_images, np.stack(policy_actions) if policy_actions else np.zeros((0, 7)), fixtures


@draccus.wrap()
def eval_perturbed(cfg: PerturbConfig) -> None:
    validate_config(cfg)
    resolve_outputs(cfg)
    assert cfg.perturb_family in PERTURBATIONS, f"Invalid perturb_family: {cfg.perturb_family}"
    assert cfg.recovery in RECOVERIES, f"Invalid recovery: {cfg.recovery}"
    set_seed_everywhere(cfg.seed)
    model, action_head, proprio_projector, noisy_action_projector, processor = initialize_model(cfg)
    tap_latent(action_head)
    resize_size = get_image_resize_size(cfg)
    task_suite = benchmark.get_benchmark_dict()[cfg.task_suite_name]()
    calib = load_calibration(cfg.calib_path)

    run_id = f"PERTURB-{cfg.recovery}-{cfg.perturb_family}-{cfg.perturb_timing:.2f}-{DATE_TIME}"
    run_id += f"--{cfg.run_id_note}" if cfg.run_id_note else ""
    os.makedirs(cfg.local_log_dir, exist_ok=True)
    log_file = open(os.path.join(cfg.local_log_dir, run_id + ".txt"), "w")
    jsonl = open(os.path.join(cfg.local_log_dir, run_id + ".jsonl"), "w")
    video_dir = os.path.join(cfg.video_dir, run_id)
    actions_dir = os.path.join(cfg.actions_dir, run_id)
    os.makedirs(actions_dir, exist_ok=True)

    total_valid, total_success = 0, 0
    for task_id in tqdm.tqdm(parse_task_ids(cfg.task_ids, task_suite.n_tasks)):
        env, task_description = get_libero_env(task_suite.get_task(task_id), cfg.model_family, resolution=cfg.env_img_res)
        initial_states = task_suite.get_task_init_states(task_id)
        task_calib = calib[task_id]
        valid, success, n_pass = 0, 0, 0
        while valid < cfg.num_valid_per_task:
            new_valid = 0
            for ep in task_calib["episodes"]:
                if valid >= cfg.num_valid_per_task:
                    break
                if not ep["success"]:  # Skip states the policy already fails without perturbation
                    continue
                k, seed = ep["init_state_idx"], ep["init_state_idx"] + n_pass * cfg.seed_stride
                perturb = PERTURBATIONS[cfg.perturb_family](
                    env, ep["target"], task_calib["phases"], cfg.perturb_timing, seed
                )
                recovery = RECOVERIES[cfg.recovery](env, perturb, cfg.recovery_delay)
                success_step, replay_images, policy_actions, fixtures = run_perturbed_episode(
                    cfg, env, task_description, model, resize_size, processor, action_head, proprio_projector,
                    noisy_action_projector, initial_states[k], perturb, recovery,
                )
                is_valid, is_success = perturb.invalid is None, success_step is not None and success_step <= cfg.max_steps
                actions_path = os.path.join(actions_dir, f"task{task_id}_init{k}_seed{seed}.npy")
                np.save(actions_path, policy_actions)
                valid, new_valid, success = valid + is_valid, new_valid + is_valid, success + (is_valid and is_success)
                rec = {"run_id": run_id, "perturb_family": cfg.perturb_family, "perturb_timing": cfg.perturb_timing,
                       "recovery_delay": cfg.recovery_delay, "actions": P.rel(actions_path), "fixtures": fixtures,
                       "task_id": task_id, "init_state_idx": k, "seed": seed, "pass": n_pass, "target": ep["target"],
                       "valid": is_valid, "invalid_reason": perturb.invalid, "fired_step": perturb.fired_step,
                       "params": perturb.params, "recovery": cfg.recovery, "recovery_info": recovery.record(),
                       "success": is_success, "success_step": success_step if is_success else None,
                       "late_success_step": None if is_success else success_step}
                jsonl.write(json.dumps(rec) + "\n")
                jsonl.flush()
                log_message(f"task {task_id} init {k} seed {seed}: valid={is_valid} ({perturb.invalid}) "
                            f"fired={perturb.fired_step} success={is_success} | valid {valid}/{cfg.num_valid_per_task}",
                            log_file)
                if cfg.save_videos == "all" or cfg.save_videos == ("success" if is_success else "failure"):
                    os.makedirs(video_dir, exist_ok=True)
                    path = f"{video_dir}/task={task_id}--init={k}--seed={seed}--valid={is_valid}--success={is_success}.mp4"
                    imageio.mimsave(path, replay_images, fps=30)
            if new_valid == 0:  # Only seed-independent invalids are left: another pass cannot help
                log_message(f"task {task_id}: no valid episode in pass {n_pass}, stopping at {valid} valid", log_file)
                break
            n_pass += 1

        total_valid, total_success = total_valid + valid, total_success + success
        log_message(f"task {task_id}: success {success}/{valid} valid", log_file)

    log_message(f"Total: success {total_success}/{total_valid} valid "
                f"({100 * total_success / max(total_valid, 1):.1f}%)", log_file)
    log_file.close()
    jsonl.close()


if __name__ == "__main__":
    eval_perturbed()
