"""
collect_detector_bank.py

Detection bank of core_detect (CoRe's detector, see calibrate_detector.py): clean (unperturbed) rollouts of every
initial state of the given tasks, run exactly like the evaluation episodes (run_perturbed_episode, with a perturbation
that never fires and no recovery) by `num_workers` simulator processes sharing one model, as in
run_libero_eval_perturb_parallel.py. Per episode, <bank_dir>/task<id>_init<k>.npz holds the action head's latent of
every policy query (run_libero_eval_perturb.tap_latent) and the success step (-1 for a failure).

    python src/panda/oft/collect_detector_bank.py --task_ids 0-4 --num_workers 5
"""

import multiprocessing as mp
import os
import queue
from dataclasses import dataclass

import draccus
import numpy as np
from libero.libero import benchmark

import src.panda.oft.run_libero_eval_perturb as E
from experiments.robot.libero.libero_utils import get_libero_env
from src.panda.recovery import Vanilla
from experiments.robot.robot_utils import get_action, get_image_resize_size, set_seed_everywhere


@dataclass
class BankConfig(E.PerturbConfig):
    # fmt: off
    bank_dir: str = ""                                       # Output directory ("" = outputs/detector/<suite>/oft/bank)
    num_workers: int = 5                                     # Simulator processes sharing the model
    worker_timeout: int = 900                                # s without any worker message before giving up
    # fmt: on


class NoPerturbation:
    """A perturbation that never fires."""

    fired_step, end_step, invalid, params, target = None, None, None, {}, None

    def reset(self):
        pass

    def before_step(self, t, action):
        return action

    def after_step(self, t, action):
        pass


def worker_main(cfg, wid, to_main, inbox):
    set_seed_everywhere(cfg.seed + 1 + wid)  # Each worker its own fixture-jitter stream
    latents = []

    def remote_get_action(cfg_, model, obs, task_label, **kwargs):
        to_main.put(("act", wid, obs, task_label))
        kind, actions, h = inbox.get()
        assert kind == "act"
        latents.append(h)
        return actions

    E.get_action = remote_get_action
    suite = benchmark.get_benchmark_dict()[cfg.task_suite_name]()
    resize_size = get_image_resize_size(cfg)
    envs = {}
    while True:
        msg = inbox.get()
        if msg[0] == "stop":
            return
        tid, k = msg[1]
        if tid not in envs:
            for t in list(envs):  # One environment per worker: each holds a GPU render context
                envs.pop(t)[0].close()
            envs[tid] = get_libero_env(suite.get_task(tid), cfg.model_family, resolution=cfg.env_img_res)
        env, desc = envs[tid]
        perturb = NoPerturbation()
        latents.clear()
        success_step, _, _, _ = E.run_perturbed_episode(
            cfg, env, desc, None, resize_size, None, None, None, None, suite.get_task_init_states(tid)[k], perturb,
            Vanilla(env, perturb, cfg.recovery_delay))
        to_main.put(("done", wid, (tid, k), {"task": desc, "success_step": -1 if success_step is None else success_step,
                                             "latents": np.stack(latents).astype(np.float32)}))


@draccus.wrap()
def collect(cfg: BankConfig) -> None:
    E.validate_config(cfg)
    E.resolve_outputs(cfg)
    cfg.bank_dir = cfg.bank_dir or os.path.join(E.out_dir("detector", cfg), "bank")
    set_seed_everywhere(cfg.seed)
    model, action_head, proprio_projector, noisy_action_projector, processor = E.initialize_model(cfg)
    E.tap_latent(action_head)
    suite = benchmark.get_benchmark_dict()[cfg.task_suite_name]()
    os.makedirs(cfg.bank_dir, exist_ok=True)
    jobs = [(tid, k) for tid in E.parse_task_ids(cfg.task_ids, suite.n_tasks)
            for k in range(len(suite.get_task_init_states(tid)))
            if not os.path.exists(os.path.join(cfg.bank_dir, f"task{tid}_init{k}.npz"))]
    print(f"{len(jobs)} episodes to run", flush=True)
    ctx = mp.get_context("spawn")
    to_main, inboxes = ctx.Queue(), [ctx.Queue() for _ in range(cfg.num_workers)]
    workers = [ctx.Process(target=worker_main, args=(cfg, w, to_main, inboxes[w]), daemon=True)
               for w in range(cfg.num_workers)]
    for w in workers:
        w.start()
    pending, done = list(jobs), 0
    for w in range(cfg.num_workers):
        if pending:
            inboxes[w].put(("job", pending.pop(0)))
    while done < len(jobs):
        try:
            msg = to_main.get(timeout=cfg.worker_timeout)
        except queue.Empty:
            raise RuntimeError(f"No worker message for {cfg.worker_timeout} s; alive: {[w.is_alive() for w in workers]}")
        if msg[0] == "act":
            _, wid, obs, task_label = msg
            actions = get_action(cfg, model, obs, task_label, processor=processor, action_head=action_head,
                                 proprio_projector=proprio_projector, noisy_action_projector=noisy_action_projector,
                                 use_film=cfg.use_film)
            inboxes[wid].put(("act", actions, E.LATENT["h"]))
            continue
        _, wid, (tid, k), r = msg
        np.savez(os.path.join(cfg.bank_dir, f"task{tid}_init{k}.npz"), **r)
        done += 1
        print(f"task {tid} init {k}: success_step {r['success_step']}, {len(r['latents'])} queries "
              f"({done}/{len(jobs)})", flush=True)
        if pending:
            inboxes[wid].put(("job", pending.pop(0)))
    for q in inboxes:
        q.put(("stop",))
    for w in workers:
        w.join(timeout=60)


if __name__ == "__main__":
    collect()
