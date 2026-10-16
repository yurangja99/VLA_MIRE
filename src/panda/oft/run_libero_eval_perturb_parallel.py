"""
run_libero_eval_perturb_parallel.py

Same evaluation as run_libero_eval_perturb.py, with `num_workers` simulator processes sharing one model on the GPU.
The episodes, the per-episode code (run_perturbed_episode) and the logs are those of the sequential script: the main
process holds the policy and answers the workers' action queries one at a time (the same get_action call, batch size
1; with the action head's latent of the query, see tap_latent), and schedules the episodes; per task, results are
logged in the sequential order and the valid-episode selection (the first num_valid_per_task valid episodes; further
passes with seed + seed_stride; stop after a pass without a valid episode) reproduces the sequential loop. Episodes
run speculatively beyond that are discarded. Videos are not supported. The only difference is the stream of LIBERO's
fixture placement jitter (global numpy RNG, consumed at every reset; recorded per episode for replay), which each
worker draws from its own seed.

    python src/panda/oft/run_libero_eval_perturb_parallel.py --num_workers 5 [run_libero_eval_perturb args]
"""

import json
import multiprocessing as mp
import os
import queue
from dataclasses import dataclass

import draccus
import numpy as np
from libero.libero import benchmark

import src.panda.oft.run_libero_eval_perturb as E
from experiments.robot.libero.libero_utils import get_libero_env
from src.panda.perturbations import PERTURBATIONS
from src.panda.recovery import RECOVERIES
from experiments.robot.robot_utils import DATE_TIME, get_action, get_image_resize_size, set_seed_everywhere


@dataclass
class ParallelConfig(E.PerturbConfig):
    # fmt: off
    num_workers: int = 5                             # Simulator processes sharing the model
    worker_timeout: int = 900                        # s without any worker message before giving up
    # fmt: on


def worker_main(cfg, wid, to_main, inbox):
    """Run the episodes sent by the main process; action queries go to the main process."""
    set_seed_everywhere(cfg.seed + 1 + wid)  # Each worker its own fixture-jitter stream

    def remote_get_action(cfg_, model, obs, task_label, **kwargs):
        to_main.put(("act", wid, obs, task_label))
        kind, actions, E.LATENT["h"] = inbox.get()
        assert kind == "act"
        return actions

    E.get_action = remote_get_action  # run_perturbed_episode looks get_action up in its module at call time
    suite = benchmark.get_benchmark_dict()[cfg.task_suite_name]()
    calib = E.load_calibration(cfg.calib_path)
    resize_size = get_image_resize_size(cfg)
    envs = {}
    while True:
        msg = inbox.get()
        if msg[0] == "stop":
            return
        job = msg[1]
        tid = job["task_id"]
        if tid not in envs:
            for t in list(envs):  # One environment per worker: each holds a GPU render context
                envs.pop(t)[0].close()
            envs[tid] = get_libero_env(suite.get_task(tid), cfg.model_family, resolution=cfg.env_img_res)
        env, desc = envs[tid]
        perturb = PERTURBATIONS[cfg.perturb_family](env, job["target"], calib[tid]["phases"], cfg.perturb_timing,
                                                    job["seed"])
        recovery = RECOVERIES[cfg.recovery](env, perturb, cfg.recovery_delay)
        success_step, _, policy_actions, fixtures = E.run_perturbed_episode(
            cfg, env, desc, None, resize_size, None, None, None, None,
            suite.get_task_init_states(tid)[job["k"]], perturb, recovery)
        to_main.put(("done", wid, job, {
            "success_step": success_step, "actions": policy_actions, "fixtures": fixtures, "invalid": perturb.invalid,
            "fired_step": perturb.fired_step, "params": perturb.params, "recovery_info": recovery.record()}))


class TaskSchedule:
    """Candidate episodes of one task in the sequential order (pass n_pass over the calibration-successful states,
    seed = k + n_pass * seed_stride) and the sequential stopping rule applied to whatever results are in."""

    def __init__(self, task_id, task_calib, cfg):
        self.task_id, self.cfg = task_id, cfg
        self.states = [ep for ep in task_calib["episodes"] if ep["success"]]
        self.results, self.next_idx, self.in_flight = {}, 0, 0
        self.final, self.stop_pass = (-1, 0) if not self.states else (None, None)  # final = last logged index

    def candidate(self, i):
        n_pass, ep = divmod(i, len(self.states))[0], self.states[i % len(self.states)]
        k = ep["init_state_idx"]
        return {"task_id": self.task_id, "idx": i, "k": k, "seed": k + n_pass * self.cfg.seed_stride,
                "pass": n_pass, "target": ep["target"]}

    def walk(self):
        """Sequential loop over the known prefix: (valid count, prefix length); sets self.final when it stops."""
        n, P, valid, new_valid, i = self.cfg.num_valid_per_task, len(self.states), 0, 0, 0
        while i in self.results:
            if i % P == 0 and i > 0:
                if new_valid == 0:  # A full pass without a valid episode: the sequential loop stops
                    self.final, self.stop_pass = i - 1, i // P - 1
                    return valid, i
                new_valid = 0
            v = self.results[i]["invalid"] is None
            valid, new_valid = valid + v, new_valid + v
            if valid >= n:
                self.final = i
                return valid, i + 1
            i += 1
        if i > 0 and i % P == 0 and new_valid == 0 and valid < n and i == self.next_idx and self.in_flight == 0:
            self.final, self.stop_pass = i - 1, i // P - 1  # Whole pass done without a valid one, nothing pending
        return valid, i

    def job(self):
        """Next candidate to run, or None if the known and pending results may already be enough."""
        if self.final is not None:
            return None
        valid, prefix = self.walk()
        if self.final is not None:
            return None
        beyond = sum(r["invalid"] is None for i, r in self.results.items() if i >= prefix)
        if valid + beyond + self.in_flight >= self.cfg.num_valid_per_task:
            return None
        self.next_idx, self.in_flight = self.next_idx + 1, self.in_flight + 1
        return self.candidate(self.next_idx - 1)

    def done(self, idx, result):
        self.in_flight -= 1
        if self.final is None:
            self.results[idx] = result
            self.walk()


@draccus.wrap()
def eval_parallel(cfg: ParallelConfig) -> None:
    E.validate_config(cfg)
    E.resolve_outputs(cfg)  # Before the workers get cfg
    assert cfg.perturb_family in PERTURBATIONS, f"Invalid perturb_family: {cfg.perturb_family}"
    assert cfg.recovery in RECOVERIES, f"Invalid recovery: {cfg.recovery}"
    assert cfg.save_videos == "none", "The parallel runner does not save videos"
    set_seed_everywhere(cfg.seed)
    model, action_head, proprio_projector, noisy_action_projector, processor = E.initialize_model(cfg)
    E.tap_latent(action_head)
    task_suite = benchmark.get_benchmark_dict()[cfg.task_suite_name]()
    calib = E.load_calibration(cfg.calib_path)

    run_id = f"PERTURB-{cfg.recovery}-{cfg.perturb_family}-{cfg.perturb_timing:.2f}-{DATE_TIME}"
    run_id += f"--{cfg.run_id_note}" if cfg.run_id_note else ""
    os.makedirs(cfg.local_log_dir, exist_ok=True)
    log_file = open(os.path.join(cfg.local_log_dir, run_id + ".txt"), "w")
    jsonl = open(os.path.join(cfg.local_log_dir, run_id + ".jsonl"), "w")
    actions_dir = os.path.join(cfg.actions_dir, run_id)
    os.makedirs(actions_dir, exist_ok=True)

    tasks = [TaskSchedule(t, calib[t], cfg) for t in E.parse_task_ids(cfg.task_ids, task_suite.n_tasks)]
    ctx = mp.get_context("spawn")
    to_main, inboxes = ctx.Queue(), [ctx.Queue() for _ in range(cfg.num_workers)]
    workers = [ctx.Process(target=worker_main, args=(cfg, w, to_main, inboxes[w]), daemon=True)
               for w in range(cfg.num_workers)]
    for w in workers:
        w.start()

    def dispatch(wid):
        for ts in tasks:
            job = ts.job()
            if job is not None:
                inboxes[wid].put(("job", job))
                return True
        return False

    idle = [w for w in range(cfg.num_workers) if not dispatch(w)]
    written, total_valid, total_success = 0, 0, 0
    while written < len(tasks):
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
        _, wid, job, result = msg
        tasks[[ts.task_id for ts in tasks].index(job["task_id"])].done(job["idx"], result)
        for w in [wid] + idle:  # Refill: the finished worker and any idle ones
            if dispatch(w):
                idle = [x for x in idle if x != w]
            elif w not in idle:
                idle.append(w)
        while written < len(tasks) and tasks[written].final is not None:  # Log finished tasks in order
            ts = tasks[written]
            valid, success = 0, 0
            for i in range(ts.final + 1):
                r, c = ts.results[i], ts.candidate(i)
                is_valid = r["invalid"] is None
                is_success = r["success_step"] is not None and r["success_step"] <= cfg.max_steps
                actions_path = os.path.join(actions_dir, f"task{ts.task_id}_init{c['k']}_seed{c['seed']}.npy")
                np.save(actions_path, r["actions"])
                valid, success = valid + is_valid, success + (is_valid and is_success)
                rec = {"run_id": run_id, "perturb_family": cfg.perturb_family, "perturb_timing": cfg.perturb_timing,
                       "recovery_delay": cfg.recovery_delay, "actions": E.P.rel(actions_path), "fixtures": r["fixtures"],
                       "task_id": ts.task_id, "init_state_idx": c["k"], "seed": c["seed"], "pass": c["pass"],
                       "target": c["target"], "valid": is_valid, "invalid_reason": r["invalid"],
                       "fired_step": r["fired_step"], "params": r["params"], "recovery": cfg.recovery,
                       "recovery_info": r["recovery_info"], "success": is_success,
                       "success_step": r["success_step"] if is_success else None,
                       "late_success_step": None if is_success else r["success_step"]}
                jsonl.write(json.dumps(rec) + "\n")
                jsonl.flush()
                E.log_message(f"task {ts.task_id} init {c['k']} seed {c['seed']}: valid={is_valid} ({r['invalid']}) "
                              f"fired={r['fired_step']} success={is_success} | valid {valid}/{cfg.num_valid_per_task}",
                              log_file)
            if ts.stop_pass is not None:
                E.log_message(f"task {ts.task_id}: no valid episode in pass {ts.stop_pass}, stopping at {valid} valid",
                              log_file)
            total_valid, total_success = total_valid + valid, total_success + success
            E.log_message(f"task {ts.task_id}: success {success}/{valid} valid", log_file)
            written += 1

    for q in inboxes:
        q.put(("stop",))
    E.log_message(f"Total: success {total_success}/{total_valid} valid "
                  f"({100 * total_success / max(total_valid, 1):.1f}%)", log_file)
    log_file.close()
    jsonl.close()
    for w in workers:
        w.join(timeout=60)


if __name__ == "__main__":
    eval_parallel()
