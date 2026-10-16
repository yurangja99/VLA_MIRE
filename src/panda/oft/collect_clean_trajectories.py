"""
collect_clean_trajectories.py

Clean trajectories for mire (recovery.Mire): clean (unperturbed) rollouts of every initial state of the given tasks,
run exactly like the evaluation episodes (run_perturbed_episode, with a perturbation that never fires and no recovery)
by `num_workers` simulator processes sharing one model, as in collect_detector_bank.py. Per episode,
<out_dir>/task<id>_init<k>.npz holds, from the end of the wait steps (index 0) and after every step: the arm joint
angles, the grip site pose, the finger joints, the pose of every object and fixture body, the truth of every BDDL
goal predicate, and the executed actions; plus the instruction and the success step (-1 for a failure).

    python src/panda/oft/collect_clean_trajectories.py --task_ids 0-4 --num_workers 5
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
class CleanConfig(E.PerturbConfig):
    # fmt: off
    out_dir: str = ""                                        # Output directory ("" = outputs/clean/<suite>/oft)
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


class StateLog:
    """Env proxy recording the state after every step (wait steps included; the caller drops them)."""

    def __init__(self, env):
        self._env, self.rows = env, []

    def snap(self, action=None):
        e = self._env.env
        d, robot = e.sim.data, e.robots[0]
        bodies = list(e.objects_dict) + list(e.fixtures_dict)
        ids = [e.obj_body_id[b] for b in bodies]
        self.rows.append({
            "q": d.qpos[robot._ref_joint_pos_indexes].copy(),
            "fingers": d.qpos[robot._ref_gripper_joint_pos_indexes].copy(),
            "tcp": d.site_xpos[robot.eef_site_id].copy(), "tcp_R": d.site_xmat[robot.eef_site_id].reshape(3, 3).copy(),
            "body_pos": d.body_xpos[ids].copy(), "body_quat": d.body_xquat[ids].copy(),
            "goals": np.array([bool(e._eval_predicate(g)) for g in e.parsed_problem["goal_state"]]),
            "action": np.full(7, np.nan) if action is None else np.asarray(action, dtype=np.float64)})
        self.bodies = bodies

    def set_init_state(self, state):
        obs = self._env.set_init_state(state)
        self.rows = []
        self.snap()
        return obs

    def step(self, action):
        out = self._env.step(action)
        self.snap(action)
        return out

    def __getattr__(self, name):
        return getattr(self._env, name)


def worker_main(cfg, wid, to_main, inbox):
    set_seed_everywhere(cfg.seed + 1 + wid)  # Each worker its own fixture-jitter stream

    def remote_get_action(cfg_, model, obs, task_label, **kwargs):
        to_main.put(("act", wid, obs, task_label))
        kind, actions, _ = inbox.get()
        assert kind == "act"
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
        log = StateLog(env)
        perturb = NoPerturbation()
        success_step, _, _, _ = E.run_perturbed_episode(
            cfg, log, desc, None, resize_size, None, None, None, None, suite.get_task_init_states(tid)[k], perturb,
            Vanilla(log, perturb, cfg.recovery_delay))
        rows = log.rows[cfg.num_steps_wait:]  # rows[0] = after the wait steps
        out = {key: np.stack([r[key] for r in rows]) for key in rows[0]}
        out["action"] = out["action"][1:]  # action[t - 1] = the action executed at step t
        to_main.put(("done", wid, (tid, k), {"task": desc, "success_step": -1 if success_step is None else success_step,
                                             "bodies": np.array(log.bodies),
                                             "goal_state": np.array([" ".join(g) for g in
                                                                     env.env.parsed_problem["goal_state"]]), **out}))


@draccus.wrap()
def collect(cfg: CleanConfig) -> None:
    E.validate_config(cfg)
    E.resolve_outputs(cfg)
    cfg.out_dir = cfg.out_dir or E.out_dir("clean", cfg)
    set_seed_everywhere(cfg.seed)
    model, action_head, proprio_projector, noisy_action_projector, processor = E.initialize_model(cfg)
    E.tap_latent(action_head)
    suite = benchmark.get_benchmark_dict()[cfg.task_suite_name]()
    os.makedirs(cfg.out_dir, exist_ok=True)
    jobs = [(tid, k) for tid in E.parse_task_ids(cfg.task_ids, suite.n_tasks)
            for k in range(len(suite.get_task_init_states(tid)))
            if not os.path.exists(os.path.join(cfg.out_dir, f"task{tid}_init{k}.npz"))]
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
        np.savez(os.path.join(cfg.out_dir, f"task{tid}_init{k}.npz"), **r)
        done += 1
        print(f"task {tid} init {k}: success_step {r['success_step']}, {len(r['action'])} steps "
              f"({done}/{len(jobs)})", flush=True)
        if pending:
            inboxes[wid].put(("job", pending.pop(0)))
    for q in inboxes:
        q.put(("stop",))
    for w in workers:
        w.join(timeout=60)


if __name__ == "__main__":
    collect()
