"""
make_qualitative.py

Qualitative videos for README.md (main episodes) and RESULTS.md section 2 (main and sub episodes), re-rendered from
the recorded runs without the model (as replay_libero.py). Episodes are paired (same perturbation, timing, initial state
and seed for every method) and chosen per scenario from the outcomes (PER_FAMILY episodes per perturbation family, one
of which is the README's main episode):

    mire_only   mire succeeds and every other method fails (RESULTS: all ten methods tiled; README: README_GRID)
    faster      core and mire both succeed and mire finishes first (as mire_only)
    regrasp     mire retakes the target with the policy's own grasp (directly or after setting it down) and succeeds,
                while volo or core fails (volo, core and mire, with the wrist camera)
    handoff     mire succeeds after handing back on agreement with the shadow policy (mire, with the wrist camera)
    limits      mire fails (mire, with the wrist camera)

The episodes shown are listed in --selection (default <out_dir>/selection.json, kept in git with the videos), per
scenario as "main" (README, one per perturbation family) and "sub" (the others, RESULTS only): edit it to swap an
episode. Without that file (or with --reselect true), they are chosen again by balance(), which spreads tasks, timings
and, for the limits, failure stages evenly, and pick_main(), and the file is written.

Every tile is framed while the perturbation (magenta) or the recovery (orange) acts, not while the VLA does, and in
green once the episode has succeeded; it ends with its outcome: the success step, or "timeout" for a failure (the step
limit), its failure stage (summarize_perturb.stage) in the limits scenario. The galleries list every method's stage.
With the wrist camera, a timeline of who acts (gray = VLA) runs under it.
Writes <out_dir>/<readme|results>/<scenario>/<family>_<timing>_task<T>_init<K>_seed<S>.mp4 with a looping preview
(.gif at the width shown, preview_gif): readme/ holds the main episodes in the README's format, results/ every episode
in the RESULTS format (a main episode whose two formats are the same is rendered once and copied). README.md and
RESULTS.md show the previews, each linked to its video. The gallery of each scenario goes between the
QUALITATIVE:<tag>:<scenario> markers of RESULTS.md, its main episodes between the QUALITATIVE_MAIN:<tag>:<scenario>
markers of README.md (the titles and text around the markers are written by hand).

    python src/panda/oft/make_qualitative.py [--scenarios faster,limits] [--num_workers 8]
"""

import concurrent.futures as cf
import functools
import json
import multiprocessing as mp
import os
import shutil
import subprocess
from collections import defaultdict
from dataclasses import dataclass
from typing import Optional

import cv2
import draccus
import imageio
import imageio_ffmpeg
import numpy as np
from matplotlib import font_manager
from PIL import Image, ImageDraw, ImageFont

import src.paths as P
import src.summarize_perturb as S
from src.panda.oft.replay_libero import ReplayConfig, WristRecorder, split
from src.panda.oft.run_libero_eval import out_dir, resolve_outputs

GRID_TILE, FOCUS_TILE, BORDER, TIMELINE_H = 192, 256, 5, 14  # px
FRAME_STEP, FPS, HOLD = 2, 20, 30  # Every 2nd step at 20 fps = twice real time (20 Hz control); frames held at the end
BORDER_COLORS = {"perturb": (220, 40, 200), "recovery": (255, 140, 0), "success": (40, 160, 60)}  # RGB (VLA: none)
TIMELINE_COLORS = {"vla": (160, 160, 160), "perturb": (220, 40, 200), "recovery": (255, 140, 0)}
FONT = font_manager.findfont(font_manager.FontProperties(family="DejaVu Sans", weight="bold"))  # Ships with matplotlib
LABEL_PX, SMALL_PX, TITLE_PX = 13, 12, 13  # Font sizes: method name, step and banner, title bar
STAGE_SHORT = ["no intervention", "recovery infeasible", "recovery exec. failed", "policy failed after recovery"]
GRASP_KIND = {"clean": "own grasp", "near": "own grasp", "set_down": "set down and retaken with the own grasp",
              "foreign": "GraspGen grasp"}
HANDOFF = {"agree": "on agreement", "pre_goal": "40 steps before the goal", "goal_unreachable": "goal pose unreachable",
           "missed_grasp": "grasp missed"}


@dataclass
class QualitativeConfig(ReplayConfig):
    # fmt: off
    tag: str = "nv10"                                 # Runs to choose from (as summarize_perturb.py)
    num_splits: int = 2                               # Task splits per condition (as summarize_perturb.py)
    scenarios: Optional[str] = None                   # Comma-separated subset of the scenarios (unset = all)
    selection: str = ""                               # Episodes shown per scenario ("" = <out_dir>/selection.json)
    reselect: bool = False                            # Choose the episodes again (balance) and overwrite --selection
    results: str = os.path.join(P.ROOT, "RESULTS.md") # Galleries between the QUALITATIVE:<tag>:<scenario> markers
    readme: str = os.path.join(P.ROOT, "README.md")   # Main episodes between the QUALITATIVE_MAIN:<tag>:<scenario> markers
    num_workers: int = 8                              # Videos rendered in parallel
    # fmt: on


def paired(res):
    """{(family, timing, task, init, seed): {method: record}} over the episodes valid for every method."""
    eps = defaultdict(dict)
    for (m, f, t), recs in res.items():
        for r in recs:
            if r["valid"]:
                eps[(f, t, r["task_id"], r["init_state_idx"], r["seed"])][m] = r
    return {k: v for k, v in eps.items() if len(v) == len(S.RECOVERIES)}


PER_FAMILY = {"mire_only": 5, "faster": 5, "regrasp": 5, "handoff": 5, "limits": 3}  # Episodes per perturbation family
MIN_LEAD = 50  # Steps (2.5 s) by which mire must finish first in the faster scenario
MAX_BEHIND = 30  # Steps (1.5 s) by which volo or core may finish before mire in the regrasp scenario
MAIN_PER_FAMILY = 1  # Of those, the README's main episodes
SHOWN = {"mire_only": S.RECOVERIES, "faster": S.RECOVERIES, "regrasp": ["volo", "core", "mire"], "handoff": ["mire"],
         "limits": ["mire"]}  # Methods in a RESULTS video
README_GRID = ["vanilla", "safeloop", "volo", "core", "mire"]  # No recovery, best rewind, regrasp tool, restoration, ours
README_SHOWN = dict(SHOWN, mire_only=README_GRID, faster=README_GRID)  # Methods in a README video
GRID = ("mire_only", "faster")  # Third-person views tiled; the other scenarios with the wrist camera and a timeline
README_COLUMNS = {"handoff": 2, "limits": 4}  # README videos per table row (the other scenarios: stacked, one per row)
GIF_FPS, GIF_COLORS = 8, 64  # The looping previews shown in README.md and RESULTS.md (a click opens the video)


def candidates(full):
    """{scenario: {key: quality in [0, 1]}} (how clearly the episode shows the scenario), and each scenario's note."""
    others = S.RECOVERIES[:-1]
    ok = lambda v, m: v[m]["success"]
    info = lambda v, m: v[m]["recovery_info"]
    early = lambda v: 1 - v["mire"]["success_step"] / 600
    cands, notes = {}, {}

    # Failures of the others at as many different stages as possible
    alone = {k: v for k, v in full.items() if ok(v, "mire") and not any(ok(v, b) for b in others)}
    cands["mire_only"] = {k: 0.8 * len({S.stage(v[b]) for b in others}) / 4 + 0.2 * early(v) for k, v in alone.items()}
    notes["mire_only"] = f"mire alone succeeds in {len(alone)} of the {len(full)} paired episodes."

    # mire at least MIN_LEAD steps ahead of the fastest other method that succeeds (the more the better)
    both = {k: v for k, v in full.items() if ok(v, "mire") and ok(v, "core")}
    lead = lambda v: min(v[b]["success_step"] for b in others if ok(v, b)) - v["mire"]["success_step"]
    cands["faster"] = {k: min(lead(v), 300) / 300 for k, v in both.items() if lead(v) >= MIN_LEAD}
    ahead = [v["core"]["success_step"] - v["mire"]["success_step"] for v in both.values()]
    notes["faster"] = (f"core and mire both succeed in {len(both)} paired episodes; mire finishes first in "
                       f"{sum(d > 0 for d in ahead)} (median lead {np.median(ahead):.0f} steps).")

    # mire retakes the target (with the policy's own grasp, directly or after setting it down) and succeeds, while volo
    # (which grasps it with a GraspGen grasp) or core (which picks it to put it back) fails; whichever of the two
    # succeeds is not more than MAX_BEHIND steps ahead of mire. Preferred: set down and retaken, volo grasping
    behind = lambda v: max([v["mire"]["success_step"] - v[m]["success_step"] for m in ("volo", "core") if ok(v, m)] + [0])
    regrasp = {k: v for k, v in full.items() if ok(v, "mire") and (info(v, "mire").get("grasps") or [None])[0] in
               ("clean", "near", "set_down") and not (ok(v, "volo") and ok(v, "core")) and behind(v) <= MAX_BEHIND}
    cands["regrasp"] = {k: 0.4 * ((info(v, "mire")["grasps"][0]) == "set_down")
                        + 0.3 * bool(info(v, "volo").get("tool") == "grasp" and info(v, "volo").get("grasped"))
                        + 0.3 * (1 - behind(v) / (MAX_BEHIND + 1)) for k, v in regrasp.items()}
    notes["regrasp"] = (f"mire retakes the target with the policy's own grasp and succeeds while volo or core fails, "
                        f"and neither finishes more than {MAX_BEHIND} steps ahead of mire, in {len(regrasp)} paired "
                        f"episodes; mire sets the target down before retaking it in "
                        f"{sum(info(v, 'mire')['grasps'][0] == 'set_down' for v in regrasp.values())} of them.")

    # A typical intervention length (near the family's median)
    agree = {k: v for k, v in full.items() if ok(v, "mire") and info(v, "mire").get("handoff") == "agree"}
    med = {f: np.median([info(v, "mire")["intervention_steps"] for k, v in agree.items() if k[0] == f])
           for f, _ in S.FAMILIES}
    cands["handoff"] = {k: max(0, 1 - abs(info(v, "mire")["intervention_steps"] - med[k[0]]) / med[k[0]])
                        for k, v in agree.items()}
    acted = [v for v in full.values() if info(v, "mire").get("handoff")]
    notes["handoff"] = (f"mire hands back on agreement in {sum(info(v, 'mire')['handoff'] == 'agree' for v in acted)} of "
                        f"the {len(acted)} episodes it intervenes in.")

    fails = {k: v for k, v in full.items() if not ok(v, "mire")}
    cands["limits"] = {k: 0.0 for k in fails}  # Spread over the failure stages by balance()
    notes["limits"] = (f"mire fails in {len(fails)} paired episodes: " + ", ".join(
        f"{name} {sum(S.stage(v['mire']) == s for v in fails.values())}" for s, name in enumerate(STAGE_SHORT)) + ".")
    return cands, notes


def balance(cands, full, iters=40000, seed=0):
    """{scenario: [key]}: PER_FAMILY[scenario] episodes per scenario and family, by local search from a greedy start
    over: quality, distinct tasks within a scenario and family, timings and tasks spread evenly within each scenario and
    family, scenario and overall, failure stages spread over the limits (within a family and overall), and no episode
    shown twice."""
    cells = {(sc, f): [k for k in c if k[0] == f] for sc, c in cands.items() for f, _ in S.FAMILIES}
    cells = {cell: ks for cell, ks in cells.items() if ks}
    want = {cell: min(PER_FAMILY[cell[0]], len(ks)) for cell, ks in cells.items()}
    spread = lambda xs, cats: sum((xs.count(c) - len(xs) / len(cats)) ** 2 for c in cats)
    tasks, timings = range(10), S.TIMINGS
    stage = lambda k: S.stage(full[k]["mire"])

    def cost(sel):
        c, picked = 0.0, [k for ks in sel.values() for k in ks]
        for (sc, f), ks in sel.items():
            c -= sum(cands[sc][k] for k in ks)
            c += 4 * (len(ks) - len({k[2] for k in ks}))  # A task twice within a scenario and family
            c += spread([k[1] for k in ks], timings)
            if sc == "limits":
                c += 4 * (len(ks) - len({stage(k) for k in ks}))
        for sc in cands:
            ks = [k for (s, _), v in sel.items() if s == sc for k in v]
            c += 0.5 * spread([k[2] for k in ks], tasks) + 0.3 * spread([k[1] for k in ks], timings)
        c += 0.1 * spread([k[2] for k in picked], tasks) + 0.05 * spread([k[1] for k in picked], timings)
        c += 0.5 * spread([stage(k) for (s, _), v in sel.items() if s == "limits" for k in v], range(4))
        c += 4 * (len(picked) - len(set(picked)))  # One episode in two scenarios
        return c

    sel = {}
    for cell, ks in cells.items():  # Greedy start: the best episode of each task not used yet
        order, out = sorted(ks, key=lambda k: -cands[cell[0]][k]), []
        for k in order:
            if len(out) < want[cell] and k[2] not in {o[2] for o in out}:
                out.append(k)
        out += [k for k in order if k not in out][: want[cell] - len(out)]
        sel[cell] = out
    rng, best = np.random.default_rng(seed), cost(sel)
    for _ in range(iters):  # Swap one episode for another of the same scenario and family when that lowers the cost
        cell = list(sel)[rng.integers(len(sel))]
        free = [k for k in cells[cell] if k not in sel[cell]]
        if not free:
            continue
        i, k = rng.integers(len(sel[cell])), free[rng.integers(len(free))]
        old, sel[cell][i] = sel[cell][i], k
        c = cost(sel)
        if c < best:
            best = c
        else:
            sel[cell][i] = old
    return {sc: [k for f, _ in S.FAMILIES for k in sel.get((sc, f), [])] for sc in cands}


def pick_main(selection, full, cands):
    """{scenario: {"main": [key], "sub": [key]}}: of each scenario's episodes of a family, the one that shows it best
    in the README's video: mire_only, the most failure stages among README_GRID; faster, the largest lead over core;
    regrasp, volo and core both failing, then volo grasping, then a set-down retake; limits, a failure stage no other
    family's main has yet; else the highest quality (candidates)."""
    ok = lambda k, m: full[k][m]["success"]
    rank = {
        "mire_only": lambda k: (len({S.stage(full[k][m]) for m in README_GRID if m != "mire"}), cands["mire_only"][k]),
        "faster": lambda k: (full[k]["core"]["success_step"] - full[k]["mire"]["success_step"],),
        "regrasp": lambda k: (not ok(k, "volo") and not ok(k, "core"), bool(full[k]["volo"]["recovery_info"].get("grasped")),
                              (full[k]["mire"]["recovery_info"].get("grasps") or [None])[0] == "set_down",
                              cands["regrasp"][k]),
    }
    out = {}
    for sc, keys in selection.items():
        main, used_stages = [], set()
        for f, _ in S.FAMILIES:
            fam = [k for k in keys if k[0] == f]
            if not fam:
                continue
            if sc == "limits":
                key = lambda k: (S.stage(full[k]["mire"]) not in used_stages, -abs(k[1] - 0.55))
            else:
                key = lambda k: rank.get(sc, lambda k: (cands[sc].get(k, 0.0),))(k) + (-abs(k[1] - 0.55),)
            main += sorted(fam, key=key, reverse=True)[:MAIN_PER_FAMILY]
            used_stages |= {S.stage(full[k]["mire"]) for k in main if k[0] == f}
        out[sc] = {"main": main, "sub": [k for k in keys if k not in main]}
    return out


def scenarios(full, selection):
    """[(name, target, layout, methods, [(key, caption)], note)]: a RESULTS gallery of every chosen episode (main and
    sub, per family) and the README's main episodes."""
    _, notes = candidates(full)
    caption = lambda sc, k: STAGE_SHORT[S.stage(full[k]["mire"])] if sc == "limits" else ""
    layout = lambda sc: "grid" if sc in GRID else "focus"
    out = []
    for sc, part in selection.items():
        both = [k for f, _ in S.FAMILIES for k in part["main"] + part["sub"] if k[0] == f]
        out.append((sc, "results", layout(sc), SHOWN[sc], [(k, caption(sc, k)) for k in both], notes[sc]))
        main = [k for f, _ in S.FAMILIES for k in part["main"] if k[0] == f]
        out.append((sc, "readme", layout(sc), README_SHOWN[sc], [(k, caption(sc, k)) for k in main], notes[sc]))
    return out


KEY_FIELDS = ("family", "timing", "task", "init", "seed")  # An episode key, as named in selection.json


def load_selection(path):
    """{scenario: {"main": [key], "sub": [key]}} (a plain list per scenario, the earlier format, is all sub)."""
    keys = lambda eps: [tuple(e[f] for f in KEY_FIELDS) for e in eps]
    return {sc: {"main": keys(v["main"]), "sub": keys(v["sub"])} if isinstance(v, dict) else {"main": [], "sub": keys(v)}
            for sc, v in json.load(open(path)).items()}


def save_selection(path, selection):
    """One episode per line, so that the file is easy to edit by hand."""
    eps = lambda keys: ",\n".join(f"      {json.dumps(dict(zip(KEY_FIELDS, k)))}" for k in keys)
    lines = [f'  "{sc}": {{\n    "main": [\n{eps(part["main"])}\n    ],\n    "sub": [\n{eps(part["sub"])}\n    ]\n  }}'
             for sc, part in selection.items()]
    open(path, "w").write("{\n" + ",\n".join(lines) + "\n}\n")


def video_name(scenario, key, target="results"):
    """<target>/<scenario>/<episode>: the video's path under the output folder, without its extension."""
    f, t, tid, k, seed = key
    return f"{target}/{scenario}/{f}_{t:.2f}_task{tid}_init{k}_seed{seed}"


@functools.lru_cache(maxsize=None)
def font(px):
    return ImageFont.truetype(FONT, px)


def text(img, s, org, px=LABEL_PX, color=(255, 255, 255), outline=True):
    """Draw s in place with its left end of the baseline at org, white with a black outline by default."""
    pil = Image.fromarray(img)
    ImageDraw.Draw(pil).text(org, s, font=font(px), fill=color, anchor="ls", stroke_width=2 if outline else 0,
                             stroke_fill=(0, 0, 0))
    img[:] = np.asarray(pil)


def tile(frame, size, method, t, phase, outcome, banner=True):
    """A frame framed by who acts (no frame while the VLA does), green once the episode has succeeded; when it is over,
    its outcome on a banner at the bottom (banner=False: the step only, as for the wrist view)."""
    img = cv2.resize(np.ascontiguousarray(frame), (size, size), interpolation=cv2.INTER_AREA)
    color = BORDER_COLORS["success"] if outcome and outcome[0] else BORDER_COLORS.get(phase)
    if color:
        cv2.rectangle(img, (0, 0), (size - 1, size - 1), color, BORDER)
    if method:
        text(img, method, (BORDER + 4, BORDER + 16))
    if outcome is None or not banner:
        text(img, f"t={t}", (BORDER + 4, size - BORDER - 6), SMALL_PX)
    else:
        ok, label = outcome
        cv2.rectangle(img, (BORDER, size - BORDER - 22), (size - BORDER - 1, size - BORDER - 1),
                      (40, 160, 60) if ok else (200, 40, 40), -1)
        width = font(SMALL_PX).getlength(label)
        text(img, label, (int((size - width) // 2), size - BORDER - 7), SMALL_PX)
    return img


def title_bar(lines, width):
    bar = np.full((10 + 18 * len(lines), width, 3), 255, np.uint8)
    for i, line in enumerate(lines):
        while font(TITLE_PX).getlength(line) > width - 16:
            line = line[:-4] + "..."
        text(bar, line, (8, 21 + 18 * i), TITLE_PX, color=(0, 0, 0), outline=False)
    return bar


def render(job):
    """Replay the episode of every method shown and write the video and its last frame."""
    from libero.libero import benchmark

    from experiments.robot.libero.libero_utils import get_libero_env
    from src.panda.oft.run_libero_eval_perturb import load_calibration, run_perturbed_episode
    from src.panda.perturbations import PERTURBATIONS
    from src.panda.recovery import RECOVERIES

    cfg, scenario, layout, methods, (f, timing, tid, k, seed), recs, path = job
    suite = benchmark.get_benchmark_dict()[cfg.task_suite_name]()
    calib = load_calibration(cfg.calib_path)
    env, desc = get_libero_env(suite.get_task(tid), cfg.model_family, resolution=cfg.env_img_res)
    tracks, mismatches = {}, []
    for m in methods:
        r = recs[m]
        rec_env = WristRecorder(env)
        perturb = PERTURBATIONS[f](rec_env, r["target"], calib[tid]["phases"], timing, seed)
        recovery = RECOVERIES[m](rec_env, perturb, r["recovery_delay"])
        recovery.recorded = r["recovery_info"]
        success_step, frames, acts, _ = run_perturbed_episode(
            cfg, rec_env, desc, None, 224, None, None, None, None, suite.get_task_init_states(tid)[k], perturb,
            recovery, replay_actions=np.load(P.find_actions(r["actions"], out_dir("actions", cfg))),
            replay_fixtures=r["fixtures"])
        if success_step != r["success_step"]:
            mismatches.append(m)
        # Frame i is the observation before step i + 1: who acted on that step
        phases = ["recovery" if np.isnan(a).any() else "vla" for a in acts]
        for s in range(perturb.fired_step, min(perturb.end_step, len(phases)) + 1):
            phases[s - 1] = "perturb"
        # The banner: the success step, or for a failure (the step limit) "timeout", its failure stage in the limits
        outcome = (True, f"success t={r['success_step']}") if r["success"] else (False, STAGE_SHORT[S.stage(r)])
        banner = outcome if r["success"] or scenario == "limits" else (False, "timeout")
        wrist = rec_env.wrist[cfg.num_steps_wait:cfg.num_steps_wait + len(frames)]
        tracks[m] = (frames, wrist, phases, banner, outcome)
    env.close()

    n = max(len(tr[0]) for tr in tracks.values())
    head = f"{dict(S.FAMILIES)[f]} | timing {timing:.2f} | task {tid}: {desc}"
    # With the wrist camera: one method as a row (third-person | wrist), several as columns (third-person over wrist),
    # each with the controller timeline under it, spanning the video's longest episode (one scale for all methods)
    if layout == "focus":
        bar_w = 2 * FOCUS_TILE if len(methods) == 1 else FOCUS_TILE
        bars = {}
        for m, (_, _, phases, _, _) in tracks.items():
            bar = np.full((TIMELINE_H, bar_w, 3), 230, np.uint8)
            for s, ph in enumerate(phases):
                bar[:, s * bar_w // n:(s + 1) * bar_w // n + 1] = TIMELINE_COLORS[ph]
            bars[m] = bar
        gap = np.full((4, bar_w, 3), 255, np.uint8)  # Keeps the timeline apart from the tile frames
    video = []
    for i in list(range(0, n, FRAME_STEP)) + [n - 1] * HOLD:
        parts = []
        for m, (frames, wrist, phases, outcome, _) in tracks.items():
            j = min(i, len(frames) - 1)
            end = outcome if i >= len(frames) - 1 else None
            if layout == "focus":
                bar = bars[m].copy()
                x = min(j * bar_w // n, bar_w - 2)
                bar[:, x:x + 2] = 0
                views = [tile(frames[j], FOCUS_TILE, m, j + 1, phases[j], end),
                         tile(wrist[j], FOCUS_TILE, "", j + 1, phases[j], end, banner=False)]
                views = np.concatenate(views, axis=1 if len(methods) == 1 else 0)
                parts.append(np.concatenate([views, gap, bar, gap, gap], axis=0))
            else:
                parts.append(tile(frames[j], GRID_TILE, m, j + 1, phases[j], end))
        if layout == "focus":
            sep = np.full((parts[0].shape[0], 4, 3), 255, np.uint8)
            body = np.concatenate([x for p in parts for x in (p, sep)][:-1], axis=1)
            lines = [f"{dict(S.FAMILIES)[f]} | timing {timing:.2f}", f"task {tid}: {desc}"] if body.shape[1] < 700 else [head]
        else:
            body = np.concatenate([np.concatenate(parts[a:a + 5], axis=1) for a in range(0, len(parts), 5)], axis=0)
            lines = [head]
        video.append(np.concatenate([title_bar(lines, body.shape[1]), body], axis=0))
    pad_h, pad_w = (-video[0].shape[0]) % 16, (-video[0].shape[1]) % 16  # H.264 macro blocks (else imageio rescales)
    video = [np.pad(v, ((0, pad_h), (0, pad_w), (0, 0)), constant_values=255) for v in video]
    imageio.mimsave(path, video, fps=FPS, quality=6, macro_block_size=16)
    facts = {}
    for m in methods:
        i = recs[m]["recovery_info"]
        facts[m] = {"outcome": tracks[m][4], "route": i.get("route"), "grasp": (i.get("grasps") or [None])[0],
                    "handoff": i.get("handoff"), "intervention_steps": i.get("intervention_steps"),
                    "tool": i.get("tool"), "grasped": i.get("grasped"), "displaced": i.get("displaced"),
                    "restored": i.get("restored"), "intervention_request": i.get("intervention_request")}
    return {"scenario": scenario, "family": f, "timing": timing, "task": tid, "desc": desc,
                                         "init": k, "seed": seed, "layout": layout, "methods": methods,
                                         "mismatches": mismatches, "facts": facts}


def describe(m, fact):
    ok, label = fact["outcome"]
    s = f"`{m}`: {'✓' if ok else '✗'} {label}"
    if m == "mire" and fact["route"]:
        extra = [f"route {fact['route']}"]
        if fact["grasp"]:
            extra.append(GRASP_KIND[fact["grasp"]])
        if fact["handoff"]:
            extra.append(f"handed back {HANDOFF[fact['handoff']]} after {fact['intervention_steps']} steps")
        s += " (" + ", ".join(extra) + ")"
    if m == "core" and fact.get("displaced"):
        s += (" (no grasp to put the target back: intervention request)" if fact["intervention_request"]
              else " (put the target back)" if fact["restored"] else " (target not restored)")
    if m == "volo" and fact["tool"] == "grasp":
        s += " (grasped with a GraspGen grasp)" if fact["grasped"] else " (grasp missed)"
    return s


def shown_width(target, scenario, layout, path):
    """px: the width at which README.md or RESULTS.md shows the video at path (its preview is made that wide): a README
    table cell, else at most S.FIG_WIDTH_PX; in RESULTS.md a grid at S.FIG_WIDTH_PX, the others at their own width."""
    frames = imageio_ffmpeg.read_frames(path)
    native = next(frames)["size"][0]  # its first item is the video's metadata
    frames.close()
    if target == "readme" and scenario in README_COLUMNS:
        return min(native, S.FIG_WIDTH_PX // README_COLUMNS[scenario] - 26)  # GitHub pads each cell by 13 px a side
    return min(native, S.FIG_WIDTH_PX) if target == "readme" or layout == "grid" else native


def preview_gif(path, width):
    """The looping preview of the video at path (.gif, `width` px wide, GIF_FPS, GIF_COLORS), made again when the
    video is newer or the width differs: GitHub plays a .gif in the page, not a repository's .mp4."""
    gif = path[:-4] + ".gif"
    if os.path.exists(gif) and os.path.getmtime(gif) >= os.path.getmtime(path) and Image.open(gif).width == width:
        return gif
    palette = (f"split[a][b];[a]palettegen=max_colors={GIF_COLORS}:stats_mode=diff[p];"
               "[b][p]paletteuse=dither=bayer:bayer_scale=5:diff_mode=rectangle")
    subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-loglevel", "error", "-y", "-i", path, "-vf",
                    f"fps={GIF_FPS},scale={width}:-1:flags=lanczos,{palette}", gif], check=True)
    return gif


def gallery(cfg, note, entries):
    rel = lambda p: os.path.relpath(p, os.path.dirname(os.path.abspath(cfg.results)))
    lines = [f"- {note}"]
    for name, e in entries:
        path = os.path.join(cfg.out_dir, name + ".mp4")
        caption = f" ({e['caption']})" if e.get("caption") else ""
        width = f' width="{S.FIG_WIDTH_PX}"' if e["layout"] == "grid" else ""
        gif = preview_gif(path, shown_width("results", e["scenario"], e["layout"], path))
        lines += ["", f"**{dict(S.FAMILIES)[e['family']]}, task {e['task']}**{caption}: {e['desc']} "
                      f"(timing {e['timing']:.2f}, initial state {e['init']}, seed {e['seed']})", "",
                  f'<a href="{rel(path)}"><img src="{rel(gif)}" alt="{os.path.basename(name)}"{width}></a>', ""]
        if e["layout"] == "focus":
            lines += [f"- {describe(m, e['facts'][m])}" for m in e["methods"]]
        else:
            lines += ["| " + " | ".join(e["methods"]) + " |", "|---" * len(e["methods"]) + "|",
                      "| " + " | ".join(("✓ " if f["outcome"][0] else "✗ ") + f["outcome"][1] for f in e["facts"].values()) + " |"]
    return "\n".join(lines)


def readme_gallery(cfg, scenario, entries):
    """The README's main episodes (one per perturbation family): each under its family's name, stacked, or in a table of
    README_COLUMNS[scenario] cells per row (with the caption, the failure stage in the limits, under the video)."""
    rel = lambda p: os.path.relpath(p, os.path.dirname(os.path.abspath(cfg.readme)))
    family = lambda e: dict(S.FAMILIES)[e["family"]]

    def video(name, e):
        path = os.path.join(cfg.out_dir, name + ".mp4")
        width = shown_width("readme", scenario, e["layout"], path)
        gif = preview_gif(path, width)
        return f'<a href="{rel(path)}"><img src="{rel(gif)}" alt="{os.path.basename(name)}" width="{width}"></a>'

    cols = README_COLUMNS.get(scenario)
    if not cols:
        return "\n\n".join(f"**{family(e)}**\n\n{video(name, e)}" for name, e in entries)
    rows = []
    for a in range(0, len(entries), cols):
        row = entries[a:a + cols]
        rows.append("<tr>" + "".join(f"<th>{family(e)}</th>" for _, e in row) + "</tr>")
        rows.append("<tr>" + "".join(f"<td>{video(name, e)}" + (f"<br>{e['caption']}" if e.get("caption") else "")
                                     + "</td>" for name, e in row) + "</tr>")
    return "<table>\n" + "\n".join(rows) + "\n</table>"


@draccus.wrap()
def make_qualitative(cfg: QualitativeConfig) -> None:
    resolve_outputs(cfg)
    cfg.out_dir = cfg.out_dir or os.path.join(P.ROOT, "assets", "qualitative", f"{cfg.task_suite_name}_oft")
    os.makedirs(cfg.out_dir, exist_ok=True)
    full = paired(S.load(S.SummaryConfig(tag=cfg.tag, log_dir=out_dir("logs", cfg), num_splits=cfg.num_splits)))
    sel_path = cfg.selection or os.path.join(cfg.out_dir, "selection.json")
    if os.path.exists(sel_path) and not cfg.reselect:
        selection = load_selection(sel_path)
    else:
        cands = candidates(full)[0]
        selection = pick_main(balance(cands, full), full, cands)
        save_selection(sel_path, selection)
    wanted = split(cfg.scenarios)
    plan = [sc for sc in scenarios(full, selection) if not wanted or sc[0] in wanted]
    manifest_path = os.path.join(cfg.out_dir, "manifest.json")
    manifest = json.load(open(manifest_path)) if os.path.exists(manifest_path) else {}
    jobs, names = {}, set()  # one render per (scenario, episode, methods): its output names (readme/ and results/)
    for scenario, target, layout, methods, keys, _ in plan:
        for key, _ in keys:
            name = video_name(scenario, key, target)
            names.add(name)
            if not os.path.exists(os.path.join(cfg.out_dir, name + ".mp4")) or cfg.overwrite or name not in manifest:
                jobs.setdefault((scenario, key, tuple(methods)), [layout, []])[1].append(name)
    print(f"{len(names)} videos in {len({p[0] for p in plan})} scenario(s), {len(jobs)} to render")
    with cf.ProcessPoolExecutor(cfg.num_workers, mp_context=mp.get_context("spawn")) as pool:
        futures = {}
        for (scenario, key, methods), (layout, outs) in jobs.items():
            path = os.path.join(cfg.out_dir, outs[0] + ".mp4")
            os.makedirs(os.path.dirname(path), exist_ok=True)
            futures[pool.submit(render, (cfg, scenario, layout, list(methods), key, full[key], path))] = outs
        for done in cf.as_completed(futures):
            outs = futures[done]
            try:
                e = done.result()
            except Exception as err:  # e.g. an EGL context the GPU could not give: run again to retry the missing ones
                print(f"FAILED {outs[0]}: {type(err).__name__}: {err}", flush=True)
                continue
            for name in outs[1:]:  # the same video in the other folder
                os.makedirs(os.path.dirname(os.path.join(cfg.out_dir, name)), exist_ok=True)
                shutil.copyfile(os.path.join(cfg.out_dir, outs[0] + ".mp4"), os.path.join(cfg.out_dir, name + ".mp4"))
            for name in outs:
                manifest[name] = e
            json.dump(manifest, open(manifest_path, "w"), indent=1)
            print(", ".join(outs) + (f": replay mismatch {e['mismatches']}" if e["mismatches"] else ""), flush=True)
    previews = [(path, shown_width(target, scenario, layout, path)) for scenario, target, layout, _, keys, _ in plan
                for path in (os.path.join(cfg.out_dir, video_name(scenario, k, target) + ".mp4") for k, _ in keys)
                if os.path.exists(path)]
    with cf.ThreadPoolExecutor(cfg.num_workers) as pool:  # ffmpeg processes: threads are enough
        list(pool.map(lambda a: preview_gif(*a), previews))
    for scenario, target, layout, methods, keys, note in plan:
        entries = []
        for key, caption in keys:
            name = video_name(scenario, key, target)
            if name in manifest:
                entries.append((name, dict(manifest[name], caption=caption)))
        if target == "results":
            S.write_block(cfg.results, f"QUALITATIVE:{cfg.tag}:{scenario}", gallery(cfg, note, entries), f"### {scenario}")
        else:
            S.write_block(cfg.readme, f"QUALITATIVE_MAIN:{cfg.tag}:{scenario}", readme_gallery(cfg, scenario, entries),
                          f"#### {scenario}")


if __name__ == "__main__":
    make_qualitative()
