"""
summarize_perturb.py

Aggregates run_libero_eval_perturb.py results into markdown and writes it between marker comments: the success tables
(per-family mean +- sample std across timings, and success within a step budget) into README.md, and the success per
timing, the failure-stage heatmap (assets/quantitative/<suite>_<backbone>/failure_stages.png, see plot_failure_stages)
and the per-method statistics into RESULTS.md. A condition (recovery x family x timing) is reported once every task
split of it has finished (its .txt log has the "Total" line).

    python src/summarize_perturb.py --tag nv10
"""

import glob
import json
import os
import re
from collections import Counter, defaultdict
from dataclasses import dataclass

import draccus
import numpy as np

import src.paths as P

FAMILIES = [("arm_collision", "R1 arm collision"), ("grasp_failure", "T2 grasp failure"),
            ("object_displace", "G5 object displacement"), ("object_tip", "G4 object tip-over")]
TIMINGS = [0.30, 0.55, 0.75]
BUDGETS = [300, 350, 400, 450, 500, 600]  # Steps: success within each (Table 2; 600 = the step limit)
RECOVERIES = ["vanilla", "replan", "rehome", "retreat", "safeloop", "rewind_il", "volo", "core", "core_detect", "mire"]
REWINDS = ["retreat", "safeloop", "rewind_il"]  # Joint-space rewinds: own statistics table
# CoRe (arXiv 2608.14822) Table A6, bare OpenVLA-OFT: S / M / D per family (200 episodes each)
CORE_OFT = {"arm_collision": [15.5, 17.0, 23.5], "grasp_failure": [54.5, 67.5, 73.0],
            "object_displace": [20.5, 12.0, 8.0], "object_tip": [13.5, 6.0, 11.5]}
# Stages at which a failed episode's recovery fell short (plot_failure_stages)
STAGES = ["No\nintervention", "Recovery\ninfeasible", "Recovery\nexecution failed", "Policy failed\nafter recovery"]
STAGE_MEANINGS = [
    "Did not act: vanilla, replan; volo / mire judged no recovery needed (or mire was still waiting); core_detect never "
    "fired after the perturbation",
    "No plan, waypoint or checkpoint; path rejected; intervention request; restoration cap; mire gave up",
    "Acted but fell short: not arrived, grasp missed, object not restored (> 3 cm), mire handed back early",
    "Recovery finished, but the policy did not complete the task within 600 steps"]
FIG_WIDTH_IN, FIG_WIDTH_PX = 8.4, 800  # Figure size, and its width in the README (about GitHub's content width)


@dataclass
class SummaryConfig:
    # fmt: off
    tag: str = "nv10"                                 # Substring of the run_id_note that selects the runs
    suite: str = "libero_10"                          # Task suite and backbone whose runs are summarized:
    backbone: str = "oft"                             #   outputs/logs/<suite>/<backbone>
    log_dir: str = ""                                 # "" = that directory
    readme: str = os.path.join(P.ROOT, "README.md")   # Success tables, between the RESULTS:<tag> markers
    results: str = os.path.join(P.ROOT, "RESULTS.md") # Failure stages and per-method statistics, between the DETAILS:<tag> markers
    num_splits: int = 2                               # Task splits per condition (e.g. GPU0 tasks 0-4, GPU1 tasks 5-9)
    figure: str = ""                                  # Failure-stage heatmap ("" = assets/quantitative/<suite>_<backbone>/failure_stages.png)
    # fmt: on


def load(cfg):
    """{(recovery, family, timing): [episode records]} for conditions whose splits have all finished."""
    recs, splits = defaultdict(list), defaultdict(lambda: [0, 0])
    for txt in glob.glob(os.path.join(cfg.log_dir, f"PERTURB-*--*{cfg.tag}*.txt")):
        m = re.match(r"PERTURB-(\w+?)-(\w+)-(\d\.\d\d)-", os.path.basename(txt))
        key = (m.group(1), m.group(2), float(m.group(3)))
        done = any("Total:" in line for line in open(txt))
        splits[key][0] += 1
        splits[key][1] += done
        if done:  # A run that died midway (and was re-run) must not add its episodes
            with open(txt[:-4] + ".jsonl") as f:
                recs[key] += [json.loads(line) for line in f]
    return {k: v for k, v in recs.items() if splits[k][1] >= cfg.num_splits}


def rate(rs):
    valid = [r for r in rs if r["valid"]]
    return 100.0 * sum(r["success"] for r in valid) / len(valid) if valid else None


def mean_std(xs):
    return f"{np.mean(xs):.1f} ± {np.std(xs, ddof=1):.1f}"


def mark_best(cells):
    """{method: (value as printed, text) or None} -> {method: cell}: the best in bold, the second best underlined."""
    ranks = sorted({c[0] for c in cells.values() if c is not None}, reverse=True)
    return {r: "…" if c is None else f"**{c[1]}**" if c[0] == ranks[0]
            else f"<ins>{c[1]}</ins>" if len(ranks) > 1 and c[0] == ranks[1] else c[1] for r, c in cells.items()}


def tables(res, cfg):
    fmt = lambda x: "…" if x is None else f"{x:.0f}"
    actions_root = P.out("actions", cfg.suite, cfg.backbone)
    timed_out = lambda x: not x["success"] and len(np.load(P.find_actions(x["actions"], actions_root))) >= 600
    per = {(r, f, t): (rate(res[(r, f, t)]) if (r, f, t) in res else None)
           for r in RECOVERIES for f, _ in FAMILIES for t in TIMINGS}
    n_valid = {k: sum(x["valid"] for x in v) for k, v in res.items()}

    valid = {r: [x for f, _ in FAMILIES for t in TIMINGS for x in res.get((r, f, t), []) if x["valid"]] for r in RECOVERIES}
    core_overall = [np.mean([CORE_OFT[f][i] for f, _ in FAMILIES]) for i in range(3)]
    n_eps = sorted({len(v) for v in valid.values() if v})
    eps_txt = f"{n_eps[0]:,} episodes per method" if len(n_eps) == 1 else "valid episodes differ by method"
    lines = ["**Table 1. Success rate per perturbation (%), mean ± sample std across the 3 timings (" + eps_txt + ")**", "",
             "| Method | " + " | ".join(name for _, name in FAMILIES) + " | Overall |",
             "|---" * (len(FAMILIES) + 2) + "|"]
    lines.append("| *CoRe paper: bare OFT* | " + " | ".join(f"*{mean_std(CORE_OFT[f])}*" for f, _ in FAMILIES)
                 + f" | *{mean_std(core_overall)}* |")
    # Per column (each perturbation, then overall): the timing values of every method (None while running)
    cols = {r: [None if None in xs else xs for xs in ([per[(r, f, t)] for t in TIMINGS] for f, _ in FAMILIES)] for r in RECOVERIES}
    for r in RECOVERIES:
        overall = [[per[(r, f, t)] for f, _ in FAMILIES] for t in TIMINGS]
        cols[r].append(None if any(None in o for o in overall) else [np.mean(o) for o in overall])
    for c in range(len(FAMILIES) + 1):  # Best in bold, second best underlined
        marked = mark_best({r: None if cols[r][c] is None else (float(f"{np.mean(cols[r][c]):.1f}"), mean_std(cols[r][c]))
                            for r in RECOVERIES})
        for r in RECOVERIES:
            cols[r][c] = marked[r]
    for r in RECOVERIES:
        lines.append(f"| {r} | " + " | ".join(cols[r]) + " |")

    # Success within a step budget, failures never counted (no selection by which episodes a method solves)
    complete = {r: all((r, f, t) in res for f, _ in FAMILIES for t in TIMINGS) for r in RECOVERIES}
    within = {r: [100.0 * sum(x["success"] and x["success_step"] <= b for x in valid[r]) / len(valid[r]) for b in BUDGETS]
              if complete[r] and valid[r] else None for r in RECOVERIES}
    lines += ["", f"**Table 2. Success rate within a step budget (%), all perturbations and timings pooled ({eps_txt})**", "",
              "| Method | " + " | ".join(f"≤ {b} steps" for b in BUDGETS) + " |", "|---" * (len(BUDGETS) + 1) + "|"]
    cells = {r: [] for r in RECOVERIES}
    for i in range(len(BUDGETS)):  # Best in bold, second best underlined, per budget
        marked = mark_best({r: None if within[r] is None else (float(f"{within[r][i]:.1f}"), f"{within[r][i]:.1f}")
                            for r in RECOVERIES})
        for r in RECOVERIES:
            cells[r].append(marked[r])
    lines += [f"| {r} | " + " | ".join(cells[r]) + " |" for r in RECOVERIES]
    lines += ["", "- Steps count from the start of the episode; an episode that fails counts as unfinished at every budget",
              "- 600 steps is the step limit, so the last column is the overall success rate"]

    main, lines = lines, []  # The rest goes to RESULTS.md
    lines += ["**Table S1. Success rate per perturbation and timing (%)**", "",
              "| Method | " + " | ".join(f"{name}<br>0.30 / 0.55 / 0.75" for _, name in FAMILIES) + " |",
              "|---" * (len(FAMILIES) + 1) + "|"]
    lines.append("| *CoRe paper: bare OFT* | " + " | ".join(
        "*" + " / ".join(f"{x:.1f}" for x in CORE_OFT[f]) + "*" for f, _ in FAMILIES) + " |")
    cells = {r: [] for r in RECOVERIES}
    for f, _ in FAMILIES:
        for t in TIMINGS:  # Best in bold, second best underlined, per perturbation and timing
            marked = mark_best({r: None if per[(r, f, t)] is None else (float(fmt(per[(r, f, t)])), fmt(per[(r, f, t)]))
                                for r in RECOVERIES})
            for r in RECOVERIES:
                cells[r].append(marked[r])
    n_t = len(TIMINGS)
    for r in RECOVERIES:
        lines.append(f"| {r} | " + " | ".join(" / ".join(cells[r][i * n_t:(i + 1) * n_t]) for i in range(len(FAMILIES))) + " |")
    lines += ["", "- 95% interval of a per-timing value: about ±8–10 pp at 100 episodes", ""]

    figure = os.path.relpath(cfg.figure, os.path.dirname(os.path.abspath(cfg.results)))
    lines += ["**Figure S1. Failed episodes by recovery stage**", "",
              f'<img src="{figure}" alt="Failed episodes by recovery stage" width="{FIG_WIDTH_PX}">', "",
              "- Failed episodes per method, split by the stage where the recovery fell short",
              "- Stages come from each episode's `recovery_info`, not from the physical outcome",
              "- Left column: total failures and failure rate (own blue scale); red box = lowest failure rate",
              "- Right columns: one color scale for all cells; lighter = fewer failures", "",
              "| Stage | Meaning |", "|---|---|"]
    lines += [f"| {name.replace(chr(10), ' ')} | {meaning} |" for name, meaning in zip(STAGES, STAGE_MEANINGS)]

    lines += ["", "**Table S2. rehome statistics (valid episodes, 3 timings pooled)**", "",
              "| Perturbation | Episodes | Moved holding the object | Arrived | Mean steps (lift + joint) | Contact besides the held object | Cut off by the episode end |",
              "|---|---|---|---|---|---|---|"]
    for f, name in FAMILIES:
        eps = [x for t in TIMINGS for x in res.get(("rehome", f, t), [])
               if x["valid"] and x["recovery_info"].get("trigger_steps")]
        info = [x["recovery_info"] for x in eps]
        if not info:
            lines.append(f"| {name} | … | … | … | … | … | … |")
            continue
        pct = lambda xs: f"{100 * np.mean(xs):.0f}%"
        lift = [i["lift_steps"] for i in info if i.get("lift_steps") is not None]
        joint = [i["joint_steps"] for i in info if i.get("joint_steps") is not None]
        held = [i.get("held", [x["target"]] if i["holding"] else []) for x, i in zip(eps, info)]
        other = [any(not any(b.startswith(o) for o in h) for b in i.get("contacts", {})) for h, i in zip(held, info)]
        lines.append(f"| {name} | {len(info)} | {pct([i['holding'] for i in info])} | "
                     f"{pct([i.get('arrived', False) for i in info])} | "
                     f"{np.mean(lift):.0f} + {np.mean(joint) if joint else 0:.0f} | {pct(other)} | "
                     f"{pct([i.get('interrupted', False) for i in info])} |")

    lines += ["", "**Table S3. Rewind baselines statistics (valid episodes, 3 timings pooled)**", "",
              "| Method | Perturbation | Episodes | Rewound | Rewind distance (steps) | Intervention steps | Arrived | Contact besides the held object | Cut off by the episode end | Notes |",
              "|---|---|---|---|---|---|---|---|---|---|"]
    for r in REWINDS:
        for f, name in FAMILIES:
            eps = [x for t in TIMINGS for x in res.get((r, f, t), [])
                   if x["valid"] and x["recovery_info"].get("trigger_steps")]
            if not eps:
                lines.append(f"| {r} | {name} | … | … | … | … | … | … | … | … |")
                continue
            info = [x["recovery_info"] for x in eps]
            moved = [i for i in info if r == "retreat" or i.get("rollback")]
            pct = lambda xs: f"{100 * np.mean(xs):.0f}%" if len(xs) else "-"
            back = [i["trigger_steps"][0] - 1 - i["target_step"] for i in moved]
            steps = [40 if r == "retreat" else i["steps"] for i in moved if not i.get("interrupted")]
            other = [any(not any(b.startswith(o) for o in i.get("held", [])) for b in i.get("contacts", {}))
                     for i in moved]
            if r == "retreat":
                note = f"clamped to the start pose {pct([i['clamped'] for i in info])}"
            elif r == "safeloop":
                note = (f"no saved waypoint {pct([i.get('skipped') == 'no_waypoint' for i in info])}, "
                        f"path rejected (collision) {pct([i.get('skipped') == 'path_collision' for i in info])}")
            else:
                kinds = [i["target"].split(":")[0] for i in moved]
                note = (f"no checkpoint {pct([i.get('skipped') == 'no_valid_checkpoint' for i in info])}, "
                        f"target grasp {pct([k == 'grasp' for k in kinds])} / subgoal {pct([k == 'goal' for k in kinds])}")
            lines.append(f"| {r} | {name} | {len(info)} | {pct([1] * len(moved) + [0] * (len(info) - len(moved)))} | "
                         f"{np.mean(back) if back else 0:.0f} | {np.mean(steps) if steps else 0:.0f} | "
                         f"{pct([i.get('arrived', False) for i in moved])} | {pct(other)} | "
                         f"{pct([i.get('interrupted', False) for i in moved])} | {note} |")
    lines += ["", "- Rewound: share of episodes in which the arm moved; the other columns are over those episodes",
              "- Rewind distance: steps from the trigger back to the target pose",
              "- Contact: anything the robot touched while rewinding, except what it held",
              "- retreat hands back after 40 steps, arrived or not"]

    lines += ["", "**Table S4. volo statistics (valid episodes, 3 timings pooled)**", "",
              "| Perturbation | Episodes | Grasp chosen | No solution → VLA | Continue reason (holding / goal met / not disturbed) | Grasped and lifted | Median planning time | Intervention steps | Contact besides the target |",
              "|---|---|---|---|---|---|---|---|---|"]
    for f, name in FAMILIES:
        eps = [x for t in TIMINGS for x in res.get(("volo", f, t), []) if x["valid"] and x["recovery_info"].get("trigger_steps")]
        info = [x["recovery_info"] for x in eps]
        if not info:
            lines.append(f"| {name} | … | … | … | … | … | … | … | … |")
            continue
        pct = lambda xs: f"{100 * np.mean(xs):.0f}%" if len(xs) else "-"
        grasp = [i for i in info if i.get("tool") == "grasp"]
        cont = [i for i in info if i.get("tool") != "grasp" and i.get("skipped") != "no_feasible_grasp"]
        reasons = (f"{pct([i.get('held_now') for i in cont])} / {pct([i.get('goal_done') and not i.get('held_now') for i in cont])} / "
                   f"{pct([not i.get('held_now') and not i.get('goal_done') for i in cont])}") if cont else "-"
        planned = [i["plan_seconds"] for i in info if "plan_seconds" in i]
        steps = [sum(i.get("steps", {}).values()) for i in grasp if not i.get("interrupted")]
        other = [any(not b.startswith(x["target"]) for b in i.get("contacts", {})) for x, i in zip(eps, info) if i.get("tool") == "grasp"]
        lines.append(f"| {name} | {len(info)} | {pct([i.get('tool') == 'grasp' for i in info])} | "
                     f"{pct([i.get('skipped') == 'no_feasible_grasp' for i in info])} | {reasons} | "
                     f"{pct([bool(i.get('grasped')) for i in grasp])} | {np.median(planned) if planned else 0:.1f}s | "
                     f"{np.mean(steps) if steps else 0:.0f} | {pct(other)} |")
    lines += ["", "- Grasp chosen, no solution: over the triggered episodes",
              "- Continue reason: over the continues that were not for lack of a solution",
              "- Grasped and lifted, intervention steps, contact: over the executed grasps",
              "- Planning time: wall clock, with the simulation paused"]

    lines += ["", "**Table S5. core statistics (valid episodes, 3 timings pooled)**", "",
              "| Perturbation | Episodes | Rewind distance (steps) | Imagined rollout steps (proximity / cap / IK failure) | Restoration needed | Restored | Infeasible (intervention request) | Intervention steps | Timeout | Success | Success (intervention request = failure) |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
    for f, name in FAMILIES:
        eps = [x for t in TIMINGS for x in res.get(("core", f, t), []) if x["valid"] and x["recovery_info"].get("trigger_steps")]
        info = [x["recovery_info"] for x in eps]
        if not info:
            lines.append(f"| {name} | … | … | … | … | … | … | … | … | … | … |")
            continue
        pct = lambda xs: f"{100 * np.mean(xs):.0f}%" if len(xs) else "-"
        ends = [i["imagined"]["end"] for i in info]
        need = [i for i in info if i.get("displaced")]
        steps = [sum(i.get("steps", {}).values()) for i in info if not i.get("interrupted")]
        lines.append(f"| {name} | {len(info)} | {np.mean([i['back_steps'] for i in info]):.0f} | "
                     f"{np.mean([i['imagined']['steps'] for i in info]):.1f} ({pct([e == 'proximity' for e in ends])} / "
                     f"{pct([e == 'cap' for e in ends])} / {pct([e == 'ik_fail' for e in ends])}) | {pct([i.get('displaced') for i in info])} | "
                     f"{pct([i.get('restored') for i in need])} | {pct([bool(i.get('intervention_request')) for i in need])} | "
                     f"{np.mean(steps) if steps else 0:.0f} | {pct([timed_out(x) for x in eps])} | "
                     f"{100 * np.mean([x['success'] for x in eps]):.1f} | "
                     f"{100 * np.mean([x['success'] and not i.get('intervention_request') for x, i in zip(eps, info)]):.1f} |")
    lines += ["", "- Restoration: pick the target and place it back (the perturbations move only the target)",
              "- Restored: within 3 cm after placing, one retry",
              "- Infeasible: no grasp fits both pick and place (the paper's intervention request); handed to the policy as is",
              "- Intervention steps: pick, carry, place and rejoin",
              "- Timeout: failures that hit the 600-step limit",
              "- Last column: intervention requests counted as failures, as in the paper"]

    lines += ["", "**Table S6. core_detect statistics (valid episodes, 3 timings pooled)**", "",
              "| Perturbation | Episodes | False trigger before the perturbation | Triggered after | Not triggered after (of which success) | Median perturbation→trigger steps (oracle) | Recoveries | Restoration needed | Restored | Intervention request | Restoration cap | Timeout | Success | Success (intervention request = failure) |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    oracle = {"arm_collision": 13, "grasp_failure": 18, "object_displace": 9, "object_tip": 12}  # injection + 8 steps
    for f, name in FAMILIES:
        eps = [x for t in TIMINGS for x in res.get(("core_detect", f, t), []) if x["valid"]]
        if not eps:
            lines.append(f"| {name} | … | … | … | … | … | … | … | … | … | … | … | … | … |")
            continue
        pct = lambda xs: f"{100 * np.mean(xs):.0f}%" if len(xs) else "-"
        trig = [x["recovery_info"]["trigger_steps"] for x in eps]
        early = [any(t < x["fired_step"] for t in tr) for x, tr in zip(eps, trig)]
        after = [[t for t in tr if t >= x["fired_step"]] for x, tr in zip(eps, trig)]
        missed = [x for x, a in zip(eps, after) if not a]
        recs = [i for x in eps for i in x["recovery_info"]["recoveries"]]
        need = [i for i in recs if i.get("displaced")]
        ir = [any(i.get("intervention_request") for i in x["recovery_info"]["recoveries"]) for x in eps]
        delay = [a[0] - x["fired_step"] for x, a in zip(eps, after) if a]
        lines.append(f"| {name} | {len(eps)} | {pct(early)} | {pct([bool(a) for a in after])} | "
                     f"{pct([not a for a in after])} ({pct([x['success'] for x in missed])}) | "
                     f"{np.median(delay) if delay else 0:.0f} ({oracle[f]}) | {np.mean([len(tr) for tr in trig]):.2f} | "
                     f"{len(need)}/{len(recs)} | {pct([i.get('restored') for i in need])} | "
                     f"{pct([bool(i.get('intervention_request')) for i in need])} | {pct([i.get('capped') for i in need])} | "
                     f"{pct([timed_out(x) for x in eps])} | "
                     f"{100 * np.mean([x['success'] for x in eps]):.1f} | "
                     f"{100 * np.mean([x['success'] and not r for x, r in zip(eps, ir)]):.1f} |")
    lines += ["", "- Trigger: CoRe's own detector instead of the oracle; several recoveries per episode, one restoration at most",
              "- False trigger before: fired at least once before the perturbation",
              "- Not triggered after: never fired after the perturbation",
              "- Perturbation→trigger: to the first trigger after it; the oracle's value in parentheses",
              "- Recoveries: mean per episode",
              "- Restoration columns: per recovery (needed / all recoveries); success: per episode"]

    lines += ["", "**Table S7. mire statistics (valid episodes, 3 timings pooled)**", "",
              "| Perturbation | Episodes | Route: none / carry / grasp / resume subgoal / gave up | Grasp: own / set down and retaken / other (success of each) | Handoff: agreement / 40 steps before the goal / goal unreachable / grasp missed | Intervention steps (agreement handoffs) | Success after an agreement handoff | Timeout | Success |",
              "|---|---|---|---|---|---|---|---|---|"]
    for f, name in FAMILIES:
        eps = [x for t in TIMINGS for x in res.get(("mire", f, t), []) if x["valid"] and x["recovery_info"].get("trigger_steps")]
        if not eps:
            lines.append(f"| {name} | … | … | … | … | … | … | … | … |")
            continue
        pct = lambda xs: f"{100 * np.mean(xs):.0f}%" if len(xs) else "-"
        info = [x["recovery_info"] for x in eps]
        route = [i.get("route") or "" for i in info]
        hand = [i.get("handoff") for i in info]
        acted = [i for i in info if i.get("handoff")]
        agree = [(x, i) for x, i in zip(eps, info) if i.get("handoff") == "agree"]
        # how the object was first taken: the policy's own grasp (clean / near), set down and retaken, or GraspGen's
        first = [((i.get("grasps") or [None])[0], x["success"]) for x, i in zip(eps, info) if i.get("grasps")]
        kinds = [("clean", "near"), ("set_down",), ("foreign",)]
        grasp_col = " / ".join(f"{pct([k in ks for k, _ in first])} ({pct([s for k, s in first if k in ks])})" for ks in kinds) \
            if first else "-"
        lines.append(
            f"| {name} | {len(eps)} | {pct([r.startswith('none') for r in route])} / {pct([r in ('carry', 'resume_carry') for r in route])} / "
            f"{pct([r in ('grasp', 'resume_grasp') for r in route])} / {pct([r == 'resume_follow' for r in route])} / "
            f"{pct([r.endswith('no_feasible_grasp') for r in route])} | "
            f"{grasp_col} | "
            f"{pct([h == 'agree' for h in hand if h])} / {pct([h == 'pre_goal' for h in hand if h])} / "
            f"{pct([h == 'goal_unreachable' for h in hand if h])} / {pct([h == 'missed_grasp' for h in hand if h])} | "
            f"{np.mean([i.get('intervention_steps', 0) for i in acted]) if acted else 0:.0f} "
            f"({np.mean([i.get('intervention_steps', 0) for _, i in agree]) if agree else 0:.0f}) | "
            f"{pct([x['success'] for x, _ in agree])} | "
            f"{pct([timed_out(x) for x in eps])} | "
            f"{100 * np.mean([x['success'] for x in eps]):.1f} |")
    lines += ["", "- Route (state at the trigger)",
              "  - held → carry; disturbed → grasp, then carry",
              "  - untouched but the arm > 8 cm off the clean rollout → resume the subgoal",
              "  - goal met or arm on its path → none; no feasible grasp → gave up",
              "- Grasp (first grasp on the grasp route; success of each in parentheses)",
              "  - own: the policy's grasp in the clean rollout, or a GraspGen candidate within 10° and 2 cm of it",
              "  - set down and retaken: taken another way, set down, retaken with the own grasp",
              "  - other: GraspGen's best",
              "- Handoff (episodes with an intervention)",
              "  - agreement: the shadow policy matched the plan on 3 checks in a row",
              "  - 40 steps before the goal: reached the clean pose 40 steps before the release",
              "  - goal unreachable, grasp missed: handed back early",
              "- Intervention steps: from the trigger to the handoff"]

    done = len(res)
    total = len(RECOVERIES) * len(FAMILIES) * len(TIMINGS)
    nv = sorted(set(n_valid.values()))
    main += ["", f"- Progress: {done}/{total} conditions (`…` = still running)",
              f"- Valid episodes per condition: {', '.join(map(str, nv)) if nv else '-'} (10 tasks pooled; CoRe paper: 200)",
              "- The CoRe paper reports CoRe only on π0.5, X-VLA and GR00T N1.7, not on OpenVLA-OFT"]
    return "\n".join(main), "\n".join(lines)


def _core_stage(i):
    """Stage of one core restoration (core, and each recovery of core_detect)."""
    if i.get("intervention_request") or i.get("capped"):
        return 1
    if (i.get("displaced") and not i.get("restored")) or i.get("interrupted") or not i.get("arrived", True):
        return 2
    return 3


def stage(r):
    """Index into STAGES of a failed episode's record."""
    m, i = r["recovery"], r["recovery_info"] or {}
    if m in ("vanilla", "replan"):
        return 0
    if m in ("rehome", "retreat"):
        return 3 if i.get("arrived") and not i.get("interrupted") else 2
    if m in ("safeloop", "rewind_il"):
        if not i.get("rollback"):
            return 1
        return 3 if i.get("arrived") and not i.get("interrupted") else 2
    if m == "volo":
        if i.get("tool") != "grasp":
            return 1 if i.get("skipped") == "no_feasible_grasp" else 0
        return 3 if i.get("grasped") and not i.get("interrupted") else 2
    if m == "core":
        return _core_stage(i)
    if m == "core_detect":
        if not any(t >= r["fired_step"] for t in i.get("trigger_steps", [])):
            return 0
        return _core_stage(i["recoveries"][-1])
    if m == "mire":
        route, handoff = i.get("route") or "", i.get("handoff")
        if route.endswith("no_feasible_grasp"):
            return 1
        if route.startswith("none") or not handoff:
            return 0
        return 2 if handoff in ("missed_grasp", "goal_unreachable") else 3
    raise ValueError(f"Unknown recovery: {m}")


def plot_failure_stages(res, cfg):
    """Failed episodes of every method, split by the stage at which the recovery fell short (read from each episode's
    recovery_info), as a heatmap:
      1 no intervention    the method did not act: vanilla, replan; volo / mire judged no recovery needed (or mire was
                           still waiting for an earlier subgoal); core_detect never fired after the perturbation
      2 infeasible         no plan, waypoint or checkpoint, rejected path, intervention request, restoration cap, gave up
      3 execution failed   acted but did not get there: not arrived, grasp missed, object not restored, or (mire)
                           handed back early because the grasp missed or the goal pose was unreachable
      4 policy failed      the recovery finished and handed back, and the policy did not complete the task in time"""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    methods = [m for m in RECOVERIES if any(k[0] == m for k in res)]
    counts, valid = np.zeros((len(methods), len(STAGES)), dtype=int), Counter()
    for (m, _, _), recs in res.items():
        for r in recs:
            if r["valid"]:
                valid[m] += 1
                if not r["success"]:
                    counts[methods.index(m), stage(r)] += 1
    # Left: total failures per method (own blue scale over the valid episodes, so it does not drown the breakdown),
    # the lowest failure rate boxed in red.
    # Right: the breakdown, one color scale for every cell: fewer failures = lighter.
    totals, n_valid = counts.sum(axis=1), np.array([valid[m] for m in methods])
    fig, (ax_t, ax) = plt.subplots(1, 2, figsize=(FIG_WIDTH_IN, 0.45 * len(methods) + 1.7), sharey=True,
                                   gridspec_kw={"width_ratios": [1.15, len(STAGES)], "wspace": 0.05})
    ax_t.imshow(totals[:, None], cmap="Blues", vmin=0, vmax=n_valid.max(), aspect="auto")
    for y in np.flatnonzero(totals / n_valid == (totals / n_valid).min()):
        ax_t.add_patch(plt.Rectangle((-0.5, y - 0.5), 1, 1, fill=False, edgecolor="red", linewidth=3, clip_on=False))
    for y, (n, v) in enumerate(zip(totals, n_valid)):
        ax_t.text(0, y, f"{n} ({100 * n / v:.0f}%)", ha="center", va="center", fontsize=10,
                  color="white" if n > 0.55 * n_valid.max() else "black")
    ax_t.set_xticks([0], ["Total failed\n(failure rate)"], fontsize=9)
    ax_t.set_yticks(range(len(methods)), methods, fontsize=10)
    im = ax.imshow(counts, cmap="OrRd", vmin=0, vmax=counts.max(), aspect="auto")
    for (y, x), n in np.ndenumerate(counts):
        ax.text(x, y, n, ha="center", va="center", fontsize=10,
                color="white" if n > 0.6 * counts.max() else "black")
    ax.set_xticks(range(len(STAGES)), STAGES, fontsize=9)
    for a in (ax_t, ax):
        a.tick_params(top=True, labeltop=True, bottom=False, labelbottom=False)
    ax.tick_params(left=False)
    valid_txt = f"{n_valid[0]} valid episodes per method" if len(set(n_valid)) == 1 else "valid episodes differ by method"
    fig.suptitle(f"Failed episodes by recovery stage ({cfg.suite}, {cfg.backbone}; {valid_txt})", fontsize=10)
    fig.colorbar(im, ax=ax, fraction=0.035, pad=0.02, label="failed episodes (breakdown)")
    fig.subplots_adjust(left=0.12, right=0.92, top=0.84, bottom=0.03)
    os.makedirs(os.path.dirname(cfg.figure), exist_ok=True)
    fig.savefig(cfg.figure, dpi=200)
    plt.close(fig)
    print(f"Saved {cfg.figure}")


def write_block(path, name, body, title):
    """Put body between the <!-- name START/END --> comments of path (appended under title if they are missing)."""
    start, end = f"<!-- {name} START -->", f"<!-- {name} END -->"
    text = open(path).read() if os.path.exists(path) else ""
    if start in text:
        text = text[: text.index(start)] + start + "\n" + body + "\n" + text[text.index(end):]
    else:
        text = text.rstrip("\n") + ("\n\n" if text else "") + f"{title}\n\n{start}\n{body}\n{end}\n"
    with open(path, "w") as f:
        f.write(text)


@draccus.wrap()
def summarize(cfg: SummaryConfig) -> None:
    cfg.log_dir = cfg.log_dir or P.out("logs", cfg.suite, cfg.backbone)
    cfg.figure = cfg.figure or os.path.join(P.ROOT, "assets", "quantitative", f"{cfg.suite}_{cfg.backbone}", "failure_stages.png")
    res = load(cfg)
    main, details = tables(res, cfg)
    write_block(cfg.readme, f"RESULTS:{cfg.tag}", main, f"## Results (`{cfg.tag}`)")
    write_block(cfg.results, f"DETAILS:{cfg.tag}", details, f"## Detailed results (`{cfg.tag}`)")
    print(main + "\n\n" + details)
    plot_failure_stages(res, cfg)


if __name__ == "__main__":
    summarize()
