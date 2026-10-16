"""
calibrate_detector.py

Calibrates core_detect's detector (CoRe Eq. 2, Table A4) from the detection bank (collect_detector_bank.py): per task,
the successful clean rollouts are split 70/30 (fixed seed) into a fit and a calibration set. On the fit set's query
latents: the per-dimension mean and standard deviation, and the mean and ridge-regularized covariance (RIDGE) of the
z-standardized features, whose inverse is stored; tau is the TAU_QUANTILE quantile of the scores of all calibration
set queries. Writes <detector_dir>/task<id>.npz, index.json (instruction -> file, read by recovery.LatentDetector)
and summary.json (per task: sizes, tau, score quantiles, and how many fit / calibration rollouts the trigger rule of
recovery.CoreDetect would fire on).

    python src/panda/calibrate_detector.py
"""

import glob
import json
import os
import re
from dataclasses import dataclass

import draccus
import numpy as np

from src.panda.recovery import CoreDetect
import src.paths as P

RIDGE, TAU_QUANTILE, CAL_FRACTION, SPLIT_SEED = 1e-2, 0.95, 0.3, 0  # CoRe Table A4; 70/30 split (CoRe)
SIGMA_FLOOR = 1e-6  # standard deviation floor for (near-)constant latent dimensions


@dataclass
class CalibConfig:
    # fmt: off
    suite: str = "libero_10"                                 # Task suite and backbone whose bank is calibrated:
    backbone: str = "oft"                                    #   outputs/detector/<suite>/<backbone>
    bank_dir: str = ""                                       # collect_detector_bank.py output ("" = <that>/bank)
    detector_dir: str = ""                                   # Output directory, read by core_detect ("" = <that>)
    # fmt: on


def first_trigger(scores, tau):
    """Query index at which CoreDetect's rule (N_CONSEC over tau or PLATEAU_N over PLATEAU_FRAC * tau) first fires."""
    above = near = 0
    for i, s in enumerate(scores):
        above = above + 1 if s > tau else 0
        near = near + 1 if s > CoreDetect.PLATEAU_FRAC * tau else 0
        if above >= CoreDetect.N_CONSEC or near >= CoreDetect.PLATEAU_N:
            return i
    return None


@draccus.wrap()
def calibrate(cfg: CalibConfig) -> None:
    cfg.detector_dir = cfg.detector_dir or P.out("detector", cfg.suite, cfg.backbone)
    cfg.bank_dir = cfg.bank_dir or os.path.join(cfg.detector_dir, "bank")
    os.makedirs(cfg.detector_dir, exist_ok=True)
    episodes = {}
    for path in sorted(glob.glob(os.path.join(cfg.bank_dir, "task*_init*.npz"))):
        tid, k = (int(x) for x in re.search(r"task(\d+)_init(\d+)", path).groups())
        z = np.load(path)
        episodes.setdefault(tid, []).append({"k": k, "task": str(z["task"]), "success": int(z["success_step"]) > 0,
                                             "latents": z["latents"].astype(np.float64)})
    index, summary = {}, {}
    for tid, eps in sorted(episodes.items()):
        ok = sorted((e for e in eps if e["success"]), key=lambda e: e["k"])
        perm = np.random.default_rng(SPLIT_SEED).permutation(len(ok))
        n_cal = int(round(CAL_FRACTION * len(ok)))
        cal, fit = [ok[i] for i in sorted(perm[:n_cal])], [ok[i] for i in sorted(perm[n_cal:])]
        X = np.concatenate([e["latents"] for e in fit])
        mu, sigma = X.mean(0), np.maximum(X.std(0), SIGMA_FLOOR)
        Z = (X - mu) / sigma
        zbar = Z.mean(0)
        cov = np.cov(Z, rowvar=False) + RIDGE * np.eye(Z.shape[1])
        w, V = np.linalg.eigh(cov)
        prec = (V / w) @ V.T

        def scores(e):
            D = (e["latents"] - mu) / sigma - zbar
            return np.sqrt(np.maximum(((D @ prec) * D).sum(1), 0.0))

        s_fit, s_cal = [scores(e) for e in fit], [scores(e) for e in cal]
        tau = float(np.quantile(np.concatenate(s_cal), TAU_QUANTILE))
        name = f"task{tid}.npz"
        np.savez(os.path.join(cfg.detector_dir, name), mu=mu, sigma=sigma, zbar=zbar, prec=prec.astype(np.float32),
                 tau=tau, fit=[e["k"] for e in fit], cal=[e["k"] for e in cal])
        index[ok[0]["task"]] = name
        q = [0.5, 0.9, 0.95, 0.99]
        summary[tid] = {
            "task": ok[0]["task"], "episodes": len(eps), "successes": len(ok), "fit": len(fit), "cal": len(cal),
            "fit_queries": len(X), "cal_queries": int(sum(len(s) for s in s_cal)), "tau": round(tau, 2),
            "fit_score_q50_90_95_99": np.round(np.quantile(np.concatenate(s_fit), q), 2).tolist(),
            "cal_score_q50_90_95_99": np.round(np.quantile(np.concatenate(s_cal), q), 2).tolist(),
            "fit_rollouts_triggered": sum(first_trigger(s, tau) is not None for s in s_fit),
            "cal_rollouts_triggered": sum(first_trigger(s, tau) is not None for s in s_cal),
            "cal_trigger_queries": [first_trigger(s, tau) for s in s_cal],
            "sigma_at_floor": int((sigma <= SIGMA_FLOOR).sum()), "cov_eig_min_max": [float(w[0]), float(w[-1])]}
        print(json.dumps({"task_id": tid, **{k: v for k, v in summary[tid].items() if k != "cal_trigger_queries"}}),
              flush=True)
    json.dump(index, open(os.path.join(cfg.detector_dir, "index.json"), "w"), indent=1)
    json.dump(summary, open(os.path.join(cfg.detector_dir, "summary.json"), "w"), indent=1)


if __name__ == "__main__":
    calibrate()
