"""
dmp.py

Joint-space dynamic movement primitives for mire (recovery.Mire): one demonstration (a joint trajectory, one row per
env step) is encoded per joint and reproduced from a new start towards a new goal, with an optional coupling term
(e.g. obstacle repulsion) added to the acceleration. The formulation is that of Hoffmann et al. 2009 / Pastor et al.
2009 (the forcing term is not scaled by goal - start, so the shape neither flips nor blows up when the new start and
goal differ from the demonstration's):

    tau * dv = K (g - x) - D v - K (g - x0) s + K f(s),    tau * dx = v,    tau * ds = -ALPHA_S s

with f a normalized Gaussian basis expansion in the phase s, fitted to the demonstration by least squares.
"""

import numpy as np

K, ALPHA_S, N_BASIS, SUBSTEPS = 150.0, 4.0, 25, 10
D = 2.0 * np.sqrt(K)  # critically damped


class JointDMP:
    def __init__(self, demo):
        """demo: (T + 1, n) joint positions at consecutive env steps (T >= 2)."""
        y = np.asarray(demo, dtype=float)
        self.T = len(y) - 1
        self.tau = float(self.T)  # in env steps
        self.y0, self.g0 = y[0].copy(), y[-1].copy()
        self.c = np.exp(-ALPHA_S * np.linspace(0, 1, N_BASIS))  # basis centers in s
        self.h = 1.0 / np.diff(self.c) ** 2
        self.h = np.append(self.h, self.h[-1])
        t = np.arange(len(y), dtype=float)
        s = np.exp(-ALPHA_S * t / self.tau)
        yd = np.gradient(y, axis=0) * self.tau  # v = tau * dx/dt
        ydd = np.gradient(yd, axis=0)  # tau * d2x/dt2, so tau * dv/dt = tau * ydd
        f = (self.tau * ydd + D * yd) / K - (self.g0 - y) + (self.g0 - self.y0) * s[:, None]
        Phi = self._features(s)
        self.w = np.linalg.lstsq(Phi, f, rcond=None)[0]  # (N_BASIS, n)

    def _features(self, s):
        psi = np.exp(-self.h * (np.atleast_1d(s)[:, None] - self.c) ** 2)
        return psi * np.atleast_1d(s)[:, None] / psi.sum(1, keepdims=True)

    def rollout(self, x0, g, tau=None, coupling=None, extra=0.5, tol=1e-3):
        """Joint positions at every env step from x0 (excluded) towards g: tau steps (default the demonstration's)
        plus up to `extra` * tau to settle (stops once within `tol` of g, at rest). coupling(x, v) -> an acceleration
        term (tau * dv units), evaluated once per env step and held over its substeps."""
        tau = float(tau or self.tau)
        x, v, s = np.array(x0, dtype=float), np.zeros(len(x0)), 1.0
        g, x0 = np.asarray(g, dtype=float), np.array(x0, dtype=float)
        dt = 1.0 / SUBSTEPS
        out = []
        for _ in range(int(np.ceil(tau * (1 + extra)))):
            c = 0.0 if coupling is None else coupling(x, v)
            for _ in range(SUBSTEPS):
                f = (self._features(s) @ self.w)[0]
                a = K * (g - x) - D * v - K * (g - x0) * s + K * f + c
                v = v + a / tau * dt
                x = x + v / tau * dt
                s = s - ALPHA_S * s / tau * dt
            out.append(x.copy())
            if s < 0.02 and np.linalg.norm(g - x) < tol and np.linalg.norm(v) / tau < tol:
                break
        return out


MAX_SLOWDOWN = 4.0  # limited_rollout gives up rather than stretch the movement beyond this factor of tau


def limited_rollout(dmp, x0, g, max_step, coupling=None, lo=None, hi=None, tau=None):
    """dmp.rollout from x0 to g slowed down (tau scaled up, at most MAX_SLOWDOWN times) until no step moves more than
    max_step (L2), within the joint limits lo / hi, ending exactly at g (a straight joint line at most max_step per
    step closes what is left when the DMP has not settled: a demonstration that does not end at rest leaves its
    forcing term pushing at the end); (None, tau) if that takes more. coupling(tau) -> the coupling function for that
    tau, or None. tau: the starting duration (default the demo's)."""
    tau0 = tau = float(tau or dmp.tau)
    g = np.array(g, dtype=float)
    for _ in range(4):
        qs = dmp.rollout(x0, g, tau=tau, coupling=None if coupling is None else coupling(tau))
        if lo is not None:
            qs = [np.clip(q, lo, hi) for q in qs]
        end = qs[-1] if qs else np.array(x0, dtype=float)
        n = max(1, int(np.ceil(np.linalg.norm(g - end) / max_step)))
        qs += [end + (g - end) * k / n for k in range(1, n + 1)]
        fastest = max(np.linalg.norm(np.diff(np.vstack([x0] + qs), axis=0), axis=1))
        if fastest <= max_step:
            return qs, tau
        tau *= fastest / max_step * 1.05
        if tau > MAX_SLOWDOWN * tau0:
            break
    return None, tau
