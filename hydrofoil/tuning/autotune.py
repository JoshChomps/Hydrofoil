"""
Autotuner: searches controller settings that minimise `scoring.cost` over the standard scenarios.

    python -m hydrofoil.tuning.autotune --preset manned --target pid  [--overrides file.json] [--generations 8]
    python -m hydrofoil.tuning.autotune --preset manned --target lqr

target "pid": the eight PID gains (heave kp ki kd, pitch kp ki kd, roll kp kd), flown on the PID alone.
target "lqr": multipliers on the LQR's Bryson weights (height, pitch, roll, heave rate, pitch rate, height integral,
              command rate), flown on the LQR (PID takeoff), so the gains are re-derived for each candidate.

Method: a staged loop (see `tune`): the current values, then classical methods (relay test + Ziegler-Nichols /
Tyreus-Luyben for the PID; Bryson's rule + iterative re-weighting for the LQR), then a cross-entropy search seeded
at the best of those. Each generation samples `--pop` candidates around the current mean,
scores all of them (scenarios in parallel on a process pool), and moves the mean and spread towards the best
quarter. Simple, derivative-free, robust to the noisy, discontinuous cost (failures add 100). Progress and the
best result so far are written to `--progress` (JSON) after every generation, so a UI can follow it.
"""

from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import time
from pathlib import Path

import numpy as np

from hydrofoil.tuning.scoring import scenarios_for, run_scenario, cost

PID_KEYS = ["kp_heave", "ki_heave", "kd_heave", "kp_pitch", "ki_pitch", "kd_pitch", "kp_roll", "kd_roll"]
LQR_KEYS = ["z", "theta", "phi", "w", "q", "iz", "du"]          # multipliers on control.lqr.DEFAULT_WEIGHTS


def base_params(preset: str, overrides: dict | None = None) -> dict:
    from hydrofoil.params import DEFAULT_PARAMS, MANNED_PARAMS
    from hydrofoil.design import apply_design
    p = dict(MANNED_PARAMS if preset == "manned" else DEFAULT_PARAMS)
    return apply_design(p, overrides or {})


def candidate_params(base: dict, target: str, x: np.ndarray) -> dict:
    """Parameter set for log-space vector x (relative to the base values / default weights)."""
    from hydrofoil.control.lqr import DEFAULT_WEIGHTS
    p = dict(base)
    if target == "pid":
        p["controller"] = "pid"
        for k, v in zip(PID_KEYS, x):
            p[k] = float(np.exp(v))
    else:
        p["controller"] = "lqr"
        w = dict(DEFAULT_WEIGHTS, **(base.get("lqr_weights") or {}))
        mult = dict(zip(LQR_KEYS, np.exp(x)))
        # Bryson weight = acceptable excursion: a larger multiplier means LESS weight on that state
        for k in ("z", "theta", "phi", "w", "q", "iz"):
            w[k] = w[k] * mult[k]
        for k in ("du_main", "du_elev", "du_roll"):
            w[k] = w[k] * mult["du"]
        p["lqr_weights"] = w
    return p


def start_vector(base: dict, target: str) -> np.ndarray:
    if target == "pid":
        return np.log(np.array([max(base[k], 1e-3) for k in PID_KEYS]))
    return np.zeros(len(LQR_KEYS))


def tune(preset: str, target: str, overrides: dict | None = None, generations: int = 6, pop: int = 10,
         sigma: float = 0.35, progress: Path | None = None, workers: int | None = None, seed: int = 0,
         bryson_rounds: int = 4) -> dict:
    """Staged tuning loop:
        1. baseline     the current values
        2. classical    PID: relay test -> Ziegler-Nichols / Tyreus-Luyben rule sets (classical.py)
                        LQR: Bryson's rule, then `bryson_rounds` of iterative re-weighting
        3. refine       cross-entropy search seeded at the best of 1-2 (Rubinstein 1999)
    Every candidate is scored on the same scenarios; progress goes to `progress` after each step."""
    from hydrofoil.tuning.classical import pid_candidates, bryson_update
    base = base_params(preset, overrides)
    scen = scenarios_for(base)                        # + the supervised takeoff where the supervisor applies
    rng = np.random.default_rng(seed)
    workers = workers or max(1, (mp.cpu_count() or 2) - 1)
    stages, hist = [], []
    t0 = time.time()
    state = {"best": None, "status": "running", "relay": None}

    def write():
        if progress is None:
            return
        b = state["best"]
        progress.write_text(json.dumps({
            "status": state["status"], "preset": preset, "target": target, "elapsed_s": round(time.time() - t0, 1),
            "stages": stages, "relay": state["relay"], "history": hist, "generations": generations,
            "best": b and {"cost": b["cost"], "values": b["values"], "metrics": b["metrics"], "source": b["source"]}},
            indent=1, default=float))

    with mp.get_context("spawn").Pool(workers) as pool:
        def evaluate(xs):
            params = [candidate_params(base, target, x) for x in xs]
            jobs = [(pp, n) for pp in params for n in scen]
            res = pool.starmap(run_scenario, jobs)
            out = []
            for i, pp in enumerate(params):
                mets = dict(zip(scen, res[i * len(scen):(i + 1) * len(scen)]))
                out.append((cost(mets, pp["flying_height"]), mets, pp))
            return out

        def consider(x, c, mets, pp, source):
            stages.append({"name": source, "cost": c, "values": _values(pp, target), "metrics": mets})
            if state["best"] is None or c < state["best"]["cost"]:
                state["best"] = {"cost": c, "x": np.array(x, float), "metrics": mets, "values": _values(pp, target),
                                 "source": source}
            write()

        write()
        x0 = start_vector(base, target)
        (c, m, pp), = evaluate([x0])
        consider(x0, c, m, pp, "current values")
        baseline = {"cost": c, "values": _values(pp, target), "metrics": m}

        if target == "pid":
            cands = pid_candidates(base)
            state["relay"] = cands.pop("_relay")
            names = list(cands)
            xs = [np.log(np.array([max(cands[n].get(k, base[k]), 1e-4) for k in PID_KEYS])) for n in names]
            for n, x, (c, m, pp) in zip(names, xs, evaluate(xs)):
                consider(x, c, m, pp, n)
        else:
            mult = dict.fromkeys(LQR_KEYS, 1.0)
            last_m = m
            for r in range(bryson_rounds):
                mult = bryson_update(mult, last_m, base["flying_height"])
                x = np.log(np.array([mult[k] for k in LQR_KEYS]))
                (c, last_m, pp), = evaluate([x])
                consider(x, c, last_m, pp, f"Bryson re-weighting, round {r + 1}")

        mean = state["best"]["x"].copy()
        sd = np.full_like(mean, sigma)
        for g in range(generations):
            xs = [mean + sd * rng.standard_normal(len(mean)) for _ in range(pop - 1)] + [mean]
            res = evaluate(xs)
            order = np.argsort([r[0] for r in res])
            elite = [xs[i] for i in order[:max(2, pop // 4)]]
            mean = 0.7 * np.mean(elite, axis=0) + 0.3 * mean
            sd = np.maximum(0.7 * np.std(elite, axis=0) + 0.3 * sd, 0.03)
            c, mets, pp = res[order[0]]
            if c < state["best"]["cost"]:
                state["best"] = {"cost": c, "x": xs[order[0]].copy(), "metrics": mets, "values": _values(pp, target),
                                 "source": f"search, round {g + 1}"}
            hist.append({"generation": g + 1, "best_cost": state["best"]["cost"], "gen_best": c,
                         "gen_median": float(np.median([r[0] for r in res]))})
            write()
    state["status"] = "done"
    write()
    b = state["best"]
    return {"baseline": baseline, "best": {"cost": b["cost"], "values": b["values"], "metrics": b["metrics"],
                                           "source": b["source"]}, "stages": stages, "relay": state["relay"]}


def _values(p: dict, target: str) -> dict:
    if target == "pid":
        return {k: round(float(p[k]), 4) for k in PID_KEYS}
    return {"lqr_weights": {k: float(v) for k, v in p["lqr_weights"].items()}}


def main():
    ap = argparse.ArgumentParser(description="Autotune the flight controller over the standard scenarios")
    ap.add_argument("--preset", choices=("model", "manned"), default="manned")
    ap.add_argument("--target", choices=("pid", "lqr"), default="pid")
    ap.add_argument("--overrides", type=Path, help="JSON design overrides (boat editor)")
    ap.add_argument("--generations", type=int, default=6)
    ap.add_argument("--pop", type=int, default=10)
    ap.add_argument("--progress", type=Path, help="JSON progress file, rewritten every generation")
    ap.add_argument("--out", type=Path, help="write the best values here (JSON)")
    a = ap.parse_args()
    ov = json.loads(a.overrides.read_text()) if a.overrides and a.overrides.exists() else None
    res = tune(a.preset, a.target, ov, a.generations, a.pop, progress=a.progress)
    b0, b1 = res["baseline"], res["best"]
    print(f"cost {b0['cost']:.2f} -> {b1['cost']:.2f} (best from: {b1['source']})")
    for st in res["stages"]:
        print(f"  {st['name']:34s} cost {st['cost']:7.2f}")
    print(json.dumps(b1["values"], indent=1))
    for n in b0["metrics"]:
        print(n, "before", {k: round(v, 3) if isinstance(v, float) else v for k, v in b0["metrics"][n].items()})
        print(n, "after ", {k: round(v, 3) if isinstance(v, float) else v for k, v in b1["metrics"][n].items()})
    if a.out:
        a.out.write_text(json.dumps(b1["values"], indent=1))


if __name__ == "__main__":
    main()
