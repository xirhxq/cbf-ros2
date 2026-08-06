#!/usr/bin/env python3
"""Offline EKF estimator replay on a recorded truth trajectory (data.json).

This is the *self-contained* counterpart to the route1h estimator-in-loop
pipeline. It reuses the pure-numpy basic EKF
(``ekf_estimator_service.EKFInLoopService``) from the ``paper/cbf2026-tvt``
branch (kept under ``.ref/``) and runs it against any recorded ``data.json``
truth trajectory, with no simulator, no ROS 2, and no Ignition.

What it produces (per robot and pooled):
- availability: fresh / coast tier counts (hold never triggers in basic EKF);
- containment: fraction of frames with |err| <= eps and |err| <= 3*eps;
- NEES: normalized estimation error squared err^T Sigma^-1 err (2-DoF chi2),
  with the expected p95 / p99 gate and the empirical exceedance rate;
- error magnitude distribution (p50/p95/max) and epsilon distribution.

Usage::

    python3 scripts/replay_estimator_offline.py \\
        --data .data/2026-03-07_00-42-23/data.json \\
        --output-root /tmp/estimator-offline \\
        [--frames N] [--seed 2026081301] [--plot]

Parameters mirror the estimator spec in the 2026-08-05 handoff
(see ``papers/cbf2026/handoff/2026-08-05-mbzirc-estimator-reproduction.md``).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

# The basic EKF lives in the read-only reference copy of paper/cbf2026-tvt,
# kept under <repo-root>/.ref (gitignored). This script lives in
# <repo-root>/.work/scripts, so walk up from here to locate <repo-root>.
_HERE = Path(__file__).resolve().parent
_REPO_ROOT = _HERE
for _candidate in (_HERE, *_HERE.parents):
    if (_candidate / ".ref" / "paper-cbf2026-tvt").is_dir():
        _REPO_ROOT = _candidate
        break
_REF = _REPO_ROOT / ".ref" / "paper-cbf2026-tvt"
if str(_REF) not in sys.path:
    sys.path.insert(0, str(_REF))

from scripts.diagnostics.ekf_estimator_service import (  # noqa: E402
    EKFInLoopService,
    build_ekf_raw_references,
)


def load_data(path: Path) -> dict:
    return json.loads(path.read_text())


def load_bases(data: dict) -> list[list[float]]:
    # Same fallback rule as replay_r1h_estimator._load_materialized_bases.
    for key in ("bases", "base-stations", "baseStations"):
        if key in data.get("config", {}):
            return data["config"][key]
    return [[-1550.0, -300.0], [-1550.0, 0.0], [-1550.0, 300.0]]


def frame_truth(frame: dict) -> dict[int, np.ndarray]:
    return {
        robot["id"]: np.asarray(
            [robot["state"]["x"], robot["state"]["y"]], dtype=float
        )
        for robot in frame["robots"]
    }


def frame_commands(prev_frame: dict) -> dict[int, list[float]]:
    """Held planar command applied during the interval ending at this frame.

    Mirrors replay_r1h_estimator.build_held_commands: the command recorded in
    frame k-1's opt.result is the one that propagated the state into frame k.
    """
    commands: dict[int, list[float]] = {}
    for robot in prev_frame["robots"]:
        result = robot.get("opt", {}).get("result", {})
        commands[robot["id"]] = [
            float(result.get("vx", 0.0)),
            float(result.get("vy", 0.0)),
        ]
    return commands


def chi2_inv_cdf(p: float, dof: int = 2) -> float:
    """Inverse CDF of the chi-square distribution via the incomplete gamma.

    Uses numpy + the series form of the lower regularized gamma; sufficient
    for dof=2 (where it reduces to 1 - exp(-x/2)).
    """
    if dof == 2:
        return -2.0 * np.log1p(-p)
    # Generic fallback via scipy if available, else approximate.
    try:
        from scipy.stats import chi2  # type: ignore

        return float(chi2.ppf(p, dof))
    except Exception:
        # Wilson–Hilferty approximation.
        z = {
            0.95: 1.6448536269514722,
            0.99: 2.3263478740408408,
        }.get(p, 1.6448536269514722)
        return float(dof * (1.0 - 2.0 / (9.0 * dof) + z * np.sqrt(2.0 / (9.0 * dof))) ** 3)


def run(
    data: dict,
    *,
    frames: int,
    seed: int,
    sigma0: float,
    range0: float,
    availability_range0: float,
    process_noise_mps: float,
    p0_std: float,
    dt: float,
    innovation_gate: float,
    anchor_covariance_scale: float,
) -> dict:
    states = data["state"]
    frames = min(frames, len(states))
    bases = load_bases(data)
    robot_ids = [robot["id"] for robot in states[0]["robots"]]
    deployment = {
        rid: frame_truth(states[0])[rid].tolist() for rid in robot_ids
    }

    service = EKFInLoopService(
        deployment_positions=deployment,
        bases=bases,
        sigma0=sigma0,
        range0=range0,
        process_noise_mps=process_noise_mps,
        p0_std=p0_std,
        dt=dt,
        innovation_gate=float(innovation_gate),
        anchor_covariance_scale=anchor_covariance_scale,
    )
    rng = np.random.default_rng(seed)

    # Per-robot logs.
    logs: dict[int, dict[str, list]] = {
        rid: {"err": [], "eps": [], "tier": [], "nees": [], "updates": []}
        for rid in robot_ids
    }

    for frame_index in range(frames):
        frame = states[frame_index]
        truth = frame_truth(frame)
        held = (
            {rid: [0.0, 0.0] for rid in robot_ids}
            if frame_index == 0
            else frame_commands(states[frame_index - 1])
        )
        frame_like = {
            "robots": [
                {
                    "id": robot["id"],
                    "state": {
                        "x": robot["state"]["x"],
                        "y": robot["state"]["y"],
                    },
                }
                for robot in frame["robots"]
            ],
            "formation": frame.get("formation", []),
            "covariance_formation": frame.get("covariance_formation", []),
        }
        references = build_ekf_raw_references(
            frame_like,
            bases,
            rng,
            sigma0=sigma0,
            range0=range0,
            availability_range0=availability_range0,
        )
        outputs = service.step(
            frame_index=frame_index,
            raw_reference_groups=references,
            held_commands=held,
        )
        for rid in robot_ids:
            out = outputs[rid]
            err = truth[rid] - np.asarray(out["estimate"], dtype=float)
            logs[rid]["err"].append(float(np.linalg.norm(err)))
            logs[rid]["eps"].append(float(out["epsilon"]))
            logs[rid]["tier"].append(out["tier"])
            logs[rid]["updates"].append(int(out["updates"]))
            # NEES needs the filter covariance; reconstruct it from epsilon.
            # epsilon = 3*sqrt(lambda_max)  =>  lambda_max = (eps/3)^2.
            # For a 2D isotropic-ish bound we use lambda_max as the larger
            # eigenvalue and back out NEES with the conservative (isotropic)
            # Sigma = lambda_max * I so the metric stays comparable across
            # frames; this is the same convention used by the route1h
            # containment reporting (eps-bound, not full ellipsoid).
            lam = (float(out["epsilon"]) / 3.0) ** 2
            sigma_iso = max(lam, 1e-12) * np.eye(2)
            nees = float(err @ np.linalg.inv(sigma_iso) @ err)
            logs[rid]["nees"].append(nees)

    return summarize(logs, frames=frames, robot_ids=robot_ids)


def summarize(logs: dict, *, frames: int, robot_ids: list[int]) -> dict:
    pooled_err = []
    pooled_eps = []
    pooled_nees = []
    tier_counts = {"fresh": 0, "coast": 0, "hold": 0}
    per_robot = {}

    for rid in robot_ids:
        err = np.asarray(logs[rid]["err"])
        eps = np.asarray(logs[rid]["eps"])
        nees = np.asarray(logs[rid]["nees"])
        tiers = logs[rid]["tier"]
        for tier in tiers:
            tier_counts[tier] = tier_counts.get(tier, 0) + 1
        within_1e = float(np.mean(err <= eps)) if len(err) else 0.0
        within_3e = float(np.mean(err <= 3.0 * eps)) if len(err) else 0.0
        per_robot[rid] = {
            "err_p50": float(np.percentile(err, 50)) if len(err) else 0.0,
            "err_p95": float(np.percentile(err, 95)) if len(err) else 0.0,
            "err_max": float(np.max(err)) if len(err) else 0.0,
            "eps_mean": float(np.mean(eps)) if len(eps) else 0.0,
            "eps_p95": float(np.percentile(eps, 95)) if len(eps) else 0.0,
            "eps_max": float(np.max(eps)) if len(eps) else 0.0,
            "containment_1e": within_1e,
            "containment_3e": within_3e,
            "nees_mean": float(np.mean(nees)) if len(nees) else 0.0,
            "nees_p95": float(np.percentile(nees, 95)) if len(nees) else 0.0,
            "fresh": int(sum(1 for t in tiers if t == "fresh")),
            "coast": int(sum(1 for t in tiers if t == "coast")),
        }
        pooled_err.append(err)
        pooled_eps.append(eps)
        pooled_nees.append(nees)

    all_err = np.concatenate(pooled_err)
    all_eps = np.concatenate(pooled_eps)
    all_nees = np.concatenate(pooled_nees)
    total = int(sum(tier_counts.values()))

    # NEES reference gates for a 2-DoF chi2.
    gate_95 = chi2_inv_cdf(0.95, dof=2)
    gate_99 = chi2_inv_cdf(0.99, dof=2)

    return {
        "frames": frames,
        "robot_count": len(robot_ids),
        "samples": int(len(all_err)),
        "availability": {
            "fresh": tier_counts.get("fresh", 0),
            "coast": tier_counts.get("coast", 0),
            "hold": tier_counts.get("hold", 0),
            "fresh_rate": tier_counts.get("fresh", 0) / total if total else 0.0,
            "availability_rate": (
                (tier_counts.get("fresh", 0) + tier_counts.get("coast", 0) + tier_counts.get("hold", 0))
                / total
                if total
                else 0.0
            ),
        },
        "containment": {
            "within_1e": float(np.mean(all_err <= all_eps)),
            "within_3e": float(np.mean(all_err <= 3.0 * all_eps)),
        },
        "error_m": {
            "p50": float(np.percentile(all_err, 50)),
            "p95": float(np.percentile(all_err, 95)),
            "max": float(np.max(all_err)),
        },
        "epsilon_m": {
            "mean": float(np.mean(all_eps)),
            "p95": float(np.percentile(all_eps, 95)),
            "max": float(np.max(all_eps)),
        },
        "nees": {
            "mean": float(np.mean(all_nees)),
            "p95": float(np.percentile(all_nees, 95)),
            "exceed_95_rate": float(np.mean(all_nees > gate_95)),
            "exceed_99_rate": float(np.mean(all_nees > gate_99)),
            "chi2_gate_95": gate_95,
            "chi2_gate_99": gate_99,
        },
        "per_robot": {str(rid): per_robot[rid] for rid in robot_ids},
    }


def maybe_plot(logs_summary: dict, data: dict, frames: int, out: Path) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        print("[plot] matplotlib unavailable, skipping")
        return

    states = data["state"][:frames]
    t = np.asarray([s.get("runtime", i * 0.5) for i, s in enumerate(states)])
    robot_ids = sorted(int(k) for k in logs_summary["per_robot"])

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    cmap = plt.get_cmap("tab20")
    for idx, rid in enumerate(robot_ids):
        pr = logs_summary["per_robot"][str(rid)]
        axes[0].plot([], [], color=cmap(idx % 20), label=f"u{rid}")
    axes[0].set_title("per-robot containment |err|<=eps (placeholder)")
    axes[0].legend(ncol=7, fontsize=7)

    pooled_err = []
    pooled_eps = []
    for s in states:
        for robot in s["robots"]:
            pooled_err.append(0.0)
            pooled_eps.append(0.0)
    plt.tight_layout()
    fig.savefig(out, dpi=120)
    print(f"[plot] saved {out}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Offline EKF estimator replay on a recorded data.json trajectory"
    )
    parser.add_argument("--data", type=Path, required=True, help="path to data.json")
    parser.add_argument(
        "--output-root", type=Path, default=Path("/tmp/estimator-offline")
    )
    parser.add_argument("--frames", type=int, default=0, help="0 = all frames")
    parser.add_argument("--seed", type=int, default=2026081301)
    parser.add_argument("--sigma0", type=float, default=0.5)
    parser.add_argument("--range0", type=float, default=850.0)
    parser.add_argument("--availability-range0", type=float, default=850.0)
    parser.add_argument("--process-noise-mps", type=float, default=1.0)
    parser.add_argument("--p0-std", type=float, default=1.0)
    parser.add_argument("--dt", type=float, default=0.5)
    parser.add_argument("--innovation-gate", type=float, default=3.0)
    parser.add_argument("--anchor-covariance-scale", type=float, default=3.0)
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args()

    data = load_data(args.data)
    frames = args.frames or len(data["state"])

    print(
        f"replaying {frames} frames x {len(data['state'][0]['robots'])} robots"
        f" from {args.data}"
    )
    summary = run(
        data,
        frames=frames,
        seed=args.seed,
        sigma0=args.sigma0,
        range0=args.range0,
        availability_range0=args.availability_range0,
        process_noise_mps=args.process_noise_mps,
        p0_std=args.p0_std,
        dt=args.dt,
        innovation_gate=args.innovation_gate,
        anchor_covariance_scale=args.anchor_covariance_scale,
    )

    args.output_root.mkdir(parents=True, exist_ok=True)
    out_json = args.output_root / "summary.json"
    out_json.write_text(json.dumps(summary, indent=1) + "\n")

    print("\n=== pooled summary ===")
    print(f"frames={summary['frames']} robots={summary['robot_count']} samples={summary['samples']}")
    av = summary["availability"]
    print(
        f"availability: fresh={av['fresh']} coast={av['coast']} hold={av['hold']}"
        f"  fresh_rate={av['fresh_rate']:.4f} availability={av['availability_rate']:.4f}"
    )
    ct = summary["containment"]
    print(f"containment: |err|<=eps={ct['within_1e']:.4f}  |err|<=3eps={ct['within_3e']:.4f}")
    er = summary["error_m"]
    print(f"error  (m): p50={er['p50']:.3f} p95={er['p95']:.3f} max={er['max']:.3f}")
    ep = summary["epsilon_m"]
    print(f"epsilon (m): mean={ep['mean']:.3f} p95={ep['p95']:.3f} max={ep['max']:.3f}")
    ns = summary["nees"]
    print(
        f"NEES: mean={ns['mean']:.3f} p95={ns['p95']:.3f}"
        f"  exceed95={ns['exceed_95_rate']:.4f} exceed99={ns['exceed_99_rate']:.4f}"
        f"  (chi2 gates 95={ns['chi2_gate_95']:.3f} 99={ns['chi2_gate_99']:.3f})"
    )
    print(f"\nfull summary -> {out_json}")

    if args.plot:
        maybe_plot(summary, data, frames, args.output_root / "offline-estimator.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
