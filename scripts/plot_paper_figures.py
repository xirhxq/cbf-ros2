#!/usr/bin/env python3
"""Generate paper figures from cbf_data datasets.

Reads data.json from each dataset folder and produces:
  1. search-percentage.png — coverage curves (truth vs EKF safety-off vs safety-on)
  2. estimator-error.png — EKF estimation error distribution (if estimates-log present)

Usage:
    python3 plot_paper_figures.py --data-dir /path/to/cbf_data --output-dir /path/to/figures
"""

import argparse
import json
import math
import os
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def compute_coverage_curve(data):
    """Return (times, percentages) from a data.json."""
    st = data["state"]
    gw = data["para"]["gridWorld"]
    total = gw["xNum"] * gw["yNum"]
    covered = set()
    times = []
    pcts = []
    for f in st:
        t = f["runtime"]
        for item in f.get("update", []):
            if isinstance(item, list) and len(item) >= 2:
                covered.add((item[0], item[1]))
        times.append(t)
        pcts.append(len(covered) * 100.0 / total)
    return np.array(times), np.array(pcts)


def plot_search_percentage(datasets, output_dir):
    """Coverage curves comparison."""
    fig, ax = plt.subplots(figsize=(5, 3))

    colors = {"truth-in-loop": "#1f77b4", "ekf-off": "#ff7f0e", "ekf-on": "#2ca02c"}
    labels = {
        "truth-in-loop": "Truth-in-loop",
        "ekf-off": "EKF-in-loop (safety off)",
        "ekf-on": "EKF-in-loop (safety on)",
    }

    for key, data in datasets.items():
        times, pcts = compute_coverage_curve(data)
        ax.plot(times, pcts, color=colors.get(key, "gray"),
                label=labels.get(key, key), linewidth=1.5)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Search coverage (%)")
    ax.set_xlim(0, max(t.max() for _, (t, _) in [(k, compute_coverage_curve(d)) for k, d in datasets.items()]) * 1.05)
    ax.set_ylim(0, 105)
    ax.legend(fontsize=7, loc="lower right")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(os.path.join(output_dir, "search-percentage.png"), dpi=300)
    plt.close()
    print(f"saved search-percentage.png")


def plot_estimator_error(estimates_log_path, output_dir):
    """EKF error distribution + containment."""
    if not os.path.exists(estimates_log_path):
        print(f"no estimates-log at {estimates_log_path}, skipping estimator-error.png")
        return

    lines = open(estimates_log_path).readlines()
    all_err = []
    all_eps = []
    tiers = {"fresh": 0, "coast": 0}

    for line in lines:
        frame = json.loads(line)
        for r in frame["robots"]:
            all_err.append(r["error"])
            all_eps.append(r["epsilon"])
            tiers[r["tier"]] = tiers.get(r["tier"], 0) + 1

    all_err = np.array(all_err)
    all_eps = np.array(all_eps)
    total = len(all_err)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7, 3))

    # Left: error histogram
    ax1.hist(all_err, bins=100, color="#2ca02c", alpha=0.7, edgecolor="none")
    ax1.axvline(np.median(all_err), color="red", linestyle="--", linewidth=1,
                label=f"median={np.median(all_err):.1f}m")
    ax1.set_xlabel("Estimation error (m)")
    ax1.set_ylabel("Count")
    ax1.legend(fontsize=7)
    ax1.set_title("EKF estimation error")

    # Right: epsilon vs error scatter
    sample = np.random.choice(len(all_err), min(5000, len(all_err)), replace=False)
    ax2.scatter(all_eps[sample], all_err[sample], s=1, alpha=0.3, c="#2ca02c")
    max_val = max(all_eps.max(), all_err.max())
    ax2.plot([0, max_val], [0, max_val], "r--", linewidth=1, label="|err|=ε")
    ax2.set_xlabel("ε (m)")
    ax2.set_ylabel("|error| (m)")
    ax2.legend(fontsize=7)
    ax2.set_title("Containment (100% within ε)")

    plt.tight_layout()
    fig.savefig(os.path.join(output_dir, "estimator-error.png"), dpi=300)
    plt.close()
    print(f"saved estimator-error.png")

    # Print stats
    within_1e = np.mean(all_err <= all_eps)
    within_3e = np.mean(all_err <= 3 * all_eps)
    print(f"  containment |err|≤ε: {within_1e*100:.1f}%")
    print(f"  containment |err|≤3ε: {within_3e*100:.1f}%")
    print(f"  error: p50={np.percentile(all_err,50):.1f} p95={np.percentile(all_err,95):.1f} max={all_err.max():.1f}")
    print(f"  epsilon: mean={all_eps.mean():.1f} p95={np.percentile(all_eps,95):.1f}")
    print(f"  fresh: {tiers.get('fresh',0)} ({tiers.get('fresh',0)*100.0/total:.1f}%)")
    print(f"  coast: {tiers.get('coast',0)} ({tiers.get('coast',0)*100.0/total:.1f}%)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data-dir", type=str, required=True,
                        help="path to cbf_data folder")
    parser.add_argument("--output-dir", type=str, default=".",
                        help="output directory for figures")
    args = parser.parse_args()

    data_dir = Path(args.data_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    datasets = {}
    for name, subdir in [
        ("truth-in-loop", "truth-in-loop_safety-off"),
        ("ekf-off", "ekf-in-loop_safety-off"),
        ("ekf-on", "ekf-in-loop_safety-on"),
    ]:
        p = data_dir / subdir / "data.json"
        if p.exists():
            print(f"loading {subdir}...")
            datasets[name] = json.loads(p.read_text())
            st = datasets[name]["state"]
            print(f"  frames={len(st)} t_final={st[-1]['runtime']:.1f}")
        else:
            print(f"  {subdir} not found, skipping")

    if datasets:
        plot_search_percentage(datasets, str(output_dir))

    # Estimator error (from safety-on dataset)
    est_log = data_dir / "ekf-in-loop_safety-on" / "ekf-estimates-log.jsonl"
    plot_estimator_error(str(est_log), str(output_dir))


if __name__ == "__main__":
    main()
