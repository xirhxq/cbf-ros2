#!/usr/bin/env python3
"""Resident EKF estimator service for the MBZIRC estimator-in-the-loop run.

This is the file-IPC counterpart to the route1h ``run_r1h_ei.py`` driver, but
the simulator is launched separately (ROS 2 ``suav``) instead of as a
subprocess. The Swarm C++ side writes ``state.json`` + ``state.json.ready``
each control frame; this service consumes it, runs the basic EKF for all 14
sUAVs, and writes ``estimates.json`` back atomically.

Parameters mirror the estimator spec in
``papers/cbf2026/handoff/2026-08-05-mbzirc-estimator-reproduction.md`` §3.1
and the paper main.tex §VI (sigma0=0.5, d0=850, kappa=3, q=process-noise
speed, p0=1.0, dt=0.5, 3sigma gate, epsilon=3*sqrt(lambda_max)).

Usage (inside the cbf-ros2 container, started by run_headless_full.sh or
manually before suav reaches PERFORM)::

    python3 src/cbf-ros2/scripts/ekf_service_mbzirc.py \\
        --state-path /dev/shm/estimator/state.json \\
        --estimates-path /dev/shm/estimator/estimates.json \\
        --bases '[[-1550,-300],[-1550,0],[-1550,300]]'

Outputs:
- ``estimates.json`` each frame (consumed & deleted by Swarm)
- ``--estimates-log`` JSONL (one payload per frame, for offline analysis)
- ``--timing-json`` per-frame solve seconds
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

# Reuse the pure-numpy basic EKF from the paper/cbf2026-tvt reference copy.
_HERE = Path(__file__).resolve().parent
_REPO_ROOT = _HERE
for _cand in (_HERE, *_HERE.parents):
    if (_cand / ".ref" / "paper-cbf2026-tvt").is_dir():
        _REPO_ROOT = _cand
        break
_REF = _REPO_ROOT / ".ref" / "paper-cbf2026-tvt"
if str(_REF) not in sys.path:
    sys.path.insert(0, str(_REF))

from scripts.diagnostics.ekf_estimator_service import (  # noqa: E402
    EKFInLoopService,
    build_ekf_raw_references,
)


def _wait_for(path: Path, timeout_s: float) -> bool:
    """Return True if ``path`` appears within timeout, else False."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path.exists():
            return True
        time.sleep(0.005)
    return False


def main() -> int:
    p = argparse.ArgumentParser(
        description="Resident EKF service for MBZIRC estimator-in-the-loop"
    )
    p.add_argument("--state-path", type=Path, required=True,
                   help="path Swarm writes state.json + .ready")
    p.add_argument("--estimates-path", type=Path, required=True,
                   help="path to write estimates.json (atomic replace)")
    p.add_argument("--bases", type=str, default="auto",
                   help="JSON list of base [x,y] or 'auto' (3 mission bases)")
    p.add_argument("--estimates-log", type=Path,
                   default=Path("/tmp/estimator-estimates.jsonl"))
    p.add_argument("--timing-json", type=Path,
                   default=Path("/tmp/estimator-timing.json"))
    # Estimator spec (handoff §3.1 / paper §VI)
    p.add_argument("--ranging-sigma0", type=float, default=0.5)
    p.add_argument("--range0", type=float, default=850.0)
    p.add_argument("--availability-range0", type=float, default=850.0)
    p.add_argument("--process-noise-mps", type=float, default=1.0,
                   help="process-noise speed q (MBZIRC高速轨迹可能需校准)")
    p.add_argument("--p0-std", type=float, default=1.0)
    p.add_argument("--dt", type=float, default=0.5)
    p.add_argument("--innovation-gate", type=float, default=3.0)
    p.add_argument("--anchor-covariance-scale", type=float, default=3.0)
    p.add_argument("--idle-timeout-s", type=float, default=30.0,
                   help="if no state.json.ready appears for this long, exit")
    p.add_argument("--max-frames", type=int, default=0,
                   help="0 = run until idle timeout")
    args = p.parse_args()

    state_path = args.state_path
    state_ready = Path(str(state_path) + ".ready")
    estimates_path = args.estimates_path
    state_path.parent.mkdir(parents=True, exist_ok=True)
    # Clean any stale IPC files from a previous run.
    for f in (state_ready, estimates_path, state_path):
        try:
            f.unlink()
        except FileNotFoundError:
            pass

    bases = (
        [[-1550.0, -300.0], [-1550.0, 0.0], [-1550.0, 300.0]]
        if args.bases == "auto"
        else json.loads(args.bases)
    )

    # Deployment positions are learned from the first frame (frame 0 truth).
    service: EKFInLoopService | None = None
    rng = np.random.default_rng(2026081301)
    estimates_log_handle = args.estimates_log.open("a", buffering=1)
    timing: list[float] = []
    frame = 0
    last_activity = time.monotonic()

    print(f"[ekf_service] state={state_path} estimates={estimates_path}")
    print(f"[ekf_service] bases={bases} q={args.process_noise_mps} kappa={args.anchor_covariance_scale}")

    try:
        while True:
            if args.max_frames and frame >= args.max_frames:
                break
            if not _wait_for(state_ready, 1.0):
                if time.monotonic() - last_activity > args.idle_timeout_s:
                    print(f"[ekf_service] idle timeout ({args.idle_timeout_s}s) at frame {frame}, exiting")
                    break
                continue
            last_activity = time.monotonic()
            state = json.loads(state_path.read_text())
            state_ready.unlink(missing_ok=True)

            if state.get("frame_index") != frame:
                print(f"[ekf_service] WARNING frame mismatch: expected {frame}, "
                      f"got {state.get('frame_index')}; resyncing")
                frame = state.get("frame_index", frame)

            entries = state["robots"]
            ids = [e["id"] for e in entries]
            positions = {
                e["id"]: np.asarray([e["x"], e["y"]], dtype=float) for e in entries
            }
            held = {e["id"]: [e["vx"], e["vy"]] for e in entries}

            # Lazy init on first frame: deployment = frame-0 truth positions.
            if service is None:
                deployment = {rid: positions[rid].tolist() for rid in ids}
                service = EKFInLoopService(
                    deployment_positions=deployment,
                    bases=bases,
                    sigma0=args.ranging_sigma0,
                    range0=args.range0,
                    process_noise_mps=args.process_noise_mps,
                    p0_std=args.p0_std,
                    dt=args.dt,
                    innovation_gate=args.innovation_gate,
                    anchor_covariance_scale=args.anchor_covariance_scale,
                )
                print(f"[ekf_service] initialized on {len(ids)} robots, frame 0")

            frame_like = {
                "robots": [
                    {"id": rid, "state": {"x": float(positions[rid][0]),
                                          "y": float(positions[rid][1])}}
                    for rid in ids
                ],
                "formation": [e.get("formation", {}) for e in entries],
                "covariance_formation": [
                    e.get("covariance_formation", {}) for e in entries
                ],
            }
            references = build_ekf_raw_references(
                frame_like, bases, rng, sigma0=args.ranging_sigma0,
                range0=args.range0,
                availability_range0=args.availability_range0,
            )

            started = time.monotonic()
            outputs = service.step(
                frame_index=frame,
                raw_reference_groups=references,
                held_commands=held,
            )
            timing.append(time.monotonic() - started)

            payload = {
                "frame_index": frame,
                "robots": [
                    {
                        "id": rid,
                        "estimate": outputs[rid]["estimate"],
                        "epsilon": outputs[rid]["epsilon"],
                        "tier": outputs[rid]["tier"],
                    }
                    for rid in sorted(outputs)
                ],
            }
            estimates_log_handle.write(json.dumps(payload) + "\n")
            tmp = estimates_path.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(payload) + "\n")
            os.replace(tmp, estimates_path)

            if frame % 100 == 0:
                fresh = sum(1 for r in payload["robots"] if r["tier"] == "fresh")
                print(f"[ekf_service] frame {frame}: fresh={fresh}/{len(ids)} "
                      f"mean_eps={np.mean([r['epsilon'] for r in payload['robots']]):.2f}")
            frame += 1
    except KeyboardInterrupt:
        print(f"[ekf_service] interrupted at frame {frame}")
    finally:
        estimates_log_handle.close()
        args.timing_json.write_text(
            json.dumps({"per_frame_seconds": timing}, indent=1) + "\n"
        )
        n = len(timing)
        if n:
            print(f"[ekf_service] done: {n} frames, mean solve "
                  f"{np.mean(timing)*1000:.1f} ms, max {np.max(timing)*1000:.1f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
