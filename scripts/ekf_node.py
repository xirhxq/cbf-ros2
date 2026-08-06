#!/usr/bin/env python3
"""EKF estimator as a standalone ROS2 node.

Subscribes to all 14 UAVs' /uav_X/pose/groundtruth (truth), runs the basic EKF
each control period, and publishes estimated positions to /uav_X/pose/estimated
plus epsilon (uncertainty radius) to /uav_X/epsilon.

This replaces both the file-IPC resident service and the in-process C++ EKF,
keeping the estimator fully outside suav's process (no OsqpEigen/EKF memory
conflicts). suav subscribes to /uav_X/pose/estimated instead of groundtruth.

Usage (inside container, started before suav):
    python3 src/cbf-ros2/scripts/ekf_node.py --num-robots 14 --process-noise 3.0
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Bool
from rosgraph_msgs.msg import Clock

# Reuse the validated Python EKF from the paper/cbf2026-tvt reference copy.
_HERE = Path(__file__).resolve().parent
_REPO_ROOT = _HERE.parent
_REF = _REPO_ROOT / ".ref" / "paper-cbf2026-tvt"
if str(_REF) not in sys.path:
    sys.path.insert(0, str(_REF))

from scripts.diagnostics.ekf_estimator_service import (
    EKFInLoopService,
    build_ekf_raw_references,
)


def quaternion_to_yaw(w, x, y, z):
    """Quaternion to yaw (rad)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class EkfNode(Node):
    def __init__(self, num_robots, bases, params):
        super().__init__("ekf_estimator")
        self.num_robots = num_robots
        self.bases = [np.asarray(b, dtype=float) for b in bases]
        self.params = params
        self.rng = np.random.default_rng(params.get("seed", 2026081301))

        # Latest truth positions {id: (x, y)} and velocities {id: (vx, vy)}
        self.truth_pos = {}
        self.prev_truth_pos = {}
        self.cmd_vel = {i + 1: [0.0, 0.0] for i in range(num_robots)}

        # EKF service (initialized lazily on first full frame)
        self.service = None

        # Gate: do not tick until suav signals PERFORM started. Otherwise epsilon
        # accumulates during PREPARE (UAVs in a tight column with poor ranging
        # geometry), producing an epsilon spike exactly when CBF begins.
        self.perform_started = False
        # Sim-time tracking for accurate predict dt (wall-clock timer interval is
        # not exactly 0.5 s under load). The EKF predict uses the real elapsed
        # sim time between ticks.
        self.last_sim_time = None

        # Subscribers: groundtruth pose for each UAV
        self.pose_subs = {}
        for i in range(1, num_robots + 1):
            topic = f"/uav_{i}/pose/groundtruth"
            self.pose_subs[i] = self.create_subscription(
                PoseStamped, topic,
                lambda msg, rid=i: self._pose_cb(msg, rid), 10)

        # Latched trigger from suav: published once when all UAVs reach PERFORM.
        self.create_subscription(
            Bool, "/cbf/perform_started",
            self._perform_trigger_cb,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        # Sim clock for accurate predict dt.
        self.create_subscription(Clock, "/clock", self._clock_cb, 10)

        # Publishers: estimated pose + epsilon for each UAV
        self.est_pubs = {}
        self.eps_pubs = {}
        for i in range(1, num_robots + 1):
            self.est_pubs[i] = self.create_publisher(
                PoseStamped, f"/uav_{i}/pose/estimated", 10)
            self.eps_pubs[i] = self.create_publisher(
                Float64, f"/uav_{i}/epsilon", 10)

        # Timer at 2 Hz (control period). _tick is a no-op until perform_started.
        dt = params.get("dt", 0.5)
        self.create_timer(dt, self._tick)

        self.frame = 0
        self.cur_sim_time = 0.0
        # Clear stale estimates log
        open("/tmp/ekf-estimates-log.jsonl", "w").close()
        self.get_logger().info(
            f"EKF node started (waiting for /cbf/perform_started): {num_robots} robots, "
            f"q={params.get('process_noise_mps', 3.0)}, "
            f"kappa={params.get('anchor_covariance_scale', 3.0)}")

    def _perform_trigger_cb(self, msg: Bool):
        if msg.data and not self.perform_started:
            self.perform_started = True
            self.last_sim_time = None  # reset so first tick dt is ~0
            self.get_logger().info("PERFORM trigger received, starting EKF ticks")

    def _clock_cb(self, msg: Clock):
        self.cur_sim_time = msg.clock.sec + msg.clock.nanosec / 1e9

    def _pose_cb(self, msg, robot_id):
        self.truth_pos[robot_id] = np.asarray(
            [msg.pose.position.x, msg.pose.position.y], dtype=float)

    def _tick(self):
        # Gate on PERFORM trigger so epsilon does not accumulate during PREPARE.
        if not self.perform_started:
            return
        # Need all robots' positions
        if len(self.truth_pos) < self.num_robots:
            return

        ids = sorted(self.truth_pos.keys())
        positions = {rid: self.truth_pos[rid] for rid in ids}

        # Real sim-time dt since last tick (fall back to nominal if clock missing).
        sim_dt = self.params.get("dt", 0.5)
        if self.last_sim_time is not None:
            sim_dt = max(self.cur_sim_time - self.last_sim_time, 1e-3)
        self.last_sim_time = self.cur_sim_time

        # Compute truth velocity (frame-to-frame difference) using real sim dt
        if self.prev_truth_pos:
            held = {rid: ((positions[rid] - self.prev_truth_pos[rid]) / sim_dt).tolist()
                    for rid in ids if rid in self.prev_truth_pos}
        else:
            held = {rid: [0.0, 0.0] for rid in ids}
        self.prev_truth_pos = {rid: pos.copy() for rid, pos in positions.items()}

        # Lazy init EKF service on first frame
        if self.service is None:
            deployment = {rid: positions[rid].tolist() for rid in ids}
            self.service = EKFInLoopService(
                deployment_positions=deployment,
                bases=self.bases,
                process_noise_mps=self.params.get("process_noise_mps", 3.0),
                anchor_covariance_scale=self.params.get("anchor_covariance_scale", 3.0),
                dt=self.params.get("dt", 0.5),
                p0_std=self.params.get("p0_std", 1.0),
                innovation_gate=self.params.get("innovation_gate", 3.0),
                sigma0=self.params.get("sigma0", 0.5),
                range0=self.params.get("range0", 850.0),
            )
            self.get_logger().info(f"EKF initialized on {len(ids)} robots")
        # Override the service's fixed dt with the real sim dt for this step's
        # predict (mean += v*dt, cov += (q*dt)^2).
        self.service.dt = sim_dt

        # Build references from formation topology
        # Simple ladder formation (matching config):
        # squad1 (1-7): 1→{base0,base1}, 2→{base1,1}, i≥3→{i-1,i-2}
        # squad2 (8-14): 8→{base1,base2}, 9→{base1,8}, i≥10→{i-1,i-2}
        # bases: 0=[-1550,-300], 1=[-1550,0], 2=[-1550,300]
        frame_like = {
            "robots": [{"id": rid, "state": {"x": float(positions[rid][0]),
                                              "y": float(positions[rid][1])}}
                       for rid in ids],
            "formation": self._build_formation(ids),
            "covariance_formation": self._build_formation(ids),
        }
        references = build_ekf_raw_references(
            frame_like, self.bases, self.rng,
            sigma0=self.params.get("sigma0", 0.5),
            range0=self.params.get("range0", 850.0),
            availability_range0=self.params.get("range0", 850.0),
        )

        outputs = self.service.step(
            frame_index=self.frame,
            raw_reference_groups=references,
            held_commands=held,
        )

        # Publish estimates + log for containment analysis
        fresh_count = 0
        log_entries = []
        for rid in ids:
            out = outputs[rid]
            truth = positions[rid]
            est = out["estimate"]
            err = float(np.hypot(truth[0] - est[0], truth[1] - est[1]))
            eps = out["epsilon"]
            log_entries.append({
                "id": rid, "truth": truth.tolist(), "estimate": est,
                "epsilon": eps, "error": err, "tier": out["tier"]
            })
            est_msg = PoseStamped()
            est_msg.header.stamp = self.get_clock().now().to_msg()
            est_msg.header.frame_id = "world"
            est_msg.pose.position.x = out["estimate"][0]
            est_msg.pose.position.y = out["estimate"][1]
            est_msg.pose.position.z = 0.0
            est_msg.pose.orientation.w = 1.0
            self.est_pubs[rid].publish(est_msg)

            eps_msg = Float64()
            eps_msg.data = out["epsilon"]
            self.eps_pubs[rid].publish(eps_msg)

            if out["tier"] == "fresh":
                fresh_count += 1

        if self.frame % 100 == 0:
            self.get_logger().info(
                f"frame {self.frame}: fresh={fresh_count}/{len(ids)}")

        # Write estimates log for containment analysis (overwrite mode, cleared on init)
        import json as _json
        with open("/tmp/ekf-estimates-log.jsonl", "a") as f:
            f.write(_json.dumps({"frame": self.frame, "robots": log_entries}) + "\n")

        self.frame += 1

    def _build_formation(self, ids):
        """Build ladder formation topology matching the config."""
        formation = []
        for rid in ids:
            entry = {"id": rid, "baseIds": [], "anchorIds": []}
            if rid <= 7:  # squad 1
                if rid == 1:
                    entry["baseIds"] = [0, 1]
                    entry["anchorIds"] = []
                elif rid == 2:
                    entry["baseIds"] = [1]
                    entry["anchorIds"] = [1]
                else:
                    entry["anchorIds"] = [rid - 1, rid - 2]
            else:  # squad 2
                if rid == 8:
                    entry["baseIds"] = [1, 2]
                    entry["anchorIds"] = []
                elif rid == 9:
                    entry["baseIds"] = [1]
                    entry["anchorIds"] = [8]
                else:
                    entry["anchorIds"] = [rid - 1, rid - 2]
            formation.append(entry)
        return formation


def main():
    parser = argparse.ArgumentParser(description="EKF estimator ROS2 node")
    parser.add_argument("--num-robots", type=int, default=14)
    parser.add_argument("--process-noise", type=float, default=3.0)
    parser.add_argument("--kappa", type=float, default=3.0)
    parser.add_argument("--dt", type=float, default=0.5)
    parser.add_argument("--sigma0", type=float, default=0.5)
    parser.add_argument("--range0", type=float, default=850.0)
    parser.add_argument("--bases", type=str, default="auto")
    args = parser.parse_args()

    bases = ([[-1550.0, -300.0], [-1550.0, 0.0], [-1550.0, 300.0]]
             if args.bases == "auto" else eval(args.bases))

    params = {
        "process_noise_mps": args.process_noise,
        "anchor_covariance_scale": args.kappa,
        "dt": args.dt,
        "sigma0": args.sigma0,
        "range0": args.range0,
        "p0_std": 1.0,
        "innovation_gate": 3.0,
        "seed": 2026081301,
    }

    rclpy.init()
    node = EkfNode(args.num_robots, bases, params)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
