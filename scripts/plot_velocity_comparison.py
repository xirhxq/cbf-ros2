#!/usr/bin/env python3
"""
Plot comparison between CBF control velocity and actual velocity.

This script reads data.json files from cbf/data/ directory and plots:
- CBF control velocity (opt.result.vx, opt.result.vy)
- Actual velocity (computed from position derivative)

Usage:
    python plot_velocity_comparison.py

    The script will automatically scan cbf/data/ directory and let you select which data to plot.
"""

import json
import os
import sys
import glob
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path


def find_data_files(folder: str, pattern: str = '*', num: int = 10):
    """Find the newest data.json files in the data folder."""
    directories = glob.glob(os.path.join(folder, pattern))
    directories = [d for d in directories if os.path.isdir(d)]
    directories = [d for d in directories if os.path.exists(os.path.join(d, 'data.json'))]
    directories = sorted(directories, key=lambda x: os.path.getmtime(os.path.join(x, 'data.json')), reverse=True)
    num = min(num, len(directories))
    return [os.path.join(d, 'data.json') for d in directories[:num]]


def interactive_selection(options):
    """Display available data files and let user select."""
    print("Select data file:")
    for idx, file_path in enumerate(options):
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)

            config = data.get('config', {})
            state = data.get('state', [])

            # Basic info
            duration = state[-1]['runtime'] if state else 0
            exec_mode = config.get('execute', {}).get('execution-mode', 'unknown')[:4]
            num_robots = config.get('num', 0)
            file_name = os.path.basename(os.path.dirname(file_path))

            # Calculate final search percentage
            grid_world = data.get('para', {}).get('gridWorld', {})
            x_num = grid_world.get('xNum', 1)
            y_num = grid_world.get('yNum', 1)
            total_cells = x_num * y_num
            searched_cells = set()
            for frame in state:
                if "update" in frame and len(frame["update"]) > 0:
                    for grid in frame["update"]:
                        cell_id = (grid[0], grid[1])
                        searched_cells.add(cell_id)
            search_pct = len(searched_cells) / total_cells * 100 if total_cells > 0 else 0

            print(f"[{idx}]: {file_name} - {duration:.1f}s | {search_pct:.1f}% | {exec_mode} | {num_robots}r")

        except Exception as e:
            print(f"[{idx}]: {os.path.basename(os.path.dirname(file_path))} - Error: {e}")

    choice = input("Choose option (or 'q' to quit): ").strip().lower()
    if choice == 'q':
        sys.exit(0)
    return options[int(choice)]


def load_data(data_file):
    """Load data.json from the specified file."""
    print(f"Loading data from: {data_file}")

    with open(data_file, 'r') as f:
        data = json.load(f)

    if 'state' not in data or len(data['state']) == 0:
        print("Error: No state data found in data.json")
        sys.exit(1)

    return data, Path(data_file).parent


def extract_velocity_data(data):
    """Extract velocity data from all robots across all timesteps.

    CBF control velocity: from opt.result (commanded velocity)
    Actual velocity: computed from position derivative
    """
    states = data['state']
    num_steps = len(states)

    # Get number of robots from first step
    num_robots = len(states[0]['robots'])

    # First pass: collect all data
    raw_data = {}  # {robot_id: {'time': [], 'x': [], 'y': [], 'cbf_vx': [], 'cbf_vy': []}}

    for step in states:
        runtime = step['runtime']
        for robot in step['robots']:
            robot_id = robot['id']

            if robot_id not in raw_data:
                raw_data[robot_id] = {
                    'time': [],
                    'x': [], 'y': [],
                    'cbf_vx': [], 'cbf_vy': []
                }

            # Position from state
            state = robot.get('state', {})
            x = state.get('x', 0.0)
            y = state.get('y', 0.0)

            # CBF control velocity from opt.result
            opt = robot.get('opt', {})
            result = opt.get('result', {})
            cbf_vx = result.get('vx', 0.0)
            cbf_vy = result.get('vy', 0.0)

            raw_data[robot_id]['time'].append(runtime)
            raw_data[robot_id]['x'].append(x)
            raw_data[robot_id]['y'].append(y)
            raw_data[robot_id]['cbf_vx'].append(cbf_vx)
            raw_data[robot_id]['cbf_vy'].append(cbf_vy)

    # Second pass: compute actual velocity from position derivative
    robot_data = {}
    for robot_id, rd in raw_data.items():
        n = len(rd['time'])
        if n < 2:
            continue

        robot_data[robot_id] = {
            'time': rd['time'],
            'cbf_vx': rd['cbf_vx'],
            'cbf_vy': rd['cbf_vy'],
            'actual_vx': [0.0] * n,
            'actual_vy': [0.0] * n,
        }

        # Compute velocity using central difference (except endpoints)
        for i in range(1, n - 1):
            dt = rd['time'][i + 1] - rd['time'][i - 1]
            if dt > 0:
                robot_data[robot_id]['actual_vx'][i] = (rd['x'][i + 1] - rd['x'][i - 1]) / dt
                robot_data[robot_id]['actual_vy'][i] = (rd['y'][i + 1] - rd['y'][i - 1]) / dt

        # Forward difference for first point
        dt = rd['time'][1] - rd['time'][0]
        if dt > 0:
            robot_data[robot_id]['actual_vx'][0] = (rd['x'][1] - rd['x'][0]) / dt
            robot_data[robot_id]['actual_vy'][0] = (rd['y'][1] - rd['y'][0]) / dt

        # Backward difference for last point
        dt = rd['time'][n - 1] - rd['time'][n - 2]
        if dt > 0:
            robot_data[robot_id]['actual_vx'][n - 1] = (rd['x'][n - 1] - rd['x'][n - 2]) / dt
            robot_data[robot_id]['actual_vy'][n - 1] = (rd['y'][n - 1] - rd['y'][n - 2]) / dt

    return robot_data


def plot_velocity_comparison(robot_data, output_dir, robot_ids=None):
    """Plot velocity comparison for each robot in a separate figure."""
    if robot_ids is None:
        robot_ids = sorted(robot_data.keys())

    if len(robot_ids) == 0:
        print("No data to plot.")
        return

    # Create output subdirectory for individual robot plots
    comparison_dir = output_dir / "velocity_comparison"
    comparison_dir.mkdir(exist_ok=True)

    # Plot each robot in its own figure
    for rid in robot_ids:
        if rid not in robot_data:
            continue

        rd = robot_data[rid]
        time = np.array(rd['time'])

        # Create figure with 2 subplots for this robot
        fig, axes = plt.subplots(1, 2, figsize=(14, 5))

        # Plot VX
        ax_vx = axes[0]
        ax_vx.plot(time, rd['cbf_vx'], 'b-', label='CBF Control vx', linewidth=1.5)
        ax_vx.plot(time, rd['actual_vx'], 'r--', label='Actual vx (from position)', linewidth=1.5)
        ax_vx.set_xlabel('Time (s)')
        ax_vx.set_ylabel('Velocity vx (m/s)')
        ax_vx.set_title(f'Robot {rid} - VX Comparison')
        ax_vx.legend()
        ax_vx.grid(True, alpha=0.3)

        # Plot VY
        ax_vy = axes[1]
        ax_vy.plot(time, rd['cbf_vy'], 'b-', label='CBF Control vy', linewidth=1.5)
        ax_vy.plot(time, rd['actual_vy'], 'r--', label='Actual vy (from position)', linewidth=1.5)
        ax_vy.set_xlabel('Time (s)')
        ax_vy.set_ylabel('Velocity vy (m/s)')
        ax_vy.set_title(f'Robot {rid} - VY Comparison')
        ax_vy.legend()
        ax_vy.grid(True, alpha=0.3)

        plt.tight_layout()

        # Save figure for this robot
        output_file = comparison_dir / f"robot_{rid}_comparison.png"
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved: {output_file}")
        plt.close()

    print(f"Total {len(robot_ids)} individual comparison plots saved to {comparison_dir}")


def plot_velocity_magnitude(robot_data, output_dir, robot_ids=None):
    """Plot velocity magnitude comparison for each robot in a separate figure."""
    if robot_ids is None:
        robot_ids = sorted(robot_data.keys())

    if len(robot_ids) == 0:
        return

    # Create output subdirectory for individual magnitude plots
    magnitude_dir = output_dir / "velocity_magnitude"
    magnitude_dir.mkdir(exist_ok=True)

    # Plot each robot in its own figure
    for rid in robot_ids:
        if rid not in robot_data:
            continue

        rd = robot_data[rid]
        time = np.array(rd['time'])

        cbf_mag = np.sqrt(np.array(rd['cbf_vx'])**2 + np.array(rd['cbf_vy'])**2)
        actual_mag = np.sqrt(np.array(rd['actual_vx'])**2 + np.array(rd['actual_vy'])**2)

        # Create figure for this robot
        fig, ax = plt.subplots(figsize=(12, 6))

        ax.plot(time, cbf_mag, 'b-', label='CBF Control', linewidth=1.5)
        ax.plot(time, actual_mag, 'r--', label='Actual (from position)', linewidth=1.5)

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity Magnitude (m/s)')
        ax.set_title(f'Robot {rid} - Velocity Magnitude Comparison')
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()

        # Save figure for this robot
        output_file = magnitude_dir / f"robot_{rid}_magnitude.png"
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"Saved: {output_file}")
        plt.close()

    print(f"Total {len(robot_ids)} individual magnitude plots saved to {magnitude_dir}")


def main():
    # Find script directory and data root
    script_dir = Path(__file__).parent
    data_root = script_dir.parent / "cbf" / "data"

    if not data_root.exists():
        print(f"Error: Data directory not found: {data_root}")
        sys.exit(1)

    # Find available data files
    data_files = find_data_files(str(data_root), '*', 10)

    if not data_files:
        print(f"Error: No data.json files found in {data_root}")
        sys.exit(1)

    # Let user select
    selected_file = interactive_selection(data_files)

    # Load and process data
    data, output_dir = load_data(selected_file)
    robot_data = extract_velocity_data(data)

    print(f"Found {len(robot_data)} robots, {len(data['state'])} timesteps")

    # Plot
    plot_velocity_comparison(robot_data, output_dir)
    plot_velocity_magnitude(robot_data, output_dir)

    print("\nDone!")


if __name__ == "__main__":
    main()
