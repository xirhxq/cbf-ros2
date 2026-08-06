#!/bin/bash
# run_suav.sh - Build + run suav controller (iterate quickly without restarting Ignition).
#
# Prerequisites: start_world.sh must have been run (Ignition + UAVs + pose_bridge alive).
#
# Usage (inside container):
#   colcon build --packages-select cbf-ros2 && bash src/cbf-ros2/scripts/run_suav.sh
set -e
source /opt/ros/galactic/setup.bash
source ~/mbzirc_ws/install/setup.bash
cd /home/developer/cbf_ws
source install/setup.bash
. src/cbf-ros2/scripts/setenv.sh

# EKF node (if enabled)
EST_ON=$(python3 -c "import json; d=json.load(open('install/cbf-ros2/share/cbf-ros2/config/config.json')).get('estimator-in-loop',{}); print('1' if d.get('on') else '0')")
if [ "$EST_ON" = "1" ]; then
    Q=$(python3 -c "import json; print(json.load(open('install/cbf-ros2/share/cbf-ros2/config/config.json')).get('estimator-in-loop',{}).get('process-noise-mps',3.0))")
    echo "=== starting EKF node (q=$Q) ==="
    python3 src/cbf-ros2/scripts/ekf_node.py --num-robots $UAV_NUM --process-noise $Q > /tmp/ekf_node.log 2>&1 &
    sleep 2
fi

echo "=== starting suav (Ctrl+C to stop, then re-run this script) ==="
exec ros2 run cbf-ros2 suav 2>&1 | tee /tmp/suav.log
