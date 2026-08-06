#!/bin/bash
# run_headless_full.sh - One-shot headless MBZIRC + CBF full-stack launcher.
#
# Runs INSIDE the cbf-ros2 container (started by scripts/run_docker_headless.sh).
# Launches the complete headless pipeline in order:
#   1. colcon build (if needed)
#   2. patch mbzirc UAV params (speed/geofence/thrust/marker) + rebuild mbzirc_ign
#   3. headless Ignition server (-s, no GUI, no camera sensors)
#   4. spawn 14 UAVs without camera/RF sensors (spawn_headless.launch.py)
#   5. pose_bridge (ign pose -> ROS2 groundtruth)
#   6. suav CBF controller (2 Hz)
#
# Usage (inside container):
#   bash src/cbf-ros2/scripts/run_headless_full.sh
#
# All logs go to /tmp/*.log. Tail them to monitor:
#   tail -f /tmp/suav.log /tmp/launch.log
#
# To stop: kill all with  `pkill -f "ign gazebo|cbf-ros2|competition_local|spawn_headless"`

set -e
source /opt/ros/galactic/setup.bash
source ~/mbzirc_ws/install/setup.bash
cd /home/developer/cbf_ws

echo "=== [1/5] colcon build ==="
colcon build --packages-select cbf-ros2 2>&1 | tail -2

# Source the workspace AFTER build so install/setup.bash exists.
source install/setup.bash
. src/cbf-ros2/scripts/setenv.sh

echo "=== [2/5] patch mbzirc params (idempotent via marker files) ==="
bash src/cbf-ros2/scripts/patch_uav_speed.sh 50    > /tmp/patch_speed.log 2>&1 || true
bash src/cbf-ros2/scripts/patch_geofence_height.sh 400 > /tmp/patch_geofence.log 2>&1 || true
bash src/cbf-ros2/scripts/patch_geofence_horizontal.sh 100000 > /tmp/patch_geofence_h.log 2>&1 || true
bash src/cbf-ros2/scripts/patch_uav_thrust.sh 1.5  > /tmp/patch_thrust.log 2>&1 || true
bash src/cbf-ros2/scripts/patch_uav_marker.sh      > /tmp/patch_marker.log 2>&1 || true
bash src/cbf-ros2/scripts/patch_disable_gamelogic.sh > /tmp/patch_gamelogic.log 2>&1 || true
# Recompile mbzirc_ign if any patch applied (only first run)
if [ -f "$HOME/.uav_speed_patched" ] || [ -f "$HOME/.uav_thrust_patched" ] || [ -f "$HOME/.uav_marker_patched" ] || [ -f "$HOME/.gamelogic_disabled" ]; then
    (cd ~/mbzirc_ws && source /opt/ros/galactic/setup.bash && colcon build --packages-select mbzirc_ign --merge-install) > /tmp/rebuild_mbzirc.log 2>&1 || true
fi

echo "=== [3/5] headless Ignition server (-s = server only, no GUI) ==="
ros2 launch mbzirc_ros competition_local.launch.py ign_args:="-v 1 -r -s coast.sdf" > /tmp/launch.log 2>&1 &
echo "   waiting 80s for coast world to load..."
sleep 80

echo "=== [4/5] spawn $UAV_NUM UAVs (no camera sensors) + pose_bridge ==="
ros2 launch cbf-ros2 spawn_headless.launch.py numbers:=$UAV_NUM > /tmp/spawn.log 2>&1 &
echo "   waiting 45s for spawn..."
sleep 45
ros2 run cbf-ros2 pose_bridge $UAV_NUM > /tmp/pose_bridge.log 2>&1 &
sleep 3

# Estimator-in-loop: when enabled, launch the EKF ROS2 node that subscribes
# to /uav_X/pose/truth (from pose_bridge) and publishes estimated positions
# to /uav_X/pose/groundtruth (which suav subscribes to). This keeps the EKF
# in a separate process from suav (no in-process memory conflicts).
EST_ON=$(python3 -c "import json; d=json.load(open('install/cbf-ros2/share/cbf-ros2/config/config.json')).get('estimator-in-loop',{}); print('1' if d.get('on') else '0')")
if [ "$EST_ON" = "1" ]; then
    Q=$(python3 -c "import json; print(json.load(open('install/cbf-ros2/share/cbf-ros2/config/config.json')).get('estimator-in-loop',{}).get('process-noise-mps',3.0))")
    echo "=== [4b/5] starting EKF ROS2 node (q=$Q) ==="
    python3 src/cbf-ros2/scripts/ekf_node.py --num-robots $UAV_NUM --process-noise $Q > /tmp/ekf_node.log 2>&1 &
    sleep 3
    echo "    EKF node pid: $(pgrep -f ekf_node.py | head -1)"
fi

echo "=== [5/5] suav CBF controller (foreground, Ctrl+C to stop) ==="
echo "    logs: tail -f /tmp/suav.log /tmp/ekf_service.log"
echo "    stop all: pkill -f 'ign gazebo|cbf-ros2|competition_local|spawn_headless|ekf_service'"
exec ros2 run cbf-ros2 suav 2>&1 | tee /tmp/suav.log
