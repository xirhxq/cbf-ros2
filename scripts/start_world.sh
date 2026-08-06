#!/bin/bash
# start_world.sh - Start Ignition + spawn + pose_bridge (run ONCE per container).
#
# This is the slow part (~125s) that doesn't change between suav iterations.
# After this, use run_suav.sh to iterate on the controller.
#
# Usage (inside container):
#   bash src/cbf-ros2/scripts/start_world.sh
set -e
source /opt/ros/galactic/setup.bash
source ~/mbzirc_ws/install/setup.bash
cd /home/developer/cbf_ws
source install/setup.bash
. src/cbf-ros2/scripts/setenv.sh

echo "=== [1/3] headless Ignition server ==="
ros2 launch mbzirc_ros competition_local.launch.py ign_args:="-v 1 -r -s coast.sdf" > /tmp/launch.log 2>&1 &
echo "   waiting 80s for coast world..."
sleep 80

echo "=== [2/3] spawn $UAV_NUM UAVs + pose_bridge ==="
ros2 launch cbf-ros2 spawn_headless.launch.py numbers:=$UAV_NUM > /tmp/spawn.log 2>&1 &
echo "   waiting 45s for spawn..."
sleep 45
ros2 run cbf-ros2 pose_bridge $UAV_NUM > /tmp/pose_bridge.log 2>&1 &
sleep 3
echo "=== World ready. Use run_suav.sh to start the controller. ==="
