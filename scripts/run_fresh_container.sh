#!/bin/bash
# run_fresh_container.sh - Full clean run in a FRESH container.
#
# Always starts a brand-new container (per researcher's standing instruction),
# applies all patches, starts the world, runs EKF+suav, and monitors until
# mission complete or crash. Leaves the container alive for inspection.
#
# Usage (from host):
#   bash scripts/run_fresh_container.sh            # full run, foreground monitor
#   Q=2.0 bash scripts/run_fresh_container.sh      # override q
#
# Output:
#   - data.json:   ~/cbf_ws/src/cbf-ros2/cbf/data/<timestamp>/data.json (in container)
#   - ekf log:     /tmp/ekf-estimates-log.jsonl (in container)
#   - suav log:    /tmp/suav.log (in container)
set -e

IMAGE="${IMAGE:-cbf-ros2:osqp}"
CONTAINER="cbf-headless"
HOST_WS="/home/tbg/Repos/cbf-ros2"

echo "=== [0] removing any stale container ==="
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true

echo "=== [1] starting fresh container ($IMAGE) ==="
docker run -d --name "$CONTAINER" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "$HOST_WS:/home/developer/cbf_ws/src/cbf-ros2" \
  -v "$HOST_WS/.tmux.conf:/home/developer/cbf_ws/.tmux.conf:ro" \
  --network host --privileged --security-opt seccomp=unconfined \
  -w /home/developer/cbf_ws \
  "$IMAGE" sleep infinity
echo "    container up"

# Fix git safe-dirs + build
docker exec "$CONTAINER" bash -lc '
  git config --global --add safe.directory /home/developer/cbf_ws/src/cbf-ros2
  git config --global --add safe.directory /home/developer/cbf_ws/src/cbf-ros2/cbf
  cd ~/cbf_ws && source /opt/ros/galactic/setup.bash
  echo "=== [2] building cbf-ros2 ==="
  colcon build --packages-select cbf-ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
'

echo "=== [3] applying patches (fresh container) ==="
docker exec "$CONTAINER" bash -lc '
  cd ~/cbf_ws/src/cbf-ros2
  bash scripts/patch_disable_gamelogic.sh >/dev/null 2>&1 && echo "    gamelogic: ok" || echo "    gamelogic: skip"
  bash scripts/patch_uav_speed.sh 25 10 4 >/tmp/p1.log 2>&1 && tail -1 /tmp/p1.log || echo "speed patch fail"
  bash scripts/patch_uav_thrust.sh >/tmp/p2.log 2>&1 && tail -1 /tmp/p2.log || echo "thrust patch fail"
  bash scripts/patch_geofence_height.sh >/dev/null 2>&1 && echo "    geofence_h: ok"
  bash scripts/patch_uav_marker.sh >/dev/null 2>&1 && echo "    marker: ok"
'

echo "=== [4] starting world (Ignition + spawn + pose_bridge, ~130s) ==="
docker exec -d "$CONTAINER" bash -c 'cd ~/cbf_ws/src/cbf-ros2 && bash scripts/start_world.sh > /tmp/start_world.log 2>&1'
# wait for world ready
for i in $(seq 1 40); do
  sleep 5
  n=$(docker exec "$CONTAINER" bash -lc 'source /opt/ros/galactic/setup.bash; source ~/mbzirc_ws/install/setup.bash; cd ~/cbf_ws && source install/setup.bash 2>/dev/null; timeout 5 ros2 topic list 2>/dev/null | grep -c "pose/groundtruth"' 2>/dev/null || echo 0)
  if [ "$n" = "14" ]; then echo "    world ready (14 groundtruth topics) after ${i}x5s"; break; fi
  echo "    waiting... ($n groundtruth topics)"
done

echo "=== [5] launching EKF + suav (run_suav.sh) ==="
docker exec -d "$CONTAINER" bash -c 'cd ~/cbf_ws && source /opt/ros/galactic/setup.bash && source ~/mbzirc_ws/install/setup.bash && source install/setup.bash && bash src/cbf-ros2/scripts/run_suav.sh > /tmp/suav.log 2>&1'

echo "=== [6] monitoring (until mission complete or crash) ==="
docker exec "$CONTAINER" bash -c 'cd ~/cbf_ws && source /opt/ros/galactic/setup.bash && source ~/mbzirc_ws/install/setup.bash && source install/setup.bash && bash src/cbf-ros2/scripts/monitor_run.sh' 2>&1 | tee /tmp/monitor_outer.log || true

echo "=== [7] checking outcome ==="
docker exec "$CONTAINER" bash -lc '
  if grep -q "Log saved" /tmp/suav.log; then
    DATA=$(grep -oE "/home/developer[^ ]*data.json" /tmp/suav.log | tail -1)
    echo "RESULT: MISSION COMPLETE, data=$DATA"
  elif grep -qi "primal_infeasible\|segmentation\|segfault\|terminate called" /tmp/suav.log; then
    echo "RESULT: CRASH"; tail -3 /tmp/suav.log | tr "\r" "\n" | tail -3
  else
    echo "RESULT: UNKNOWN (monitor exited)"; tail -3 /tmp/suav.log | tr "\r" "\n" | tail -3
  fi
'
