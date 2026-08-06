#!/bin/bash
# monitor_run.sh - High-frequency anomaly monitor for a running cbf-headless sim.
#
# Checks every MONITOR_INTERVAL seconds (default 30) for common failures:
#   - suav process died (segfault)
#   - any UAV below sea level (z < 0, crashed/underwater)
#   - sim paused (mbzirc finished or other pause)
#   - frames not advancing (step() stalled)
#   - new segfault in dmesg
#
# Usage (inside container):  bash src/cbf-ros2/scripts/monitor_run.sh
#   or with interval:         MONITOR_INTERVAL=15 bash .../monitor_run.sh

set +e
source /opt/ros/galactic/setup.bash 2>/dev/null
source ~/mbzirc_ws/install/setup.bash 2>/dev/null
cd /home/developer/cbf_ws
source install/setup.bash 2>/dev/null

INTERVAL=${MONITOR_INTERVAL:-30}
LAST_FRAMES=0
LAST_DMESGline=""

echo "[monitor] starting, interval=${INTERVAL}s"

while true; do
    sleep $INTERVAL

    # 1. suav process alive?
    SUAV_PID=$(ps aux | grep "suav" | grep -v grep | grep -v bash | awk '{print $2}' | head -1)
    if [ -z "$SUAV_PID" ]; then
        echo "[monitor] $(date +%H:%M:%S) *** ALERT: suav process DEAD (segfault?) ***"
        dmesg 2>/dev/null | grep "suav\[" | tail -1
        # check if it was primal_infeasible
        grep -E "terminate|primal_infeasible|invalid_solution" /tmp/suav.log 2>/dev/null | tail -1
        echo "[monitor] recommend: restart container"
        break
    fi

    # 2. Any UAV underwater (z < 0) or stuck on ground (z < 10 after sim > 60s)?
    BAD_UAVS=""
    for i in $(seq 1 14); do
        Z=$(timeout 3 ros2 topic echo /uav_$i/pose/groundtruth 2>/dev/null | grep "z:" | head -1 | awk '{print $2}')
        if [ -n "$Z" ]; then
            if python3 -c "exit(0 if float('$Z') < 0 else 1)" 2>/dev/null; then
                BAD_UAVS="$BAD_UAVS uav_$i(z=$Z UNDERWATER)"
            elif [ -n "$SIM_T1" ] && [ "$SIM_T1" -gt 60 ] && python3 -c "exit(0 if float('$Z') < 10 else 1)" 2>/dev/null; then
                BAD_UAVS="$BAD_UAVS uav_$i(z=$Z STUCK)"
            fi
        fi
    done
    if [ -n "$BAD_UAVS" ]; then
        echo "[monitor] $(date +%H:%M:%S) *** ALERT: UAV anomaly:$BAD_UAVS ***"
        echo "[monitor] recommend: restart container"
        break
    fi

    # 3. Sim time advancing?
    SIM_T1=$(timeout 3 ros2 topic echo /clock 2>/dev/null | grep -oE "sec: [0-9]+" | head -1 | awk '{print $2}')
    sleep 5
    SIM_T2=$(timeout 3 ros2 topic echo /clock 2>/dev/null | grep -oE "sec: [0-9]+" | head -1 | awk '{print $2}')
    if [ "$SIM_T1" = "$SIM_T2" ] && [ -n "$SIM_T1" ]; then
        echo "[monitor] $(date +%H:%M:%S) WARNING: sim clock stalled at $SIM_T1"
    fi

    # 4. Frames advancing?
    LATEST=$(ls -t /home/developer/cbf_ws/src/cbf-ros2/cbf/data/ 2>/dev/null | head -1)
    if [ -n "$LATEST" ] && [ -f "/home/developer/cbf_ws/src/cbf-ros2/cbf/data/$LATEST/frames.jsonl" ]; then
        FRAMES=$(wc -l < "/home/developer/cbf_ws/src/cbf-ros2/cbf/data/$LATEST/frames.jsonl" 2>/dev/null)
        if [ "$FRAMES" = "$LAST_FRAMES" ] && [ "$FRAMES" -gt 0 ]; then
            # frames not advancing — check if all PERFORM or still flying
            CBF=$(grep -c "starting CBF" /tmp/suav.log 2>/dev/null)
            echo "[monitor] $(date +%H:%M:%S) frames stalled at $FRAMES (CBF=$CBF)"
        fi
        LAST_FRAMES=$FRAMES
    fi

    # 5. Mission complete?
    if grep -q "Log saved\|Step 6" /tmp/suav.log 2>/dev/null; then
        echo "[monitor] $(date +%H:%M:%S) *** MISSION COMPLETE *** data saved"
        echo "[monitor] frames=$FRAMES sim=$SIM_T2"
        break
    fi

    # Status line
    CBF=$(grep -c "starting CBF" /tmp/suav.log 2>/dev/null)
    echo "[monitor] $(date +%H:%M:%S) OK suav=$SUAV_PID sim=$SIM_T2 frames=${FRAMES:-0} CBF=$CBF"
done
