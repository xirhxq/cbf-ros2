#!/bin/bash
# patch_uav_speed.sh - Patch UAV maximum speed limit in MBZIRC simulation
#
# This script modifies the UAV speed limit and recompiles mbzirc_ign
# It only runs once - subsequent runs detect the patch is already applied
#
# Usage: ./patch_uav_speed.sh [speed]
# Default speed: 25 (m/s)

PATCH_MARKER="$HOME/.uav_speed_patched"
TARGET_SPEED=${1:-25}

# Check if already patched
if [ -f "$PATCH_MARKER" ]; then
    echo "[patch_uav_speed.sh] Already patched (marker: $PATCH_MARKER), skipping."
    cat "$PATCH_MARKER"
    exit 0
fi

echo "[patch_uav_speed.sh] Patching UAV speed limit to ${TARGET_SPEED} m/s..."

# Path to model file inside Docker container
MODEL_FILE="$HOME/mbzirc_ws/src/mbzirc/mbzirc_ign/models/mbzirc_quadrotor/model.sdf.erb"

if [ ! -f "$MODEL_FILE" ]; then
    echo "[patch_uav_speed.sh] Error: model.sdf.erb not found at $MODEL_FILE"
    echo "[patch_uav_speed.sh] Searching for alternative locations..."
    find "$HOME" -name "model.sdf.erb" -path "*quadrotor*" 2>/dev/null
    exit 1
fi

# Show current value
echo "[patch_uav_speed.sh] Current speed limit:"
grep -n "maximumLinearVelocity" "$MODEL_FILE" || true

# Replace maximumLinearVelocity: 5 5 5 -> TARGET_SPEED
sed -i "s/<maximumLinearVelocity>5 5 5</<maximumLinearVelocity>${TARGET_SPEED} ${TARGET_SPEED} ${TARGET_SPEED}</g" "$MODEL_FILE"

# Show new value
echo "[patch_uav_speed.sh] New speed limit:"
grep -n "maximumLinearVelocity" "$MODEL_FILE" || true

# Recompile mbzirc_ign (only takes ~2 seconds)
echo "[patch_uav_speed.sh] Recompiling mbzirc_ign..."
cd "$HOME/mbzirc_ws"
source /opt/ros/galactic/setup.bash
colcon build --packages-select mbzirc_ign --merge-install 2>&1 | tail -5

if [ $? -eq 0 ]; then
    echo "[patch_uav_speed.sh] Patch applied successfully!"
    echo "Speed limit: ${TARGET_SPEED} m/s" > "$PATCH_MARKER"
else
    echo "[patch_uav_speed.sh] Warning: Compilation may have issues"
    echo "Speed limit: ${TARGET_SPEED} m/s (may have issues)" > "$PATCH_MARKER"
fi
