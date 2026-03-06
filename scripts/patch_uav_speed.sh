#!/bin/bash
# patch_uav_speed.sh - Patch UAV maximum speed limit in MBZIRC simulation
#
# Usage: ./patch_uav_speed.sh [speed_mps] [horizontal_accel_mpss] [vertical_accel_mpss]
# Defaults: 25 10 4

TARGET_SPEED_MPS=${1:-25}
MAX_HORIZONTAL_ACCEL_MPSS=${2:-10}
MAX_VERTICAL_ACCEL_MPSS=${3:-4}

PATCH_MARKER="$HOME/.uav_speed_patched"
MODEL_FILE="$HOME/mbzirc_ws/src/mbzirc/mbzirc_ign/models/mbzirc_quadrotor/model.sdf.erb"

if [ -f "$PATCH_MARKER" ]; then
    echo "[patch_uav_speed.sh] Already patched, skipping."
    exit 0
fi

echo "[patch_uav_speed.sh] Patching UAV speed to ${TARGET_SPEED_MPS} m/s..."
echo "[patch_uav_speed.sh] Acceleration: ${MAX_HORIZONTAL_ACCEL_MPSS} ${MAX_HORIZONTAL_ACCEL_MPSS} ${MAX_VERTICAL_ACCEL_MPSS}"

if [ ! -f "$MODEL_FILE" ]; then
    echo "[patch_uav_speed.sh] Error: model.sdf.erb not found at $MODEL_FILE"
    echo "[patch_uav_speed.sh] Searching for alternative locations..."
    find "$HOME" -name "model.sdf.erb" -path "*mbzirc_quadrotor*" 2>/dev/null | head -5
    exit 1
fi

echo "[patch_uav_speed.sh] Current limits:"
grep -n "maximumLinearVelocity" "$MODEL_FILE" || true
grep -n "maximumLinearAcceleration" "$MODEL_FILE" || true

sed -i "s/<maximumLinearVelocity>5 5 5</<maximumLinearVelocity>${TARGET_SPEED_MPS} ${TARGET_SPEED_MPS} ${TARGET_SPEED_MPS}</g" "$MODEL_FILE"
sed -i "s/<maximumLinearAcceleration>1 1 2</<maximumLinearAcceleration>${MAX_HORIZONTAL_ACCEL_MPSS} ${MAX_HORIZONTAL_ACCEL_MPSS} ${MAX_VERTICAL_ACCEL_MPSS}</g" "$MODEL_FILE"

echo "[patch_uav_speed.sh] New limits:"
grep -n "maximumLinearVelocity" "$MODEL_FILE" || true
grep -n "maximumLinearAcceleration" "$MODEL_FILE" || true

echo "[patch_uav_speed.sh] Recompiling mbzirc_ign..."
cd "$HOME/mbzirc_ws"
. /opt/ros/galactic/setup.bash
colcon build --packages-select mbzirc_ign --merge-install 2>&1 | tail -5

if [ $? -eq 0 ]; then
    echo "[patch_uav_speed.sh] Patch applied successfully!"
    echo "${TARGET_SPEED_MPS} ${MAX_HORIZONTAL_ACCEL_MPSS} ${MAX_HORIZONTAL_ACCEL_MPSS} ${MAX_VERTICAL_ACCEL_MPSS}" > "$PATCH_MARKER"
else
    echo "[patch_uav_speed.sh] Warning: Compilation may have issues"
    echo "${TARGET_SPEED_MPS} ${MAX_HORIZONTAL_ACCEL_MPSS} ${MAX_HORIZONTAL_ACCEL_MPSS} ${MAX_VERTICAL_ACCEL_MPSS}" > "$PATCH_MARKER"
fi
