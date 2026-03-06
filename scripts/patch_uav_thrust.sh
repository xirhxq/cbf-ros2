#!/bin/bash
# patch_uav_thrust.sh - Patch UAV thrust (motorConstant) in MBZIRC simulation
#
# Usage: ./patch_uav_thrust.sh [motor_constant_multiplier]
# Defaults: 1.5 (increase thrust by 50%)
#
# Reference: mbzirc/docs/uav_thrust_tuning.md
# Thrust = motorConstant × motor_speed²

MULTIPLIER=${1:-1.5}

PATCH_MARKER="$HOME/.uav_thrust_patched"
MODEL_FILE="$HOME/mbzirc_ws/src/mbzirc/mbzirc_ign/models/mbzirc_quadrotor/model.sdf.erb"

if [ -f "$PATCH_MARKER" ]; then
    echo "[patch_uav_thrust.sh] Already patched, skipping."
    exit 0
fi

echo "[patch_uav_thrust.sh] Patching UAV thrust with multiplier ${MULTIPLIER}x..."

if [ ! -f "$MODEL_FILE" ]; then
    echo "[patch_uav_thrust.sh] Error: model.sdf.erb not found at $MODEL_FILE"
    echo "[patch_uav_thrust.sh] Searching for alternative locations..."
    find "$HOME" -name "model.sdf.erb" -path "*mbzirc_quadrotor*" 2>/dev/null | head -5
    exit 1
fi

echo "[patch_uav_thrust.sh] Current motor constants:"
grep -n "\$motor_constant = " "$MODEL_FILE" | head -5

# Extract current default motor constant (line 13)
CURRENT_CONSTANT=$(grep -m1 "\$motor_constant = " "$MODEL_FILE" | sed "s/.*'\([^']*\)'.*/\1/")
echo "[patch_uav_thrust.sh] Current default: $CURRENT_CONSTANT"

# Calculate new motor constant using awk for scientific notation
NEW_CONSTANT=$(awk -v c="$CURRENT_CONSTANT" -v m="$MULTIPLIER" 'BEGIN { printf "%.5e", c * m }')
echo "[patch_uav_thrust.sh] New default: $NEW_CONSTANT"

# Patch the default motor constant (line 13: $motor_constant = '8.54858e-06')
sed -i "s/\$motor_constant = '$CURRENT_CONSTANT'/\$motor_constant = '$NEW_CONSTANT'/g" "$MODEL_FILE"


echo "[patch_uav_thrust.sh] New motor constants:"
grep -n "\$motor_constant = " "$MODEL_FILE" | head -5

echo "[patch_uav_thrust.sh] Recompiling mbzirc_ign..."
cd "$HOME/mbzirc_ws"
. /opt/ros/galactic/setup.bash
colcon build --packages-select mbzirc_ign --merge-install 2>&1 | tail -5

if [ $? -eq 0 ]; then
    echo "[patch_uav_thrust.sh] Patch applied successfully!"
    echo "${MULTIPLIER}" > "$PATCH_MARKER"
else
    echo "[patch_uav_thrust.sh] Warning: Compilation may have issues"
    echo "${MULTIPLIER}" > "$PATCH_MARKER"
fi
