#!/bin/bash
# patch_geofence_height.sh - Patch geofence height limit in MBZIRC coast.sdf
#
# Usage: ./patch_geofence_height.sh [max_height]
# Default max_height: 200 (meters)

PATCH_MARKER="$HOME/.geofence_height_patched"
TARGET_HEIGHT=${1:-200}

# Check if already patched
if [ -f "$PATCH_MARKER" ]; then
    echo "[patch_geofence_height.sh] Already patched, skipping."
    exit 0
fi

echo "[patch_geofence_height.sh] Patching geofence height to ${TARGET_HEIGHT}m..."

WORLD_FILE="$HOME/mbzirc_ws/src/mbzirc/mbzirc_ign/worlds/coast.sdf"

if [ ! -f "$WORLD_FILE" ]; then
    echo "[patch_geofence_height.sh] Error: coast.sdf not found at $WORLD_FILE"
    echo "[patch_geofence_height.sh] Searching for alternative locations..."
    find "$HOME" -name "coast.sdf" -path "*mbzirc*" 2>/dev/null | head -5
    exit 1
fi

# Show current values
echo "[patch_geofence_height.sh] Current geofence:"
grep -A2 "<geofence>" "$WORLD_FILE" | head -5

# Calculate new center and size
# Keep min at -100m, set max to TARGET_HEIGHT
# New center Z = (TARGET_HEIGHT + (-100)) / 2
# New size Z = TARGET_HEIGHT - (-100) = TARGET_HEIGHT + 100
NEW_CENTER_Z=$(echo "scale=2; ($TARGET_HEIGHT - 100) / 2" | bc)
NEW_SIZE_Z=$((TARGET_HEIGHT + 100))

# Replace geofence center and size
sed -i "s/<center>0 0 [0-9.]*</<center>0 0 ${NEW_CENTER_Z}</g" "$WORLD_FILE"
sed -i "s/<size>3162.28 3162.28 [0-9.]*</<size>3162.28 3162.28 ${NEW_SIZE_Z}</g" "$WORLD_FILE"

# Show new values
echo "[patch_geofence_height.sh] New geofence:"
grep -A2 "<geofence>" "$WORLD_FILE" | head -5

echo "[patch_geofence_height.sh] Patch applied. Max height: ${TARGET_HEIGHT}m"
echo "${TARGET_HEIGHT}" > "$PATCH_MARKER"
