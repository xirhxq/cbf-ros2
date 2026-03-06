#!/bin/bash
# patch_geofence_height.sh - Patch geofence height limit in MBZIRC coast.sdf
#
# Usage: ./patch_geofence_height.sh [max_height_m] [min_height_m]
# Defaults: 200 -100

TARGET_HEIGHT_M=${1:-200}
GEOFENCE_MIN_M=${2:--100}

PATCH_MARKER="$HOME/.geofence_height_patched"
WORLD_FILE="$HOME/mbzirc_ws/src/mbzirc/mbzirc_ign/worlds/coast.sdf"

if [ -f "$PATCH_MARKER" ]; then
    echo "[patch_geofence_height.sh] Already patched, skipping."
    exit 0
fi

echo "[patch_geofence_height.sh] Patching geofence height to ${TARGET_HEIGHT_M}m..."

if [ ! -f "$WORLD_FILE" ]; then
    echo "[patch_geofence_height.sh] Error: coast.sdf not found at $WORLD_FILE"
    echo "[patch_geofence_height.sh] Searching for alternative locations..."
    find "$HOME" -name "coast.sdf" -path "*mbzirc*" 2>/dev/null | head -5
    exit 1
fi

echo "[patch_geofence_height.sh] Current geofence:"
grep -A2 "<geofence>" "$WORLD_FILE" | head -5

NEW_CENTER_Z=$(echo "scale=2; ($TARGET_HEIGHT_M + $GEOFENCE_MIN_M) / 2" | bc)
NEW_SIZE_Z=$((TARGET_HEIGHT_M - GEOFENCE_MIN_M))

sed -i "s/<center>0 0 [0-9.]*</<center>0 0 ${NEW_CENTER_Z}</g" "$WORLD_FILE"
sed -i "s/<size>3162.28 3162.28 [0-9.]*</<size>3162.28 3162.28 ${NEW_SIZE_Z}</g" "$WORLD_FILE"

echo "[patch_geofence_height.sh] New geofence:"
grep -A2 "<geofence>" "$WORLD_FILE" | head -5

echo "[patch_geofence_height.sh] Patch applied. Max height: ${TARGET_HEIGHT_M}m"
echo "${TARGET_HEIGHT_M}" > "$PATCH_MARKER"
