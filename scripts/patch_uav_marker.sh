#!/bin/bash
# patch_uav_marker.sh - Copy UAV model with red sphere marker

PATCH_MARKER="$HOME/.uav_marker_patched"
SRC_MODEL="$HOME/cbf_ws/src/cbf-ros2/scripts/models/mbzirc_quadrotor_base/model.sdf"
DST_DIR="$HOME/mbzirc_ws/src/mbzirc/mbzirc_ign/models/mbzirc_quadrotor_base"
DST_MODEL="$DST_DIR/model.sdf"

if [ -f "$PATCH_MARKER" ]; then
    echo "[patch_uav_marker.sh] Already patched, skipping."
    exit 0
fi

echo "[patch_uav_marker.sh] Adding red sphere marker to UAV model..."

if [ ! -f "$SRC_MODEL" ]; then
    echo "[patch_uav_marker.sh] Error: Source model not found at $SRC_MODEL"
    exit 1
fi

if [ ! -d "$DST_DIR" ]; then
    echo "[patch_uav_marker.sh] Error: Destination directory not found at $DST_DIR"
    exit 1
fi

# Backup original and copy new model
cp "$DST_MODEL" "${DST_MODEL}.original"
cp "$SRC_MODEL" "$DST_MODEL"

echo "[patch_uav_marker.sh] Model copied successfully!"

# No need to recompile for model.sdf changes (only .sdf.erb needs recompile)
touch "$PATCH_MARKER"
