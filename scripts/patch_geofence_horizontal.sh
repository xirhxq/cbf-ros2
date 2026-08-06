#!/bin/bash
# patch_geofence_horizontal.sh - Widen the horizontal geofence boundary.
#
# The default coast.sdf geofence is 3162.28 x 3162.28 (±1581m), only ~81m
# beyond the ±1500m search area. Under estimator-in-the-loop control, EKF
# position errors (tens of meters) can push UAVs past this boundary, triggering
# mbzirc GameLogicPlugin's exceed_boundary_2 -> Finish() -> pause, which
# terminates the run prematurely. This patch widens the horizontal geofence so
# the simulation completes and the estimator-in-loop behavior can be evaluated.
#
# Usage: ./scripts/patch_geofence_horizontal.sh [size]
#   size: full width of the square geofence in meters (default 4000 -> ±2000m,
#         giving ±500m beyond the search area to absorb estimator error).
set -e

SIZE_M=${1:-100000}

PATCH_MARKER="$HOME/.geofence_horizontal_patched"
SRC_FILE="$HOME/cbf_ws/src/cbf-ros2/scripts/../../cbf/config/config.json"  # not used, kept for layout parity
WORLD_FILE="$HOME/mbzirc_ws/src/mbzirc/mbzirc_ign/worlds/coast.sdf"

if [ -f "$PATCH_MARKER" ]; then
    echo "[patch_geofence_horizontal.sh] Already patched, skipping."
    exit 0
fi

if [ ! -f "$WORLD_FILE" ]; then
    echo "[patch_geofence_horizontal.sh] Error: coast.sdf not found at $WORLD_FILE"
    exit 1
fi

echo "[patch_geofence_horizontal.sh] Widening geofence to ${SIZE_M} x ${SIZE_M}..."
# Replace the horizontal size inside <geofence><size>X Y Z</size></geofence>.
# Keep Z (500) as-is; only widen X/Y.
python3 - "$WORLD_FILE" "$SIZE_M" << 'PY'
import re, sys
path, size = sys.argv[1], sys.argv[2]
text = open(path).read()
# Fix center to explicit 3 values (original "0 0 " with trailing space can
# cause Vector3d parse issues in GameLogicPlugin), then widen size.
new = re.sub(r"(<geofence>.*?<center>)[^<]*(</center>)",
             r"\g<1>0 0 0\g<2>", text, count=1, flags=re.DOTALL)
new = re.sub(
    r"(<geofence>.*?<size>)\S+\s+\S+\s+500(</size>.*?</geofence>)",
    lambda m: m.group(1) + f"{size} {size} 500" + m.group(2),
    new, count=1, flags=re.DOTALL)
if new == text:
    print("[patch_geofence_horizontal.sh] WARNING: pattern not matched, no change")
    sys.exit(1)
open(path, "w").write(new)
print(f"[patch_geofence_horizontal.sh] geofence center -> 0 0 0, size -> {size} x {size}")
PY

touch "$PATCH_MARKER"
echo "[patch_geofence_horizontal.sh] Done (recompile mbzirc_ign not needed for .sdf)"
