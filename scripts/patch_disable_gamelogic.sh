#!/bin/bash
# patch_disable_gamelogic.sh - Disable the MBZIRC GameLogicPlugin.
#
# The GameLogicPlugin enforces competition boundary penalties, run duration,
# and can pause/finish the simulation when UAVs exceed the geofence. For
# estimator-in-loop research runs we don't want the sim to terminate on
# boundary excursions (especially while tuning EKF parameters). This script
# comments out the <plugin> block so Ignition loads the coast world without
# the competition logic. The /clock topic (from ros_ign_bridge) is unaffected.
#
# Usage: ./scripts/patch_disable_gamelogic.sh
set -e

PATCH_MARKER="$HOME/.gamelogic_disabled"
WORLD_FILE="$HOME/mbzirc_ws/src/mbzirc/mbzirc_ign/worlds/coast.sdf"

if [ -f "$PATCH_MARKER" ]; then
    echo "[patch_disable_gamelogic.sh] Already patched, skipping."
    exit 0
fi

if [ ! -f "$WORLD_FILE" ]; then
    echo "[patch_disable_gamelogic.sh] Error: coast.sdf not found at $WORLD_FILE"
    exit 1
fi

echo "[patch_disable_gamelogic.sh] Disabling GameLogicPlugin..."

python3 - "$WORLD_FILE" << 'PY'
import sys, re
path = sys.argv[1]
text = open(path).read()
# Remove the GameLogicPlugin <plugin>...</plugin> block entirely.
# (Cannot use XML comments because the block itself contains <!-- --> comments,
# which would break XML nesting.)
pattern = r'\s*<plugin\s+filename="libGameLogicPlugin\.so".*?</plugin>'
new, n = re.subn(pattern, '', text, flags=re.DOTALL)
if n == 0:
    print("[patch_disable_gamelogic.sh] WARNING: GameLogicPlugin not found")
    sys.exit(1)
open(path, "w").write(new)
print(f"[patch_disable_gamelogic.sh] GameLogicPlugin removed ({n} block)")
PY

touch "$PATCH_MARKER"
echo "[patch_disable_gamelogic.sh] Done"
