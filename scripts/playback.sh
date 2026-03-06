#!/bin/bash
# playback.sh - Interactive playback script for simulation logs
#
# Usage: ./scripts/playback.sh
#
# This script:
# 1. Lists available log directories in cbf/data/
# 2. Copies selected log to /tmp/ign (standard playback location)
# 3. Launches ign gazebo with playback.sdf

set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
DATA_DIR="$SCRIPT_DIR/../cbf/data"

# Find all directories containing state.tlog
echo "Scanning for playback logs in $DATA_DIR..."
LOG_DIRS=()
while IFS= read -r -d '' tlog; do
    # Get parent directory of ign/mbzirc/playback (the directory containing ign)
    playback_dir=$(dirname "$tlog")  # .../ign/mbzirc/playback
    mbzirc_dir=$(dirname "$playback_dir")  # .../ign/mbzirc
    ign_dir=$(dirname "$mbzirc_dir")  # .../ign
    log_dir=$(dirname "$ign_dir")  # .../timestamp
    LOG_DIRS+=("$log_dir")
done < <(find "$DATA_DIR" -name "state.tlog" -print0 2>/dev/null)

if [ ${#LOG_DIRS[@]} -eq 0 ]; then
    echo "No playback logs found in $DATA_DIR"
    echo "Make sure you have copied /tmp/ign to cbf/data/<timestamp>/ign"
    exit 1
fi

# Sort by time (directory names are timestamps, newest first)
IFS=$'\n' SORTED_DIRS=($(sort -r <<<"${LOG_DIRS[*]}")); unset IFS

# Display list
echo ""
echo "Available playback logs:"
echo "------------------------"
for i in "${!SORTED_DIRS[@]}"; do
    timestamp=$(basename "${SORTED_DIRS[$i]}")
    data_json="${SORTED_DIRS[$i]}/data.json"
    if [ -f "${data_json}" ]; then
        size=$(du -h "${data_json}" 2>/dev/null | cut -f1)
        echo "  [$((i+1))] ${timestamp} (data.json: ${size})"
    else
        echo "  [$((i+1))] ${timestamp}"
    fi
done
echo ""

# User selection
read -p "Select log to playback [1-${#SORTED_DIRS[@]}]: " choice

if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt ${#SORTED_DIRS[@]} ]; then
    echo "Invalid selection"
    exit 1
fi

SELECTED_DIR="${SORTED_DIRS[$((choice-1))]}"
SOURCE_IGN_DIR="$SELECTED_DIR/ign"
TARGET_IGN_DIR="/tmp/ign"

echo ""
echo "Selected: $(basename "$SELECTED_DIR")"
echo "Source: $SOURCE_IGN_DIR"
echo "Target: $TARGET_IGN_DIR"
echo ""

# Check if /tmp/ign already exists
if [ -d "$TARGET_IGN_DIR" ]; then
    echo "[playback.sh] Warning: $TARGET_IGN_DIR already exists"
    read -p "Overwrite? [y/N]: " overwrite
    if [[ "$overwrite" =~ ^[Yy]$ ]]; then
        echo "[playback.sh] Removing existing $TARGET_IGN_DIR ..."
        rm -rf "$TARGET_IGN_DIR"
    else
        echo "[playback.sh] Aborted"
        exit 0
    fi
fi

# Copy log directory to /tmp/ign
echo "[playback.sh] Copying log files to $TARGET_IGN_DIR ..."
echo "[playback.sh] This may take a while for large logs..."
cp -r "$SOURCE_IGN_DIR" "$TARGET_IGN_DIR"
echo "[playback.sh] Copy complete"

# Verify the copy
if [ -f "$TARGET_IGN_DIR/mbzirc/playback/state.tlog" ]; then
    TLOG_SIZE=$(du -h "$TARGET_IGN_DIR/mbzirc/playback/state.tlog" | cut -f1)
    echo "[playback.sh] Verified: state.tlog ($TLOG_SIZE)"
else
    echo "[playback.sh] Error: state.tlog not found after copy"
    exit 1
fi

# Check for journal file (unclean shutdown)
JOURNAL_FILE="$TARGET_IGN_DIR/mbzirc/playback/state.tlog-journal"
RECOVERED_FILE="$TARGET_IGN_DIR/mbzirc/playback/state_recovered.tlog"

if [ -f "$JOURNAL_FILE" ]; then
    echo ""
    echo "[playback.sh] Journal file detected (unclean shutdown)"

    # Check for sqlite3
    if ! command -v sqlite3 &> /dev/null; then
        echo "[playback.sh] Warning: sqlite3 not installed"
        echo "[playback.sh] Install with: sudo apt-get update && sudo apt-get install -y sqlite3"
        echo "[playback.sh] Continuing with original file (may have issues)"
    else
        echo "[playback.sh] Recovering database..."

        TLOG_FILE="$TARGET_IGN_DIR/mbzirc/playback/state.tlog"
        sqlite3 "$TLOG_FILE" ".recover" | sqlite3 "$RECOVERED_FILE" 2>/dev/null || true

        if [ -f "$RECOVERED_FILE" ] && [ -s "$RECOVERED_FILE" ]; then
            # Replace original with recovered
            mv "$RECOVERED_FILE" "$TLOG_FILE"
            rm -f "$JOURNAL_FILE"
            echo "[playback.sh] Database recovered successfully"
        else
            echo "[playback.sh] Warning: Recovery failed, using original file"
            rm -f "$RECOVERED_FILE"
        fi
    fi
fi

echo ""
echo "[playback.sh] Log ready at: $TARGET_IGN_DIR/mbzirc/playback"
echo ""
echo "Starting playback..."
echo "  - Wait for GUI to load (may take several minutes)"
echo "  - Click Play button to start playback"
echo "  - Use scrubber to navigate through the recording"
echo ""

# Launch playback using standard playback.sdf
ign gazebo -v 4 $(ros2 pkg prefix mbzirc_ign)/share/mbzirc_ign/worlds/playback.sdf
