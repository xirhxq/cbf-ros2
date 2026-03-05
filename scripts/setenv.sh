#!/bin/bash
# setenv.sh - Load UAV_NUM from config.json

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_PATH="$SCRIPT_DIR/cbf/config/config.json"

# Parse config.json using python
eval $(python3 -c "
import json
with open('$CONFIG_PATH', 'r') as f:
    config = json.load(f)
print(f'export UAV_NUM={config.get(\"num\", 6)}')
")

echo "[setenv.sh] UAV_NUM=$UAV_NUM"
