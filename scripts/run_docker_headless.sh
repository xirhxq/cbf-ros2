#!/bin/bash
# run_docker_headless.sh - Launch cbf-ros2 container for headless (no GUI) MBZIRC runs.
#
# Differences from docker/run_docker.sh:
#   - No X11/xauth/xhost (Ignition runs server-only with -s, no GUI client).
#   - GPU optional: pass --gpus to enable (useful for future camera rendering).
#     Without --gpus the container uses CPU, which is enough for -s server-only.
#   - Mounts the host repo at /home/developer/cbf_ws/src/cbf-ros2 (same as original).
#
# Usage:
#   .work/scripts/run_docker_headless.sh                # CPU, interactive bash
#   .work/scripts/run_docker_headless.sh --gpus         # GPU enabled
#   .work/scripts/run_docker_headless.sh -- <cmd>       # run <cmd> instead of bash

set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
HOST_WS_DIR="$SCRIPT_DIR/../.."

IMAGE_NAME="cbf-ros2"
CONTAINER_WS_DIR="/home/developer/cbf_ws"

if [ $# -ge 1 ] && [ "$1" != "--gpus" ] && [ "$1" != "--" ]; then
    IMAGE_NAME="$1"
    shift
fi

DOCKER_OPTS=""
INTERACTIVE="-it"

# Parse our own flags
while [ $# -gt 0 ]; do
    case "$1" in
        --gpus)
            DOCKER_OPTS="$DOCKER_OPTS --gpus all"
            shift
            ;;
        --)
            shift
            break
            ;;
        *)
            break
            ;;
    esac
done

# Remaining args become the container command (default: bash).
if [ $# -gt 0 ]; then
    INTERACTIVE=""
    CONTAINER_CMD=("$@")
else
    CONTAINER_CMD=("/bin/bash")
fi

VIMRC=~/.vimrc
[ -f "$VIMRC" ] && DOCKER_OPTS="$DOCKER_OPTS -v $VIMRC:/home/developer/.vimrc:ro"

exec docker run --rm \
    $INTERACTIVE \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "${HOST_WS_DIR}:${CONTAINER_WS_DIR}/src/cbf-ros2" \
    -v "${HOST_WS_DIR}/.tmux.conf:${CONTAINER_WS_DIR}/.tmux.conf:ro" \
    --network host \
    --privileged \
    --security-opt seccomp=unconfined \
    $DOCKER_OPTS \
    -w "$CONTAINER_WS_DIR" \
    "$IMAGE_NAME" \
    "${CONTAINER_CMD[@]}"
