#!/bin/bash
# run_with_gui.sh - Launch cbf-ros2 container WITH GUI (X11 + GPU).
#
# Same as docker/run_docker.sh but kept separate so headless and GUI runs
# don't interfere. Uses the original tmuxinator.cbf-ros2.yml (with GUI).
#
# Usage:
#   scripts/run_with_gui.sh           # X11 + GPU, interactive bash
#   scripts/run_with_gui.sh -- <cmd>  # run specific command
#
# Inside the container, launch the sim with GUI:
#   tmuxinator start . -p src/cbf-ros2/tmuxinator.cbf-ros2.yml
#
# Note: GUI mode has ~0.2x RTF (much slower than headless). Use only when
# you need to visually verify UAV behavior.

set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
HOST_WS_DIR="$SCRIPT_DIR/.."
IMAGE_NAME="cbf-ros2"
CONTAINER_WS_DIR="/home/developer/cbf_ws"

if [ $# -ge 1 ] && [ "$1" != "--" ]; then
    IMAGE_NAME="$1"; shift
fi

# X11 setup (same as docker/run_docker.sh)
XAUTH=/tmp/.docker.xauth
xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/' 2>/dev/null || true)
if [ ! -f "$XAUTH" ]; then
    touch "$XAUTH"; chmod a+r "$XAUTH"
    [ -n "$xauth_list" ] && echo "$xauth_list" | xauth -f "$XAUTH" nmerge - 2>/dev/null || true
fi
xhost +local:root > /dev/null 2>&1 || true

DOCKER_OPTS=""
DOCKER_VER=$(dpkg-query -f='${Version}' --show docker-ce 2>/dev/null | sed 's/[0-9]://' || echo "0")
if dpkg --compare-versions 19.03 gt "$DOCKER_VER" 2>/dev/null; then
    DOCKER_OPTS="$DOCKER_OPTS --runtime=nvidia"
else
    DOCKER_OPTS="$DOCKER_OPTS --gpus all"
fi

VIMRC=~/.vimrc
[ -f "$VIMRC" ] && DOCKER_OPTS="$DOCKER_OPTS -v $VIMRC:/home/developer/.vimrc:ro"

CONTAINER_CMD="/bin/bash"
[ "$1" = "--" ] && shift
[ $# -gt 0 ] && CONTAINER_CMD="$@"

exec docker run --rm -it \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -v "$XAUTH:$XAUTH" \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "/dev/input:/dev/input" \
  -v "${HOST_WS_DIR}:${CONTAINER_WS_DIR}/src/cbf-ros2" \
  -v "${HOST_WS_DIR}/.tmux.conf:${CONTAINER_WS_DIR}/.tmux.conf:ro" \
  --network host \
  --privileged \
  --security-opt seccomp=unconfined \
  $DOCKER_OPTS \
  -w "$CONTAINER_WS_DIR" \
  "$IMAGE_NAME" \
  $CONTAINER_CMD
