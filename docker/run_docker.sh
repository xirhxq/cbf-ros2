#!/bin/bash
set -e

IMAGE_NAME="cbf-ros2:galactic-highs"

# 这里指定你本地的工作空间路径，例如 ~/cbf_ws
HOST_WS_DIR="$HOME/cbf_ws"

# 容器内映射路径，例如 /home/appuser/cbf_ws
CONTAINER_WS_DIR="/home/appuser/cbf_ws"

docker run --rm -it \
    --network host \
    -v "${HOST_WS_DIR}:${CONTAINER_WS_DIR}" \
    -w "${CONTAINER_WS_DIR}" \
    "${IMAGE_NAME}" \
    /bin/bash
