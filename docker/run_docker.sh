#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
HOST_WS_DIR="$SCRIPT_DIR/.."

IMAGE_NAME="xirhxq/cbf-ros2"

CONTAINER_WS_DIR="/home/appuser/cbf_ws"

docker run --rm -it \
    --network host \
    -v "${HOST_WS_DIR}:${CONTAINER_WS_DIR}/src/cbf-ros2" \
    -w "${CONTAINER_WS_DIR}" \
    "${IMAGE_NAME}" \
    /bin/bash
