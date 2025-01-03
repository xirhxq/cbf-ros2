#!/bin/bash
set -e

IMAGE_NAME="cbf-ros2"
DOCKERHUB_USER="xirhxq"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${SCRIPT_DIR}"

docker build -t "${IMAGE_NAME}" .
docker tag "${IMAGE_NAME}" "${DOCKERHUB_USER}/${IMAGE_NAME}"
docker push "${DOCKERHUB_USER}/${IMAGE_NAME}"
echo "Built and pushed ${DOCKERHUB_USER}/${IMAGE_NAME}"