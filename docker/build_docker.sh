#!/bin/bash
set -e

IMAGE_NAME="cbf-ros2:galactic-highs"

# 切换到脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${SCRIPT_DIR}"

# 构建镜像
docker build -t "${IMAGE_NAME}" -f Dockerfile .
