#!/bin/bash
set -e

IMAGE_NAME="cbf-ros2"

echo "🚀 Building ${IMAGE_NAME}..."
docker build -t "${IMAGE_NAME}" .
echo "✅ Built ${IMAGE_NAME}"
