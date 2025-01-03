#!/bin/bash
set -e

IMAGE_NAME="cbf-ros2"
DOCKERHUB_USER="xirhxq"

PLATFORMS=("amd64" "arm64")

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "${SCRIPT_DIR}"

ARCH=$(uname -m)

if [[ "$ARCH" == "x86_64" ]]; then
  PLATFORM="amd64"
elif [[ "$ARCH" == "arm64" || "$ARCH" == "aarch64" ]]; then
  PLATFORM="arm64"
else
  echo "❌ Unsupported architecture: $ARCH"
  exit 1
fi

FULL_TAG="${DOCKERHUB_USER}/${IMAGE_NAME}:${PLATFORM}"

echo "🚀 Building and pushing ${FULL_TAG} for ${PLATFORM}..."
docker build -t "${FULL_TAG}" .
docker push "${FULL_TAG}"
echo "✅ Built and pushed ${FULL_TAG}"

echo "❓ Do you want to update the Docker Manifest? (y/n)"
read -r UPDATE_MANIFEST

if [[ "$UPDATE_MANIFEST" == "y" || "$UPDATE_MANIFEST" == "Y" ]]; then
    echo "🚧 Creating and pushing manifest..."
    docker manifest create "${DOCKERHUB_USER}/${IMAGE_NAME}:latest" \
        "${DOCKERHUB_USER}/${IMAGE_NAME}:amd64" \
        "${DOCKERHUB_USER}/${IMAGE_NAME}:arm64" \
        --amend

    docker manifest push "${DOCKERHUB_USER}/${IMAGE_NAME}:latest"
    echo "✅ Manifest updated successfully!"
else
  echo "⏭️ Skipping manifest update."
fi
