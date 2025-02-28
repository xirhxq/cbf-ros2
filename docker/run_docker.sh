#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
HOST_WS_DIR="$SCRIPT_DIR/.."

IMAGE_NAME="cbf-ros2"
CONTAINER_WS_DIR="/home/developer/cbf_ws"

if [ $# -ge 1 ]; then
    IMAGE_NAME="$1"
    shift
fi

XAUTH=/tmp/.docker.xauth
xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    touch $XAUTH
    chmod a+r $XAUTH
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    fi
fi

if [ ! -f $XAUTH ]; then
  exit 1
fi

xhost +local:root > /dev/null

DOCKER_OPTS=""
DOCKER_VER=$(dpkg-query -f='${Version}' --show docker-ce 2>/dev/null | sed 's/[0-9]://')

if dpkg --compare-versions 19.03 gt "$DOCKER_VER"
then
    echo "Docker version is less than 19.03, using nvidia-docker2 runtime"
    if ! dpkg --list | grep nvidia-docker2
    then
        echo "Please either update docker-ce to a version greater than 19.03 or install nvidia-docker2"
	exit 1
    fi
    DOCKER_OPTS="$DOCKER_OPTS --runtime=nvidia"
else
    DOCKER_OPTS="$DOCKER_OPTS --gpus all"
fi

VIMRC=~/.vimrc
[ -f $VIMRC ] && DOCKER_OPTS="$DOCKER_OPTS -v $VIMRC:/home/developer/.vimrc:ro"

docker run -it \
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
  "$@"