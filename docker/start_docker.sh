#!/usr/bin/env bash

set -e
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../" && pwd -P)"
TARGET_ARCH="$(uname -m)"

DOCKER_EXEC=""

if [ -f "/usr/bin/nvidia-docker" ]; then
  DOCKER_EXEC=nvidia-docker
else
  DOCKER_EXEC=docker
fi


function local_volume(){
  local __retval="$1"
  local volumes="-v /media:/media
                 -v /tmp/.X11-unix:/tmp/.X11-unix:rw
                 -v /etc/localtime:/etc/localtime:ro
                 -v /usr/src:/usr/src
                 -v /lib/modules:/lib/modules
                 -v /dev:/dev
		 -v /sys:/sys"
  volumes="$(tr -s " " <<<"${volumes}")"
  eval "${__retval}='${volumes}'"
}

function main(){
  local SHM_SIZE="4G"
  local DEV_IMAGE=
  local DEV_CONTAINER=

  if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    DEV_IMAGE="osrf/ros:noetic-desktop-full"
    DEV_CONTAINER="tinyCar_x86_${USER}"
  elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
    DEV_IMAGE="tiryoh/ros-desktop-vnc:arm64_v2.2"
    DEV_CONTAINER="tinyCar_arm64_${USER}"
  fi
  echo ${DEV_IMAGE}   
  echo ${DEV_CONTAINER}  
  echo ${ROOT_DIR}   

  local display="${DISPLAY:-:0}"
  local volume_dir
  local_volume volume_dir



  ${DOCKER_EXEC} run  -itd  \
    --privileged \
    --name "${DEV_CONTAINER}" \
    -e DISPLAY="${display}" \
    ${volume_dir} \
    -v ${ROOT_DIR}:/home/tinyCar_ws \
    -w /home/tinyCar_ws \
    --device=/dev/ttyUSB*:/dev/ttyUSB* \
    --net host \
    --hostname "${DEV_INSIDE}" \
    --pid=host  \
    --shm-size "${SHM_SIZE}" \
    ${DEV_IMAGE} \
    /bin/bash
}
main
