TARGET_ARCH="$(uname -m)"

DOCKER_USER=${USER}

DEV_CONTAINER=

if [[ "${TARGET_ARCH}" == "x86_64" ]]; then
    DEV_CONTAINER="tinyCar_x86_${USER}"
elif [[ "${TARGET_ARCH}" == "aarch64" ]]; then
    DEV_CONTAINER="tinyCar_arm64_${USER}"
fi

DOCKER_EXEC=""

if [ -f "/usr/bin/nvidia-docker" ]; then
  DOCKER_EXEC=nvidia-docker
else
  DOCKER_EXEC=docker
fi

${DOCKER_EXEC} start "${DEV_CONTAINER}"
xhost +local:root 1>/dev/null 2>&1
${DOCKER_EXEC} exec \
    -it "${DEV_CONTAINER}" \
    /bin/bash

xhost -local:root 1>/dev/null 2>&1

