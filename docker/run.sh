set -euo pipefail
export SW9_DOCKER_WORKING_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

MACHINE="$(uname -m)"

if [ "${MACHINE}" = "aarch64" ]; then
    if [ -f /etc/nv_tegra_release ]; then
        echo "[run.sh] Jetson detected (L4T $(cat /etc/nv_tegra_release | head -1))"

        export SW9_ARCH="jetson"
        export TARGETARCH="arm64"
        export DEPTHAI_TAG="kilted-arm64-latest"
        export YOLO_DOCKERFILE="Dockerfile.ARM"

        export NVIDIA_VISIBLE_DEVICES=all
        export NVIDIA_DRIVER_CAPABILITIES=all

    else
        echo "[run.sh] Generic ARM64 detected"
        export SW9_ARCH="arm"
        export DEPTHAI_TAG="kilted-arm64-latest"
        export YOLO_DOCKERFILE="Dockerfile"
    fi
else
    echo "[run.sh] x86-64 detected"
    export SW9_ARCH="x86"
    export TARGET_ARCH=""
    export DEPTHAI_TAG="kilted-latest"
    export YOLO_DOCKERFILE="Dockerfile"
fi

export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${SW9_DOCKER_WORKING_DIR}/fastdds_no_shm.xml"

echo "[run.sh] SW9_ARCH         = ${SW9_ARCH}"
echo "[run.sh] DEPTHAI_TAG      = ${DEPTHAI_TAG}"
echo "[run.sh] YOLO_DOCKERFILE  = ${YOLO_DOCKERFILE}"
echo "[run.sh] RMW              = ${RMW_IMPLEMENTATION}"

cd "${SW9_DOCKER_WORKING_DIR}"
docker compose up
