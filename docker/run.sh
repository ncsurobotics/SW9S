export SW9_DOCKER_WORKING_DIR="$(pwd)"

if [ "$(uname -m)" = "aarch64" ]; then
    export SW9_ARCH="kilted-arm64-latest"
else
    export SW9_ARCH="kilted-latest"
fi
export FASTDDS_DEFAULT_PROFILES_FILE="${SW9_DOCKER_WORKING_DIR}/fastdds_no_shm.xml"

docker compose up
