#!/bin/bash

# Docker helper script for INEKF

IMAGE_NAME="inekf"
CONTAINER_NAME="inekf-container"
HOST_UID=$(id -u)
HOST_GID=$(id -g)

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

function print_usage() {
    echo "Usage: ./docker.sh [command]"
    echo ""
    echo "Commands:"
    echo "  build       - Build the Docker image"
    echo "  run         - Run the container interactively"
    echo "  exec        - Execute a command in running container"
    echo "  configure   - Configure CMake with Clang, ccache, and Ninja"
    echo "  fix-perms   - Fix workspace file ownership to current user"
    echo "  stop        - Stop the running container"
    echo "  clean       - Remove container and image"
    echo "  rebuild     - Clean and rebuild the image"
    echo ""
}

function build_image() {
    echo -e "${BLUE}Building Docker image: ${IMAGE_NAME}${NC}"
    docker build -t ${IMAGE_NAME} .
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Image built successfully${NC}"
    else
        echo -e "${RED}✗ Build failed${NC}"
        exit 1
    fi
}

function run_container() {
    echo -e "${BLUE}Running Docker container: ${CONTAINER_NAME}${NC}"
    mkdir -p .ccache && chmod 777 .ccache
    docker run -it --rm \
        --name ${CONTAINER_NAME} \
        --user ${HOST_UID}:${HOST_GID} \
        -e HOME=/tmp \
        -e CCACHE_DIR=/workspace/.ccache \
        -e CC=clang \
        -e CXX=clang++ \
        -e CMAKE_GENERATOR=Ninja \
        -v /etc/passwd:/etc/passwd:ro \
        -v /etc/group:/etc/group:ro \
        -v $(pwd):/workspace \
        ${IMAGE_NAME} \
        /bin/bash
}

function exec_container() {
    if [ -z "$2" ]; then
        docker exec -it --user ${HOST_UID}:${HOST_GID} ${CONTAINER_NAME} /bin/bash
    else
        docker exec -it --user ${HOST_UID}:${HOST_GID} ${CONTAINER_NAME} "${@:2}"
    fi
}

function configure_cmake() {
    echo -e "${BLUE}Configuring CMake with Clang, ccache, and Ninja${NC}"
    if [ ! -f /.dockerenv ]; then
        mkdir -p .ccache && chmod 777 .ccache
        docker run --rm \
            --user ${HOST_UID}:${HOST_GID} \
            -e HOME=/tmp \
            -e CCACHE_DIR=/workspace/.ccache \
            -e CC=clang \
            -e CXX=clang++ \
            -e CMAKE_GENERATOR=Ninja \
            -v /etc/passwd:/etc/passwd:ro \
            -v /etc/group:/etc/group:ro \
            -v $(pwd):/workspace \
            ${IMAGE_NAME} \
            /bin/bash -lc "./docker.sh configure"
        return $?
    fi

    mkdir -p build .ccache
    rm -f build/CMakeCache.txt build/build.ninja build/.ninja_deps build/.ninja_log
    rm -rf build/CMakeFiles
    cmake -S . -B build -G Ninja \
        -DCMAKE_C_COMPILER=clang \
        -DCMAKE_CXX_COMPILER=clang++ \
        -DCMAKE_CXX_COMPILER_LAUNCHER=/usr/bin/ccache
}

function fix_permissions() {
    echo -e "${BLUE}Fixing workspace ownership to UID:GID ${HOST_UID}:${HOST_GID}${NC}"
    docker run --rm \
        -v $(pwd):/workspace \
        ${IMAGE_NAME} \
        chown -R ${HOST_UID}:${HOST_GID} /workspace
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Workspace ownership fixed${NC}"
    else
        echo -e "${RED}✗ Failed to fix ownership${NC}"
        exit 1
    fi
}

function stop_container() {
    echo -e "${BLUE}Stopping container: ${CONTAINER_NAME}${NC}"
    docker stop ${CONTAINER_NAME}
}

function clean() {
    echo -e "${BLUE}Cleaning up Docker resources${NC}"
    docker rm -f ${CONTAINER_NAME} 2>/dev/null
    docker rmi ${IMAGE_NAME} 2>/dev/null
    echo -e "${GREEN}✓ Cleanup complete${NC}"
}

function rebuild() {
    clean
    build_image
}

# Main script logic
case "$1" in
    build)
        build_image
        ;;
    run)
        run_container
        ;;
    exec)
        exec_container "$@"
        ;;
    configure)
        configure_cmake
        ;;
    fix-perms)
        fix_permissions
        ;;
    stop)
        stop_container
        ;;
    clean)
        clean
        ;;
    rebuild)
        rebuild
        ;;
    *)
        print_usage
        ;;
esac
