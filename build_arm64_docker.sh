#!/bin/bash
# Build ARM64 library using Docker with QEMU emulation

set -e

# Accept optional platform argument (e.g. --platform amd64 or --platform aarch64)
HOST_ARCH=""
for arg in "$@"; do
    case "$arg" in
        --platform=*) HOST_ARCH="${arg#--platform=}" ;;
        --platform)   shift; HOST_ARCH="$1" ;;
    esac
    shift 2>/dev/null || true
done

# Auto-detect if not provided
if [ -z "$HOST_ARCH" ]; then
    HOST_ARCH="$(uname -m)"
fi

echo "================================================"
echo "ARM64 Docker Build Script"
echo "================================================"
echo ""
echo "This builds the library for ARM64 using Docker"
echo "with QEMU emulation (if not on ARM64 device)"
echo "Host architecture: $HOST_ARCH"
echo ""

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "ERROR: Docker not found!"
    echo "Please install Docker: https://docs.docker.com/get-docker/"
    exit 1
fi

# Enable ARM64 emulation when not running on an ARM64 host
if [ "$HOST_ARCH" != "aarch64" ] && [ "$HOST_ARCH" != "arm64" ]; then
    echo "Non-ARM64 host detected ($HOST_ARCH). Setting up ARM64 emulation..."
    docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
    echo "ARM64 emulation enabled."
    echo ""
fi

# Build the Docker image
echo "Building ARM64 Docker image (this may take a while)..."
docker build -t inekf-arm64 -f Dockerfile.arm64 .

# Run the build inside Docker
echo ""
echo "Building INEKF library for ARM64..."
docker run --rm --platform linux/arm64 \
    -v "$(pwd):/workspace" \
    inekf-arm64 \
    bash -c "cd /workspace && mkdir -p build_arm64_docker && cd build_arm64_docker && \
             cmake .. -DCMAKE_BUILD_TYPE=Release && \
             cmake --build . -j2 && \
             echo '' && \
             echo 'Build complete!' && \
             file libINEKF_Estimator.so"

echo ""
echo "================================================"
echo "✓ ARM64 library built successfully!"
echo "Output: build_arm64_docker/libINEKF_Estimator.so"
echo "================================================"