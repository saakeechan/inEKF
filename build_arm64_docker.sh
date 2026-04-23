#!/bin/bash
# Build ARM64 library using Docker with QEMU emulation

set -e

echo "================================================"
echo "ARM64 Docker Build Script"
echo "================================================"
echo ""
echo "This builds the library for ARM64 using Docker"
echo "with QEMU emulation (if not on ARM64 device)"
echo ""

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "ERROR: Docker not found!"
    echo "Please install Docker: https://docs.docker.com/get-docker/"
    exit 1
fi

# Enable ARM64 emulation (for x86_64 hosts)
echo "Setting up ARM64 emulation..."
docker run --rm --privileged multiarch/qemu-user-static --reset -p yes

# Build the Docker image
echo "Building ARM64 Docker image (this may take a while)..."
docker build -t inekf-arm64 -f Dockerfile.arm64 .

# Run the build inside Docker
echo ""
echo "Building INEKF library for ARM64..."
echo "Note: Using multi-threaded build (8 cores, 7GB RAM limit)"
docker run --rm --platform linux/arm64 \
    --memory=7g \
    --memory-swap=7g \
    --cpus=12 \
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
