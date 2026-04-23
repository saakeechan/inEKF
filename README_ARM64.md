# Building INEKF Library for ARM64

This project supports building the INEKF library for ARM64 devices (Raspberry Pi, ARM servers, Jetson, etc.)

## Option 1: Docker Build (RECOMMENDED) ⭐

The easiest and most reliable method. Works on any x86_64 machine.

```bash
./build_arm64_docker.sh
```

**Output:** `build_arm64_docker/libINEKF_Estimator.so` (ARM64)

## Verifying the Build

Check that the library is indeed ARM64:

```bash
file build_arm64_docker/libINEKF_Estimator.so
```

Should output: `...ARM aarch64...`

## Dependencies for ARM64

The library requires ARM64 versions of:
- Eigen3 (header-only, no special build needed)
- Pinocchio
- Boost libraries
- yaml-cpp
- ONNX Runtime
- urdfdom and related libraries

The Docker method installs these automatically.
