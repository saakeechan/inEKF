#!/bin/bash

# Build script for InEKF with Python bindings

set -e  # Exit on error

echo "=========================================="
echo "Building InEKF Estimator with Python Bindings"
echo "=========================================="

# Install pybind11 if not already installed
echo ""
echo "Checking for pybind11..."
if ! python3 -c "import pybind11" 2>/dev/null; then
    echo "pybind11 not found. Installing..."
    pip3 install "pybind11[global]"
else
    echo "pybind11 already installed"
fi

# Navigate to build directory
cd "$(dirname "$0")/build"

echo ""
echo "Running CMake..."
cmake ..

echo ""
echo "Building project..."
if [ -f "build.ninja" ]; then
    echo "Using Ninja build system..."
    ninja
else
    echo "Using Make build system..."
    make -j$(nproc)
fi

echo ""
echo "=========================================="
echo "Build complete!"
echo "=========================================="

# Check if Python module was built
if [ -f "inekf_estimator*.so" ]; then
    echo ""
    echo "✓ Python module built successfully: $(ls inekf_estimator*.so)"
    echo ""
    echo "To use the Python module, either:"
    echo "  1. Copy it to your Python site-packages:"
    echo "     sudo cp inekf_estimator*.so /usr/local/lib/python3.*/dist-packages/"
    echo ""
    echo "  2. Add build directory to PYTHONPATH:"
    echo "     export PYTHONPATH=$PWD:\$PYTHONPATH"
    echo ""
    echo "  3. Copy to your project directory:"
    echo "     cp inekf_estimator*.so ../src/"
    echo ""
    
    # Test import
    echo "Testing Python import..."
    cd ..
    if PYTHONPATH=build:$PYTHONPATH python3 -c "import inekf_estimator; print('✓ Import successful! Version:', inekf_estimator.__version__)" 2>/dev/null; then
        echo ""
        echo "You can now run the ROS2 node with:"
        echo "  PYTHONPATH=build:\$PYTHONPATH python3 src/ros2_inekf_bridge.py"
    else
        echo "⚠ Import test failed, but module was built"
    fi
else
    echo ""
    echo "⚠ Python module was not built. Make sure pybind11 is properly installed."
    echo "   Install with: pip3 install 'pybind11[global]'"
fi

echo ""
echo "C++ executable: ./INEKF_Estimator_Test"
echo "=========================================="
