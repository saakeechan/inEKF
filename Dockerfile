FROM ubuntu:22.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies (cached layer)
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    pkg-config \
    libboost-all-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    liburdfdom-dev \
    libconsole-bridge-dev \
    python3 \
    python3-dev \
    python3-pip \
    python3-numpy \
    ccache \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install Python packages required for GTSAM Python wrapper (cached layer)
RUN pip3 install --upgrade pip && \
    pip3 install pyparsing pybind11 numpy

# Install additional Boost libraries for GTSAM (cached layer)
RUN apt-get update && apt-get install -y \
    libboost-system-dev \
    libboost-filesystem-dev \
    libboost-thread-dev \
    libboost-date-time-dev \
    libboost-timer-dev \
    libboost-chrono-dev \
    libboost-serialization-dev \
    libtbb-dev \
    && rm -rf /var/lib/apt/lists/*

# Install GTSAM (cached layer - this takes a long time)
WORKDIR /tmp
RUN git clone https://github.com/borglab/gtsam.git && \
    cd gtsam && \
    mkdir build && cd build && \
    cmake .. -DGTSAM_BUILD_PYTHON=ON && \
    make -j1 && \
    make install && \
    cd ../.. && rm -rf gtsam
    

# Install Pinocchio dependencies (cached layer)
RUN apt-get update && apt-get install -y \
    robotpkg-py3*-pinocchio \
    || (apt-get install -y lsb-release curl gnupg && \
        echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list && \
        curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | apt-key add - && \
        apt-get update && \
        apt-get install -y robotpkg-pinocchio robotpkg-example-robot-data) \
    && rm -rf /var/lib/apt/lists/*

# Set up Pinocchio paths
ENV PATH="/opt/openrobots/bin:${PATH}" \
    PKG_CONFIG_PATH="/opt/openrobots/lib/pkgconfig" \
    LD_LIBRARY_PATH="/opt/openrobots/lib" \
    CMAKE_PREFIX_PATH="/opt/openrobots"



# Install Ninja build system (after GTSAM to preserve cache)
RUN apt-get update && apt-get install -y ninja-build && rm -rf /var/lib/apt/lists/*

# Install ONNX Runtime (cached layer)
WORKDIR /tmp
RUN wget https://github.com/microsoft/onnxruntime/releases/download/v1.25.1/onnxruntime-linux-x64-1.25.1.tgz && \
    tar -xzf onnxruntime-linux-x64-1.25.1.tgz && \
    cp -r onnxruntime-linux-x64-1.25.1/include/* /usr/local/include/ && \
    cp -r onnxruntime-linux-x64-1.25.1/lib/* /usr/local/lib/ && \
    ldconfig && \
    rm -rf onnxruntime-linux-x64-1.25.1*

# Enable ccache for faster rebuilds
ENV CCACHE_DIR=/ccache
ENV PATH="/usr/lib/ccache:${PATH}"

# Install GTSAM Python package and matplotlib for plotting
RUN pip3 install gtsam pandas matplotlib

# Set working directory
WORKDIR /workspace

# Copy CMakeLists.txt and source files (for cmake configuration)
COPY CMakeLists.txt .
COPY src/ src/
COPY config/ config/
COPY convex/ convex/

# Build the project with Ninja
RUN mkdir -p build && cd build && \
    cmake -GNinja .. && \
    ninja -j4


CMD ["/bin/bash"]
