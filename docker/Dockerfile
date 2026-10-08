FROM ubuntu:22.04

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install basic dependencies (cached layer)
RUN apt-get update && apt-get install -y \
    build-essential \
    clang \
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
    ninja-build \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python3-venv \
 && rm -rf /var/lib/apt/lists/*

RUN python3 -m venv --system-site-packages /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Enable ccache for faster rebuilds
ENV CCACHE_DIR=/ccache
ENV PATH="/usr/lib/ccache:${PATH}"

# Default container builds to Clang, ccache, and Ninja.
ENV CC=clang
ENV CXX=clang++
ENV CMAKE_GENERATOR=Ninja

# Upgrade pip (cached layer)
RUN /opt/venv/bin/pip install --upgrade pip && \
    /opt/venv/bin/pip install numpy

# Install Pinocchio dependencies (cached layer)
RUN apt-get update && apt-get install -y \
    robotpkg-py3*-pinocchio \
    || (apt-get install -y lsb-release curl gnupg && \
        echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list && \
        curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | gpg --dearmor -o /etc/apt/trusted.gpg.d/robotpkg.gpg && \
        apt-get update && \
        apt-get install -y robotpkg-pinocchio robotpkg-example-robot-data) \
    && rm -rf /var/lib/apt/lists/*

# Set up Pinocchio paths
ENV PATH="/opt/openrobots/bin:${PATH}" \
    PKG_CONFIG_PATH="/opt/openrobots/lib/pkgconfig" \
    LD_LIBRARY_PATH="/opt/openrobots/lib" \
    CMAKE_PREFIX_PATH="/opt/openrobots"



# Install ONNX Runtime (cached layer)
WORKDIR /tmp
RUN wget https://github.com/microsoft/onnxruntime/releases/download/v1.25.1/onnxruntime-linux-x64-1.25.1.tgz && \
    tar -xzf onnxruntime-linux-x64-1.25.1.tgz && \
    cp -r onnxruntime-linux-x64-1.25.1/include/* /usr/local/include/ && \
    cp -r onnxruntime-linux-x64-1.25.1/lib/* /usr/local/lib/ && \
    ldconfig && \
    rm -rf onnxruntime-linux-x64-1.25.1*

# Install Python packages for data analysis and plotting
RUN /opt/venv/bin/pip install pandas matplotlib

# Install meshcat for Pinocchio visualization
RUN /opt/venv/bin/pip install meshcat

# Set working directory
WORKDIR /workspace

# Copy CMakeLists.txt and source files (for cmake configuration)
COPY CMakeLists.txt .
COPY src/ src/
COPY config/ config/
COPY Extra_Files/ Extra_Files/

# Use all available cores for plain cmake --build in interactive shells.
RUN echo 'export CMAKE_BUILD_PARALLEL_LEVEL=$(nproc)' >> /etc/bash.bashrc

CMD ["/bin/bash"]
