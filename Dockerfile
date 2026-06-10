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

# Upgrade pip and install Python packages required for GTSAM Python wrapper (cached layer)
RUN /opt/venv/bin/pip install --upgrade pip && \
    /opt/venv/bin/pip install pyparsing pybind11 numpy

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
    cmake .. -DGTSAM_BUILD_PYTHON=ON \
        -DPython3_EXECUTABLE=/opt/venv/bin/python3 \
        -DGTSAM_PYTHON_VERSION=3 \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache && \
    make -j$(nproc) && \
    make install && \
    cd ../.. && rm -rf gtsam
    

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



# Install GTSAM Python package and matplotlib for plotting
RUN /opt/venv/bin/pip install gtsam pandas matplotlib



# Install cvxpy with solvers for convex optimization
RUN /opt/venv/bin/pip install cvxpy ecos scs osqp clarabel

# Install LaTeX packages for document generation
RUN apt-get update && apt-get install -y \
    texlive-latex-base \
    texlive-latex-extra \
    texlive-fonts-recommended \
    texlive-fonts-extra \
    texlive-publishers \
    texlive-science \
    && rm -rf /var/lib/apt/lists/*

# Install meshcat for Pinocchio visualization
RUN /opt/venv/bin/pip install meshcat

# Set working directory
WORKDIR /workspace

# Copy CMakeLists.txt and source files (for cmake configuration)
COPY CMakeLists.txt .
COPY src/ src/
COPY config/ config/
COPY convex/ convex/
COPY Extra_Files/ Extra_Files/

CMD ["/bin/bash"]
