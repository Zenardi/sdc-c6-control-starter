FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# System packages
# ---------------------------------------------------------------------------
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    # Build toolchain
    build-essential \
    g++ \
    cmake \
    make \
    git \
    # SSL / compression (uWS runtime deps)
    libssl-dev \
    libz-dev \
    libuv1-dev \
    # Boost (used by CARLA stubs and CMakeLists.txt)
    libboost-all-dev \
    # Google logging + testing
    libgoogle-glog-dev \
    libgtest-dev \
    # LAPACK (linked by pid_controller target)
    liblapack-dev \
    # Python 3 + pip (simulatorAPI.py, plot_pid.py, twiddle_tune.py)
    python3 \
    python3-pip \
    python3-dev \
    # Plotting
    gnuplot \
    # Convenience
    curl \
    wget \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Build GTest static libraries
# (apt provides sources under /usr/src/gtest, not prebuilt .a files)
# ---------------------------------------------------------------------------
RUN cd /usr/src/gtest && cmake . && make \
    && cp lib/*.a /usr/local/lib/ 2>/dev/null || cp *.a /usr/local/lib/ 2>/dev/null || true

# ---------------------------------------------------------------------------
# uWebSockets — pinned to the commit used in install-ubuntu.sh
# ---------------------------------------------------------------------------
RUN git clone https://github.com/uWebSockets/uWebSockets /tmp/uws \
    && cd /tmp/uws && git checkout e94b6e1 \
    && mkdir build && cd build && cmake .. && make && make install \
    && ln -sf /usr/local/lib/libuWS.so /usr/lib/libuWS.so \
    && rm -rf /tmp/uws

# ---------------------------------------------------------------------------
# Python dependencies
# ---------------------------------------------------------------------------
RUN pip3 install --no-cache-dir numpy matplotlib pandas

# ---------------------------------------------------------------------------
# Non-root user for VS Code Dev Containers
# ---------------------------------------------------------------------------
RUN useradd -ms /bin/bash vscode \
    && echo "vscode ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER vscode
WORKDIR /workspaces/sdc-c6-control-starter
