# ============================================================
# ROS2 Jazzy — Desktop Image
# Base: osrf/ros:jazzy-desktop
# ============================================================
FROM osrf/ros:jazzy-desktop

# ---------- build args (override at build time if needed) ---
ARG WORKSPACE=/ros2_ws
ARG DEBIAN_FRONTEND=noninteractive

# ---------- system dependencies -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # essentials
    build-essential \
    cmake \
    git \
    curl \
    wget \
    vim \
    nano \
    bash-completion \
    # networking / debug
    iputils-ping \
    net-tools \
    # ROS extras
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    && rm -rf /var/lib/apt/lists/*

# ---------- rosdep init (skip if already initialised) -------
RUN rosdep update

# ---------- workspace setup ---------------------------------
WORKDIR ${WORKSPACE}

# Copy your repository into the image.
# If you prefer to mount it at runtime (dev workflow), comment
# out the COPY line and rely on the volume in docker-compose.
COPY . ${WORKSPACE}/src/

# Install rosdep dependencies declared in your packages
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y"

# Build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install"

# ---------- environment -------------------------------------
# Auto-source ROS and the workspace on every shell session
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "source ${WORKSPACE}/install/setup.bash" >> /etc/bash.bashrc

ENV WORKSPACE=${WORKSPACE}

# ---------- entrypoint --------------------------------------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
