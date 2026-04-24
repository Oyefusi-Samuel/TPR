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
    build-essential \
    cmake \
    git \
    curl \
    wget \
    vim \
    nano \
    bash-completion \
    iputils-ping \
    net-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    ros-jazzy-topic-tools \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-controller-manager \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-moveit \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-localization \
    ros-jazzy-twist-mux \
    ros-jazzy-joy-linux \
    ros-jazzy-imu-filter-madgwick \
    ros-jazzy-interactive-marker-twist-server \
    ros-jazzy-urdf-launch \
    ros-jazzy-warehouse-ros-sqlite \
    && rm -rf /var/lib/apt/lists/*

# ---------- workspace setup ---------------------------------
WORKDIR ${WORKSPACE}

# Copy repository into the image (build/ install/ log/ excluded by .dockerignore)
COPY . ${WORKSPACE}/src/

# Build the workspace
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    cd ${WORKSPACE}/src && \
    colcon build --symlink-install"

# ---------- environment -------------------------------------
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "source ${WORKSPACE}/src/install/setup.bash" >> /etc/bash.bashrc

ENV WORKSPACE=${WORKSPACE}

# ---------- entrypoint --------------------------------------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
