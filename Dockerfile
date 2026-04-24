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
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-interfaces \
    ros-jazzy-moveit \
    && rm -rf /var/lib/apt/lists/*
# ---------- rosdep init (skip if already initialised) -------
RUN rosdep update

# ---------- workspace setup ---------------------------------
WORKDIR ${WORKSPACE}

# Copy your repository into the image.
COPY . ${WORKSPACE}/src/

# Install rosdep dependencies for all workspaces
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    rosdep install --from-paths \
      src --ignore-src -r -y"

# [1/3] Build TPR root workspace (a_star_smooth_planner)
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    cd ${WORKSPACE}/src && \
    colcon build --symlink-install"

# [2/3] Build ar4_ros_driver
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    source ${WORKSPACE}/src/install/setup.bash && \
    cd ${WORKSPACE}/src/ar4_ros_driver && \
    colcon build --symlink-install"

# [3/3] Build goal_manager_pkg
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    source ${WORKSPACE}/src/install/setup.bash && \
    source ${WORKSPACE}/src/ar4_ros_driver/install/setup.bash && \
    cd ${WORKSPACE}/src/goal_manager_pkg && \
    colcon build --symlink-install"

# ---------- environment -------------------------------------
# Auto-source ROS and all workspaces on every shell session
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "source ${WORKSPACE}/src/install/setup.bash" >> /etc/bash.bashrc && \
    echo "source ${WORKSPACE}/src/ar4_ros_driver/install/setup.bash" >> /etc/bash.bashrc && \
    echo "source ${WORKSPACE}/src/goal_manager_pkg/install/setup.bash" >> /etc/bash.bashrc

ENV WORKSPACE=${WORKSPACE}

# ---------- entrypoint --------------------------------------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
