FROM osrf/ros:jazzy-desktop

# Install core dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-moveit \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Setup workspace
ENV WORKSPACE=/root/jackal_ar4_ws
WORKDIR $WORKSPACE

# Copy source code
COPY ./src $WORKSPACE/src

# Install remaining dependencies via rosdep
RUN . /opt/ros/jazzy/setup.sh && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# Entrypoint
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "jackal_ar4_description", "jackal_ar4.launch"]
