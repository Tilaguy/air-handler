# Stage 1: Base image with ROS 2 Humble and Gazebo Harmonic
FROM osrf/ros:humble-desktop-full

# Install core dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    libignition-common5-dev \
    libignition-math7-dev \
    libignition-msgs10-dev \
    libignition-transport13-dev \
    libsdformat14-dev \
    libgz-sim8-dev \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    && rm -rf /var/lib/apt/lists/*

# Setup environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Default workspace location (overridden by mount)
WORKDIR /ros2_ws

ENTRYPOINT ["/entrypoint.sh"]