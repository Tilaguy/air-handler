FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# Base tools
RUN apt-get update && apt-get install -y \
    locales curl tree gnupg lsb-release wget git \
    build-essential cmake python3-pip \
    && locale-gen en_US.UTF-8 && update-locale \
    && rm -rf /var/lib/apt/lists/*

# 1. Install ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-ament-package \
    ros-humble-rosidl-typesupport-c \
    && rm -rf /var/lib/apt/lists/*

# 2. Install Ignition Fortress (v7) and its dependencies
RUN curl -sSL https://packages.osrfoundation.org/gazebo.key | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
    > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y \
    ignition-fortress \
    libignition-gazebo7-dev \
    libignition-math7-dev \
    libignition-msgs9-dev \
    libignition-transport12-dev \
    libignition-common5-dev \
    libignition-fuel-tools8-dev \
    ros-humble-ros-ign-bridge \
    ros-humble-ros-ign-gazebo \
    ros-humble-ros-ign-interfaces \
    && rm -rf /var/lib/apt/lists/*

# 3. ROS developing tools
RUN pip3 install -U colcon-common-extensions vcstool rosdep pytest

# 4. Workspace configurations
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# 5. Envirometn configurations
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

COPY ros2_env.sh /ros2_env.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]