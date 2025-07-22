FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# Base tools
RUN apt-get update && apt-get install -y \
    locales curl tree gnupg lsb-release wget git \
    build-essential cmake python3-pip\
    && locale-gen en_US.UTF-8 && update-locale \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y \
    python3-ament-package\
    ros-humble-rosidl-typesupport-c

# Gazebo Fortress
RUN curl -sSL https://packages.osrfoundation.org/gazebo.key | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
    https://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
    > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y \
    libgazebo-dev ignition-fortress \
    && rm -rf /var/lib/apt/lists/*

# colcon & vcstool via pip
RUN pip3 install -U colcon-common-extensions vcstool rosdep


# Setup workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Enviroment variables
COPY ros2_env.sh /ros2_env.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
