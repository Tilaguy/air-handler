# ================================
# Dockerfile: ROS 2 Humble + Gazebo Fortress + OpenGL Debug Tools
# ================================

# Use official ROS 2 Humble desktop image as base
FROM osrf/ros:humble-desktop-full

# Metadata
LABEL maintainer="tu@email.com"
LABEL version="1.3"
LABEL description="ROS 2 Humble + Gazebo Fortress + OpenGL debug tools"

# Environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Install basic dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg \
    lsb-release \
    wget \
    git \
    && rm -rf /var/lib/apt/lists/*

# Add OSRF repository for Gazebo Fortress
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# Install Gazebo Fortress (new naming scheme gz-*)
RUN apt-get update && apt-get install -y --no-install-recommends \
    gz-fortress \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Gazebo bridge packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-msgs \
    ros-humble-gazebo-dev \
    && rm -rf /var/lib/apt/lists/*

# === OpenGL & Debugging tools ===
RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libegl1-mesa \
    && rm -rf /var/lib/apt/lists/*

# Setup environment for ROS 2 and Gazebo
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "export GZ_VERSION=fortress" >> /root/.bashrc

# Expose Gazebo default port
EXPOSE 11345

# Workspace directory
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Copy entrypoint and environment configuration
COPY entrypoint.sh /entrypoint.sh
COPY ros2_env.sh /ros2_env.sh
RUN chmod +x /entrypoint.sh /ros2_env.sh

# Entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
