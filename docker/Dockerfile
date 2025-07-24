# ================================
# Dockerfile: ROS 2 Humble + Gazebo Fortress + OpenGL Debug Tools
# ================================

FROM osrf/ros:humble-desktop-full

LABEL maintainer="tu@email.com"
LABEL version="2.0"
LABEL description="ROS 2 Humble + Gazebo Fortress (Ignition) + OpenGL debug tools"

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg \
    lsb-release \
    wget \
    git \
    && rm -rf /var/lib/apt/lists/*

# ========================
# Add Gazebo Fortress repo
# ========================
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# Install Gazebo Fortress
RUN apt-get update && apt-get install -y --no-install-recommends \
    gz-fortress \
    && rm -rf /var/lib/apt/lists/*

# ========================
# ROS 2 ↔ Gazebo Fortress bridge
# (ros-gz instead of gazebo_ros_pkgs)
# ========================
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# === OpenGL & Debugging tools ===
RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libegl1-mesa \
    && rm -rf /var/lib/apt/lists/*

# Configure environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "export GZ_VERSION=fortress" >> /root/.bashrc

EXPOSE 11345

# Workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Copy entrypoint
COPY entrypoint.sh /entrypoint.sh
COPY ros2_env.sh /ros2_env.sh
COPY validate_env.sh /validate_env.sh
RUN chmod +x /entrypoint.sh /ros2_env.sh /validate_env.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
