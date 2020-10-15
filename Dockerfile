FROM nvidia/cudagl:11.1-base-ubuntu16.04
LABEL maintainer="jameslin3118@gmail.com"

SHELL ["/bin/bash", "-c"]

# Run a full upgrade and install utilities for development.
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    mesa-utils \
    wget \
    vim \
    build-essential gdb \
    cmake cmake-curses-gui \
    git \
    ssh \
 && rm -rf /var/lib/apt/lists/*

# Register the ROS package sources.
ENV UBUNTU_RELEASE=xenial
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $UBUNTU_RELEASE main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS.
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full \
  && rm -rf /var/lib/apt/lists/*

# Install Gazebo && Packages && Initialise Catkin Workspace
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-kinetic-gazebo-* \
    ros-kinetic-eigen-conversions \
    ros-kinetic-xacro \
    ros-kinetic-rosbridge-server \
    ros-kinetic-robot-state-publisher \
    ros-kinetic-joint-state-controller \
    ros-kinetic-joint-trajectory-controller \
  && rm -rf /var/lib/apt/lists/* \
  && mkdir -p /root/catkin_ws/src/ur5_t2_4230

# Set Working Directory and Copy Files
WORKDIR /root/catkin_ws/src/ur5_t2_4230
COPY . .

# Build Files
RUN source /opt/ros/kinetic/setup.bash \
  && cd /root/catkin_ws \
  && catkin_make

# # Copy and Set Entrypoint
COPY docker-entrypoint.sh /
ENTRYPOINT [ "/docker-entrypoint.sh" ]

CMD [ "bash" ]