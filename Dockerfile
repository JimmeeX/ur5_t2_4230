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
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $UBUNTU_RELEASE main" > /etc/apt/sources.list.d/ros-latest.list' \
  && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
  && apt-get update && apt-get install -y \
    ros-kinetic-ros-base \
  && rm -rf /var/lib/apt/lists/*

# Install Gazebo7
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
  && wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
  && apt-get update && apt-get install -y \
    gazebo7 \
    ros-kinetic-gazebo-msgs \
    ros-kinetic-gazebo-plugins \
    ros-kinetic-gazebo-ros \
    ros-kinetic-gazebo-ros-control \
  && rm -rf /var/lib/apt/lists/*

# Install Project Dependencies && Initialise Catkin Workspace
RUN apt-get update && apt-get install -y \
    ros-kinetic-image-transport-plugins \
    ros-kinetic-controller-manager \
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