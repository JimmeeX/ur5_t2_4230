ARG PROJECT_NAME=ur5_t2_4230

FROM ros:kinetic-ros-base

# Install Gazebo
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-kinetic-gazebo-* \
    ros-kinetic-eigen-conversions \
    ros-kinetic-xacro \
    ros-kinetic-rosbridge-server \
    ros-kinetic-robot-state-publisher
#   && rm -rf /var/lib/apt/lists/*

# Initialise Catkin Workspace
RUN mkdir -p /root/catkin_ws/src/$PROJECT_NAME

# Initialise ROS Commands (eg, roscd)
# RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash"

WORKDIR /root/catkin_ws/src/$PROJECT_NAME

COPY . .

# Build Files
RUN . /opt/ros/kinetic/setup.sh && \
    cd /root/catkin_ws && \
    catkin_make

RUN . /root/catkin_ws/devel/setup.sh
# Initialise ROS Packages (ie, ur5_t2_4230)

# CMD roslaunch ur5_t2_4230 ur5_world.launch