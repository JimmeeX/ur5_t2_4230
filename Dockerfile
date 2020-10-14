ARG PROJECT_NAME=ur5_t2_4230

FROM ros:kinetic-ros-base

SHELL ["/bin/bash", "-c"]

# Install Gazebo && Packages && Initialise Catkin Workspace
RUN apt-get update && apt-get install --no-install-recommends -y \
    ros-kinetic-gazebo-* \
    ros-kinetic-eigen-conversions \
    ros-kinetic-xacro \
    ros-kinetic-rosbridge-server \
    ros-kinetic-robot-state-publisher \
  && rm -rf /var/lib/apt/lists/* \
  && mkdir -p /root/catkin_ws/src/$PROJECT_NAME

# Initialise ROS Commands (eg, roscd)
# RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash"

# Set Working Directory and Copy Files
WORKDIR /root/catkin_ws/src/$PROJECT_NAME
COPY . .

# Build Files
RUN source /opt/ros/kinetic/setup.bash \
  && cd /root/catkin_ws \
  && catkin_make

# Copy and Set Entrypoint
COPY docker-entrypoint.sh /
ENTRYPOINT [ "/docker-entrypoint.sh" ]

CMD [ "bash" ]

# Initialise ROS Packages (ie, ur5_t2_4230)

# CMD roslaunch ur5_t2_4230 ur5_world.launch