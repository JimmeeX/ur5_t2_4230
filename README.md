# MTRN4230 Group 6 Project 
MTRN4230 Group Project source code.

## Project Structure (TODO)
TODO

## 3rd Party Packages (TODO)
Name | Description | Notes
---|---|---
[Universal Robotics](https://github.com/ros-industrial/universal_robot) | Provides UR Models for Simulation | Already installed
[ur5_ROS-Gazebo](https://github.com/lihuang3/ur5_ROS-Gazebo) | UR5 Examples | Alread Installed


## Installation & Setup

Environment: **ROS Kinetic (Ubuntu 16.04)**

1. Download the code into ROS Workspace
    ```
    git clone git@github.com:JimmeeX/ur5_t2_4230.git ~/simulation_ws/src
    ```

2. Build Environment

    2.1. Navigate to the ROS Workspace
    ```
    cd ~/simulation_ws
    ```

    2.2. Enable ROS Commands & Access to Packages (Protip: Add this to ~/.bashrc)
    ```
    source ~/simulation_ws/devel/setup.bash
    ```

    2.3. Disable GPU Config for Gazebo (otherwise Gazebo will crash) (Protip: Add this to ~/.bashrc)
    ```
    export SVGA_VGPU10=0
    ```

    2.4. (Required if using MATLAB) Configure ROS IP address to allow communication with host environment. (Protip: Add this to ~/.bashrc)
    ```
    export ROS_IP=<UBUNTU IP ADDRESS>
    ```

    2.5. Build

    ```
    catkin_make
    ```

3. Run (for now)

    3.1. Launch Gazebo Simulation Environment
    ```
    roslaunch ur5_t2_4230 ur5_world.launch
    ```