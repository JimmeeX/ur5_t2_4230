# MTRN4230 Group 6 Project 
MTRN4230 Group Project source code.

## Project Structure (TODO)
TODO

## 3rd Party Packages (TODO)
Name | Description | Notes
---|---|---
[universal_robot](https://github.com/ros-industrial/universal_robot) | Provides UR Models for Simulation | Already installed
[ur5_ROS-Gazebo](https://github.com/lihuang3/ur5_ROS-Gazebo) | UR5 Examples | Alread Installed


## Installation & Setup

Environment: **ROS Kinetic (Ubuntu 16.04)**
1. Download Code

    1.1. Delete existing ur5_t2_4230 directory (backup if you made any changes).
    ```
    rm -rf ~/simulation_ws/src/ur5_t2_4230
    ```

    1.2. Download the Github code into ROS Workspace
    ```
    cd ~/simulation_ws/src && git clone git@github.com:JimmeeX/ur5_t2_4230.git
    ```

2. Build Environment

    2.1. Enable ROS Commands & Access to Packages (Protip: Add this to ~/.bashrc)
    ```
    source ~/simulation_ws/devel/setup.bash
    ```

    2.2. Disable GPU Config for Gazebo (otherwise Gazebo will crash) (Protip: Add this to ~/.bashrc)
    ```
    export SVGA_VGPU10=0
    ```

    2.3. (Required if using MATLAB) Configure ROS IP address to allow communication with host environment. (Protip: Add this to ~/.bashrc)
    ```
    export ROS_IP=<UBUNTU IP ADDRESS>
    ```

    2.4. Build Environment

    ```
    cd ~/simulation_ws && catkin_make
    ```

3. Run (for now)

    3.1. Launch Gazebo Simulation Environment
    ```
    roslaunch ur5_t2_4230 ur5_world.launch
    ```