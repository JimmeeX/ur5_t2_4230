# MTRN4230 Group 6 Project

MTRN4230 Group Project source code.

## Project Structure

- /config - UR5 Controller Manager Config Files
- /include - C++ header files (mostly for plugins)
- /launch - Launch Files to run the application
- /models - Gazebo Models (.urdf, .sdf, .xacro, etc)
  - 4230_objects - Pickable objects of various colours and shapes
  - break_beam - Laser Proximity Sensor
  - conveyor_belt_in - Conveyor Belt for Pickable Objects
  - conveyor_belt_out - Conveyor Belt for Blue Delivery Containers
  - kinect - RGB-D Sensor
  - ur5 - UR5 Robot Arm Manipulator
- /msg - Custom Messages for Publishers & Subscribers
  - ConveyorBeltState - Conveyor Belt custom message
  - Proximity - Break Beam custom message
- /src - ROS Nodes + Custom Scripts
  - spawner.py - Spawns objects & containers
- /srv - Custom Service request / response definitions
  - ConveyorBeltControl - Sets power to conveyor belts
- /world - Gazebo World Files

## 3rd Party Packages

Name | Description | Notes
---|---|---
[universal_robot](https://github.com/ros-industrial/universal_robot) | Provides UR Models for Simulation | Already installed
[ur5_ROS-Gazebo](https://github.com/lihuang3/ur5_ROS-Gazebo) | UR5 Examples | Already installed

## Installation & Setup

Environment: **ROS Kinetic (Ubuntu 16.04)**

1. Download Code

    1.1. Delete existing ur5_t2_4230 directory (backup if you made any changes).

    ```bash
    rm -rf ~/simulation_ws/src/ur5_t2_4230
    ```

    1.2. Download the Github code into ROS Workspace

    ```bash
    cd ~/simulation_ws/src && git clone git@github.com:JimmeeX/ur5_t2_4230.git
    ```

    1.3. Install required packages for Gazebo Math Ignition

    ```bash
    sudo apt-get install libignition-math2-dev
    ```

    1.4. Install required package for custom srv for Matlab

    ```bash
    sudo apt-get install ros-kinetic-rosbridge-library
    ```

2. Build Environment

    2.1. Enable ROS Commands & Access to Packages (Protip: Add this to ~/.bashrc)

    ```bash
    source ~/simulation_ws/devel/setup.bash
    ```

    2.2. Disable GPU Config for Gazebo (otherwise Gazebo will crash) (Protip: Add this to ~/.bashrc)

    ```bash
    export SVGA_VGPU10=0
    ```

    2.3. (Required if using MATLAB) Configure ROS IP address to allow communication with host environment. (Protip: Add this to ~/.bashrc)

    ```bash
    export ROS_IP=<UBUNTU IP ADDRESS>
    ```

    2.4. Build Environment

    ```bash
    cd ~/simulation_ws && catkin_make
    ```

3. Run

    3.1. Launch Gazebo Simulation Environment

    ```bash
    roslaunch ur5_t2_4230 ur5_world.launch
    ```

## Development

- Make branches and try not to push straight to master :)
