# MTRN4230 Group 6 Project

MTRN4230 Group Project source code.

## Project Structure
- /config - UR5 Controller Manager Config Files
- /include - C++ header files (mostly for plugins)
- /launch - Launch Files to run the application
- /models - Gazebo Models (.urdf, .sdf, .xacro, etc)
- /msg - Custom Messages for Publishers & Subscribers
- /src - ROS Nodes, plugin scripts, test scripts, utility functions
- /srv - Custom Service request / response definitions
- /web - React-based Web GUI
- /world - Gazebo World Files

## 3rd Party ROS Packages

Name | Description | Notes
---|---|---
[universal_robot](https://github.com/ros-industrial/universal_robot) | Provides UR Models for Simulation | Already installed
[ur5_ROS-Gazebo](https://github.com/lihuang3/ur5_ROS-Gazebo) | UR5 Examples | Already installed
[rosbridge_server, rosbridge_library](https://github.com/RobotWebTools/rosbridge_suite) | Required for Web-based GUI | Not Installed

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

    1.4. Install rosbridge_server to communicate with Web GUI
    ```bash
    sudo apt-get install ros-kinetic-rosbridge-server
    ```

    1.5. Install npm (or Yarn) to launch the Web GUI
    ```bash
    curl -sL https://deb.nodesource.com/setup_14.x | sudo -E bash -
    sudo apt-get install -y nodejs
    ```


2. Build Environment

    2.1. Enable ROS Commands & Access to Packages (Protip: Add this to ~/.bashrc)

    ```bash
    source ~/simulation_ws/devel/setup.bash
    ```

    2.2. Disable GPU Config for Gazebo (otherwise Gazebo will crash) (Protip: Add this to ~/.bashrc). Note that if your environment has a GPU driver, you can ignore this.

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

    2.5. Update npm packages for Web GUI

    ```bash
    cd ~/simulation_ws/src/ur5_t2_4230/web/roswebcontroller
    npm install
    ```

3. Run

    3.1. Launch Gazebo Simulation Environment

    ```bash
    roslaunch ur5_t2_4230 ur5_world.launch
    ```

    3.2. Launch Web GUI

    ```bash
    cd ~/simulation_ws/src/ur5_t2_4230/web/roswebcontroller
    npm start
    ```

## Development

- Make branches and try not to push straight to master :)
