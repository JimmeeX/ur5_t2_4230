# UNSW MTRN4230 Group 6 Project

MTRN4230 Group Project source code for a Gazebo-simulated pick-and-place robot.

## Demo
[Video](https://www.youtube.com/watch?v=OMTVitTJu90)

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

## Installation & Setup

### 1. Docker (requires NVIDIA GPU)

1. Setting Up

    1.1. Install [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker)

    1.2. Install xhost for your system

    1.3. [Configure](https://github.com/docker/compose/issues/6691#issuecomment-696465142) Docker Daemon to use GPU for docker-compose

2. Run Environment

Run the ```run.sh``` script:

```
./run.sh
```

If configured properly, the Gazebo Simulation environment will spawn. You can visit [http://localhost:3000](http://localhost:3000) to connect the Web GUI that monitors and controls the Gazebo environment and orders (see linked video for more info).


### 2. Manual (Requires Ubuntu 16.04 w/ ROS Kinetic)

1. Setting Up

    1.1. Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) (for future steps, I will be assuming ```ros-kinetic-ros-base``` was installed)

    1.2. Install [Gazebo7](http://gazebosim.org/tutorials?tut=install_ubuntu&ver=7.0&cat=install) simulator

    1.3. Install project ROS dependencies:

    ```bash
    sudo apt-get update && apt-get install \
        ros-kinetic-gazebo-msgs \
        ros-kinetic-gazebo-plugins \
        ros-kinetic-gazebo-ros \
        ros-kinetic-gazebo-ros-control \
        ros-kinetic-image-transport-plugins \
        ros-kinetic-controller-manager \
        ros-kinetic-eigen-conversions \
        ros-kinetic-xacro \
        ros-kinetic-rosbridge-server \
        ros-kinetic-robot-state-publisher \
        ros-kinetic-joint-state-controller \
        ros-kinetic-joint-trajectory-controller \
    ```

    1.4. Install npm (or Yarn) to launch the Web GUI
    ```bash
    curl -sL https://deb.nodesource.com/setup_14.x | sudo -E bash -
    sudo apt-get install -y nodejs
    ```

    1.5. Download the Github code into ROS Workspace

    ```bash
    cd ~/catkin_ws/src && git clone git@github.com:JimmeeX/ur5_t2_4230.git
    ```


2. Build Environment

    2.1. Enable ROS Commands & Access to Packages (Protip: Add this to ~/.bashrc)

    ```bash
    source ~/catkin_ws/devel/setup.bash
    ```

    2.2. If you are using a virtual environment, disable GPU config for Gazebo (prevent Gazebo from crashing) (Protip: Add this to ~/.bashrc).

    ```bash
    export SVGA_VGPU10=0
    ```

    2.3. Build Environment

    ```bash
    cd ~/catkin_ws && catkin_make
    ```

    2.5. Update npm packages for Web GUI

    ```bash
    cd ~/catkin_ws/src/ur5_t2_4230/web
    npm install
    ```

3. Run

    3.1. Launch Gazebo Simulation Environment

    ```bash
    roslaunch ur5_t2_4230 ur5_world.launch
    ```

    3.2. Launch Web GUI (accessible via [http://localhost:3000](http://localhost:3000))

    ```bash
    cd ~/simulation_ws/src/ur5_t2_4230/web/roswebcontroller
    npm start
    ```

## Inspirations
- [ur5_ROS-Gazebo](https://github.com/lihuang3/ur5_ROS-Gazebo)
- [Gilbreth](https://github.com/swri-robotics/gilbreth)
- [OSRF ARIAC Gear](https://bitbucket.org/osrf/ariac/src/master/osrf_gear/)
- [universal_robot](https://github.com/ros-industrial/universal_robot)

