# ROS2 Jazzy Robot Project

This repository contains a ROS2 Jazzy project for robot simulation and control, designed to run on Ubuntu 24.04.
Prerequisites

  - Operating System: Ubuntu 24.04
  
  - ROS2 Distribution: Jazzy

## Installation
1. Install ROS2 Jazzy

Follow the comprehensive installation tutorial from 
https://learnbydoing.dev/installing-ros-2-jazzy-on-ubuntu-24-04-step-by-step-tutorial/ 

to set up ROS2 Jazzy on your Ubuntu 24.04 system.

2. Install ROS2 Packages


    sudo apt-get install ros-jazzy-ros2-control
    sudo apt-get install ros-jazzy-ros2-controllers
    sudo apt-get install ros-jazzy-xacro
    sudo apt-get install ros-jazzy-ros-gz-*
    sudo apt-get install ros-jazzy-*-ros2-control
    sudo apt-get install ros-jazzy-joint-state-publisher-gui
    sudo apt-get install ros-jazzy-tf-transformations
    sudo apt-get install ros-jazzy-moveit*

3. Install Python Dependencies


    sudo apt-get install python3-pip
    sudo apt-get install python3-transforms3d
    sudo apt-get install python3-flask

Install Python packages with pip (note: using --break-system-packages flag):

    pip3 install pyserial --break-system-packages
    pip3 install flask-ask-sdk --break-system-packages
    pip3 install ask-sdk --break-system-packages

4. Install Additional System Dependencies

    sudo apt-get install libserial-dev

## Building the Project

Build the workspace:

    colcon build

Source the setup file:

    source install/setup.bash

or

    . install/setup.bash

## Running the Simulation

Launch the simulated robot with the following command:
bash

    ros2 launch arduinobot_bringup simulated_robot.launch.py


## Features

  - ROS2 control system integration

  - Gazebo simulation support

  - MoveIt motion planning

  - Web interface capabilities (Flask)

  - Serial communication for hardware integration

  - Joint state visualization and control

## Notes

  - Ensure you have sourced the ROS2 Jazzy environment before building and running

  - The project uses the latest ROS2 Jazzy distribution packages

  - Python packages are installed with --break-system-packages flag to accommodate Ubuntu 24.04's strict package isolation

  - Make sure all dependencies are installed in the specified order

## Troubleshooting

If you encounter issues:

  - Verify all ROS2 Jazzy packages are installed correctly

  - Ensure the workspace builds without errors

  - Confirm that the ROS2 environment is properly sourced

  - Check Python package installation if web/serial features don't work

  - Validate Ubuntu 24.04 and ROS2 Jazzy compatibility

For detailed troubleshooting, refer to the ROS2 Jazzy installation guide.
