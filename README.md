# Preliminary Exoskeleton Simulation Using ROS 2

This repository is for the ROS2-based exoskeleton simulation developed by LLEAP, a Cal Poly club project. The simulated Gazebo model of our biped, lower-limb exoskeleton is based on custom SolidWorks designs. The goal of the simulation is to provide a test bed for control code, and to root out design flaws before the expense of manufacturing. This is still a very preliminary simulation, although work is underway to flush out geometries and physical properties. See below for details on how to download, run, and control our simulation.

## Prerequisites
- ROS2 Humble
- ros_control 
- Gazebo Sim
- A catkin workspace
- git
- A Github account connected to our group

## Project Structure
The project is organized into a few different ROS packages that were reformatted from our previous repo which can be found on gitlab here https://gitlab.com/lleap/simulation/preliminary-simulation. Packages and their contents are split by their function for modularity and ease of simultaneous development.
* `models` describes the physical model of our robot. This package contains the URDF/xacro files and related physical property files.
* `launch` contains the launch files and associated launch config files
* `meshes` contains the Solidworks meshes for the urdf
* `rviz` contains the rviz configuration files
* `src` Is currently experimental, containing an example node

## Downloading the project
1. Launch a terminal window and navigate to the `src` folder of your catkin workspace.

2. To download this project (essentially all of its packages), type the following into the command line: 
    ```
    git clone https://github.com/MaxLewter16/Preliminary-Simulation-ROS-2.git
    ```
3. Back out of the `src` folder into the top level of your catkin workspace.
    ```
    cd ..
    ```
4. Build the package (i.e. make it runnable) by typing the following into terminal:
    ```
    colcon_build
    ```
    The `lleap_exo_XXX` packages should now be ready and runnable.  

    You can control the simulation through the terminal or with `rqt`.
  

## Controlling the simulation through the terminal

1. Launch a terminal window and navigate to your catkin workspace. Once there, type:
    ```
    source devel/setup.bash
    ```
2. In the same terminal window, type (Note: Should have tab auto-fill):
    ```
    roslaunch launch preliminary_exo.launch.py
    ```
    After a few seconds, the model should open in RVIZ.
