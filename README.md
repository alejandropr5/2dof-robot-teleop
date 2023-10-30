# ROS Noetic packages for 2dof-robot-teleop
## The repo consists on develop the teleoperation of a 2 degree cylindrical robot 

## Instalation

### :hammer: How to Build

To build the packages in this repository follow these steps:

1. Clone this repository in the `src` folder of your ROS Noetic [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):

   ```console
   git clone https://github.com/alejandropr5/2dof-robot-teleop.git
   ```
2. Install the requried binary dependencies of all packages in the catkin workspace using the following [`rosdep` command](http://wiki.ros.org/rosdep#Install_dependency_of_all_packages_in_the_workspace):

   ```
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. After installing the required dependencies build the catkin workspace, either with [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make):

   ```console
   catkin_make
   ```
## Usage

The following sections describe how to run the robot simulation

### Gazebo Simulation with ROS Control

Control the robot inside Gazebo and view what it sees in RViz using the following launch file:

```console
roslaunch teleoperation_lab1 robot2DoF.launch
```
### Run keyboard Teleoperation node for the robot2DoF

```console
rosrun teleoperation_lab1 teleop_robot2DoF_keyboard.py
```
