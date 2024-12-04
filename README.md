# MSD Spot Arm
This repository is for all of the ROS2 packages made for the MSD Spot robot arm. 
The project ran during the 2024 semesters and this repository contains all of
the ROS2 Humble code.

# Required software
- Ubuntu 22.04 (Jammy)
- ROS2 Humble
- Ros2\_control
- Moveit

It is possible to do all development 

# Project Layout

`arm_motor_controller`
: This has the `ros2_control` controllers for the gripper and the arm.

`arm_moveit_config`
: The moveit launch and launch configurations for inverse kinematics solving, and
motion planning

`arm_servoing`
: Contains the realtime servoing custom node for robot control and its
respective launch files and launch file configs

`sw_arm_desc`
: "Solidworks arm description". This is the result of the [Solidworks URDF
exporter](http://wiki.ros.org/sw_urdf_exporter), and the
[sw2urdf\_ros2](https://github.com/xiaoming-sun6/sw2urdf_ros2) tool. Contains
the true arm URDF that gets referenced by many other packages.

`sauron`
: This oversees the arm, and has a homing button, which publishes to the
`/home_request` topic. This package isn't strictly needed, as the homing could
be tied to a button press in the custom servoing node that will be discussed
later.

`marge`
: Marge controls the homing algorithm. (It controls homer) It is a
`ros2_control` controller, who's node subscribes to the `/home_request` topic,
and sends a value down to the hardware interfaces specified in the controller's
yaml configuration.


`*_old`
: These two are old, deprecated packages that contained URDF stuff regarding an
older revision of the robotic arm

`arm_pkg`
: This was intended to be a package containing the highest level scripts and
launches for working with the real arm, and putting everything together.


# Building the packages

Build like you would any other ROS2 package. 

Before building the packages, the repository needs to be cloned to a ROS workspace. This can be setup by running
To set up a ROS2 workspace 

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <REPOSITORY PAGE>
cd ~/ros2_ws/
rosdep update --rosdistro=$ROS_DISTRO
sudo rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO}
```
You may need to `sudo apt-get update` before the `rosdep install` step

Once the workspace is setup along with any dependencies, the packages can be built with

```bash
cd ~/ros2_ws/
. /opt/ros/${ROS_DISTRO}/setup.sh
colcon build --merge-install
source install/setup.sh
```

An important thing to note is the compile option `--merge-install`.
The merge-install option will put all of the installed things into one big
folder. We used merge-install most times because Matt was having difficulty in
his WSL environment. You can read more about [colcon, and merge install
here](https://colcon.readthedocs.io/en/released/user/isolated-vs-merged-workspaces.html)
Some tutorials reccomend using `--simlink-install`, which is very helpful for
rapid development.

# Extremely useful resources
This was a "baptism by fire" for me and matt, so a lot of this code is hacked up
examples. The main portions of the project can be broken into the following:

1. `ros2_control` hardware interfaces
2. Solidworks arm CAD modeling/ URDF export
3. Moveit configuration inverse kinematics 
4. Custom controller (marge) and RViz Panel (Sauron) for triggering homing sequence


# Future Work
Given more time, the following would've been done

- The URDF of the arm could be adjusted to have a better zero position, as I
  believe the arm is very close to a singularity in the state it was exported.

- In the joint trajectory controller for the Servo42D motor, for whatever
  reason, the spline generation just doesn't work, and the motors were never
  made fully operational. The behaviour was super weird.

- Tuning the various constants for the servoing node could be helpful to make
  the arm slow down before a singularity, have a large window around those
  singularities so its obvious to move in the opposite direction could be nice.

- Getting the motor velocities sync'd up and the posistions tuned so that the
  simulation of the arm (in RViz) matches the real world motor positions is a
  big one

- Gazebo very quickly turned into an after thought when the true scope of the
  project was realized. This is very un-researched on our end.
