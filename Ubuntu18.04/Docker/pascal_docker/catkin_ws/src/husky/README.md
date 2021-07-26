husky (AML Teleop)
=====

Common ROS packages for the Clearpath Husky, useable for both simulation and
real robot operation, with added functionalities for simulating various remote teleoperation setups.

Husky packages:

 - husky_control : Control configuration
 - husky_description : Robot description (URDF)
 - husky_msgs : Message definitions
 - husky_navigation : Navigation configurations and demos

Added teleoperation functionalities:

 - Optional omnidirectional camera
 - Dense slam and pathplanning with RTABmap
 - RTABmap helper node to improve bandwidth use for remote dense reconstruction

For Husky instructions and tutorials, please see [Robots/Husky](http://wiki.ros.org/Robots/Husky).

To create a custom Husky description or simulation, please fork [husky_customization](https://github.com/husky/husky_customization).


Prerequisites
=============

The following ros packages are required:

 - [rtabmap & rtabmap_ros](https://github.com/introlab/rtabmap_ros)
 - [rtabmap_helper](https://github.com/HalfManHalfWookey/rtabmap_helper)
 - [image_tools](https://github.com/HalfManHalfWookey/image_tools)
 - [pointcloud_tools](https://github.com/HalfManHalfWookey/pointcloud_tools)

husky_simulator
==============

Simulator ROS packages for the Clearpath Husky.

 - husky_gazebo : Gazebo plugin definitions and extensions to the robot URDF.

### Runing teleoperation setups:

 - Launch a desired gazebo world (eg ` roslaunch husky_gazebo playpen.launch ` )
 - Launch husky\_teleop to spawn the husky in the world: ` roslaunch husky_gazebo husky_teleop.launch `)

husky_desktop
=============

Desktop ROS packages for the Clearpath Husky, which may pull in graphical dependencies.

 - husky_viz : Visualization (rviz) configuration and bringup

For Husky instructions and tutorials, please see http://wiki.ros.org/Robots/Husky

husky_robot
===========

Robot ROS packages for the Clearpath Husky, for operating robot hardware.

 - husky_bringup : Bringup launch files and scripts.
 - husky_base : Hardware driver for communicating with the onboard MCU.

For Husky instructions and tutorials, please see http://wiki.ros.org/Robots/Husky


For Husky instructions and tutorials, please see http://wiki.ros.org/Robots/Husky
