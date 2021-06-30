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

The following ros packages are required to obtain all features of the expanded interface:

| **Feature**|**Package(s)** |
|-|-|
|Depth Camera|Included|
|Lidar|Included|
|Omnidirectional 360 Camera|[image_tools](https://github.com/HalfManHalfWookey/image_tools)|
|Rtabmap|[rtabmap & rtabmap_ros](https://github.com/introlab/rtabmap_ros)|
|Unity Rtabmap Reconstruction|[rtabmap & rtabmap_ros](https://github.com/introlab/rtabmap_ros) [rtabmap_helper](https://github.com/HalfManHalfWookey/rtabmap_helper)  [pointcloud_tools](https://github.com/HalfManHalfWookey/pointcloud_tools)|

nb: The current setup of all launchfiles assumes that all packages are present and has not been tested otherwise. Some launchfiles might need editing to ensure things work if all packages arent installed!

husky_simulator
==============

Simulator ROS packages for the Clearpath Husky.

 - husky_gazebo : Gazebo plugin definitions and extensions to the robot URDF.

### Husky Launch Files
* playpen.launch - Launches the playpen gazebo environment. 
* husky_teleop.launch - Spawns a husky robot in the current gazebo environment (launched seperately). Enable sensors through arguments `realsense_enabled` `laser_enabled` and `omnicamera_enabled`
* husky_playpen_teleop.launch  - Launches the playpen test environment and spawns a husky with all sensors enabled and all teleop features. Requires all packages installed.
* husky_arena_teleop.launch - Launches the arena test environment and spawns a husky with all sensors enabled and all teleop features. Requires all packages installed.

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
