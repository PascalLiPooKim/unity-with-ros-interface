# AML Unity Immersive Interface backup - Unity Version 2019.4.13f

Backup of the Unity based teleop interface. This is a project that is in *active development*, with changes occuring regularly. Therefore, the project can be quite confusing and is not well organised. This version is built for Unity 2019.4.13f to overcome encountered bugs with RosSharp, and is the continuation of [mav-interface-unity](https://github.com/HalfManHalfWookey/mav-interface-unity).

## Installation
To add the Unity project into the Unity hub:
* Download Unity project in this repository and move to desired location on your machine.
* Download and install the Unity hub [https://unity3d.com/get-unity/download](https://unity3d.com/get-unity/download).
* Open the hub, click the `ADD` button and locate the project

We must now add the correct version of Unity to the hub to start the project
* Click the orange warning sign next to the project's listed Unity version to bring up a popup. On this popup select `Install`
* Leave the default module settings and click `Install`
* Once downloaded, launch into the Unity project
  
## Unity Scenes

### Husky Interfaces

As this project is used to prototype and design many differernt interface settups, the Scene functionality within Unity (designed to load/save different levels) is used to load into different interface setups.
To load into a new scene, click `File>Open Scene` and navigate to `Assets/Scenes/HuskyInterfaces/TeleopSetups` to find the interfaces designed using the Husky Gazebo Simulation:

* RTSP+Lidar - RGB Stream using RTSP and pointcloud representation of Lidar data
* RTSP+LidarTopDown - Same as previous, but with a top down perspective
* Realsense+Lidar - Pointcloud representation of Realsense data and pointcloud representation of Lidar data
* Realsense+Lidar - Same as previous, but with a top down perspective
* 360RTSP - RGB stream of 360 camera rendered on a inverted sphere
* RTABmapReconstruction - RtabmapHelper script that connects with RTABmap algorithm to construct dynamic pointcloud representation of environment
* RTABmapReconstructionTopDown - Same as previous, but with a top down perspective

## Using interface for basic teleoperation

To connect the interface to the gazebo server/robot and start teleoperation, the ROS and RTSP connections must first be set up

### Setting up the RosSharp connection
The RosConnector script deals with connecting to the remote rosbridge_server and must be connected to every gameObject that contains a RosSharp publisher/subscriber. 

**The RosConnector must have the correct server url :** 

**(ws://[IP ADDRESS]:9090)**

**and each publisher/subscriber must have the correct rostopic.**
 
To ensure the connection is setup correctly, go to all the following gameobjects and ensure the RosConnector IP and topic is correct:

* [**GameObject**]: [**Publisher/Subscriber**] [**topic:=**]
* Husky: OdometrySubscriber topic:= `/odometry/filtered` 
* RtabmapHelper: PointCloudMapDataSubscriber topic:= `/rtabmap_helper/pcl_mapdata` 
* TeleopControls: TwistTeleopPublisher topic:= `/husky_velocity_controller/cmd_vel`
* TeleopControls: PosePublisher topic:= `/move_base_simple/goal`
* ScanPointcloud: PcxPointCloudSubscriber topic:=`/scan/pointcloud`
* RealsensePointcloud: PcxPointCloudSubscriber topic:=`/realsense/depth/color/points/filtered`


### Setting up the RTSP connection

The RTSPConnector script deals with connecting and rendering any RTSP camera streams. 

**For a succesful connection, RTSPConnector must have the correct stream URL**

**rtsp://[IP ADDRESS]:8554/test**

For the RTSP scenes (360RTSP, RTSP+Lidar, RTSP+LidarTopDown), the relevant RTSPConnector scripts are found on the 360RTSP and CameraRTSP gameObjects.

The main settings are as follows:
* Uri: The RTSP stream Uri
* isUDP: Set as `true` if the connection uses UDP protocal, otherwise the system will default to TCP
* isForceGC: Forces a regular garbage collection to free up memory. This will slow Unity performance
* Renderer: This is the gameObject that the stream will be rendered on within Unity. Drag the gameObject from the Unity Hierarchy to change (CameraRTSP and 360RTSP are setup to render on themselves).
* Texture: This is generated at runtime so can be left blank.

In addition to this, the interface has custom shaders and materials to further customise how the stream is presented in Unity. 360Stream contains an example of this, as the stream is rendered on the inside of the sphere. To achieve this:
* On the gameObject's MeshRenderer, set the Material's Element0 to `VideoMaterial`
* Then, on the gameObject's Material (which is now `VideoMaterial`) set the shader to either `InsideVisibleBGR` \ `InsideVisibleRGB`

If you encounter visual bugs with the RTSP stream or Unity crashes, check and ensure that only one stream activated (Select the gameObject in the hierarchy menu and untick the object in the inspector) or restart the RTSP server.
Freely available test RTSP server for debugging [rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov](rtsp://wowzaec2demo.streamlock.net/vod/mp4:BigBuckBunny_115k.mov)

### Teleoperation controls

All teleoperation interfaces have keyboard and mouse controls, which are associated with the TeleopControls gameObject in each scene.

Keyboard controls: use WASD to move in fiward, backward, left and right directions. The speed of linear/angular velocities can be edited through the public variables on the TeleopKeyboard script attatched to TeleopControls
Mouse controls: Click on a position within the scene to place down a waypoint. Holding the click allows for the orientation of this waypoint to be set. Pressing the space bar will then send the command to the path planner.

## Info on Old scenes/Other scripts

* SampleScene - The first scene that was auto generated with the Unity project. This was used to prototype the first drone teleop setup, including ROS connection with RosBridge, an RTSP stream and simple inputs using menu GUIs. This is set up to use a custom MavState publisher to publish multiple waypoints at once (creating trajectories for the drone to follow) 

* DroneFollow/DroneOverview/SingleDrone - These are slight edits on the previous setup with different user sizes during operation. These werent setup for use with the VR interface

* MultiDrone/MultiDroneImage - Interface set up for use with VR headset. Multidrone use case, with logic of selcting drones achieved through menu system hooked up to "RobotOrganiser" script using Unity events (Logic of menus purely event based for now). This setup is how we found out that the RTSP client couldnt handle multiple streams at once (an ongoing issue), therefore MultiDroneImage is a copy that subscribes to image topics through RosSharp instead

* 2DMapping - Interface testing out basic mapping functionality with drones. "MapOrganiser" takes input from images from drone and sorts it into 2D tiles at set positions

* DroneGrouping - More sophisticated grouping logic. Combining scripts "RobotOrganiser", "MenuOrganiser" and "GroupOrganiser", users can use custom menus to create, select and add and remove drones to custom groups. Able to then issue commands to groups of drones instead of single drones.

* VRPaperScene - Project developed for a previous VR paper deadline. Setup - Multidrone mapping a scene, with ground based vehicle (husky) to complete simulated tasks. In this case, intended design was finding and putting out forest fires. Combines all functionality of previous developed scenes

* Husky - Interface set up for teleoperation of ClearPath husky robot. Includes "RobotOrganiser" and "MenuOrganiser" scripts to enable control of the robot ("GroupOrganiser" can also be added but currently this interface is a single robot use case). Custom RosSharp subscribers have been created to visualise OccupancyGrids and a custom PointCloud render is included to visualise PointCloud maps. 

* Husky Reconstruction - Interface set up for teleoperation experiments (See below section of how to set up). Includes stripped down Husky setup with simple keyboard teleoperation controls, various RTSP stream gameobjects for rendering different camera streams, as well as custom RosSharp subscribers for dense reconstruction of RTABmap pointcloud maps.

### Walkthrough of other interface scripts

The other interfaces can be quite daunting to wrap your head around, so here is a basic rundown of how they work

### Robot gameObjects

The robot gameObjects represent the robot. These contain publishers/subscribers for the current postion of the robot (Odom/Pose sub) and control for the robot (Twist/Pose pub)

### Organiser scripts and Prefabs

The interface comes with a couple of prefabs and scripts to allow for quick setup of multi-robot interfaces.

* SelectionMenu - This prefab deals with user input on the VR controllers and allows for quick design of handheld menus. New "SelectionSections" can be defined, with names and various Unity events for different press functions to create many different menu setups
* MenuOrganiser - This controls controls the visibility of two types of menus in the interface: SelectionMenus and the HierachicalMenu. Sets of SelectionMenus can be defined (SourceMenu, GroupMenu, RobotMenu) to create definitive levels within the interface. This also deals with the logic of the HierachicalMenu.
* HierachicalMenu - This menu is made up of different menu nodes and is presented as a graph within the Unity scene. Users can operate this menu using a laser pointer to select which level they want to interact with.  
* RobotOrganiser - This script deals with the control logic of the various robots. Using a SelectionMenu, the current robot can be selected. Robot controlls in the script will then map to the specified publishers on the robot gameobject, meaning that you can reuse a SelectionMenu for control schemes across multiple robots
* TwistControl - Hooks up to the RobotOrganiser, allowing users to use motion controls and generate Twist messages. 
* TrajecoryControl - Hooks up to the RobotOrganiser, allowing users to set position waypoints and generate Pose messages.
* GroupOrganiser - This script deals with the control logic of grouping the robots, including creating, deleting and adding and removing robots to groups. The script also contains functions for setting group waypoints.

