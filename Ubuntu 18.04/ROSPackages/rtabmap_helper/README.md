# rtabmap_helper
A ROS package to simplify outgoing data from rtabmap

## Installation
### Prerequisites
This package requires **rtabmap** and **rtabmap\_ros** to be built from source. Full steps can be found on the [**rtabmap_ros repo**](https://github.com/introlab/rtabmap_ros).

1. Install rtabmap_ros from source. Ensure to use the extra step to install in the catkin workspace without sudo command, as otherwise the install won't work
	```
	cd ~
	git clone https://github.com/introlab/rtabmap.git rtabmap
	cd rtabmap/build
	cmake -DCMAKE_INSTALL_PREFIX=~/catkin_ws/devel ..
	make 
	make install
	```
2. Install rtabmap_ros
	```
	cd ~/catkin_ws
	git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
	catkin_make
	```

# Nodes
## mapdata_to_pointcloud
The mapdata\_to\_pointcloud node accesses the default _/rtabmap/mapData_ message and uses the compressed image and depth from the inserted frame to construct a PointCloud. This then gets published with the relevant message information as before.

### Parameters
- voxelsize: Size of voxel for downsampling generated pointcloud (default = 0.05)
- camera\_frame: Frame id of the camera 
- pcl\_data\_topic: See published topics below (default = /rtabmap\_helper/pcl\_frame)
- pcl\_mapdata\_topic: See published topics below (default = /rtabmap\_helper/pcl\_mapdata)

### Subscribed Topics
rtabmap/mapData ([rtabmap\_ros/mapData](http://docs.ros.org/en/api/rtabmap_ros/html/msg/MapData.html)) - RTAB-Map's optimised graph and latest node data. 

### Published Topics
rtabmap\_helper/pcl\_frame ([sensor\_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)) - PointCloud constructed from latest image and depth data inserted into RTAB-Map

rtabmap\_helper/\_pcl_mapdata (rtabmap\_helper\PointCloudMapData) - RTAB-Map's optimised graph and latest node data in PointCloud form
