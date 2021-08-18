#PointCloud tools

Loosely defined ros package for various tools related to pointclouds

## Current scripts:

### Laserscan\_to\_pointcloud
Script for converting sensor\_msgs/LaserScan msgs to sensor\_msgs/PointCloud2

Parameters:

* scan\_topic: sensor\_msgs/LaserScan topic to be converted
* pcl\_topic: Topic name of the sensor\_msgs/PointCloud2 msg with data equivalent to scan\_topic
* update\_rate: Desired publish rate of pcl\_topic
* frame: Frame for explicit pointcloud transform

### voxel\_filter
Script to apply a voxel filter to a sensor\_msgs/PointCloud2 msg

Parameters:

* pcl\_topic: sensor\_msgs/PointCloud2 topic to be filtered
* pcl\_filtered\_topic: sensor\_msgs/PointCloud2 topic to publish the resulting filtered pointcloud
* voxel\_size: Size of the voxel used for filtering (default:=0.05)
* update\_rate: Desried publish rate of pcl\_filtered\_topic
* frame: Frame for explicit pointcloud transform
