# Image tools

Loosely defined ros package for various tools related to images

## Current scripts:

### omnistitch

Script for projecting two fisheye images into one equitriangular image, with rudimentary stitching to improve resulting image quality.
Note: this implementation assumes both images are the same resolution and have the same FOV

Parameters:

* left\_image\_topic: sensor\_msgs/Image topic which will be positioned on the left of the resulting stitched image
* right\_image\_topic: sensor\_msgs/Image topic which will be positioned on the right of the resulting stitched image
* src\_width: Width of input image
* src\_height: Height of input image
* out\_width: Width of ouput image
* out\_height: Height of output image
* fov: Field of view of the fisheye lens
* buffer: Size of blended region between resulting left and right sides of the ouput image
* update\_rate: Desried publish rate of stitched image

### response delay

Script for analysing the response delay in teleoperation interfaces. 
Subscribed topic: /latency/command, a geometry\_msgs/Point message topic. X, Y and Z data relate to R, G and B of the desired colour.
Published topic: /latency/image, a sensor\_msgs/Image messge topic

Parameters:

* out\_width: Width of output image
* out\_height: Height of output image
* update\_rate: Desired publish rate of image
* visualise: Debug option to render image in openCV window
