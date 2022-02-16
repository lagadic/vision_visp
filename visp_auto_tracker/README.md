# visp_auto_tracker


visp_auto_tracker wraps model-based trackers provided by ViSP visual 
servoing library into a ROS package. The tracked object should have a 
QRcode, Flash code or April tag pattern. Based on the pattern, the object is 
automaticaly detected. The detection allows then to initialise the 
model-based trackers. When lost of tracking achieves a new detection 
is performed that will be used to re-initialize the tracker.

This computer vision algorithm computes the pose (i.e. position and
orientation) of an object in an image. It is fast enough to allow
object online tracking using a camera.

This package is composed of one node called 'visp_auto_tracker'. The 
node tries first to detect the QRcode, the Flash or the April tag code associated to 
the object. Once the detection is performed, the node tracks the object. 
When a lost of tracking occurs the node tries to detect once again the 
object and then restart a tracking.

The viewer comming with visp_tracker package can be used to monitor the 
tracking result.

* [Project webpage on ros.org: tutorial and API reference] [ros-homepage]
* [Project webpage: source code download, bug report] [github-homepage]


## Setup

This package contains submodules. It can be compiled like any other ROS package using `catkin_make`. 

### Prerequisities

visp_auto_tracker depends on visp_bridge and visp_tracker packages available from <https://github.com/lagadic/vision_visp> (indigo-devel branches).

visp_auto_tracker depends also on libdmtx-dev and libzbar-dev system dependencies. To install them run:

	$ sudo apt-get install libdmtx-dev libzbar-dev

### How to get and build visp_tracker 

Supposed you have a catkin work space just run:

	$ cd ~/catkin_ws/src 
	$ git clone -b indigo-devel https://github.com/lagadic/vision_visp.git
	$ cd ..
	$ catkin_make --pkg visp_auto_tracker

## Documentation

The documentation is available on the project [ROS homepage]
[ros-homepage].

For more information, refer to the [ROS tutorial]
[ros-tutorial-building-pkg].

[github-homepage]: https://github.com/lagadic/visp_auto_tracker
[ros-homepage]: http://www.ros.org/wiki/visp_auto_tracker
[ros-tutorial-building-pkg]: http://www.ros.org/wiki/ROS/Tutorials/BuildingPackages "Building a ROS Package"
