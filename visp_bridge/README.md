# visp_bridge

visp_bridge is a small interface between the ViSP library and ROS. For instance it converts between the different data types used by each library.

To date, the supported functionnality sums up to:
 * ViSP [vpImage][visp-doc-homepage] / ROS [sensor_msgs::Image] conversion
 * ViSP [vpCameraParameter][visp-doc-homepage] / ROS [sensor_msgs::CameraInfo] conversion
 * ViSP [vpHomogeneousMatrix][visp-doc-homepage] / ROS [geometry_msgs::Transform] conversion
 * ViSP [vpHomogeneousMatrix][visp-doc-homepage] / ROS [geometry_msgs::Pose] conversion


## Setup

This package can be compiled like any other catkin package using `catkin_make`. In that case you have to consider the `hydro-devel` branch.

### Prerequisities

First you need to install ViSP as a system dependency. This can be achived using `ros-indigo-visp` package available for Ubuntu. Just run:

	$ sudo apt-get install ros-indigo-visp

If the package is not available (this is for example the case for Fedora) or if you want to use a more recent version of ViSP, you can also install ViSP from source:

	$ cd ~
	$ svn checkout svn://scm.gforge.inria.fr/svn/visp/trunk/ViSP
	$ cd ViSP
	$ cmake -DBUILD_SHARED_LIBS=ON .
	$ make -j8

Then to use this version you have to setup `VISP_DIR` environment variable to the folder that contains the build. In ou case it becomes:

	$ export VISP_DIR=~/ViSP

### How to get and build visp_bridge 

Supposed you have a catkin work space just run:

	$ cd ~/catkin_ws/src 
	$ git clone -b hydro-devel https://github.com/lagadic/vision_visp.git
	$ cd ..
	$ catkin_make --pkg visp_bridge

Documentation
-------------

* [Project webpage on ros.org] [ros-homepage]
* [Project webpage: source code download, bug report] [github-homepage]
* [API documentation][api-homepage] 


[github-homepage]: https://github.com/lagadic/visp_bridge
[ros-homepage]: http://www.ros.org/wiki/visp_bridge
[api-homepage]: http://ros.org/doc/api/visp_bridge/html/namespacemembers.html
[sensor_msgs::Image]: http://www.ros.org/doc/api/sensor_msgs/html/msg/Image.html|sensor_msgs::Image
[sensor_msgs::CameraInfo]: http://www.ros.org/doc/api/sensor_msgs/html/msg/CameraInfo.html|sensor_msgs::CameraInfo
[geometry_msgs::Transform]: http://www.ros.org/doc/api/geometry_msgs/html/msg/Transform.html|geometry_msgs::Transform
[geometry_msgs::Pose]: http://www.ros.org/doc/api/geometry_msgs/html/msg/Pose.html|geometry_msgs::Pose
[visp-doc-homepage]: http://team.inria.fr/lagadic/visp/publication.html
