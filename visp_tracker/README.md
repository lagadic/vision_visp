# visp_tracker

visp_tracker wraps the ViSP moving edge tracker provided by the ViSP
visual servoing library into a ROS package.

This computer vision algorithm computes the pose (i.e. position and
orientation) of an object in an image. It is fast enough to allow
object online tracking using a camera.


This package is composed of one node called 'tracker' and two
additional binaries 'client' and 'viewer'.

The node tries to track the object as fast as possible but needs to be
initialized using the client. The viewer can be used to monitor the
tracking result.

* [Project webpage on ros.org: tutorial and API reference] [ros-homepage]
* [Project webpage: source code download, bug report] [github-homepage]


## Setup

This package can be compiled like any other catkin package using `catkin_make`. 

### Prerequisities

First you need to install ViSP as a system dependency. This can be achived using `ros-indigo-visp` package for Ubuntu. Just run:

	$ sudo apt-get install ros-indigo-visp

If the package is not available (this is for example the case for Fedora) or if you want to use a more recent version of ViSP, you can also install ViSP from source:

	$ cd ~
	$ svn checkout svn://scm.gforge.inria.fr/svn/visp/trunk/ViSP
	$ cd ViSP
	$ cmake -DBUILD_SHARED_LIBS=ON .
	$ make -j8

Then to use this version you have to setup `VISP_DIR` environment variable to the folder that contains the build. In ou case it becomes:

	$ export VISP_DIR=~/ViSP

### How to get and build visp_tracker 

Supposed you have a catkin work space just run:

	$ cd ~/catkin_ws/src 
	$ git clone -b hydro-devel https://github.com/lagadic/vision_visp.git
	$ cd ..
	$ catkin_make --pkg visp_tracker

## Documentation

The documentation is available on the project [ROS homepage]
[ros-homepage].

For more information, refer to the [ROS tutorial]
[ros-tutorial-building-pkg].

[github-homepage]: https://github.com/laas/visp_tracker
[ros-homepage]: http://www.ros.org/wiki/visp_tracker
[ros-tutorial-building-pkg]: http://www.ros.org/wiki/ROS/Tutorials/BuildingPackages "Building a ROS Package"
