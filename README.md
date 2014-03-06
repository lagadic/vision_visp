# ViSP stack for ROS

[![Build Status](https://travis-ci.org/lagadic/vision_visp.png?branch=hydro-devel)](https://travis-ci.org/lagadic/vision_visp)

This repository provides a ViSP stack for ROS. [ViSP] [visp] is the
Visual Servoing Platform and [ROS] [ros] a robotics middleware.

## Setup

This package contains submodules. It can be compiled like any other ROS package using `catkin_make`. In that case you have to consider the `hydro-devel` branch.

### Prerequisities

1/ First you need to install ViSP as a system dependency. This can be achived using `ros-hydro-visp` package available for Ubuntu. Just run:

	$ sudo apt-get install ros-hydro-visp

If the package is not available (this is for example the case for Fedora) or if you want to use a more recent version of ViSP, you can also install ViSP from source:

	$ cd ~
	$ svn checkout svn://scm.gforge.inria.fr/svn/visp/trunk/ViSP
	$ cd ViSP
	$ cmake -DBUILD_SHARED_LIBS=ON .
	$ make -j8

Then to use this version of ViSP build from source you have to setup `VISP_DIR` environment variable to the folder that contains the build. In our case it becomes:

	$ export VISP_DIR=~/ViSP

2/ vision_visp stack contains visp_auto_tracker package that depends on libdmtx-dev and libzbar-dev system dependencies. To install them run:

	$ sudo apt-get install libdmtx-dev libzbar-dev


### How to get and build vision_visp 

Supposed you have a catkin work space, if you want to build all the packages just run:

	$ cd ~/catkin_ws/src 
	$ git clone -b hydro-devel --recursive https://github.com/lagadic/vision_visp.git
	$ cd ..
	$ catkin_make 

If you want to build a specific package (like visp_bridge) run either:

	$ catkin_make --pkg visp_bridge


## Documentation

The documentation is available on the project [ROS homepage]
[ros-homepage].

[github-homepage]: https://github.com/lagadic/vision_visp
[ros-homepage]: http://www.ros.org/wiki/vision_visp
[visp]: http://team.inria.fr/lagadic/visp/visp.html
[ros]: http://www.ros.org
