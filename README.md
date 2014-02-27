# ViSP stack for ROS

[![Build Status](https://travis-ci.org/laas/vision_visp.png?branch=master)](https://travis-ci.org/laas/vision_visp)

This repository provides a ViSP stack for ROS. [ViSP] [visp] is the
Visual Servoing Platform and [ROS] [ros] a robotics middleware.

## Setup

This package contains submodules. It can be compiled like any other ROS package using `catkin_make`. In that case you have to consider the `groovy-devel` branch.

Supposed you have a catkin work space just run:

	$ cd ~/catkin_ws/src 
	$ git clone -b groovy-devel --recursive https://github.com/lagadic/vision_visp.git
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
