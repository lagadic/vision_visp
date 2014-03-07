# visp_hand2eye_calibration

visp_hand2eye_calibration is a ROS package that estimates the camera position with respect to its effector using the ViSP library avalaible from <http://team.inria.fr/lagadic/visp>. 

## Setup

This package can be compiled like any other catkin package using `catkin_make`. In that case you have to consider the `hydro-devel` branch.

### Prerequisities

visp_hand2eye_calibration depends on visp_bridge package available from <https://github.com/lagadic> (hydro-devel branch). Install first visp_bridge package.

### How to get and build visp_camera_calibration

Supposed you have a catkin work space just run:

	$ cd ~/catkin_ws/src 
	$ git clone -b hydro-devel https://github.com/lagadic/visp_hand2eye_calibration.git
	$ cd ..
	$ catkin_make --pkg visp_hand2eye_calibration

Documentation
-------------

* [Project webpage on ros.org] [ros-homepage]
* [Project webpage: source code download, bug report] [github-homepage]

[github-homepage]: https://github.com/lagadic/visp_camera_calibration
[ros-homepage]: http://www.ros.org/wiki/visp_camera_calibration

