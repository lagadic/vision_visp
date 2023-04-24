# visp_hand2eye_calibration

visp_hand2eye_calibration is a ROS package that estimates the camera position with respect to its effector using the ViSP library avalaible from <http://team.inria.fr/lagadic/visp>.

## Setup

This package can be compiled like any other ros2 package using `colcon`. In that case you have to consider the `ros2` branch.

### Prerequisities

visp_hand2eye_calibration depends on visp_bridge package available from <https://github.com/lagadic/vision_visp> (`ros2` branch).

### How to get and build visp_camera_calibration

Supposed you have a ros2 work space just run:

	$ cd ~/colcon_ws/src
	$ git clone -b ros2 https://github.com/lagadic/vision_visp.git
	$ cd ..
	$ colcon build --symlink-install --packages-select visp_hand2eye_calibration

Documentation
-------------

* [Project webpage on ros.org] [ros-homepage]
* [Project webpage: source code download, bug report] [github-homepage]

[github-homepage]: https://github.com/lagadic/visp_camera_calibration
[ros-homepage]: http://www.ros.org/wiki/visp_camera_calibration
