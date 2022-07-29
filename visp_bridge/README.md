# visp_bridge

visp_bridge is a small interface between the ViSP library and ROS. For instance it converts between the different data types used by each library.

To date, the supported functionnality sums up to:
 * ViSP [vpImage][visp-doc-homepage] / ROS [sensor_msgs::Image] conversion
 * ViSP [vpCameraParameter][visp-doc-homepage] / ROS [sensor_msgs::CameraInfo] conversion
 * ViSP [vpHomogeneousMatrix][visp-doc-homepage] / ROS [geometry_msgs::msg::Transform] conversion
 * ViSP [vpHomogeneousMatrix][visp-doc-homepage] / ROS [geometry_msgs::msg::Pose] conversion


## Setup

This package can be compiled like any other ros2 package using `colcon`. In that case you have to consider the `ros2` branch.

### Prerequisities

#### Install ViSP from ros2 package

First you need to install ViSP as a system dependency. This can be achived using `ros-${ROS-DISTRO}-visp` package available for Ubuntu. Just run:

	$ sudo apt-get install ros-${ROS-DISTRO}-visp

#### Install ViSP from source

If the ros2 package is not available or if you want to use a more recent version of ViSP, you can also install ViSP from source following [ViSP Quick Installation](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-install-ubuntu.html#install_ubuntu_quick). We recall here after the main steps:

    $ cd $VISP_WS
    $ git clone https://github.com/lagadic/visp.git
    $ mkdir -p $VISP_WS/visp-build
    $ cd $VISP_WS/visp-build
    $ cmake ../visp
    $ make -j$(nproc)

Then to use this version you have to setup `VISP_DIR` environment variable to the folder that contains the build. In our case it becomes:

	$ export VISP_DIR=$VISP_WS/visp-build

### How to get and build visp_bridge

Supposed you have a ros2 work space just run:

	$ cd ~/colcon_ws/src
	$ git clone -b ros2 https://github.com/lagadic/vision_visp.git
	$ cd ..
	$ colcon build --symlink-install --packages-select visp_bridge

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
[geometry_msgs::msg::Transform]: http://www.ros.org/doc/api/geometry_msgs/html/msg/Transform.html|geometry_msgs::msg::Transform
[geometry_msgs::msg::Pose]: http://www.ros.org/doc/api/geometry_msgs/html/msg/Pose.html|geometry_msgs::msg::Pose
[visp-doc-homepage]: http://team.inria.fr/lagadic/visp/publication.html
