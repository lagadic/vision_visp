visp_bridge
===========

visp_bridge is a small interface between the ViSP library and ROS. For instance it converts between the different data types used by each library.

To date, the supported functionnality sums up to:
 * ViSP [[http://www.irisa.fr/lagadic/visp/publication.html|vpImage]] / ROS [[http://www.ros.org/doc/api/sensor_msgs/html/msg/Image.html|sensor_msgs::Image]] conversion
 * ViSP [[http://www.irisa.fr/lagadic/visp/publication.html|vpCameraParameter]] / ROS [[http://www.ros.org/doc/api/sensor_msgs/html/msg/CameraInfo.html|sensor_msgs::CameraInfo]] conversion
 * ViSP [[http://www.irisa.fr/lagadic/visp/publication.html|vpHomogeneousMatrix]] / ROS [[http://www.ros.org/doc/api/geometry_msgs/html/msg/Transform.html|geometry_msgs::Transform]] conversion
 * ViSP [[http://www.irisa.fr/lagadic/visp/publication.html|vpHomogeneousMatrix]] / ROS [[http://www.ros.org/doc/api/geometry_msgs/html/msg/Pose.html|geometry_msgs::Pose]] conversion

For usage, see the [[http://ros.org/doc/api/visp_bridge/html/namespacemembers.html|code API]].


* [Project webpage on ros.org] [ros-homepage]
* [Project webpage: source code download, bug report] [github-homepage]


Setup
-----

This package can be compiled like any other ROS package using `rosmake`.

Documentation
-------------

The documentation is available on the project [ROS homepage]
[ros-homepage].


[github-homepage]: https://github.com/lagadic/visp_bridge
[ros-homepage]: http://www.ros.org/wiki/visp_bridge

