visp_bridge
===========

visp_bridge is a small interface between the ViSP library and ROS. For instance it converts between the different data types used by each library.

To date, the supported functionnality sums up to:
 * ViSP [vpImage][visp-doc-homepage] / ROS [sensor_msgs::Image] conversion
 * ViSP [vpCameraParameter][visp-doc-homepage] / ROS [sensor_msgs::CameraInfo] conversion
 * ViSP [vpHomogeneousMatrix][visp-doc-homepage] / ROS [geometry_msgs::Transform] conversion
 * ViSP [vpHomogeneousMatrix][visp-doc-homepage] / ROS [geometry_msgs::Pose] conversion



* [Project webpage on ros.org] [ros-homepage]
* [Project webpage: source code download, bug report] [github-homepage]


Setup
-----

This package can be compiled like any other ROS package using `rosmake`.

Documentation
-------------

The general documentation is available on the project [ROS homepage]
[ros-homepage].

The code documentation is available on [API homepage][api-homepage] 


[github-homepage]: https://github.com/lagadic/visp_bridge
[ros-homepage]: http://www.ros.org/wiki/visp_bridge
[api-homepage]: http://ros.org/doc/api/visp_bridge/html/namespacemembers.html|code API
[sensor_msgs::Image]: http://www.ros.org/doc/api/sensor_msgs/html/msg/Image.html|sensor_msgs::Image
[sensor_msgs::CameraInfo]: http://www.ros.org/doc/api/sensor_msgs/html/msg/CameraInfo.html|sensor_msgs::CameraInfo
[geometry_msgs::Transform]: http://www.ros.org/doc/api/geometry_msgs/html/msg/Transform.html|geometry_msgs::Transform
[geometry_msgs::Pose]: http://www.ros.org/doc/api/geometry_msgs/html/msg/Pose.html|geometry_msgs::Pose
[visp-doc-homepage]: http://www.irisa.fr/lagadic/visp/publication.html
