visp_tracker
============

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


Setup
-----

This package can be compiled like any other ROS package using `rosdep`
and `rosmake`.

For more information, refer to the [ROS tutorial]
[ros-tutorial-building-pkg].


Documentation
-------------

The documentation is available on the project [ROS homepage]
[ros-homepage].


[github-homepage]: https://github.com/lagadic/visp_tracker
[ros-homepage]: http://www.ros.org/wiki/visp_tracker
[ros-tutorial-building-pkg]: http://www.ros.org/wiki/ROS/Tutorials/BuildingPackages "Building a ROS Package"
