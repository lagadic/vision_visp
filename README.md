ViSP stack for ROS
==================

[![Build Status](https://travis-ci.org/laas/vision_visp.png?branch=master)](https://travis-ci.org/laas/vision_visp)

This repository provides a ViSP stack for ROS. [ViSP] [visp] is the
Visual Servoing Platform and [ROS] [ros] a robotics middleware.


This package contains Git submodules. To clone it, run:

    git clone --recursive git://github.com/laas/vision_visp.git

### Setup with rosbuild

    $ roscd
    $ rosws set vision_visp --git https://github.com/lagadic/vision_visp.git -v fuerte-devel
    $ rosws update vision_visp
    $ source setup.bash
    $ rosdep install vision_visp
    $ rosmake vision_visp


[visp]: http://www.irisa.fr/lagadic/visp/visp.html
[ros]: http://www.ros.org
