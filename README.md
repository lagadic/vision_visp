ViSP stack for ROS
==================

![GPL-2](https://www.gnu.org/graphics/gplv3-127x51.png)

`vision_visp` provides ViSP algorithms as ROS components. [ViSP]
[visp] is the Visual Servoing Platform and [ROS] [ros] a robotics
middleware.

These packages are released under the [GPL-2](COPYING) license.


Components documentation is hosted on the [ros.org wiki] [vision_visp-wiki].

Support is provided through [ROS Answers] [vision_visp-answers] .


Which branch should I use?
--------------------------

Branches come in two flavors:

 * development branch,
 * release branch

Package for each ROS release is maintained on separate
branches. I.e. `jade-devel` is the Jade development branch whereas
`jade` is the Jade release branch.

`master` means the next ROS release.

If you are a user you should use a release branch as they contain
stable and tested versions of the packages. If you are a developper
you must provide new patches against `master`. You may also provide
version-specific bug fix again older releases.


 - Never implement new features in old branches (i.e. not
   master). These Pull Requests will not be accepted. If you provide a
   bug fix then you may ask for it to be backported. ABI/API breakage
   prevent patches from being backported.
 - The *only* action allowed in release branches is merging the
   development branch in the current branch.


*Warning:* the Fuerte branches still rely on the legacy `rosbuild`
 build system. We recommend you to update to a newer ROS release. Only
 minimum maintained will be done for this release.


Additional development guidelines are provided in
[CONTRIBUTING.md](CONTRIBUTING.md).



Build Status
------------

This stack supports the following ROS releases:

 * Hydro
 * Groovy
 * Fuerte
 * Indigo
 * Jade

The master branch holds the development that will be available in the
next ROS release.


| ROS Release   | Development Branch           | Development branch (ros.org) | Release Branch | Documentation (ros.org) |
| ------------- | ---------------------------- | ---------------------------- | -------------- | ----------------------- |
| Master        | [![Build Status](https://travis-ci.org/lagadic/vision_visp.png?branch=master)](https://travis-ci.org/lagadic/vision_visp) | N/A | N/A | N/A |
| Jade          | [![Build Status](https://travis-ci.org/lagadic/vision_visp.png?branch=jade-devel)](https://travis-ci.org/lagadic/vision_visp) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=devel-jade-vision_visp)](http://jenkins.ros.org/job/devel-jade-vision_visp/) | [![Build Status](https://travis-ci.org/lagadic/vision_visp.png?branch=jade)](https://travis-ci.org/lagadic/vision_visp) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=doc-jade-vision_visp)](http://jenkins.ros.org/job/doc-jade-vision_visp/) |
| Indigo         | [![Build Status](https://travis-ci.org/lagadic/vision_visp.png?branch=indigo-devel)](https://travis-ci.org/lagadic/vision_visp) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=devel-indigo-vision_visp)](http://jenkins.ros.org/job/devel-indigo-vision_visp/) | [![Build Status](https://travis-ci.org/lagadic/vision_visp.png?branch=indigo)](https://travis-ci.org/lagadic/vision_visp) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=doc-indigo-vision_visp)](http://jenkins.ros.org/job/doc-indigo-vision_visp/) |
| Hydro         | [![Build Status](https://travis-ci.org/lagadic/vision_visp.png?branch=hydro-devel)](https://travis-ci.org/lagadic/vision_visp) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=devel-hydro-vision_visp)](http://jenkins.ros.org/job/devel-hydro-vision_visp/) | [![Build Status](https://travis-ci.org/lagadic/vision_visp.png?branch=hydro)](https://travis-ci.org/lagadic/vision_visp) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=doc-hydro-vision_visp)](http://jenkins.ros.org/job/doc-hydro-vision_visp/) |
| Groovy         | [![Build Status](https://travis-ci.org/lagadic/vision_visp.png?branch=groovy-devel)](https://travis-ci.org/lagadic/vision_visp) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=devel-groovy-vision_visp)](http://jenkins.ros.org/job/devel-groovy-vision_visp/) | [![Build Status](https://travis-ci.org/lagadic/vision_visp.png?branch=groovy)](https://travis-ci.org/lagadic/vision_visp) | [![Build Status](http://jenkins.ros.org/buildStatus/icon?job=doc-groovy-vision_visp)](http://jenkins.ros.org/job/doc-groovy-vision_visp/) |
| Fuerte         | [![Build Status](https://travis-ci.org/lagadic/vision_visp.png?branch=fuerte-devel)](https://travis-ci.org/lagadic/vision_visp) | N/A | [![Build Status](https://travis-ci.org/lagadic/vision_visp.png?branch=fuerte)](https://travis-ci.org/lagadic/vision_visp) | N/A |



[visp]: http://www.irisa.fr/lagadic/visp/visp.html
[ros]: http://www.ros.org
[vision_visp-wiki]: http://wiki.ros.org/vision_visp
[vision_visp-answers]: http://answers.ros.org/questions/scope:all/sort:activity-desc/tags:vision_visp/page:1/
