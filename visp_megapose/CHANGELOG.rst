^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package visp_auto_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.13.1 (2022-02-20)
-------------------

0.13.0 (2022-02-16)
-------------------

0.12.1 (2021-04-22)
-------------------
* CMake fixes
* Contributors: Fabien Spindler

0.12.0 (2021-04-21)
-------------------
* Fix warnings detected by catkin_lint
* Fix warning detected during packaging
* Update README with Noetic support
* Use python3 on neotic
* Fix compat with Ubuntu 20.04
* Moves tracker_ref_frame from extern to param (#97)
* Contributors: Fabien Spindler

0.11.0 (2019-05-27)
-------------------
* Update with Melodic
* Fix memory leak
* Introduce vpMbGenericTracker and visp3 headers in visp_auto_tracker
* Introduce visp3 headers
* Code indentation
* Introduce vpMbGenericTracker
* Introduce vpHandEyeCalibration class
* Fix build for melodic (Closes #88)
* Fix compilation on 17.04 by adding missing boost/format.hpp inclusion
* Contributors: Fabien Spindler

0.10.0 (2017-02-10)
-------------------
* Fix catkin_lint warnings level 2
* kinetic-0.9.3
* Prepare changelogs
* Contributors: Fabien Spindler

0.7.0 (2014-03-12)
------------------
* Add flashcode_mbt as subdirectory.
* Merge visp_auto_tracker as our subdirectory
* Contributors: Thomas Moulard

0.7.2 (2014-04-07)
------------------
* Fix various dependency issues in the CMakeLists.txt and package.xml files
* :lipstick: Aesthetic changes
* [visp_auto_tracker] Add missing dependency
* Add missing dependency to ViSP
* Fix errors detected with catkin_lint
* Contributors: Fabien Spindler, Thomas Moulard

0.7.3 (2014-04-10)
------------------
* Fix to install models folder
* indigo-0.7.2
* Prepare changelogs
* Contributors: Fabien Spindler

0.8.0 (2015-04-01)
------------------
* Remove catkin_lint issues and warnings
* Fix doc url location
* Update launch files
* Comment viewer node. Only interesting when visp_auto_tracker debug_display param is False
* Improve pose computation using Lagrange and Dementhon methods as initialisation
* Increase quality value to avoid keypoint detection on the black part of the target
* Introduce code_message parameter as a comment to show how to use.
* New parameter "code_message" to specify the target to track from the code message.
  If this parameter is not set, the code that is tracked is the largest in the image.
* Add frame_size parameter to the tracker client and viewer nodes that allow to 
  specify the lenght (in meter) of the frame axis
* Publish the code message of the tracked target on /visp_auto_tracker/code_message topic.
  If no target is tracked, publish an empty message.
  Remove fps printing
* Fix to publish covariance matrix associated to the pose estimation
* Fix to publish an empty list of moving edges and klt points on their respective topics
  when the tracker fails
* Makes sure klt points that are sent via ros are from visible polygons.
* Show how to use the viewer with visp_auto_tracker
* Consider also .cao file to describe the cad model of the object to track with the 
  hybrid model-based tracker
* Remove useless include
* Fix issue https://github.com/lagadic/vision_visp/issues/45 to take into account 
  model_name parameter set in the launch file.
* Changes done to use flash code detectors introduced in ViSP 2.10.0 rather than 
  the one in visp_auto_tracker/flashcode_mbt/detectors.
* Fix to avoid an OpenCV exception when the object roi is outside the image
* Fix to run the viewer node
* Fix to publish camera_info with the images
* Fix compat with ViSP 2.10.0
* Update and fix content of README files
* Download the tutorial-qrcode.bag bag file from a new location that allows the 
  download without SSL certificate
* Install visp_auto_tracker/models folder
* 0.7.3
* Prepare changelogs
* Prepare changelogs
* Contributors: Aurelien Yol, Fabien Spindler

0.9.0 (2015-12-20)
------------------
* Fix catkin_lint error and issues
* Compat with ViSP 3.0.0
* Fix to build with ViSP 2.10.0 when VISP_BUILD_DEPRECATED=OFF
* jade-0.8.0
* Prepare changelogs
* Contributors: Aurelien Yol, Fabien Spindler

0.9.1 (2015-12-21)
------------------
* Revert build_depend visp removal that is mandatory.
* jade-0.9.0
* Prepare changelogs
* Contributors: Fabien Spindler

0.9.3 (2016-05-20)
------------------
* Fix CMAKE_CXX_FLAGS as separated list
* kinetic-0.9.2
* Contributors: Fabien Spindler


