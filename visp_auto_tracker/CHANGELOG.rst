^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package visp_auto_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.8.1 (2015-04-08)
------------------
* hydro-0.8.0
* Prepare changelogs
* Contributors: Fabien Spindler

0.8.0 (2015-04-01)
------------------
* Merge branch 'hydro-devel' into hydro
* Reintroduce link_directories() requested to work with ros-hydro-visp-2.4.9
* Merge branch 'master' into hydro-devel
* Remove catkin_lint issues and warnings
* Fix doc url location
* Update launch files
* Comment viewer node. Only interesting when visp_auto_tracker debug_display param is False
* Improve pose computation using Lagrange and Dementhon methods as initialisation
* Increase quality value to avoid keypoint detection on the black part of the target
* Introduce code_message parameter as a comment to show how to use.
* New parameter "code_message" to specify the target to track from the code message.
  If this parameter is not set, the code that is tracked is the largest in the image.
* Add frame_size parameter to the tracker client and viewer nodes that allow to specify 
  the lenght (in meter) of the frame axis
* Publish the code message of the tracked target on /visp_auto_tracker/code_message topic. 
  If no target is tracked, publish an empty message.
  Remove fps printing
* Fix to publish covariance matrix associated to the pose estimation
* Fix to publish an empty list of moving edges and klt points on their respective topics when
  the tracker fails
* Makes sure klt points that are sent via ros are from visible polygons.
* Show how to use the viewer with visp_auto_tracker
* Consider also .cao file to describe the cad model of the object to track with the hybrid
  model-based tracker
* Remove useless include
* Fix issue https://github.com/lagadic/vision_visp/issues/45 to take into account 
  model_name parameter set in the launch file.
* Changes done to use flash code detectors introduced in ViSP 2.10.0 rather than the 
  one in visp_auto_tracker/flashcode_mbt/detectors.
* Fix to publish camera_info with the images
* Fix to avoid an OpenCV exception when the object roi is outside the image
* Fix to run the viewer node
* Fix compat with ViSP 2.10.0
* Fix to avoid an OpenCV exception when the object roi is outside the image
* Fix to run the viewer node
* Fix to publish camera_info with the images
* Fix compat with ViSP 2.10.0
* hydro-0.7.5
* Prepare changelogs
* Contributors: Aurelien Yol, Fabien Spindler, Riccardo Spica

0.7.5 (2014-08-01)
------------------
* hydro-0.7.4
* Prepare changelogs
* Contributors: Fabien Spindler

0.7.4 (2014-07-03)
------------------
* Update and fix content of README files
* Fixed bag dowload in tracker
* Prepare changelogs
* Contributors: Fabien Spindler, Riccardo Spica

0.7.3 (2014-04-10)
------------------
* Fix to install model folder
* hydro-0.7.2
* Contributors: Fabien Spindler

0.7.2 (2014-03-14)
------------------
* Merge branch 'hydro-devel' into hydro
* Fix various dependency issues in the CMakeLists.txt and package.xml files
* Merge branch 'hydro-devel' into hydro
* :lipstick: Aesthetic changes
* [visp_auto_tracker] Add missing dependency
* Add missing dependency to ViSP
* hydro-0.7.1
* Prepare changelogs
* Fix errors detected with catkin_lint
* hydro-0.7.0
* Run catkin_generate_changelog, catkin_tag_changelog, bump version to 0.6.0
* Contributors: Fabien Spindler, Thomas Moulard

0.7.1 (2014-03-13)
------------------
* Fix errors detected with catkin_lint
* hydro-0.7.0
* Run catkin_generate_changelog, catkin_tag_changelog, bump version to 0.6.0
* Contributors: Fabien Spindler, Thomas Moulard

0.7.0 (2014-03-12)
------------------



