^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package visp_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.10.0 (2017-02-13)
-------------------
* Fix catkin_lint warnings level 2
* Fix OpenCV issue when reconfiguring the visp_tracker (Closes `#58 <https://github.com/lagadic/vision_visp/issues/58>`_)
* jade-0.9.1
* Prepare changelogs
* Contributors: Fabien Spindler

0.7.0 (2014-03-12)
------------------
* visp_tracker/package.xml: swithc license to GPLv2 to match the other packages.
* Remove useless call to setup.py
* Identify Fabien as the principal maintainer.
* CMakeLists.txt: update bag file URL (use new GitHub Release API).
* Merge visp_tracker as our subdirectory
* Contributors: Thomas Moulard

0.7.2 (2014-04-07)
------------------
* Remove bullet usage and dependency by using ViSP instead. This was done to avoid errors when releasing vision_visp on oneiric/groovy where bullet is not packaged.
* Reorganize launch files and fix since viewer and client nodes where renamed due to catkin_lint errors
* Fix various dependency issues in the CMakeLists.txt and package.xml files
* Use Boost Filesystem V3.
* [visp_tracker] package.xml: add back Bullet dependency
* Add missing dependency to ViSP
* Fix errors detected with catkin_lint
* Contributors: Benjamin Chrétien, Fabien Spindler, Thomas Moulard

0.7.3 (2014-04-10)
------------------
* Prepare changelogs
* Contributors: Fabien Spindler

0.8.0 (2015-04-01)
------------------
* Remove catkin_lint issues and warnings
* Fix doc url location
* Fix hard coded "protected to public" for Model based trackers parameters.
* Change warning message when viewer is waiting for initialization.
* Change default quality value of KLT points detection.
* Add config files for dynamic reconfigure of Edge and KLT Model based trackers.
* Fix dynamic reconfigure for bother client and tracker.
  Add forgotten mutex.unlock() when catching errors.
* Remove "bad suppress value" message and improve suppress state detection based on ViSP enum
* Add frame_size parameter to the tracker client and viewer nodes that allow to 
  specify the lenght (in meter) of the frame axis
* Fix compat with ViSP-2.9.0
* Fix to save/read initial pose from /tmp/${USER}/${model_name}.0.pos file
* Show optional help file during initialization. To this end 2
  options are possible:
  1/ If visp_tracker_client has help_image_path parameter, we
  use this setting to display the corresponding image. Extension
  could be jpeg, not only ppm. For example, in the launch file we can have:
  <node pkg="visp_tracker" type="visp_tracker_client" name="tracker_mbt_client">
  <param name="model_path" value="package://visp_tracker/models" />
  <param name="model_name" value="laas-box" />
  <param name="help_image_path" value="/home/user/laas-box.ppm"/>
  In that case we will open a display windows with /home/user/laas-box.ppm image.
  2/ If the previous parameter is not set, we consider
  model_path/model_name/model_name + .ppm extension as image full name and path.
  Following previous example, we will display
  package://visp_tracker/models/laas-box/laas-box.ppm
* Adapt visp_tracker to handle new message for trackers common parameters.
  Add mutex_lock() in visp_tracker_client to fix dynamic reconfigure utilisation.
  Modify ModelBasedSettings.cfg for better usage of dynamic reconfigure.
* Add new message for trackers common parameters.
* Show object frame
* Introduce bug fix when tracker viewer reads wrong /model_description value.
* Support not only wrml but also cao model files.
* Allow to read comments in init file.
* Remove useless debug messages.
* Fix to set Klt config at initialization.
  To check if really necessary.
* Merge branch 'master' of https://github.com/lagadic/vision_visp
* From /model_decription parameter decode the type of model that is used 
  (vrml or cao) to be able to consider both descriptions in the viewer
* Tracker client and viewer can now consider not only /camera_prefix but 
  also /node_name/camera_prefix parameters
* Introduce comment arround /camera_prefix parameter
* Rename model file name parameter to be generic.
  Delete useless setPose().
  Add other forgotten setPose().
* Rename model name variable to be generic.
* Add comment to the model
* Sync with ViSP 2.10.0
* Fix compat with ViSP 2.10.0
* Update and fix content of README files
* Contributors: Aurelien Yol, Fabien Spindler

0.9.0 (2015-12-20)
------------------
* Fix catkin_lint error and issues
* Compat with ViSP 3.0.0
* Fix to build with ViSP 2.10.0 when VISP_BUILD_DEPRECATED=OFF
* Fix bug to display the last computed pose in the tracker client.
* Improve data synchronization test based only on pose, klt points, and moving edges features
* Make ROS warn messages more explicit
* Make dynamic reconfigure working with ViSP 2.9.0.
  Ensure that the image is ready (test image size != 0) during dynamic reconfigure initialisation.
* Use VP_VERSION_INT
* Fix compat with ViSP 2.9.0. Fix ROS_INFO message. Code indentation.
* Improve ROS debug messages to be more generic.
  Remove parameters that should not be modified by the user in dynamic reconfigure files.
* Improve viewer node to handle dynamic reconfigure modifications.
  Modify tutorials so that they use the new functionnalities.
* Fix bug in visp_tracker_client to work without visp_tracker_viewer.
* jade-0.8.0
* Prepare changelogs
* Remove useless call to setKltOpencv and setMovingEdge (as it is done by default by the reconfigure server).
  Remove useless vpKltOpencv and vpMe variables.
* Modify dynamic reconfigure files to suppressed deprecated values.
  Adapt library to work with those modifications.
  Fix ROS debug message error.
* Reorganise ROS debug message to display trackers, edges and KLT parameters value.
* Contributors: Aurelien Yol, Fabien Spindler

0.9.1 (2015-12-21)
------------------
* Revert build_depend visp removal that is mandatory.
* jade-0.9.0
* Prepare changelogs
* Contributors: Fabien Spindler


