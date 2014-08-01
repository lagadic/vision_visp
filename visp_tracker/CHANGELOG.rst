^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package visp_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.5 (2014-08-01)
------------------
* indigo-0.7.4
* Prepare changelogs
* Contributors: Fabien Spindler

0.7.4 (2014-07-03)
------------------
* Update and fix content of README files
* Contributors: Fabien Spindler

0.7.3 (2014-04-10)
------------------
* Prepare changelogs
* Contributors: Fabien Spindler

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

0.7.0 (2014-03-12)
------------------
* visp_tracker/package.xml: swithc license to GPLv2 to match the other packages.
* Remove useless call to setup.py
* Identify Fabien as the principal maintainer.
* CMakeLists.txt: update bag file URL (use new GitHub Release API).
* Merge visp_tracker as our subdirectory
* Contributors: Thomas Moulard

