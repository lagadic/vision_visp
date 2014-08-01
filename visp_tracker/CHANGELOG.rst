^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package visp_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.5 (2014-08-01)
------------------
* 0.7.4
* Prepare changelogs
* Contributors: Fabien Spindler

0.7.4 (2014-07-02)
------------------
* Update and fix content of README files
* 0.7.3
* Prepare changelogs
* Contributors: Fabien Spindler

0.7.3 (2014-04-11)
------------------
* Remove unicode character that makes bloom-release failing
* Update changelog
* Contributors: Fabien Spindler

0.7.2 (2014-03-25)
------------------
* Remove bullet usage and dependency by using ViSP instead. This was done to avoid errors when releasing vision_visp on oneiric/groovy where bullet is not packaged.
* Reorganize launch files and fix since viewer and client nodes where renamed due to catkin_lint errors
* Merge branch 'groovy-devel' into groovy
* Fix various dependency issues in the CMakeLists.txt and package.xml files
* Use Boost Filesystem V3.
* [visp_tracker] package.xml: add back Bullet dependency
* Merge branch 'groovy-devel' into groovy
* Add missing dependency to ViSP
* groovy-0.7.1
* Prepare changelogs
* Contributors: Benjamin Chretien, Fabien Spindler, Thomas Moulard

0.7.1 (2014-03-13)
------------------
* Fix errors detected with catkin_lint
* groovy-0.7.0
* Run catkin_generate_changelog, catkin_tag_changelog, bump version to 0.7.0
* Fix package.xml version number (set to current version, i.e. 0.6.0)
* [visp_tracker] Remove unused bullet dependency
* [visp_tracker] Remove useless call to setup.py
* Identify Fabien as the principal maintainer.
* [visp_tracker] CMakeLists.txt: update bag file URL (use new GitHub Release API).
* Merge visp_tracker as our subdirectory
* Contributors: Fabien Spindler, Thomas Moulard

0.7.0 (2014-03-12)
------------------
* Fix package.xml version number (set to current version, i.e. 0.6.0)
* [visp_tracker] Remove unused bullet dependency
* [visp_tracker] Remove useless call to setup.py
* Identify Fabien as the principal maintainer.
* [visp_tracker] CMakeLists.txt: update bag file URL (use new GitHub Release API).
* Merge visp_tracker as our subdirectory
* Contributors: Thomas Moulard
