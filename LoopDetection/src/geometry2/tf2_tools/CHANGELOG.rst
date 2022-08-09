^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.5 (2020-09-01)
------------------

0.7.4 (2020-09-01)
------------------

0.7.3 (2020-08-25)
------------------

0.7.2 (2020-06-08)
------------------
* fix shebang line for python3 (`#466 <https://github.com/ros/geometry2/issues/466>`_)
* Contributors: Mikael Arguedas

0.7.1 (2020-05-13)
------------------

0.7.0 (2020-03-09)
------------------
* Bump CMake version to avoid CMP0048 warning (`#445 <https://github.com/ros/geometry2/issues/445>`_)
* Merge pull request `#377 <https://github.com/ros/geometry2/issues/377>`_ from InstitutMaupertuis/melodic-devel
  Allow to choose output precision in echo
* Merge pull request `#373 <https://github.com/ros/geometry2/issues/373>`_ from mikaelarguedas/yaml_safe_load
  use yaml.safe_load instead of deprecated yaml.load
* Python 3 compatibility: relative imports and print statement
* Contributors: Mikael Arguedas, Shane Loretz, Timon Engelke, Tully Foote, Victor Lamoine

0.6.5 (2018-11-16)
------------------

0.6.4 (2018-11-06)
------------------

0.6.3 (2018-07-09)
------------------

0.6.2 (2018-05-02)
------------------
* Tf2 tools echo (`#289 <https://github.com/ros/geometry2/issues/289>`_)
  * tf2_tools echo is working but not yet printing the rotation `#287 <https://github.com/ros/geometry2/issues/287>`_
  * install echo.py
  * Added quaternion output but importing from tf1 for euler_from_quaternion seems wrong (`#222 <https://github.com/ros/geometry2/issues/222>`_) so not doing that yet.  Also made count exit after n counts even if exceptions occurred, also printing time of lookup for exceptions `#287 <https://github.com/ros/geometry2/issues/287>`_
  * Fixed time query option, also changing message text to be more clear `#287 <https://github.com/ros/geometry2/issues/287>`_
  * Added bsd license, code from transform3d transformations.py `#287 <https://github.com/ros/geometry2/issues/287>`_
  * Get rid of tabs
  * docstring for each function
* Contributors: Lucas Walter

0.6.1 (2018-03-21)
------------------

0.6.0 (2018-03-21)
------------------

0.5.17 (2018-01-01)
-------------------
* Merge pull request `#268 <https://github.com/ros/geometry2/issues/268>`_ from smnogar/indigo-devel
  Fixed for cases of non-standard python install
* Contributors: Steve Nogar, Tully Foote

0.5.16 (2017-07-14)
-------------------

0.5.15 (2017-01-24)
-------------------

0.5.14 (2017-01-16)
-------------------
* Remove old load_manifest from view_frames (`#182 <https://github.com/ros/geometry2/issues/182>`_)
* Contributors: Jochen Sprickerhof

0.5.13 (2016-03-04)
-------------------
* casted el to string in view_frames
* Contributors: g_gemignani

0.5.12 (2015-08-05)
-------------------

0.5.11 (2015-04-22)
-------------------

0.5.10 (2015-04-21)
-------------------

0.5.9 (2015-03-25)
------------------

0.5.8 (2015-03-17)
------------------
* remove useless Makefile files
* Contributors: Vincent Rabaud

0.5.7 (2014-12-23)
------------------

0.5.6 (2014-09-18)
------------------

0.5.5 (2014-06-23)
------------------

0.5.4 (2014-05-07)
------------------

0.5.3 (2014-02-21)
------------------

0.5.2 (2014-02-20)
------------------

0.5.1 (2014-02-14)
------------------

0.5.0 (2014-02-14)
------------------

0.4.10 (2013-12-26)
-------------------

0.4.9 (2013-11-06)
------------------

0.4.8 (2013-11-06)
------------------
* updating install rule for view_frames.py fixes `#44 <https://github.com/ros/geometry_experimental/issues/44>`_

0.4.7 (2013-08-28)
------------------

0.4.6 (2013-08-28)
------------------

0.4.5 (2013-07-11)
------------------

0.4.4 (2013-07-09)
------------------

0.4.3 (2013-07-05)
------------------

0.4.2 (2013-07-05)
------------------

0.4.1 (2013-07-05)
------------------

0.4.0 (2013-06-27)
------------------
* splitting rospy dependency into tf2_py so tf2 is pure c++ library.
* Restoring test packages and bullet packages.
  reverting 3570e8c42f9b394ecbfd9db076b920b41300ad55 to get back more of the packages previously implemented
  reverting 04cf29d1b58c660fdc999ab83563a5d4b76ab331 to fix `#7 <https://github.com/ros/geometry_experimental/issues/7>`_

0.3.6 (2013-03-03)
------------------

0.3.5 (2013-02-15 14:46)
------------------------
* 0.3.4 -> 0.3.5

0.3.4 (2013-02-15 13:14)
------------------------
* 0.3.3 -> 0.3.4

0.3.3 (2013-02-15 11:30)
------------------------
* 0.3.2 -> 0.3.3

0.3.2 (2013-02-15 00:42)
------------------------
* 0.3.1 -> 0.3.2

0.3.1 (2013-02-14)
------------------
* 0.3.0 -> 0.3.1

0.3.0 (2013-02-13)
------------------
* switching to version 0.3.0
* removing packages with missing deps
* catkinizing geometry-experimental
* catkinizing tf2_tools
* strip out rx dependencies
* Some fixes to make things work with rxbag
* Threading ns list
* merge tf2_cpp and tf2_py into tf2_ros
* Now catching exceptions correctly with echo
* Working version of tf echo
* Making sure to clear details when switching frames
* Changing file format to tf
* First cut at loading, saving, and exporting support
* tf frame viewer is now an rxbag plugin
* Can now connect to any node in the system that has a tf2 buffer
* Now populates namespaces as well
* Now populates a frame list on the fly
* Got the GUI set up for a bunch of features, now just have to implement the backend of them
* Persistent service call to speed things up. Also, coloring on click
* Adding a first version of frame_viewer
* Adding xdot as a dep in prep for frame_viewer
* working view frames
* call new service
* new version of view_frames in new tf2_tools package
