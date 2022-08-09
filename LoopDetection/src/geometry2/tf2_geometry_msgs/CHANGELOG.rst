^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_geometry_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.5 (2020-09-01)
------------------

0.7.4 (2020-09-01)
------------------

0.7.3 (2020-08-25)
------------------
* Use list instead of set to make build reproducible (`#473 <https://github.com/ros/geometry2/issues/473>`_)
* Contributors: Jochen Sprickerhof

0.7.2 (2020-06-08)
------------------

0.7.1 (2020-05-13)
------------------
* import setup from setuptools instead of distutils-core (`#449 <https://github.com/ros/geometry2/issues/449>`_)
* Contributors: Alejandro Hernández Cordero

0.7.0 (2020-03-09)
------------------
* Replace kdl packages with rosdep keys (`#447 <https://github.com/ros/geometry2/issues/447>`_)
* Bump CMake version to avoid CMP0048 warning (`#445 <https://github.com/ros/geometry2/issues/445>`_)
* Make kdl headers available (`#419 <https://github.com/ros/geometry2/issues/419>`_)
* FIx python3 compatibility for noetic (`#416 <https://github.com/ros/geometry2/issues/416>`_)
* add <array> from STL (`#366 <https://github.com/ros/geometry2/issues/366>`_)
* use ROS_DEPRECATED macro for portability (`#362 <https://github.com/ros/geometry2/issues/362>`_)
  * use ROS_DEPRECATED for better portability
  * change ROS_DEPRECATED position (`#5 <https://github.com/ros/geometry2/issues/5>`_)
* Contributors: James Xu, Shane Loretz, Tully Foote

0.6.5 (2018-11-16)
------------------
* Fix python3 import error
* Contributors: Timon Engelke

0.6.4 (2018-11-06)
------------------

0.6.3 (2018-07-09)
------------------
* Changed access to Vector to prevent memory leak (`#305 <https://github.com/ros/geometry2/issues/305>`_)
* Added WrenchStamped transformation (`#302 <https://github.com/ros/geometry2/issues/302>`_)
* Contributors: Denis Štogl, Markus Grimm

0.6.2 (2018-05-02)
------------------

0.6.1 (2018-03-21)
------------------

0.6.0 (2018-03-21)
------------------
* Boilerplate for Sphinx (`#284 <https://github.com/ros/geometry2/issues/284>`_)
  Fixes `#264 <https://github.com/ros/geometry2/issues/264>`_
* tf2_geometry_msgs added doTransform implementations for not stamped types (`#262 <https://github.com/ros/geometry2/issues/262>`_)
  * tf2_geometry_msgs added doTransform implementations for not stamped Point, Quaterion, Pose and Vector3 message types
* New functionality to transform PoseWithCovarianceStamped messages. (`#282 <https://github.com/ros/geometry2/issues/282>`_)
  * New functionality to transform PoseWithCovarianceStamped messages.
* Contributors: Blake Anderson, Tully Foote, cwecht

0.5.17 (2018-01-01)
-------------------

0.5.16 (2017-07-14)
-------------------
* remove explicit templating to standardize on overloading. But provide backwards compatibility with deprecation.
* adding unit tests for conversions
* Copy transform before altering it in do_transform_vector3 [issue 233] (`#235 <https://github.com/ros/geometry2/issues/235>`_)
* store gtest return value as int (`#229 <https://github.com/ros/geometry2/issues/229>`_)
* Document the lifetime of the returned reference for getFrameId and getTimestamp
* tf2_geometry_msgs: using tf2::Transform in doTransform-functions, marked gmTransformToKDL as deprecated
* Switch tf2_geometry_msgs to use package.xml format 2 (`#217 <https://github.com/ros/geometry2/issues/217>`_)
* tf2_geometry_msgs: added missing conversion functions
* Contributors: Christopher Wecht, Sebastian Wagner, Tully Foote, dhood, pAIgn10

0.5.15 (2017-01-24)
-------------------

0.5.14 (2017-01-16)
-------------------
* Add doxygen documentation for tf2_geometry_msgs
* Contributors: Jackie Kay

0.5.13 (2016-03-04)
-------------------
* Add missing python_orocos_kdl dependency
* make example into unit test
* vector3 not affected by translation
* Contributors: Daniel Claes, chapulina

0.5.12 (2015-08-05)
-------------------
* Merge pull request `#112 <https://github.com/ros/geometry_experimental/issues/112>`_ from vrabaud/getYaw
  Get yaw
* add toMsg and fromMsg for QuaternionStamped
* Contributors: Tully Foote, Vincent Rabaud

0.5.11 (2015-04-22)
-------------------

0.5.10 (2015-04-21)
-------------------

0.5.9 (2015-03-25)
------------------

0.5.8 (2015-03-17)
------------------
* remove useless Makefile files
* tf2 optimizations
* add conversions of type between tf2 and geometry_msgs
* fix ODR violations
* Contributors: Vincent Rabaud

0.5.7 (2014-12-23)
------------------
* fixing transitive dependency for kdl. Fixes `#53 <https://github.com/ros/geometry_experimental/issues/53>`_
* Contributors: Tully Foote

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

0.4.7 (2013-08-28)
------------------

0.4.6 (2013-08-28)
------------------

0.4.5 (2013-07-11)
------------------

0.4.4 (2013-07-09)
------------------
* making repo use CATKIN_ENABLE_TESTING correctly and switching rostest to be a test_depend with that change.

0.4.3 (2013-07-05)
------------------

0.4.2 (2013-07-05)
------------------

0.4.1 (2013-07-05)
------------------

0.4.0 (2013-06-27)
------------------
* moving convert methods back into tf2 because it does not have any ros dependencies beyond ros::Time which is already a dependency of tf2
* Cleaning up unnecessary dependency on roscpp
* converting contents of tf2_ros to be properly namespaced in the tf2_ros namespace
* Cleaning up packaging of tf2 including:
  removing unused nodehandle
  cleaning up a few dependencies and linking
  removing old backup of package.xml
  making diff minimally different from tf version of library
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
* add setup.py
* added setup.py etc to tf2_geometry_msgs
* adding tf2 dependency to tf2_geometry_msgs
* adding tf2_geometry_msgs to groovy-devel (unit tests disabled)
* fixing groovy-devel
* removing bullet and kdl related packages
* disabling tf2_geometry_msgs due to missing kdl dependency
* catkinizing geometry-experimental
* catkinizing tf2_geometry_msgs
* add twist, wrench and pose conversion to kdl, fix message to message conversion by adding specific conversion functions
* merge tf2_cpp and tf2_py into tf2_ros
* Got transform with types working in python
* A working first version of transforming and converting between different types
* Moving from camelCase to undescores to be in line with python style guides
* Fixing tests now that Buffer creates a NodeHandle
* add posestamped
* import vector3stamped
* add support for Vector3Stamped and PoseStamped
* add support for PointStamped geometry_msgs
* add regression tests for geometry_msgs point, vector and pose
* Fixing missing export, compiling version of buffer_client test
* add bullet transforms, and create tests for bullet and kdl
* working transformations of messages
* add support for PoseStamped message
* test for pointstamped
* add PointStamped message transform methods
* transform for vector3stamped message
