^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_bullet
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.5 (2020-09-01)
------------------

0.7.4 (2020-09-01)
------------------

0.7.3 (2020-08-25)
------------------

0.7.2 (2020-06-08)
------------------

0.7.1 (2020-05-13)
------------------
* [noetic] cherry-pick Windows fixes from melodic-devel (`#450 <https://github.com/ros/geometry2/issues/450>`_)
  * [Windows][melodic-devel] Fix install locations (`#442 <https://github.com/ros/geometry2/issues/442>`_)
  * fixed install locations of tf2
  * [windows][melodic] more portable fixes. (`#443 <https://github.com/ros/geometry2/issues/443>`_)
  * more portable fixes.
* Contributors: Sean Yen

0.7.0 (2020-03-09)
------------------
* Bump CMake version to avoid CMP0048 warning (`#445 <https://github.com/ros/geometry2/issues/445>`_)
* Fix compile error missing ros/ros.h (`#400 <https://github.com/ros/geometry2/issues/400>`_)
  * ros/ros.h -> ros/time.h
  * tf2_bullet doesn't need ros.h
  * tf2_eigen doesn't need ros/ros.h
* use find_package when pkg_check_modules doesn't work (`#364 <https://github.com/ros/geometry2/issues/364>`_)
  * use find_package bullet for MSVC
  * use find_package when pkg_check_modules fails (`#8 <https://github.com/ros/geometry2/issues/8>`_)
  * use find_package() when pkg_check_modules() doesn't work
* Contributors: James Xu, Shane Loretz

0.6.5 (2018-11-16)
------------------

0.6.4 (2018-11-06)
------------------

0.6.3 (2018-07-09)
------------------

0.6.2 (2018-05-02)
------------------

0.6.1 (2018-03-21)
------------------

0.6.0 (2018-03-21)
------------------

0.5.17 (2018-01-01)
-------------------

0.5.16 (2017-07-14)
-------------------
* store gtest return value as int (`#229 <https://github.com/ros/geometry2/issues/229>`_)
* Contributors: dhood

0.5.15 (2017-01-24)
-------------------

0.5.14 (2017-01-16)
-------------------
* Improve documentation
* Contributors: Jackie Kay

0.5.13 (2016-03-04)
-------------------
* Don't export catkin includes
  They only point to the temporary include in the build directory.
* Contributors: Jochen Sprickerhof

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
* fix ODR violations
* Contributors: Vincent Rabaud

0.5.7 (2014-12-23)
------------------
* fixing install rules and adding backwards compatible include with #warning
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
* adding missing buildtool dependency on pkg-config

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
* removing unused dependency on rostest

0.4.1 (2013-07-05)
------------------
* stripping tf2_ros dependency from tf2_bullet.  Test was moved to test_tf2

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

0.3.4 (2013-02-15 13:14)
------------------------

0.3.3 (2013-02-15 11:30)
------------------------

0.3.2 (2013-02-15 00:42)
------------------------

0.3.1 (2013-02-14)
------------------

0.3.0 (2013-02-13)
------------------
* fixing groovy-devel
* removing bullet and kdl related packages
* catkinizing geometry-experimental
* catkinizing tf2_bullet
* merge tf2_cpp and tf2_py into tf2_ros
* A working first version of transforming and converting between different types
* Fixing tests now that Buffer creates a NodeHandle
* add frame unit tests to kdl and bullet
* add first regression tests for kdl and bullet tf
* add btTransform transform
* add bullet transforms, and create tests for bullet and kdl
