^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_sensor_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge pull request `#378 <https://github.com/ros/geometry2/issues/378>`_ from peci1/tf2_sensor_msgs_isometry
  Affine->Isometry
* Python 3 compatibility: relative imports and print statement
* Contributors: Martin Pecka, Shane Loretz, Timon Engelke, Tully Foote

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
* Merge pull request `#257 <https://github.com/ros/geometry2/issues/257>`_ from delftrobotics-forks/python3
  Make tf2_py python3 compatible again
* Use python3 print function.
* Contributors: Maarten de Vries, Tully Foote

0.5.16 (2017-07-14)
-------------------
* Fix do_transform_cloud for multi-channelled pointcloud2. (`#241 <https://github.com/ros/geometry2/issues/241>`_)
* store gtest return value as int (`#229 <https://github.com/ros/geometry2/issues/229>`_)
* Document the lifetime of the returned reference for getFrameId and getTimestamp
* Find eigen in a much nicer way.
* Switch tf2_sensor_msgs over to package format 2.
* Contributors: Atsushi Watanabe, Chris Lalancette, dhood

0.5.15 (2017-01-24)
-------------------

0.5.14 (2017-01-16)
-------------------

0.5.13 (2016-03-04)
-------------------
* add missing Python runtime dependency
* fix wrong comment
* Adding tests to package
* Fixing do_transform_cloud for python
  The previous code was not used at all (it was a mistake in the __init_\_.py so
  the do_transform_cloud was not available to the python users).
  The python code need some little correction (e.g there is no method named
  read_cloud but it's read_points for instance, and as we are in python we can't
  use the same trick as in c++ when we got an immutable)
* Contributors: Laurent GEORGE, Vincent Rabaud

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
* ODR violation fixes and more conversions
* Fix keeping original pointcloud header in transformed pointcloud
* Contributors: Paul Bovbel, Tully Foote, Vincent Rabaud

0.5.7 (2014-12-23)
------------------
* add support for transforming sensor_msgs::PointCloud2
* Contributors: Vincent Rabaud
