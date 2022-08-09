^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.5 (2020-09-01)
------------------

0.7.4 (2020-09-01)
------------------

0.7.3 (2020-08-25)
------------------
* Use correct frame service name in docstrings. (`#476 <https://github.com/ros/geometry2/issues/476>`_)
  Replaces the deprecated names
  {tf_frames, view_frames} -> tf2_frames
* Cherry-picking various commits from Melodic (`#471 <https://github.com/ros/geometry2/issues/471>`_)
  * Revert "rework Eigen functions namespace hack" (`#436 <https://github.com/ros/geometry2/issues/436>`_)
  * Fixed warnings in message_filter.h (`#434 <https://github.com/ros/geometry2/issues/434>`_)
  the variables are not used in function body and caused -Wunused-parameter to trigger with -Wall
  * Fix ambiguous call for tf2::convert on MSVC (`#444 <https://github.com/ros/geometry2/issues/444>`_)
  * rework ambiguous call on MSVC.
* Contributors: Michael Grupp, Robert Haschke

0.7.2 (2020-06-08)
------------------

0.7.1 (2020-05-13)
------------------
* StatisTransformBroadcaster: simplify/modernize code
* [noetic] cherry-pick Windows fixes from melodic-devel (`#450 <https://github.com/ros/geometry2/issues/450>`_)
  * [Windows][melodic-devel] Fix install locations (`#442 <https://github.com/ros/geometry2/issues/442>`_)
  * fixed install locations of tf2
  * [windows][melodic] more portable fixes. (`#443 <https://github.com/ros/geometry2/issues/443>`_)
  * more portable fixes.
* import setup from setuptools instead of distutils-core (`#449 <https://github.com/ros/geometry2/issues/449>`_)
* Contributors: Alejandro Hernández Cordero, Robert Haschke, Sean Yen

0.7.0 (2020-03-09)
------------------
* Bump CMake version to avoid CMP0048 warning (`#445 <https://github.com/ros/geometry2/issues/445>`_)
* Add arguments to TransformListener constructors that accept TransportHints for the tf topic subscriber (`#438 <https://github.com/ros/geometry2/issues/438>`_)
* Merge pull request `#404 <https://github.com/ros/geometry2/issues/404>`_ from otamachan/remove-load-manifest
  Remove roslib.load_manifest
* Merge pull request `#402 <https://github.com/ros/geometry2/issues/402>`_ from rhaschke/fix-message-filter
  Fix message filter
* resolve virtual function call in destructor
* remove pending callbacks in clear()
* Merge pull request `#372 <https://github.com/ros/geometry2/issues/372>`_ from lucasw/patch-1
  spelling fix: seperate -> separate
* Merge pull request `#369 <https://github.com/ros/geometry2/issues/369>`_ from magazino/fix-dangling-reference
* Fix dangling iterator references in buffer_server.cpp
* Remove some useless code from buffer_server_main.cpp (`#368 <https://github.com/ros/geometry2/issues/368>`_)
* Mark check_frequency as deprecated in docstring.
* Follow `#337 <https://github.com/ros/geometry2/issues/337>`_: use actionlib API in BufferClient::processGoal()
* Test for equality to None with 'is' instead of '==' (`#355 <https://github.com/ros/geometry2/issues/355>`_)
* added parameter to advertise tf2-frames as a service, if needed
* Contributors: Daniel Ingram, Emre Sahin, JonasTietz, Lucas Walter, Michael Grupp, Robert Haschke, Shane Loretz, Tamaki Nishino, Tully Foote, toliver

0.6.5 (2018-11-16)
------------------
* Protect the time reset logic from a race condition.
  Fixes `#341 <https://github.com/ros/geometry2/issues/341>`_
  This could incorrectly trigger a buffer clear if two concurrent callbacks were invoked.
* Contributors: Tully Foote

0.6.4 (2018-11-06)
------------------
* fix(buffer-client): Use actionlib api for obtaining result
  Use the API provided by actionlib for waiting for result. This will improve the response time and prevent problems with custom solutions (see `#178 <https://github.com/ros/geometry2/issues/178>`_). This change makes constructor parameter check_frequency obsolute and deprecates it.
* Add check to buffer_client.py to make sure result is available
  Related issue: `#178 <https://github.com/ros/geometry2/issues/178>`_
* Add check to reset buffer when rostime goes backwards
* Fixed the value of expected_success_count\_
* Added a tf2_ros message filter unittest with multiple target frames and non-zero time tolerance
* Contributors: Ewoud Pool, Jørgen Borgesen, Stephen Williams

0.6.3 (2018-07-09)
------------------

0.6.2 (2018-05-02)
------------------
* update buffer_server_name (`#296 <https://github.com/ros/geometry2/issues/296>`_)
  * use nodename as namespace
  * Update `#209 <https://github.com/ros/geometry2/issues/209>`_ to provide backwards compatibility.
* Contributors: Jihoon Lee, Tully Foote

0.6.1 (2018-03-21)
------------------

0.6.0 (2018-03-21)
------------------
* tf2_ros::Buffer: canTransform can now deal with timeouts smaller than 10ms by using the hunderdth of the timeout for sleeping (`#286 <https://github.com/ros/geometry2/issues/286>`_)
* More spinning to make sure the message gets through for `#129 <https://github.com/ros/geometry2/issues/129>`_ `#283 <https://github.com/ros/geometry2/issues/283>`_
* Contributors: Tully Foote, cwecht

0.5.17 (2018-01-01)
-------------------
* Merge pull request `#260 <https://github.com/ros/geometry2/issues/260>`_ from randoms/indigo-devel
  fix python3 import error
* Merge pull request `#257 <https://github.com/ros/geometry2/issues/257>`_ from delftrobotics-forks/python3
  Make tf2_py python3 compatible again
* Use python3 print function.
* Contributors: Maarten de Vries, Tully Foote, randoms

0.5.16 (2017-07-14)
-------------------
* Merge pull request `#144 <https://github.com/ros/geometry2/issues/144>`_ from clearpathrobotics/dead_lock_fix
  Solve a bug that causes a deadlock in MessageFilter
* Clear error string if it exists from the external entry points.
  Fixes `#117 <https://github.com/ros/geometry2/issues/117>`_
* Make buff_size and tcp_nodelay and subscriber queue size mutable.
* Remove generate_rand_vectors() from a number of tests. (`#227 <https://github.com/ros/geometry2/issues/227>`_)
  * Remove generate_rand_vectors() from a number of tests.
* Log jump duration on backwards time jump detection. (`#234 <https://github.com/ros/geometry2/issues/234>`_)
* replaced dependencies on tf2_msgs_gencpp by exported dependencies
* Use new-style objects in python 2
* Solve a bug that causes a deadlock in MessageFilter
* Contributors: Adel Fakih, Chris Lalancette, Christopher Wecht, Eric Wieser, Koji Terada, Stephan, Tully Foote, koji_terada

0.5.15 (2017-01-24)
-------------------
* tf2_ros: add option to unregister TransformListener (`#201 <https://github.com/ros/geometry2/issues/201>`_)
* Contributors: Hans-Joachim Krauch

0.5.14 (2017-01-16)
-------------------
* Drop roslib.load_manifest (`#191 <https://github.com/ros/geometry2/issues/191>`_)
* Adds ability to load TF from the ROS parameter server.
* Code linting & reorganization
* Fix indexing beyond end of array
* added a static transform broadcaster in python
* lots more documentation
* remove BufferCore doc, add BufferClient/BufferServer doc for C++, add Buffer/BufferInterface Python documentation
* Better overview for Python
* Contributors: Eric Wieser, Felix Duvallet, Jackie Kay, Mikael Arguedas, Mike Purvis

0.5.13 (2016-03-04)
-------------------
* fix documentation warnings
* Adding tests to package
* Contributors: Laurent GEORGE, Vincent Rabaud

0.5.12 (2015-08-05)
-------------------
* remove annoying gcc warning
  This is because the roslog macro cannot have two arguments that are
  formatting strings: we need to concatenate them first.
* break canTransform loop only for non-tiny negative time deltas
  (At least) with Python 2 ros.Time.now() is not necessarily monotonic
  and one can experience negative time deltas (usually well below 1s)
  on real hardware under full load. This check was originally introduced
  to allow for backjumps with rosbag replays, and only there it makes sense.
  So we'll add a small duration threshold to ignore backjumps due to
  non-monotonic clocks.
* Contributors: Vincent Rabaud, v4hn

0.5.11 (2015-04-22)
-------------------
* do not short circuit waitForTransform timeout when running inside pytf. Fixes `#102 <https://github.com/ros/geometry_experimental/issues/102>`_
  roscpp is not initialized inside pytf which means that ros::ok is not
  valid. This was causing the timer to abort immediately.
  This breaks support for pytf with respect to early breaking out of a loop re `#26 <https://github.com/ros/geometry_experimental/issues/26>`_.
  This is conceptually broken in pytf, and is fixed in tf2_ros python implementation.
  If you want this behavior I recommend switching to the tf2 python bindings.
* inject timeout information into error string for canTransform with timeout
* Contributors: Tully Foote

0.5.10 (2015-04-21)
-------------------
* switch to use a shared lock with upgrade instead of only a unique lock. For `#91 <https://github.com/ros/geometry_experimental/issues/91>`_
* Update message_filter.h
* filters: fix unsupported old messages with frame_id starting with '/'
* Enabled tf2 documentation
* make sure the messages get processed before testing the effects. Fixes `#88 <https://github.com/ros/geometry_experimental/issues/88>`_
* allowing to use message filters with PCL types
* Contributors: Brice Rebsamen, Jackie Kay, Tully Foote, Vincent Rabaud, jmtatsch

0.5.9 (2015-03-25)
------------------
* changed queue_size in Python transform boradcaster to match that in c++
* Contributors: mrath

0.5.8 (2015-03-17)
------------------
* fix deadlock `#79 <https://github.com/ros/geometry_experimental/issues/79>`_
* break out of loop if ros is shutdown. Fixes `#26 <https://github.com/ros/geometry_experimental/issues/26>`_
* remove useless Makefile files
* Fix static broadcaster with rpy args
* Contributors: Paul Bovbel, Tully Foote, Vincent Rabaud

0.5.7 (2014-12-23)
------------------
* Added 6 param transform again
  Yes, using Euler angles is a bad habit. But it is much more convenient if you just need a rotation by 90° somewhere to set it up in Euler angles. So I added the option to supply only the 3 angles.
* Remove tf2_py dependency for Android
* Contributors: Achim Königs, Gary Servin

0.5.6 (2014-09-18)
------------------
* support if canTransform(...): in python `#57 <https://github.com/ros/geometry_experimental/issues/57>`_
* Support clearing the cache when time jumps backwards `#68 <https://github.com/ros/geometry_experimental/issues/68>`_
* Contributors: Tully Foote

0.5.5 (2014-06-23)
------------------

0.5.4 (2014-05-07)
------------------
* surpressing autostart on the server objects to not incur warnings
* switch to boost signals2 following `ros/ros_comm#267 <https://github.com/ros/ros_comm/issues/267>`_, blocking `ros/geometry#23 <https://github.com/ros/geometry/issues/23>`_
* fix compilation with gcc 4.9
* make can_transform correctly wait
* explicitly set the publish queue size for rospy
* Contributors: Tully Foote, Vincent Rabaud, v4hn

0.5.3 (2014-02-21)
------------------

0.5.2 (2014-02-20)
------------------

0.5.1 (2014-02-14)
------------------
* adding const to MessageEvent data
* Contributors: Tully Foote

0.5.0 (2014-02-14)
------------------
* TF2 uses message events to get connection header information
* Contributors: Kevin Watts

0.4.10 (2013-12-26)
-------------------
* adding support for static transforms in python listener. Fixes `#46 <https://github.com/ros/geometry_experimental/issues/46>`_
* Contributors: Tully Foote

0.4.9 (2013-11-06)
------------------

0.4.8 (2013-11-06)
------------------
* fixing pytf failing to sleep https://github.com/ros/geometry/issues/30
* moving python documentation to tf2_ros from tf2 to follow the code
* Fixed static_transform_publisher duplicate check, added rostest.

0.4.7 (2013-08-28)
------------------
* fixing new conditional to cover the case that time has not progressed yet port forward of `ros/geometry#35 <https://github.com/ros/geometry/issues/35>`_ in the python implementation
* fixing new conditional to cover the case that time has not progressed yet port forward of `ros/geometry#35 <https://github.com/ros/geometry/issues/35>`_

0.4.6 (2013-08-28)
------------------
* patching python implementation for `#24 <https://github.com/ros/geometry_experimental/issues/24>`_ as well
* Stop waiting if time jumps backwards.  fixes `#24 <https://github.com/ros/geometry_experimental/issues/24>`_
* patch to work around uninitiaized time. `#30 https://github.com/ros/geometry/issues/30`_
* Removing unnecessary CATKIN_DEPENDS  `#18 <https://github.com/ros/geometry_experimental/issues/18>`_

0.4.5 (2013-07-11)
------------------
* Revert "cherrypicking groovy patch for `#10 <https://github.com/ros/geometry_experimental/issues/10>`_ into hydro"
  This reverts commit 296d4916706d64f719b8c1592ab60d3686f994e1.
  It was not starting up correctly.
* fixing usage string to show quaternions and using quaternions in the test app
* cherrypicking groovy patch for `#10 <https://github.com/ros/geometry_experimental/issues/10>`_ into hydro

0.4.4 (2013-07-09)
------------------
* making repo use CATKIN_ENABLE_TESTING correctly and switching rostest to be a test_depend with that change.
* reviving unrun unittest and adding CATKIN_ENABLE_TESTING guards

0.4.3 (2013-07-05)
------------------

0.4.2 (2013-07-05)
------------------

0.4.1 (2013-07-05)
------------------
* adding queue accessors lost in the new API
* exposing dedicated thread logic in BufferCore and checking in Buffer
* adding methods to enable backwards compatability for passing through to tf::Transformer

0.4.0 (2013-06-27)
------------------
* splitting rospy dependency into tf2_py so tf2 is pure c++ library.
* moving convert methods back into tf2 because it does not have any ros dependencies beyond ros::Time which is already a dependency of tf2
* Cleaning up unnecessary dependency on roscpp
* converting contents of tf2_ros to be properly namespaced in the tf2_ros namespace
* fixing return by value for tranform method without preallocatoin
* Cleaning up packaging of tf2 including:
  removing unused nodehandle
  cleaning up a few dependencies and linking
  removing old backup of package.xml
  making diff minimally different from tf version of library
* Restoring test packages and bullet packages.
  reverting 3570e8c42f9b394ecbfd9db076b920b41300ad55 to get back more of the packages previously implemented
  reverting 04cf29d1b58c660fdc999ab83563a5d4b76ab331 to fix `#7 <https://github.com/ros/geometry_experimental/issues/7>`_
* Added link against catkin_LIBRARIES for tf2_ros lib, also CMakeLists.txt clean up

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
* Merge pull request `#2 <https://github.com/ros/geometry_experimental/issues/2>`_ from KaijenHsiao/groovy-devel
  added setup.py and catkin_python_setup() to tf2_ros
* added setup.py and catkin_python_setup() to tf2_ros
* fixing cmake target collisions
* fixing catkin message dependencies
* removing packages with missing deps
* catkin fixes
* catkinizing geometry-experimental
* catkinizing tf2_ros
* catching None result in buffer client before it becomes an AttributeError, raising tf2.TransformException instead
* oneiric linker fixes, bump version to 0.2.3
* fix deprecated use of Header
* merged faust's changes 864 and 865 into non_optimized branch: BufferCore instead of Buffer in TransformListener, and added a constructor that takes a NodeHandle.
* add buffer server binary
* fix compilation on 32bit
* add missing file
* build buffer server
* TransformListener only needs a BufferCore
* Add TransformListener constructor that takes a NodeHandle so you can specify a callback queue to use
* Add option to use a callback queue in the message filter
* move the message filter to tf2_ros
* add missing std_msgs dependency
* missed 2 lines in last commit
* removing auto clearing from listener for it's unexpected from a library
* static transform tested and working
* subscriptions to tf_static unshelved
* static transform publisher executable running
* latching static transform publisher
* cleaning out old commented code
* Only query rospy.Time.now() when the timeout is greater than 0
* debug comments removed
* move to tf2_ros completed. tests pass again
* merge tf2_cpp and tf2_py into tf2_ros
