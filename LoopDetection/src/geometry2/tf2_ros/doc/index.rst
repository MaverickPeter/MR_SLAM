tf2_ros Overview
================

This is the Python API reference for the tf2_ros package.

To broadcast transforms using ROS:
- Call :meth:`rospy.init` to initialize a node.
- Construct a :class:`tf2_ros.TransformBroadcaster`.
- Pass a :class:`geometry_msgs.TransformStamped` message to :meth:`tf2_ros.TransformBroadcaster.sendTransform`.

    - Alternatively, pass a vector of :class:`geometry_msgs.TransformStamped` messages.

To listen for transforms using ROS:
- Construct an instance of a class that implements :class:`tf2_ros.BufferInterface`.

    - :class:`tf2_ros.Buffer` is the standard implementation which offers a tf2_frames service that can respond to requests with a :class:`tf2_msgs.FrameGraph`.
    - :class:`tf2_ros.BufferClient` uses an :class:`actionlib.SimpleActionClient` to wait for the requested transform to become available.

- Pass the :class:`tf2_ros.Buffer` to the constructor of :class:`tf2_ros.TransformListener`.
    - Optionally, pass a :class:`ros.NodeHandle` (otherwise TransformListener will connect to the node for the process).
    - Optionally, specify if the TransformListener runs in its own thread or not.

- Use :meth:`tf2_ros.BufferInterface.transform` to apply a transform on the tf server to an input frame.
    - Or, check if a transform is available with :meth:`tf2_ros.BufferInterface.can_transform`.
    - Then, call :meth:`tf2_ros.BufferInterface.lookup_transform` to get the transform between two frames.

For more information, see the tf2 tutorials: http://wiki.ros.org/tf2/Tutorials

Or, get an `overview`_ of data type conversion methods in geometry_experimental packages.

See http://wiki.ros.org/tf2/Tutorials for more detailed usage.

.. _overview: http://wiki.ros.org/tf2/Tutorials/Migration/DataConversions

Classes and Exceptions
======================

.. toctree::
    :maxdepth: 2

    tf2_ros


Indices and tables
==================

* :ref:`genindex`
* :ref:`search`
