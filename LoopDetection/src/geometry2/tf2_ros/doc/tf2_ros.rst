tf_ros2 Python API
==================

Exceptions
----------

.. exception:: tf2.TransformException

    base class for tf exceptions.  Because :exc:`tf2.TransformException` is the
    base class for other exceptions, you can catch all tf exceptions
    by writing::

        try:
            # do some tf2 work
        except tf2.TransformException:
            print "some tf2 exception happened"
        

.. exception:: tf2.ConnectivityException

   subclass of :exc:`TransformException`.
   Raised when that the fixed_frame tree is not connected between the frames requested.

.. exception:: tf2.LookupException

   subclass of :exc:`TransformException`.
   Raised when a tf method has attempted to access a frame, but
   the frame is not in the graph.
   The most common reason for this is that the frame is not
   being published, or a parent frame was not set correctly 
   causing the tree to be broken.  

.. exception:: tf2.ExtrapolationException

   subclass of :exc:`TransformException`
   Raised when a tf method would have required extrapolation beyond current limits.


.. exception:: tf2.InvalidArgumentException

   subclass of :exc:`TransformException`.
   Raised when the arguments to the method are called improperly formed.  An example of why this might be raised is if an argument is nan. 

.. autoexception:: tf2_ros.buffer_interface.TypeException

.. autoexception:: tf2_ros.buffer_interface.NotImplementedException


BufferInterface
---------------
.. autoclass:: tf2_ros.buffer_interface.BufferInterface
    :members:

Buffer
------
.. autoclass:: tf2_ros.buffer.Buffer
    :members:

BufferClient
------------
.. autoclass:: tf2_ros.buffer_client.BufferClient
    :members:


TransformBroadcaster
--------------------
.. autoclass:: tf2_ros.transform_broadcaster.TransformBroadcaster
    :members:

TransformListener
-----------------
.. autoclass:: tf2_ros.transform_listener.TransformListener
    :members:
