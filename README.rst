Description
===========

This is a naoqi driver module that bridges with ROS. It publishes
several sensor data as well as the robot position.

On the other hand it enables ROS to call parts of the
NAOqi API.

What it does
============

The **naoqi_driver** module is in charge of providing some
bridge capabilities between ROS and NAOqiOS.

How it works
============

The **naoqi_driver** module is a NAOqi module that also acts
as a ROS node. It needs to be given the IP of the **roscore** in order to be
registered as a node in the ROS processing graph. Usually,
you will start your **roscore** on your local desktop 
(or on a robot in case of instaling ROS there).

Once connected, ROS communicates with NAOqi OS running on your robot.


For further information, you can go `here <http://ros-naoqi.github.io/naoqi_driver/>`_ or build the doc:

.. code-block:: sh

  cd doc
  doxygen Doxyfile
  sphinx-build -b html ./source/ ./build/
