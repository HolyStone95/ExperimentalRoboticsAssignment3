Code docs -- *Detectibot_magnifier.cpp*
=========================================

About the node
---------------

This node is devoted to the detection of ARUCO's markers made through a single camera mounted on the front side of the robot.
It also implements a service that allows for retrieving the IDs identified through Aruco.

Generalities
-------------
For realising such a node, the ``vision_openCV`` packages, aimed at interfacing ROS with OpenCV have been emploied. 
OpenCV basically consists in a library of programming functions for real time computer vision. Hence this node 
employs a bridge between OpenCV and ROS. Due to the fact that ROS sends Images in ``sensor_msgs/Image`` format, our 
goal is to obtain images in ``cv_bridge`` format.

.. note::
    Please note that By using ``image_transport::Publisher image_pub_`` and subscribing to the topic ``/robot/camera1/image_raw`` 
    we are able to significantly **decrease the bandwidth**!

Code Reference
---------------

.. doxygenfile:: detectibot_magnifier.cpp
    :project: ExperimentalRoboticsAssignment3
