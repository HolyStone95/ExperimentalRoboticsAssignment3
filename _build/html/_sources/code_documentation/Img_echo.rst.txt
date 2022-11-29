Code docs -- *Img_echo.cpp*
============================

:Authors:
    HolyStone95
:Version:
    1.0

About the node 
---------------

This node reads the input image from the robot's camera. Secondly, it print it on a floating window, 
namely **DetectiCam**, by means of a ``cv_ptr``. (the ``cv_bridge::CvImagePtr cv_ptr`` returns a ROS image into an appropriate
format compatible with OpenCV). Thirdly it publish the video stream!

.. note::
    Since we have to deal with the image, multiple copies of it will be needed; 
    For this purpose the BGR8 image encoding has been chosen, being it less susceptible against typos. 
    
.. note::
    ImageTransport's methods have been employed for creating image publishers and subscribers,
    being ``image_transport`` a package that provides transparetn support for transporting images in **low-bandwidth** compressed 
    formats. 
    
.. warning::
    Please remember to include cv_bridge in your xml package! Also do not forget to add the following headers to your cpp file



Code Reference
---------------

.. doxygenfile:: img_echo.cpp
    :project: ExperimentalRoboticsAssignment3
