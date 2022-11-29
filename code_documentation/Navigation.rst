Code docs -- *Navigation.py*
=================================

:Authors:
    HolyStone95
:Version:
    1.0

This nodes implements two different service servers 

The "go_to_point" service 
--------------------------

the robot moves from a waypoint to another by means of a 
service call to the ``move_base`` action (navigation stack).
Hence, it waits until the robot reaches the given target (or a
neighbouring position of 35 cm).  
Here below the prototype of the robot is reported

.. code-block::
	   
     ## service file 'GoToPoint.srv'

     # the target point to go at 
     geometry_msgs/Point target
     
     ---
     
     # res to set the service as succeded or failed 
     bool success


..
    Once the target gets reached, the service boolean response 
    turns to be True, indicating the successfull outcome of
    the service



The "turn the robot" service
-----------------------------

The aim of this service is to listen for a request containing an
**angular velocity** we want the robot to acquire, for making it 
capable of turning around itself within a specific time interval.
Here below the prototype of the robot is reported 

.. code-block::
    
    ## service file 'TurnRobot.srv'

    # angular velocity request 
    float32 angularVel

    # time allocated for turning around 
    float32 time

    ---

    # res to set the service as succeded or failed 
    bool success

..
    This service will be functional to the actual detection of
    the aruco markers scattered around the map 


Code Reference
---------------

.. automodule:: navigation
	:members:
	:noindex:
