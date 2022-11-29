# Experimental Robotics Laboratory - third assignment

This package contains the source code implemented during the third assignment of the Experimental Robotics Laboratory, course of the first semester of the second year of Robotics Engineering Master Degree course of University of Genoa, Italy.

## Index

- Index:
  - [Required packages](#required-packages)
  - [Assignment request](#assignment-request)
	- [Before Getting Started](#before-getting-started)
  - [Software Architecture and Functioning](#software-architecture-and-functioning)
  - [ROS nodes description](#ros-nodes-description)
    - [The state_machine.py node](#the-state_machine.py-node)
    - [The navigation.py node](#the-navigation.py-node)
    - [The cluedo_kb.py node](#the-cluedo_kb.py-node)
    - [The simulation.cpp node](#the-simulation.cpp-node)
    - [The image_echo.cpp node](#the-image_echo.cpp-node)
    - [The detectibot_magnifier.cpp node](#the-detectibot_magnifier.cpp-node)
    - [Rqt_graph](#rqt_graph)
    - [UML temporal diagram](#uml-temporal-diagram)
		- [Summarizing working architecture](#summarizing-working-architecture)
  - [How to launch and additional documentation](#how-to-launch-and-additional-documentation)
		- [Running the Navigation module](#running-the-navigation-module)
		- [Running the Visio  module](#running-the-vision-module)
		- [Running the State machine module](#running-the-state-machine-module)
  - [System limitations](#system-limitations)
  - [Possible Technical Improvements](#possible-technical-improvements)
  - [Contact](#contact)

## Required packages

	* **ROS**
	* **smach**
	* **OpenCV**
	* **MoveIt Frameowrk**
	* **move_base**

## Assignment request

This assignment improves the architectures developed in the first and second assignments but unlike them, the environment in which the robot moves is much more complex. Indeed, it presents several rooms, and 30 ArUco markers (5 markers for each room)

This time, each marker corresponds to a hint, which are always given with the following structure:

```
int32 ID
string key
string value
```

Also, markers may me in three different positions: placed on the walls (height 1 m ca.), and on the
floor (placed vertically or horizonally).

The idea is the same of the previous two assignment: the robot should keep receiving hints until it has a complete and consistent hypothesis. However, as in the previous assignments, only one ID source is the trustable one.

As soon as the robot gets a complete hypothesis, it should go in the center of the arena (**x=0.0**, **y=-1.0**, which should be also the starting position of the robot) and «tells» its solution.

If the solution is the correct one, the game ends.

> **REMARK** x and y coordinates belonging to each room's point where known a priori as shown in the table below   

	| room  | x,y coordinates  |
	|--|--|
	| Room1 | ( -4 , -3 ) |
	| Room2 | ( -4 , +2 ) |
	| Room3 | ( -4 , +7 ) |
	| Room4 | ( 5 , -7 )  |
	| Room5 | ( 5 , -3 )  |
	| Room6 | ( 5 , +1 )  |

There are different values for z, meaning that the robot needs to be able to reach them with it's arm, precisely with its *cluedo_link*.

Concerining the simulation environment, there are small walls around the robot aimed at impeding the movements of its mobile base

Hence the robot moves from one «hint» coordinate to another one, while receiving hints. This holds until it has a complete
and consistent hypothesis

<!-- Note about  consistent huèothesis -->

 Please consider that **consistent hypothesis** have been defined as COMPLETED but NOT INCONSISTENT

> *REMARK* A consistent hypothesis is  defined as *completed* when there occurs one role for each class (i.e., one occourence of what, one occourence for who, one occourence for where ).
A straightforward example of such hypothesis is [ID2][12], whose definition is here below reported

```txt
ID2_1: ['where', 'Study']
ID2_2: ['who', 'Col.Mustard']
ID2_3: ['what', 'Rope']
```

As in the first assignment:
- only one ID source is the trustable one.
- Whenever a robot gets a complete hypothesis, it should go in the center of the arena
- Once the center has been reached, it should «tell» its solution (as in the first assignment).
- If the solution is the **correct one**, the game ends


## Before Getting Started


In order to find the marker models, befire the build, navigate through the directories:

```sh
cd /ros_ws/src/ExperimentalRoboticsAssignment3/erl3/models
```
copy all files inside the `erl3` models folder and navigate to the `root/.gazebo` directory

```sh
cd /root/.gazebo/models
```
paste the previously copied files, containing all the marker's models.

Then you can build the workspace.

## Software Architecture and Functioning

The most relevant aspects of the project and a brief video tutorial on how to launch the simulation can be found here below



https://user-images.githubusercontent.com/61761835/188322232-e940bd45-e460-4505-8004-16a02879690c.mp4

<p align="right">(<a href="#top">back to top</a>)</p>

## ROS nodes description

Here there is the UML components diagram of the project

<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/master/media/component_diagrams/v1/component_diagram.jpg" >

The aforementioned architechture can be seen as a **deliberative** one, being its pipeline structured as *"sense-plan-act"*
More specifically, there are *three types* of sense in this architechture
- **Vision**:      It is implemented by means of Aruco and OpenCV frameworks
- **Localisation**: It is implemented by means of the Odom topic, in Gazebo
- **Mapping**: made possible by laser sensors and GMAPPING algorithm

Moreover:
- Concerning the *"plan"* module, it is implemented through a **Smach** state machine
- Finally, the `move_base` pkg is responsible for the detectibot's movement around the environment
- The mapping is performed using **SLAM GMAPPING** module

Modules:


- **cluedo_kb.py**              <!-- PLEASE INSERT HERE -->

- **navigation.py**           <!-- PLEASE INSERT HERE -->

- **state_machine.py**         <!-- PLEASE INSERT HERE -->

- **simulation.cpp**         <!-- PLEASE INSERT HERE -->

- **img_echo.cpp**             <!-- PLEASE INSERT HERE -->

- **detectibot_magnifier.cpp**  <!-- PLEASE INSERT HERE -->


<p align="right">(<a href="#top">back to top</a>)</p>


### The state_machine.py node

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/master/media/component_diagrams/v1/erl_assignment_3_state_machine_py.jpg" width= 300 height=300>
</p>

It implements a state machine that controls the operations of the robot using **smach_ros**

In particular the machine organises the investigation into four states.
- **move** → moves the robot between rooms inside the simulated indoor environment
- **collect** → the robot rotates on itself to read the largest number of hints within the room
- **check** → takes hints from the sensing system via a service, and uses the ontology to work out whether there are possible solutions or not. If there occurs no possible solutions, the outcome is `mistery_not_solvable`, and the robot transitions back to the "move" state. Otherwise, if there actually occurs possible solutions, the state machine makes a transition to the "show" state, responsible for querying the oracle about the solution's truthfulness
- **show** → questions the oracle about the solution

Here below we can find a hand-made state diagram, representing how the system works
<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/master/media/state_diagrams/state_diagram.jpg" width= 500 height=500>
</p>

Thanks to smach, we can visualize the state of the state machine in which the program is running. Just type:
```sh
rosrun smach_viewer smach_viewer.py
```
> _REMARK:_ Please, remember to import the correct libraries (i.e `import smach, smach_ros`) otherwise some errors may occur!

![state_machine_functioning](https://user-images.githubusercontent.com/61761835/188338883-71fa5cc8-ad81-4621-87c5-35a7ca21c44a.gif)

Node interfaces:
```Plain txt
Node [/state_machine]
Publications:
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]

Services:
 * /state_machine/get_loggers
 * /state_machine/set_logger_level
```
<p align="right">(<a href="#top">back to top</a>)</p>

### The navigation.py node :knot:

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/master/media/component_diagrams/v1/erl_assignment_3_navigation_py.jpg" width= 500 height=500>
</p>

This node implements two different services aimed at letting the robot reach different rooms and to collect hints.
The `/go_to_point` service this time implements MoveBase and waits until the robot hasn't reached the given target.

 The `/turn_robot` service listens for a request containing the angular velocity around x to keep, and the time during which the robot has to turn at a specific angular velocity.

- The **move_base** module combines a global and a local planner and provides a much more robust navigation module than the previously used.
- In addition the navigation service provides a **service** to rotate the robot (erl_assignment_3_msgs/TurnRobot) by a certain angle speed for a certain time; this functionality is necessary for the collection of clues.

Node Interfaces:
```Plain txt
Node [/navigation]
Publications:
 * /cmd_vel [geometry_msgs/Twist]
 * /move_base/cancel [actionlib_msgs/GoalID]
 * /move_base/goal [move_base_msgs/MoveBaseActionGoal]
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]
 * /odom [nav_msgs/Odometry]

Services:
 * /go_to_point
 * /navigation/get_loggers
 * /navigation/set_logger_level
 * /turn_robot
```

### The cluedo_kb.py node

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/master/media/component_diagrams/v1/erl_assignment_3_cluedo_kb_py.jpg" width= 500 height=500>
</p>

This node represents the knowledge base of the robot, allowing for storing and processing of hypotheses, and interrogation of the oracle in order to check if the problem is being resolved.

More specifically, When the robot starts roaming around looking for Aruco markers, (where the hints' IDs are stored), it  makes a service request through `/add_hint` for soliciting the oracle to announce the found hint. This latter, consists in a request of type `erl3/Marker` here below reported

```Plain txt
# erl3/Marker service implementation

int32 markerId
---
# erl3/ErlOracle oracle_hint
ErlOracle oracle_hint
```
Since could happen that sometimes the Oracle sends a wrong hint (i.e. it may occurs that some fields are empty strings and/or some fields has value -1), a function responsible for checking its quality, has been implemented.


Node interfaces:
```Plain txt
Node [/cluedo_kb]
Publications:
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]

Services:
 * /add_hint
 * /cluedo_kb/get_loggers
 * /cluedo_kb/set_logger_level
 * /get_id
 * /mark_wrong_id
```
<p align="right">(<a href="#top">back to top</a>)</p>


### The simulation.cpp node

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/master/media/component_diagrams/v1/erl3_final_oracle_cpp.jpg" width= 500 height=500>
</p>

The architecture is based on the simulation.cpp node which is the same node we were provided by our Professors
Implements two services:
- 1. (/oracle_hint [erl3/Marker]), once it has been provided with a certain ID, it returns the clue corresponding to that ID (Identifier of an index in an array of messages yield by the oracle)
- 2. Concerning the second one (/oracle_solution [erl3/Oracle]), it is needed to check the correctness of a proposed hypothesis at the end of the case

Node interfaces:
```Plain txt
Node [/final_oracle]
Publications:
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]

Services:
 * /final_oracle/get_loggers
 * /final_oracle/set_logger_level
 * /oracle_hint
 * /oracle_solution
```
<p align="right">(<a href="#top">back to top</a>)</p>

### The img_echo.cpp node

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/component_diagrams/v1/erl_assignment_3_img_echo_cpp.jpg" width= 500 height=500>
</p>

Briefly, this node reads the input image from the robot's camera. Secondly, it  print it on a floating window, namely DetectiCAm, by means of a `cv_ptr`. (the `cv_bridge::CvImagePtr cv_ptr` returns a ROS image into an appropriate format compatible with OpenCV).
It then publishes the video stream.

![detecticam_optimised](https://user-images.githubusercontent.com/61761835/188857662-adec563d-f4ec-4517-bd43-c74b1b9261d7.gif)

> Further Remark: ImageTransport's  methods have been employed for creating image publishers and subscribers, being `image_transport` a package that provides transparetn support for transporting images in low-bandwidth compressed formats.

Node interfaces:
```Plain txt
Node [/img_echo]
Publications:
 * /img_echo [sensor_msgs/Image]
 * /img_echo/compressed [sensor_msgs/CompressedImage]
 * /img_echo/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /img_echo/compressed/parameter_updates [dynamic_reconfigure/Config]
 * /img_echo/compressedDepth [sensor_msgs/CompressedImage]
 * /img_echo/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /img_echo/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
 * /img_echo/theora [theora_image_transport/Packet]
 * /img_echo/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /img_echo/theora/parameter_updates [dynamic_reconfigure/Config]
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]
 * /robot/camera1/image_raw [sensor_msgs/Image]

Services:
 * /img_echo/compressed/set_parameters
 * /img_echo/compressedDepth/set_parameters
 * /img_echo/get_loggers
 * /img_echo/set_logger_level
 * /img_echo/theora/set_parameters

 --------------------------------------------------------------------------------
Node [/gazebo]
Publications:
 * /clock [rosgraph_msgs/Clock]
 * /gazebo/link_states [gazebo_msgs/LinkStates]
 * /gazebo/model_states [gazebo_msgs/ModelStates]
 * /gazebo/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /gazebo/parameter_updates [dynamic_reconfigure/Config]
 * /odom [nav_msgs/Odometry]
 * /robot/camera1/camera_info [sensor_msgs/CameraInfo]
 * /robot/camera1/image_raw [sensor_msgs/Image]
 * /robot/camera1/image_raw/compressed [sensor_msgs/CompressedImage]
 * /robot/camera1/image_raw/compressed/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /robot/camera1/image_raw/compressed/parameter_updates [dynamic_reconfigure/Config]
 * /robot/camera1/image_raw/compressedDepth [sensor_msgs/CompressedImage]
 * /robot/camera1/image_raw/compressedDepth/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /robot/camera1/image_raw/compressedDepth/parameter_updates [dynamic_reconfigure/Config]
 * /robot/camera1/image_raw/theora [theora_image_transport/Packet]
 * /robot/camera1/image_raw/theora/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /robot/camera1/image_raw/theora/parameter_updates [dynamic_reconfigure/Config]
 * /robot/camera1/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /robot/camera1/parameter_updates [dynamic_reconfigure/Config]
 * /rosout [rosgraph_msgs/Log]
 * /scan [sensor_msgs/LaserScan]
 * /tf [tf2_msgs/TFMessage]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]
 * /cmd_vel [geometry_msgs/Twist]
 * /gazebo/set_link_state [unknown type]
 * /gazebo/set_model_state [unknown type]

Services:
 * /controller_manager/list_controller_types
 * /controller_manager/list_controllers
 * /controller_manager/load_controller
 * /controller_manager/reload_controller_libraries
 * /controller_manager/switch_controller
 * /controller_manager/unload_controller
 * /gazebo/apply_body_wrench
 * /gazebo/apply_joint_effort
 * /gazebo/clear_body_wrenches
 * /gazebo/clear_joint_forces
 * /gazebo/delete_light
 * /gazebo/delete_model
 * /gazebo/get_joint_properties
 * /gazebo/get_light_properties
 * /gazebo/get_link_properties
 * /gazebo/get_link_state
 * /gazebo/get_loggers
 * /gazebo/get_model_properties
 * /gazebo/get_model_state
 * /gazebo/get_physics_properties
 * /gazebo/get_world_properties
 * /gazebo/pause_physics
 * /gazebo/reset_simulation
 * /gazebo/reset_world
 * /gazebo/set_joint_properties
 * /gazebo/set_light_properties
 * /gazebo/set_link_properties
 * /gazebo/set_link_state
 * /gazebo/set_logger_level
 * /gazebo/set_model_configuration
 * /gazebo/set_model_state
 * /gazebo/set_parameters
 * /gazebo/set_physics_properties
 * /gazebo/spawn_sdf_model
 * /gazebo/spawn_urdf_model
 * /gazebo/unpause_physics
 * /robot/camera1/image_raw/compressed/set_parameters
 * /robot/camera1/image_raw/compressedDepth/set_parameters
 * /robot/camera1/image_raw/theora/set_parameters
 * /robot/camera1/set_camera_info
 * /robot/camera1/set_parameters
```
<p align="right">(<a href="#top">back to top</a>)</p>

### The detectibot_magnifier.cpp node

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/master/media/component_diagrams/v1/erl_assignment_3_detectibot_magnifier.jpg" width= 500 height=500>
</p>

This node is devoted to the detection of ARUCO's markers made through a single camera mounted on the front side of the robot.
It also implements a service that allows for retrieving the IDs identified through Aruco.

For realising such a node, the **vision_openCV** package is needed. This node employs a bridge between OpenCV and ROS. Due to the fact that ROS sends Images in `sensor_msgs/Image` format, our goal is to obtain images in `cv_bridge` format.

Node interfaces:
```Plain txt
Node [/detectibot_magnifier]
Publications:
 * /rosout [rosgraph_msgs/Log]

Subscriptions:
 * /clock [rosgraph_msgs/Clock]
 * /robot/camera1/image_raw [sensor_msgs/Image]

Services:
 * /aruco_markers
 * /detectibot_magnifier/get_loggers
 * /detectibot_magnifier/set_logger_level
```
<p align="right">(<a href="#top">back to top</a>)</p>

### Rqt_graph

<img src= "https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/master/media/rqt/rosgraph_nodes_topics_all.png" />

### UML temporal diagram

By means of this diagram it is possible to show how the system works. As the state_machine gets launched, the robot enters the MOVE state, responsible for the activation of the `/go_to_point` service. Hence, it reaches the center of the room and it starts to collect as many marker as possible.

This has been made possible through the implementation of a `/turn_robot` service that, as the name explicitly suggests, makes detectibot turning around its own position. Only after, the system transitions to the CHECK state, where a request is made by the `/aruco_marker` service to retrieve the detected marker's IDs (by means of a topic subscription). Whenever a new hint gets detected, the knowledge base represented by cluedo_kb node is issued (with a `/oracle_hint` service request).

By means of a further request, made to the final_oracle node through the `/oracle_solution` service, the True ID gets compared and it is chosen whether to terminate the investigation (ending up in a MISTERY_SOLVED state) or pursuing it, transitioning back to the MOVE state

<img src= "https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/master/media/temporal_diagrams/erl_temporal_diagram.jpg" />

<p align="right">(<a href="#top">back to top</a>)</p>

### Summarizing working architecture

The hypothesis IDs are contained within Aruco markers. The True ID instead, is  randomly chosen before starting the game.

Path planning, has been implemented using MoveBase package

The robot is equipped with a single camera, mounted frontally.

It is also equipped with laser range finders which make possible, together with Odometry data, to employ a SLAM gmapping algorithm.
To ensure its employability, the following requirements were met:
- laser outputs are published onto `/scan`  topic
- the robot model is endowed with two frames required for mapping algorithms, which are: `link_chassis` and `odom`

Concerning the Aruco detection, it has been implemented by the **detectibotMagnifier**. Indeed, ROS images messages are sent over `/robot/camera/image_raw` for being converted im a openCV handable format. This is done by means of `cv_bridge` package.

<p align="right">(<a href="#top">back to top</a>)</p>

## How to launch and additional documentation

Additional documentation of this project can be found opening in the browser the file *_build/html/index.html*

### Running the entire project

<!-- including all the steps to display the robot's behavior -->

To test the **project**, first of all:

- Open a shell and run:

```sh
roslaunch erl_assignment_3_robot detectibot_environment_2.launch 2>/dev/null
```

- Open a second shell and run

```sh
roslaunch erl_assignment_3 launch_nodes.launch
```

- Open a third shell and type:

```sh
rosrun erl_assignment_3 state_machine.py
```

### Running the Navigation module


To test the navigation module, first of all:

- Open a shell and run:

```sh
roslaunch erl_assignment_3_robot detectibot_environment_2.launch
```

- Open a second shell and run the navigation node

```sh
rosrun erl_assignment_3 navigation.py
```

### Running the Vision module

To test the vision module, first of all:

- Open a shell and run:

```sh
roslaunch erl_assignment_3_robot detectibot_environment_2.launch
```

- Open a second shell and run

```sh
rosrun erl_assignment_3 img_echo
```

- Open a third shell and type:

```sh
rosrun erl_assignment_3 detectibot_magnifier
```

### Running the State machine module

To test the state machine's module , first of all:

- Open a shell and run:

```sh
roslaunch erl_assignment_3_robot detectibot_environment.launch 2>/dev/null
```

- Open a second shell and run

```sh
rosrun erl_assignment_3 img_echo &
rosrun erl_assignment_3 detectibot_magnifier &
rosrun erl_assignment_3 navigation.py
```

- Open a third shell and type:

```sh
rosrun erl_assignment_3 cluedo_kb.py
```

- Open a fourth shell and run:

```sh
rosrun erl_assignment_3 state_machine.py
```

## System's limitations

Here below, some of the major system limitations are listed:

- Adding cameras will mean to reconfigure **detectibot_magnifier** module. Camera data simulated is costly, it required a powerfull computer.
- A significant numebrs of Aruco Markers are not perceived by the single camera. Modifying the orientation of the camera could be a solution or even better, endowing the manipulator arm with multiple cameras (with different pan and tilt) could work as well.
- There's some issues with the move base evaluating the target: sometimes the robot remains stuck for a while before resuming navigation.
- As already pointed out the simulation is very costly and required the use a powerful machine in order to run rapidly enough.

<p align="right">(<a href="#top">back to top</a>)</p>

## Possible technical Improvements

- The current KB is being chosen because there was problems and limitations in using ARMOR due to the nature of the generation flow of ID, decided a priori. By modification of the ID generation flow and by creating an interface in order to initially filter the data, it is fair to say that the integration with ARMOR would be preferable.

- The navigation system using move_base still has some limitations and seems to fatigue in reaching certain positions and orientations.

- The current robot model is quite unstable. It should be adjusted so that it does not oscillate during its movements. This is due to the simplicity of its mobile base with respect to the shape of its robotic arm. A more robust and stable model should eliminate some of these issues.

- Possible improvements in the manipulation controls used by MoveIt for obtaining a better, more robust, more precise manipulation.

- Possible code and computational improvements in order to further reduce the cost of execution

<p align="right">(<a href="#top">back to top</a>)</p>

## Contact

Iacopo Pietrasanta - s4119821@studenti.unige.it

Project Link: [https://github.com/HolyStone95/ExperimentalRoboticsAssignment3](https://github.com/HolyStone95/ExperimentalRoboticsAssignment3)
