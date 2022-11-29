<div id="top"></div>




<!-- PROJECT SHIELDS -->
<!--
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]





<!-- PROJECT LOGO -->
<br />
<div align="center">
<a href="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3">
	<img src="media/miscellaneous/logo-black.png" alt="Logo" width="200" height="100">
</a>

<h3 align="center">Experimental Robotics Laboratory</h3>

<p align="center">
	First assignment for the Experimental Robotics laboratory course 
	<br />
	<a href="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/doc"><strong>Explore the docs »</strong></a>
	<br />
	<br />
	<a href="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/demo">View Demo</a>
	·
	<a href="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/issues">Report Bug</a>
	·
	<a href="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/issues">Request Feature</a>
</p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
	<summary>Table of Contents</summary>
	<ol>
		<li>
			<a href="#about-the-project">About The Project</a>
			<ul>
				<li><a href="#assignments-prerequisites">Assignment's prerequisites</a></li>
				<li><a href="#built-with">Built With</a></li>
			</ul>
		</li>
		<li>
			<a href="#getting-started">Getting Started</a>
			<ul>
				<li><a href="#installation-procedure">Installation procedure</a></li>
				<li><a href="#workspace-building-and-launch">Workspace building and launch</a></li>
				<li><a href="#running-pocedure">Running procedure</a></li>
			</ul>
		</li>
		<li>
			<a href="#usage">Usage</a>
		</li>
		<li>
			<a href="#ros-node-description-an-overview">ROS node description: An overview</a>
			<ul>
				<li><a href="#the-main.py-node ">the main.py node</a></li>
				<li><a href="#the-cluedo_kb.py-node ">the cluedo_kb.py node</a></li>
				<li><a href="#the-action_interface.cpp-node">the action_interface.cpp node</a></li>
				<li><a href="#the-manipulation.cpp-node ">the manipulation.cpp node</a></li>
				<li><a href="#the-my_simulation.cpp-node ">the my_simulation.cpp node </a></li>
				<li><a href="#rqt_graph">rqt_graph</a></li>
			</ul>
		<li>
				<a href="#Working-hypothesis-and-environment">Working hypothesis and environment</a>
		</li>
			<ul>
				<li><a href="#System's features">System's features</a></li>
				<li><a href="#System's limitations">System's limitations</a></li>
				<li><a href="#Possible technical Improvements">Possible technical Improvements</a></li>
			</ul>
		<li>
			<a href="#roadmap">Roadmap</a>
		</li>
		<li>
			<a href="#contributing">Contributing</a>
		</li>
		<li>
			<a href="#license">License</a>
		</li>
		<li>
			<a href="#contact">Contact</a>
		</li>
		<li>
			<a href="#acknowledgments">Acknowledgments</a>
		</li>
	</ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

This project takes inspiration from the earlier ones ([ExperimentalRoboticsAssignment1][9] and [ExperimentalRoboticsAssignment2][10], respectively) but unlike them, the environment in which detectiBot moves is much more complex. Indeed, it presents several rooms, and 30 ArUco markers (5 markers for each room)

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
 
Having differen values for z, it is needed that detectibot reaches both quotes with its cluedo_link

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

> *REMARK* An hypothesis, is defined as *inconsistent* when there occurs more than one role for each class (i.e. 2 or more occourences of who, where, what) 

A clear example of such hypothesis is ID4  whose definition is here below reported

```txt
ID4_1: ['where', 'Library']
ID4_2: ['who', 'Mrs.White']
ID4_3: ['what', 'LeadPipe']
ID4_4: ['where', 'Diningroom']
```
### Assignment's prerequisites

In this assignment:
-  Here using ROSPlan is **not mandatory**
- the robot of the model has no limitations, meaning that it can be modelled in whatever fashion 
- the usage of the ArUco libraries is mandatory for the detection of the markers. 
- using functionalities such as mapping, path planning and following may greatly help in performing
the assignment
- use, as starting point the [following package][8], provided by our Professors!

What does the [starting package][8] contain:
-  a node, which implements the oracle. Concerning the implementation of the Oracle consider that:
		- there are in total 6 IDs [0...5];
		- ome of these IDs (randomly chosen) may generate inconsistent hypotheses (e.g. multiple persons,
rooms, objects)
		-  the «trustable» ID is also randomly chosen (among the IDs which do not generate inconsisten
		-  the oracle node implements two services: the first one (/oracle_hint) recevies an Int32 as request (the id of
the marker) and returns the hint as a erl2/ErlOracle message;
		- The oracle node implements also a service (/oracle_solution) which returns the trustable ID (erl2/Oracle.h,
with an empty message for the request, and a int32 for the reply).
- similarly to the second assignment, some markers correspond to malformed hints (e.g, all fields are
empty, or just one field is empty, ....)


<p align="right">(<a href="#top">back to top</a>)</p>



### Built With 🏗️

<!-- PLEASE INSERT HERE -->

* [ROS][4]
* [smach][1]
* [OpenCV][115]
* [MoveIt Frameowrk][6]
* [move_base][7]


<p align="right">(<a href="#top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

Under the following sections, the software architecture is briefly introduced, along with the prerequisites and installation procedures. Then, a quick video demonstration showing the overall functioning is provided and system’s limitations are discussed

### Installation procedure

> :warning: To avoid further issues, please use this docker image provided by our professors 

```sh
docker run -it -p 6080:80 -p 5900:5900 --name MyDockerContainer carms84/exproblab

```
Also remember to update and upgrade the container

```sh
sudo apt get update
sudo apt get upgrade
```
Then run `catkin_make` on your workspace; in my case:

- Navigate to your ROS workspace
	```sh
	cd /home/ros_ws/
	```
- Run catkin
	```sh
	catkin_make
	```

You can now download the repository inside the `src` folder 

```sh 
cd /home/ros_ws/src
git clone https://github.com/HolyStone95/ExperimentalRoboticsAssignment3
```
Also download `MoveIt 1.1.5` (If you haven't already done so)
```sh
git clone https://github.com/ros-planning/moveit.git
cd moveit
git checkout 2b881e5e3c4fd900d4d4310f4b12f9c4a63eb5dd
cd ..
git clone https://github.com/ros-planning/moveit_resources.git
cd moveit_resources
git checkout f6a7d161e224b9909afaaf621822daddf61b6f52
cd ..
git clone https://github.com/ros-planning/srdfdom.git
cd srdfdom
git checkout b1d67a14e45133928f9793e9ee143998219760fd
cd ..
apt-get install -y ros-noetic-rosparam-shortcuts
cd ..
catkin_make
catkin_make
catkin_make
```
Then navigate through the directory, in order to find the marker models 

```sh
cd /ros_ws/src/ExperimentalRoboticsAssignment3/erl3/models 
```
copy all files inside the `erl3` models folder and navigate to the `root/.gazebo` directory

```sh
cd /root/.gazebo/models
```
paste the previously copied files, containing all the marker's models, as showm in the animated gif, here below

![optimized_install_models](https://user-images.githubusercontent.com/61761835/188335313-a364d084-6098-49a8-8f91-d8eda227416b.gif)


## Workspace building e launch

Navigate to you workspace

```sh
cd /home/ros_ws/

```

- clone the repository

```sh
https://github.com/HolyStone95/ExperimentalRoboticsAssignment3

```

- source your workspace by typing

```sh
source devel/setup.bash

```

<p align="right">(<a href="#top">back to top</a>)</p>

### Running procedure

#### Running the entire project

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

#### Running the Navigation module


To test the navigation module, first of all:

- Open a shell and run:
	
```sh
roslaunch erl_assignment_3_robot detectibot_environment_2.launch
```

- Open a second shell and run the navigation node 

```sh
rosrun erl_assignment_3 navigation.py
```


#### Running the Vision module


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

#### Running the State machine  module

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

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage

The most relevant aspects of the project and a brief video tutorial on how to launch the simulation can be found here below



https://user-images.githubusercontent.com/61761835/188322232-e940bd45-e460-4505-8004-16a02879690c.mp4




<!-- A commented small video, a GIF or screenshots showing the relevant parts of the running code. -->





<p align="right">(<a href="#top">back to top</a>)</p>

## ROS node description: An overview 

Here there is the UML components diagram of the project

<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/component_diagrams/v1/component_diagram.jpg" >

The aforementioned architechture can be seen as a **Deliberative** one, being its pipeline structured as "sense-plan-act"
- Concerning the "sense" module, there are three types of sense in this architechture 
		1. **Vision** -       it is implemented by means of Aruco and OpenCV frameworks
		2. **Localisation** - It is implemented by means of the Odom topic, in Gazebo
		3. **Mapping** - made possible by laser sensors and GMAPPING algorithm
- Concerning the "plan" module, it is implemented through a [smach][1] state machine 
- Finally, the move_base pkg is responsible for the detectibot's movement around the environment  

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/miscellaneous/deliberative_sketch_arch.png" width= 500 height=100>
</p>

As shown in the above component diagram, this software architechture relies on the synergy of varius modules: 



- [cluedo_kb.py][20]              <!-- PLEASE INSERT HERE -->

- [navigation.py][21]             <!-- PLEASE INSERT HERE -->

- [state_machine.py][22]          <!-- PLEASE INSERT HERE -->

- [final_oracle.cpp][23]          <!-- PLEASE INSERT HERE -->

- [img_echo.cpp][24]              <!-- PLEASE INSERT HERE -->

- [detectibot_magnifier.cpp][25]  <!-- PLEASE INSERT HERE -->


<p align="right">(<a href="#top">back to top</a>)</p>


### the state_machine.py node :knot:

Let's start with the `state_machine.py` node


<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/component_diagrams/v1/erl_assignment_3_state_machine_py.jpg" width= 300 height=300>
</p>
 
It implements a state machine that controls the operations of the robot; it is the core node of the architecture that interacts with and directs all remaining parts 

In particular the machine organises the investigation into four states.
- **move** → moves the robot between rooms inside the simulated indoor environment 
- **collect** → the robot rotates on itself to read the largest number of hints within the room 
- **check** → takes hints from the sensing system via a service, and uses the ontology to work out whether there are possible solutions or not. If there occurs no possible solutions, the outcome is `mistery_not_solvable`, and the robot transitions back to the "move" state. Otherwise, if there actually occurs possible solutions, the state machine makes a transition to the "show" state, responsible for querying the oracle about the solution's truthfulness
- **show** → questions the oracle about the solution

Here below we can find a hand-made state diagram, representing how the system works 
<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/state_diagrams/state_diagram.jpg" width= 500 height=500>
</p>

Moreover, an **introspection Server** has been implemented in order to visualize the possible transitions between states, as well as the currently active state and the values of user data that is passed around 
For visualising it, just type:
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

### the navigation.py node :knot:

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/component_diagrams/v1/erl_assignment_3_navigation_py.jpg" width= 500 height=500>
</p>

This node implements two different services aimed at letting the robot reach ifferent rooms in order to fulfill its investigation-related tasks.
The `/go_to_point` service calls MoveBase and waits until the robot hasn't reached the given target whereas the `/turn_robot` service listens for a request containing the angular velocity around x to keep, and the time during which the robot has to turn at "that" specific angular velocity. Moreover:

- Localisation takes place through the subscription to the **odom** (nav_msgs/Odom) topic 
- The node uses **move_base** (from move_base pkg) to perform the navigation. The main function of this package is to move a robot from its current position to a goal position with the help of other navigation nodes. The move_base node inside this package links the global-planner and the local-planner for the path planning, connecting to the rotate-recovery package if the robot is stuck in some obstacle and connecting global costmap and local costmap for getting the map. The move_base node is basically an implementation of SimpleActionServer, which takes a goal pose with message type (geometry_msgs/PoseStamped). We can send a goal position to this node using a SimpleActionClient node.
- In addition the navigation service provides a **service** to rotate the robot (erl_assignment_3_msgs/TurnRobot) by a certain angle speed for a certain time; this functionality is aimed at the collection of clues!

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
<p align="right">(<a href="#top">back to top</a>)</p>

### the cluedo_kb.py node :knot:

Concerning the `cluedo_kb.py` node:

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/component_diagrams/v1/erl_assignment_3_cluedo_kb_py.jpg" width= 500 height=500>
</p>

cluedo_KB is a node that serves as a specialised ontology for the problem in hand; it supplies a processing/reasoning system that provides the functionality of:  

- Registering clues 
- Building and processing hypotheses based on the added information 
- Finding possible solutions to the case 
- Rejecting hypotheses (whether needed)

More specifically, When the robot starts roaming around looking for Aruco markers, (where the hints' IDs are stored), it  makes a service request through `/add_hint` for soliciting the oracle to announce the found hint. This latter, consists in a request of type `erl3/Marker` here below reported

```Plain txt
# erl3/Marker service implementation

int32 markerId
---
# erl3/ErlOracle oracle_hint
ErlOracle oracle_hint
```
Since could happen that sometimes the Oracle sends a wrong hint (i.e. it may occurs that some fields are empty strings and/or some fields has value -1), a function responsible for checking its quality, has been implemented. 

> *Remark* In the previous section we mentioned the difference between consistent and inconsistent hypothesis; It is worthmentioning that this nodes also implements a function  to cope with the removal of inconsistent hypothesis from the list of possible hints




> ***REMARK*** the KB listens to the oracle's topic and as soon as the oracle transmits the clue, the KB adds the message to the ontology without the need for an explicit request

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


### the simulation.cpp node (final_oracle) :knot:

Concerning the `simulation.cpp` node:

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/component_diagrams/v1/erl3_final_oracle_cpp" width= 500 height=500>
</p>

The architecture is based on the simulation.cpp node which is the same node we were provided by our Professors
This latter supplies two services:
- Concerning the first one (/oracle_hint [erl3/Marker]), once it has been provided with a certain ID, it returns the clue corresponding to that ID (Identifier of an index in an array of messages yield by the oracle)
- Concerning the second one (/oracle_solution [erl3/Oracle]), it is needed to check the correctness of a proposed hypothesis at the end of the case 

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

### the img_echo.cpp node :knot:

Concerning the `img_echo.cpp` node :

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/component_diagrams/v1/erl_assignment_3_img_echo_cpp.jpg" width= 500 height=500>
</p>

Briefly, this node reads the input image from the robot's camera. Secondly, it  print it on a floating window, namely DetectiCAm, by means of a `cv_ptr`. (the `cv_bridge::CvImagePtr cv_ptr` returns a ROS image into an appropriate format compatible with OpenCV). Thirdly it publish the video stream!

![detecticam_optimised](https://user-images.githubusercontent.com/61761835/188857662-adec563d-f4ec-4517-bd43-c74b1b9261d7.gif)

> Remark: Since we have to deal with the image, multiple copies of it will be needed; For this purpose the BGR8 image encoding  has been chosen, being it less susceptible against typos.
> Further Remark: ImageTransport's  methods have been employed for creating image publishers and subscribers, being `image_transport` a package that provides transparetn support for transporting images in low-bandwidth compressed formats.
> *Further Further Remark*: Please remember to include `cv_bridge` in your `xml` package! Also do not forget to add the following headers to your cpp file

```cpp
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp>
```

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

### The detectibot_magnifier.cpp node :knot:

<p align="center">
<img src="https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/component_diagrams/v1/erl_assignment_3_detectibot_magnifier.jpg" width= 500 height=500>
</p>

This node is devoted to the detection of ARUCO's markers made through a single camera mounted on the front side of the robot. 
It also implements a service that allows for retrieving the IDs identified through Aruco.

For realising such a node, the [vision_openCV][117] packages, aimed at interfacing ROS with OpenCV have been emploied. OpenCV basically consists in a library of programming functions for real time computer vision. Hence this node employs a bridge between OpenCV and ROS. Due to the fact that ROS sends Images in `sensor_msgs/Image` format, our goal is to obtain images in `cv_bridge` format.

> *REMARK* Please note that By using `image_transport::Publisher image_pub_ `and subscribing to the topic `/robot/camera1/image_raw` we are able to decrease the bandwidth!

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

### rqt_graph

In the figure below, circles represent nodes and squares represent topic messages. The arrow instead, indicates the transmission of the message!

<img src= "https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/rqt/rosgraph_nodes_topics_all.png" />

### UML temporal diagram

By means of this diagram it is possible to show how the system works. As the state_machine gets launched, the robot enters the MOVE state, responsible for the acrivation of the `/go_to_point` service. Hence, it reaches the center of the room and it starts to collect as many marker as possible. 

This has been made possible through the implementation of a `/turn_robot` service that, as the name explicitly suggests, makes detectibot turning around its own position. Only after, the system transitions to the CHECK state, where a request is made by the `/aruco_marker` service to retrieve the detected marker's IDs (by means of a topic subscription). Whenever a new hint gets detected, the knowledge base represented by cluedo_kb node is issued (with a `/oracle_hint` service request). 

By means of a further request, made to the final_oracle node through the `/oracle_solution` service, the True ID gets compared and it is chosen whether to terminate the investigation (ending up in a MISTERY_SOLVED state) or pursuing it, transitioning back to the MOVE state

<img src= "https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/temporal_diagrams/erl_temporal_diagram.jpg" />



<p align="right">(<a href="#top">back to top</a>)</p>

## Working hypothesis and environment 

This architecture is designed for providing a reinterpretation of the Cluedo Game. Markers are set a-priori both on the ground and on wall-fixed boxes. The hypothesis IDs are contained within Aruco markers. The True ID instead, is  randomly chosen before starting the game. 

Detectibot (the robot involved in the investigation), moves in a obstacle-free environment charachterised by a perfectly flat floor (without irregularities), within a indoor environment, provided with rooms without furnitures. Path planning, has been implemented by menas of the MoveBase package It has been designed to mount a single camera, pointing toward the front side. 

It is also equipped with laser range finders which make possible, together with Odometry data, to employ a SLAM gmapping algorithm. This approach uses a particle filter in which each particle carries an individual map of the environment. To ensure its employability, the following requirements were met:
- laser outputs are published onto `/scan`  topic
- the robot model is endowed with two frames required for mapping algorithms, which are: `link_chassis` and `odom`

Concerning the Aruco detection, it has been implemented by the [detectibotMagnifier][25]. This reads images on a specific topic (`topic_info`)

All choices were made with the aim of making the system as modular and flexible as possible. Despite this, certain limitations make the system quite unrealistic but functional.

<p align="right">(<a href="#top">back to top</a>)</p>

### System's features

Most of them have been already discussed in the Software architecture’s section. 

The project implements the robot behaviour so that it can keep roaming around, looking for Aruco Markers inside the environmeent. This serves for solving the case. 

Indeed, while it navigates through the environment it tries to combine them in order to find a solution. This is where the reasoning & AI module, represented by the [cluedo_kb.py][20], comes imto play

Concerning the architecture, it is centralised and designed in such a way that individual components can be replaced as long as they meet the same required interface 

<p align="right">(<a href="#top">back to top</a>)</p>

### System's limitations

Here below, some of the major system limitations are listed:

- If the robot had more than one camera, the detection system (detectibot_magnifirer) would have to be re-implemented to ensure a certain performance from the system 

<p align="right">(<a href="#top">back to top</a>)</p>

### Possible technical Improvements

As for the system limitations, some of the most relevant potential techincal improvements:

- The current KB can be modified to implement the same functionalities on a different ontology system (i.e. Armor); the component can be extended for more accurate  hypotheses processing or for providing, for instance, a ontology backup feature
	
- The current navigation system is rather poor; it should be replaced with a more elaborate navigation system. In particular, the new navigation system should make it possible to achieve a certain orientation as well as a final position.
	
- The manipulation could be replaced with a more advanced node that performs a finer (more precise) control on moveit
	
- The current robot model is quite unstable. It should be adjusted so that it does not oscillate during its movments
	
- the robot needs a lot of manoeuvring space to move; There should be the need of seeking an appropriate navigation algorithm to reduce the necessary manoeuvring space

- The architecture could also be executed in a distributed manner by splitting the components over several devices. However, this possibility was not considered during the design of the system. It is therefore necessary to identify possible criticalities in the communication protocol (i.e. to better manage service calls that fail based on the quality of the connection) and deal with them appropriately 

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- ROADMAP -->
## Roadmap

- [x] Complete the introduction of the template 
- [x] Describe the software architechture
	- [x] Component diagram (*not mandatory*)
	- [ ] Temporal diagram + comments
	- [x] States diagrams, whether applicable + comments
	- [x] Create a list describing ROS messages and parameters 
- [x] Describe the installation steps and the running procedures
		- [x] Create a dedicated paragraph
		- [x] Include all the steps to display the robot's behaviour
- [x] Show in the "usage" section the running code
	- [x] Create a small video tutorial of the launch
	- [x] Create a small animated gif of the terminal while running code
- [x] Describe the Working hypothesis and environment
	- [x] Dedicated section for System's features
	- [x] Dedicated section for System's limitations
	- [x] Dedicated section for Possible technical improvements

	

See the [open issues](https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement"

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under none License. 

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- CONTACT -->
## Contact

Iacopo Pietrasanta - s4119821@studenti.unige.it

Project Link: [https://github.com/HolyStone95/ExperimentalRoboticsAssignment2](https://github.com/HolyStone95/ExperimentalRoboticsAssignment2)

<p align="right">(<a href="#top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->
## List of resources

* [Viewing state machine](http://wiki.ros.org/smach/Tutorials/Smach%20Viewer)
* [Smach](http://wiki.ros.org/smach)
* [Armor](https://github.com/EmaroLab/armor)
* [Protègè](https://protege.stanford.edu)




<p align="right">(<a href="#top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->


[forks-shield]: 	https://img.shields.io/github/forks/HolyStone95/ExperimentalRoboticsAssignment3
[forks-url]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/network/members
[stars-shield]: 	https://img.shields.io/github/stars/HolyStone95/ExperimentalRoboticsAssignment3
[stars-url]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/stargazers
[issues-shield]: 	https://img.shields.io/github/issues/HolyStone95/ExperimentalRoboticsAssignment3
[issues-url]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/issues
[license-shield]: https://img.shields.io/github/license/HolyStone95/ExperimentalRoboticsAssignment3

<!-- general resources -->
[1]:  http://wiki.ros.org/smach
[3]:  http://wiki.ros.org/smach/Tutorials/Smach%20Viewer
[4]:  http://wiki.ros.org
[5]:  https://github.com/KCL-Planning/ROSPlan
[6]:  https://moveit.ros.org/
[7]:  http://wiki.ros.org/move_base
[8]:  https://github.com/CarmineD8/exp_assignment3
[9]:  https://github.com/HolyStone95/ExperimentalRoboticsAssignment1
[10]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment2
[11]: < insert here >
[12]: < insert here >
[13]: < insert here >
[14]: < insert here >
[15]: < insert here >
[16]: https://ontogenesis.knowledgeblog.org/1260/
[17]: http://emarolab.github.io/armor_py_api/armor_utils_client.html
[18]: https://docs.python.org/3/library/queue.html
[19]: https://protege.stanford.edu

<!-- Nodes -->
[20]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl_assignment_3/scripts/cluedo_kb.py
[21]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl_assignment_3/scripts/navigation.py
[22]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl_assignment_3/scripts/state_machine.py
[23]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl3/src/simulation.cpp
[24]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl_assignment_3/src/img_echo.cpp
[25]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl_assignment_3/src/detectibot_magnifier.cpp



<!-- Launchers -->
[28]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl_assignment_3/launch/launch_nodes.launch
[29]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl_assignment_3_robot/launch/detectibot_environment_2.launch
[30]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl_assignment_3_robot/launch/detectibot_environment.launch


<!-- srvs -->
[32]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl_assignment_3_msgs/srv/GetId.srv
[33]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl_assignment_3_msgs/srv/MarkWrongId.srv
[34]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl_assignment_3_msgs/srv/AddHint.srv
[35]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl_assignment_3_msgs/srv/GetArucoIds.srv
[36]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl_assignment_3_msgs/srv/GoToPoint.srv
[37]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl_assignment_3_msgs/srv/TurnRobot.srv
[38]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl3/srv/Marker.srv
[39]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/erl3/srv/Oracle.srv



<!-- diagrams  -->
[100]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/component_diagrams/v1/component_diagram.jpg
[101]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/component_diagrams/v1/erl_assignment_3_action_interface_cpp.jpg
[102]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/component_diagrams/v1/erl3_my_simulation_cpp.jpg
[103]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/component_diagrams/v1/erl_assignment_3_cluedo_kb_py.jpg
[104]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/component_diagrams/v1/erl_assignment_3_main_py.jpg
[105]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/component_diagrams/v1/erl_assignment_3_manipulation_cpp.jpg
[106]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/component_diagrams/v1/erl_assignment_3_test_nav_py.jpg
[107]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/component_diagrams/v1/erl_assignment_go_to_point_py.jpg


<!-- Temporal diagram -->
[116]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/blob/main/media/temporal_diagrams/erl_temporal_diagram.jpg

<!-- rqt graphs -->
[108]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/tree/main/media/rqt/rosgraph_nodes_only.png
[109]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/tree/main/media/rqt/rosgraph_nodes_topics_active.png
[110]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/tree/main/media/rqt/rosgraph_nodes_topics_all.png

<!-- temporal diagram -->
[111]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/tree/main/media/temporal_diagrams/Diagrams_erl_img.jpg


<!-- MoveIt -->
[112]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/tree/main/erl_moveit_pkg/launch
[113]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment3/tree/main/erl_moveit_pkg/config

<!-- Previous project links -->
[114]: https://github.com/HolyStone95/ExperimentalRoboticsAssignment2

<!-- OpemCV -- Aruco -->
[115]: https://www.github.com/opencv/opencv/wiki
[117]: https://github.com/ros-perception/vision_opencv
