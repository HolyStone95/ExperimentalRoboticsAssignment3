Code docs -- *State_machine.py*
=================================

:Authors:
    HolyStone95
:Version:
    1.0

About the node 
---------------

It implements a state machine that controls the operations of the robot; it is the core node of the architecture that interacts with and directs all remaining parts

In particular the machine organises the investigation into **four** states.

- **move** → moves the robot between rooms inside the simulated indoor environment
- **collect** → the robot rotates on itself to read the largest number of hints within the room
- **check** → takes hints from the sensing system via a service, and uses the ontology to work out whether there are possible solutions or not. If there occurs no possible solutions, the outcome is mistery_not_solvable, and the robot transitions back to the "move" state. Otherwise, if there actually occurs possible solutions, the state machine makes a transition to the "show" state, responsible for querying the oracle about the solution's truthfulness
- **show** → questions the oracle about the solution



Therefore, as the state_machine gets launched, the robot enters the MOVE state, responsible for the acrivation of the ``/go_to_point`` service. Hence, it reaches the center of the room and it starts to collect as many marker as possible.
This has been made possible through the implementation of a ``/turn_robot`` service that, as the name explicitly suggests, makes detectibot turning around its own position. Only after, the system transitions to the CHECK state, where a request is made by the ``/aruco_marker`` service to retrieve the detected marker's IDs (by means of a topic subscription).
Whenever a new hint gets detected, the knowledge base represented by **cluedo_kb** node is issued (with a ``/oracle_hint`` service request).

By means of a further request, made to the final_oracle node through the ``/oracle_solution`` service, the True ID gets compared and it is chosen whether to terminate the investigation (ending up in a MISTERY_SOLVED state) or pursuing it, transitioning back to the MOVE state

Why Smach?
------------
For the state machine SMACH has been employed  since it was needed a robot capable of achieving a Plan where all possible states and state transitions were described explicitly. 
Some of the most relevant aspects, offered by such a package are:

- **Fast prototyping**: The straightforward Python-based SMACH syntax makes it easy to quickly prototype a state machine and start running it.

- **Complex state machines:** SMACH allows you to design, maintain and debug large, complex hierarchical state machines. You can find an example of a complex hierarchical state machine here.

- **Introspection:** SMACH gives you full introspection in your state machines, state transitions, data flow, etc. See the smach_viewer for more details. 


Code Reference
---------------

.. automodule:: state_machine
	:members:
	:noindex:
