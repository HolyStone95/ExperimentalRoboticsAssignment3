Code docs -- *final_oracle.cpp*
=================================


Observations
--------------------

- **the node contains up to 30 precomputed hints**, see in the code: ``erl3::ErlOracle oracle_msgs[30];``
	sometimes in the space, the robot could find a ID greater than 30: *it must be discarded*. 
	
- the array ``int markerID[30];`` contains the ID associated to the respective message into ``erl3::ErlOracle oracle_msgs[30];``. For instance, the id of the i-th hint``oracle_msgs[i]`` is ``markerID[i]``
- in particular, ``winID`` is the ID of the solution of the case
- there are two types of ID: there's the one in the message *Marker* which is an index of the array of hints; and there's the ID of the other message *ErlOracle* that is an identifier of a hypothesis of the case. 
- each hypothesis identifier goes from 0 up to 5


How to receive a hint
----------------------

the robot goes around in search of a ArUco code containing the ID of the hint. When it finds one hint, it makes a service request to the Oracle through ``/oracle_hint`` service,  declaring the found hint. This request is of type ``erl3/Marker``, made as follows:

.. code-block::
	
	int32 markerId
	---
	erl3/ErlOracle oracle_hint

the field of type *erl3/ErlOracle* has these fields:

.. code-block::
	
	int32 ID
	string key
	string value


How to check if the hint is valid
----------------------------------

to check whether a specific ID is the solution of the mystery, call the service ``/oracle_solution`` of type ``erl3/Oracle``. Here's the prototype of the service:

.. code-block::
	
	---
	int32 ID


Code Reference
---------------

.. doxygenfile:: simulation.cpp
    :project: ExperimentalRoboticsAssignment3
