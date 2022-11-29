"""
.. module:: state_machine
	:platform: Unix
	:synopsis: Core Python module aimed at implementong the Smach state machine
.. moduleauthor:: Iacopo Pietrasanta iacopo.pietrasanta@gmail.com

The state_machine node implements a state machine (by means of ROS Smach pkg) that
establishes the operations the robot can actually perform; It also represent
the core module of the overall architechture 
 
In particular the machine organises the investigation into four states.
- **move** → moves the robot between rooms inside the simulated indoor environment 
- **collect** → the robot rotates on itself to read the largest number of hints within the room 
- **check** → takes hints from the sensing system via a service, and uses the ontology to work out whether there are possible solutions or not. If there occurs no possible solutions, the outcome is `mistery_not_solvable`, and the robot transitions back to the "move" state. Otherwise, if there actually occurs possible solutions, the state machine makes a transition to the "show" state, responsible for querying the oracle about the solution's truthfulness
- **show** → questions the oracle about the solution
Subscribes to:
		/clock [rosgraph_msgs/Clock]
	
Publishes to:
		/rosout [rosgraph_msgs/Log]
Service :
		/state_machine/get_loggers
		/state_machine/set_logger_level
"""


import rospy
import smach
import smach_ros
import math
from random import seed
from random import random

from erl3.srv import Oracle, OracleRequest, OracleResponse
from erl3.srv import Marker, MarkerRequest, MarkerResponse
from erl_assignment_3_msgs.srv import GetId, GetIdRequest, GetIdResponse
from erl_assignment_3_msgs.srv import GoToPoint, GoToPointRequest, GoToPointResponse
from erl_assignment_3_msgs.srv import TurnRobot, TurnRobotRequest, TurnRobotResponse
from erl_assignment_3_msgs.srv import GetArucoIds, GetArucoIdsRequest, GetArucoIdsResponse
from erl_assignment_3_msgs.srv import AddHint, AddHintRequest, AddHintResponse
from erl_assignment_3_msgs.srv import MarkWrongId, MarkWrongIdRequest, MarkWrongIdResponse
from geometry_msgs.msg import Point

rooms = [(-4,-3), (-4,2), (-4,7), (5,-7), (5,-3), (5,1)]
''' coordinates of the rooms to visit
'''

room_idx = 0
''' current room to explore
'''

def next_room():
	global rooms
	global room_idx
	
	room_idx = room_idx + 1
	if room_idx >= len(rooms):
		room_idx = 0

winnerID = -1
''' the id at the end of the case
'''

to_show = -1;
''' the id to show to the Oracle in the SHOW status
'''

case_data = {"who":"", "where":"", "what":""}
''' the final solution of the case
'''

cl_go_to_point = None
''' client /go_to_point : erl_assignment_3_msgs/GoToPoint
'''

cl_turn_robot = None
''' client /turn_robot : erl_assignment_3_msgs/TurnRobot
'''

cl_aruco = None
''' client /aruco_markers : erl_assignment_3_msgs/GetArucoIds
'''

cl_add_hint = None
''' client /add_hint : erl_assignment_3_msgs/AddHint
'''

cl_get_hint = None
''' client /get_hint : erl_assignment_3_msgs/GetId
'''

cl_oracle_solution = None
''' client /oracle_solution : erl3/Oracle
'''

cl_delete_hint = None
''' client /mark_wrong_id : erl_assignment_3_msgs/MarkWrongId
'''

max_r = 1.0
''' for the movement target (maximum distance from the target)
'''



class MOVE(smach.State):
	def __init__(self, outcomes=['NEXT']):
		smach.State.__init__(self, outcomes=outcomes)

	def execute(self, userdata):
		''' move to the room, then increment the index.
		
		Note:
			if the robot can't reach the correct result after having visited
			every room, the exploration restart from the first room
		'''
		global rooms
		global room_idx
		global cl_go_to_point
		
		room = rooms[room_idx]
		th = 2.0 * math.pi * random()
		radius = max_r * random()
		x = room[0] + radius * math.cos(th)
		y = room[1] + radius * math.sin(th)
		rospy.loginfo(f"(MOVE) target=({x}, {y}) idx={room_idx}")
		
		# send request and wait
		tg = GoToPointRequest()
		tg.target.x = x
		tg.target.y = y
		tg.target.z = 0.0
		tgres = GoToPointResponse()
		tgres.success = False
		while not tgres.success:
			tgres = cl_go_to_point(tg)
			if not tgres.success:
				rospy.loginfo("(MOVE) movement action failure; retrying")
		
		rospy.loginfo("(MOVE) on the target")
		return 'NEXT'


class COLLECT(smach.State):
	def __init__(self, outcomes=['NEXT']):
		smach.State.__init__(self, outcomes=outcomes)
	
	def execute(self, userdata):
		''' turn and gather the hints
		
		the robot turns areound itself and tries to grasp as many
		markers as the camera can detect. 
		'''
		global cl_turn_robot
		
		rospy.loginfo("(COLLECT) collecting hints")
		cmd = TurnRobotRequest()
		cmd.angularVel = math.pi / 3
		cmd.time = 10
		cl_turn_robot(cmd)
		
		rospy.loginfo("(COLLECT) done")
		return 'NEXT'


class CHECK(smach.State):
	def __init__(self, outcomes=['NEXT', 'IMPOSSIBLE', 'AGAIN']):
		smach.State.__init__(self, outcomes=outcomes)
	
	def execute(self, userdata):
		''' ask if there are valid hints to the knowledge base
		'''
		global cl_aruco
		global cl_add_hint
		global cl_get_hint
		global to_show
		
		# ask for hints to aruco
		aruco_hints = cl_aruco()
		if aruco_hints.isEmpty:
			rospy.loginfo("(CHECK) empty from aruco, retry")
		
		# register the hints in the knowledge base
		if not aruco_hints.isEmpty:
			rospy.loginfo(f"(CHECK) received {len(aruco_hints.ids)} hints from ArUco")
			for idh in aruco_hints.ids:
				if idh < 30:
					rospy.loginfo(f"(CHECK) request for hint index={idh}")
					add_hint_req = AddHintRequest()
					add_hint_req.ID = idh
					cl_add_hint(add_hint_req)
				else:
					rospy.loginfo(f"(CHECK) DISCARD request for hint index={idh}")
		
		# check for available hints
		kb_res = cl_get_hint()
		if kb_res.consistent_found:
			if kb_res.consistent_id >= 0:
				to_show = kb_res.consistent_id
				return 'NEXT'
			else:
				rospy.loginfo("(CHECK) no consistent ID to show, retrying")
				next_room()
				return 'AGAIN'
		else:
			rospy.loginfo("(CHECK) all hypotheses discarded, FAIL")
			return 'IMPOSSIBLE'


class SHOW(smach.State):
	def __init__(self, outcomes=['AGAIN', 'SUCCESS']):
		smach.State.__init__(self, outcomes=outcomes)
	
	def execute(self, userdata):
		''' final checking: ask to the oracle, and try to show a solution.
		'''
		global cl_oracle_solution
		global cl_delete_hint
		global cl_marker
		global to_show
		global case_data
		global room_idx
		global rooms 
		
		sol = cl_oracle_solution()
		rospy.loginfo(f"to_show={to_show} -- oracle={sol.ID}")
		
		if to_show == sol.ID:
			rospy.loginfo("MYSTERY SOLVED")
			return "SUCCESS"
		
		else:
			rospy.loginfo("TRY AGAIN...")
			next_room()
			
			return 'AGAIN'



if __name__ == "__main__":
	rospy.init_node("state_machine")
	
	seed(82749387492)
	
	# client navigation service -- go to point
	rospy.loginfo("cl go_to_point")
	cl_go_to_point = rospy.ServiceProxy("/go_to_point", GoToPoint)
	
	# client navigation service -- turn robot
	rospy.loginfo("cl turn_robot")
	cl_turn_robot = rospy.ServiceProxy("/turn_robot", TurnRobot)
	
	# client aruco markers
	rospy.loginfo("cl aruco markers")
	cl_aruco = rospy.ServiceProxy("/aruco_markers", GetArucoIds)
	
	# client knowledge base -- add hint
	rospy.loginfo("cl add hint")
	cl_add_hint = rospy.ServiceProxy("/add_hint", AddHint)
	
	# client get hint
	rospy.loginfo("cl get id")
	cl_get_hint = rospy.ServiceProxy("/get_id", GetId)
	
	# client oracle -- solution
	rospy.loginfo("cl oracle solution")
	cl_oracle_solution = rospy.ServiceProxy("/oracle_solution", Oracle)
	
	# client kb -- delete hint
	rospy.loginfo("cl delete hint")
	cl_delete_hint = rospy.ServiceProxy("/mark_wrong_id", MarkWrongId)
	
	# client -- oracle marker
	rospy.loginfo("cl oracle marker")
	cl_marker = rospy.ServiceProxy("/oracle_hint", Marker)
	
	
	sm = smach.StateMachine( outcomes = ['mystery_solved', 'mystery_not_solvable'] )
	with sm:
		smach.StateMachine.add( 'MOVE', MOVE(), transitions={'NEXT':'COLLECT'}, remapping={} )
		smach.StateMachine.add( 'COLLECT', COLLECT(), transitions={'NEXT':'CHECK'}, remapping={} )
		smach.StateMachine.add( 'CHECK', CHECK(), transitions={'NEXT':'SHOW', 'IMPOSSIBLE':'mystery_not_solvable', 'AGAIN':'MOVE'}, remapping={} )
		smach.StateMachine.add( 'SHOW', SHOW(), transitions={'AGAIN':'MOVE', 'SUCCESS':'mystery_solved'}, remapping={} )
	
	# starting the introspection server needed for smachviewer 
	sis = smach_ros.IntrospectionServer('sm_server', sm, '/sm_root')
	sis.start()
	
	outcome = sm.execute()
	if outcome=="mystery_solved":
		rospy.loginfo(f"mystery solved! ID={winnerID} with data(who={case_data['who']},where={case_data['where']},what={case_data['what']}")
	else:
		rospy.logerr("mystery not solvable.")
	
	# rospy.spin()
