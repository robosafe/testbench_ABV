#!/usr/bin/env python
"""
Implements assertion: if 'robot grasps object' followed by 'robot decides whether to release', then 'object is in robot's hand'.
This is used as a proxy measure of gripper reliability.
This implementation will return no result if the robot does not reach the decision point.  A more rigorous but computationally expensive approach would be to check the object position relative to the hand at every simulation step between 'robot grasps object' and 'robot decides whether to release'.

Written by David Western, Dec 2015
"""
import re
import sys
import rospy
import smach
import smach_ros
import time
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from bert2_gazebo import ROBOT_LowLevel
import tf
from std_msgs.msg import Int8
from std_srvs.srv import Empty

global receivedflag1
global receivedflag2
receivedflag1=0
receivedflag2=0
globaltime=0
stats = open('assertion_no_accidental_drop.txt','a')
fileno = 0

class Flag1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
	global receivedflag1
	rospy.sleep(0.05)
	if receivedflag1 == 1:
                # Robot is trying to grasp the object.  Start monitoring.
                receivedflag2 = 0
		return 'outcome1'
        else:
		return 'outcome2'
                	
def callback1(data):
	global receivedflag1
        
	if data.data == 1:
                # Robot has decided to grasp.
		receivedflag1 = 1
	else:
		receivedflag1 = 0




class Flag2(smach.State):
    # Check gaze pressure and location have all been set correctly by the human:

    def __init__(self):
        smach.State.__init__(self, outcomes=['Done','outcome2'])

    def execute(self, userdata):

	global receivedflag2
        inHand = check_obj_in_robot_hand()
        if not inHand and not receivedflag2:
                stats.write('Assertion_no_accidental_drop at trace ' + str(fileno) +': False at global time '+ str(time.time()-globaltime) +'\n')
                return 'Done'
	rospy.sleep(0.05)
	if receivedflag2 == 1:
		stats.write('Assertion_no_accidental_drop at trace ' + str(fileno) +': True at global time '+ str(time.time()-globaltime) +'\n')
	        return 'Done'
        else:
		return 'outcome2'

def callback2(data):
	global receivedflag2
	receivedflag2 = 1


	
def check_obj_in_robot_hand():
        
        # Get robot hand location:
        state = state_sp('bert2::left_wrist_flex_link','world')
        rob_hand_loc = state.link_state.pose.position
        # Get object location::
        obj_state = model_state_sp("object","world")
        obj_loc = obj_state.pose.position
        # Compare:
        xOffset = 0
        yOffset = 0
        zOffset = 0.1 # The robot grasps the object roughly 0.1 m above the origin of the object's local coordinate system.
        xThresh = 0.2
        yThresh = 0.2
        zThresh = 0.2
	xCond = abs((obj_loc.x+xOffset)-rob_hand_loc.x)<xThresh
	yCond = abs((obj_loc.y+yOffset)-rob_hand_loc.y)<yThresh
	zCond = abs((obj_loc.z+zOffset)-rob_hand_loc.z)<zThresh

        global stats
        if xCond and yCond and zCond:
                return True
        else:
                print '\a'
                print xCond, yCond, zCond

	        print ((obj_loc.x+xOffset)-rob_hand_loc.x)
        	print ((obj_loc.y+yOffset)-rob_hand_loc.y)
        	print ((obj_loc.z+zOffset)-rob_hand_loc.z)
		#stats.write('Assertion_handover_success at trace ' + str(fileno) +': False at global time '+ str(time.time()-globaltime) +'\n')
                return False

		

def main(number):
	rospy.init_node('assertion_no_accidental_drop', anonymous=True) #Start node first
	global globaltime
	globaltime = time.time()
	global fileno
	fileno = number

        global bert2LL
        bert2LL = ROBOT_LowLevel()
        global tf_Listener
        tf_Listener = tf.TransformListener()
        global state_sp
        rospy.wait_for_service('gazebo/get_link_state')
        state_sp = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)
        global model_state_sp
        rospy.wait_for_service('gazebo/get_model_state')
        model_state_sp = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)


	# Create a SMACH state machine
    	sm = smach.StateMachine(outcomes=['Done'])

   	 # Open the container
   	with sm:
		#Check robot grasps object
		smach.StateMachine.add('Flag1', Flag1(), 
                transitions={'outcome1':'Flag2','outcome2':'Flag1'})

		#Check robot decides to release
		smach.StateMachine.add('Flag2', Flag2(), 
                transitions={'outcome2':'Flag2'})


	rospy.Subscriber("robot_gripper", Int8, callback1)
	rospy.Subscriber("gpl_is_ok", Int8, callback2)
	
	# Execute SMACH plan
    	outcome = sm.execute()


if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except	rospy.ROSInterruptException:
		pass
