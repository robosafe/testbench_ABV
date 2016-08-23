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
        #rospy.Subscriber("robot_gripper", Int8, callback1)
	rospy.sleep(0.01)
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
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
	global receivedflag2
	#receivedflag2 = 0
	rospy.sleep(0.01)
	if receivedflag2 == 1:
                # Robot has decided whether to release. Check the object is in hand.
                receivedflag2 = 0
		return 'outcome1'
        else:
		return 'outcome2'
                	
def callback2(data):
	global receivedflag2
        
	# Robot has finished sensing
	receivedflag2 = 1


		
class Gazebo_check1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Done'])

    def execute(self, userdata):
        check_obj_in_robot_hand()
    	rospy.sleep(0.1)
	return 'Done'
		
def check_obj_in_robot_hand():

        # Old method: Gazebo only:
        
        # Get robot hand location:
        state_sp = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)
        state = state_sp('bert2::left_wrist_flex_link','world')
        rob_hand_loc = state.link_state.pose.position
        # Get object location::
        rospy.wait_for_service('gazebo/get_model_state')
        model_state_sp = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
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
        print xCond, yCond, zCond

	print ((obj_loc.x+xOffset)-rob_hand_loc.x)
	print ((obj_loc.y+yOffset)-rob_hand_loc.y)
	print ((obj_loc.z+zOffset)-rob_hand_loc.z)

        global stats
        if xCond and yCond and zCond:
		stats.write('Assertion_no_accidental_drop at trace ' + str(fileno) +': True at global time '+ str(time.time()-globaltime) +'\n')
        else:
                print '\a'
		stats.write('Assertion_no_accidental_drop at trace ' + str(fileno) +': False at global time '+ str(time.time()-globaltime) +'\n')
		#stats.write(str((obj_loc.x+xOffset)-rob_hand_loc.x) +'\n')
		#stats.write(str((obj_loc.y+yOffset)-rob_hand_loc.y) +'\n')
		#stats.write(str((obj_loc.z+zOffset)-rob_hand_loc.z) +'\n')
                #rospy.wait_for_service('gazebo/pause_physics')
                #pause_gazebo = rospy.ServiceProxy('gazebo/pause_physics',Empty)
                #print "Pausing gazebo"
                #pause_gazebo()

		
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
	# Create a SMACH state machine
    	sm = smach.StateMachine(outcomes=['Done'])

   	 # Open the container
   	with sm:
		#Check robot grasps object
		smach.StateMachine.add('Flag1', Flag1(), 
                transitions={'outcome1':'Flag2','outcome2':'Flag1'})

		#Check robot decides to release
		smach.StateMachine.add('Flag2', Flag2(), 
                transitions={'outcome1':'Check1','outcome2':'Flag2'})

		#Check object in robot hand(one-shot, so no transitions specified.)
		smach.StateMachine.add('Check1', Gazebo_check1())

        sub1 = rospy.Subscriber("robot_gripper", Int8, callback1)
	sub2 = rospy.Subscriber("gpl_is_ok", Int8, callback2)

	# Execute SMACH plan
    	outcome = sm.execute()


if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except	rospy.ROSInterruptException:
		pass
