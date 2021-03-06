#!/usr/bin/env python
"""
Implements assertion: if 'robot grasps object', check 'object is in robot's hand', until sensing is finished, then check that the sensors are satisfied, then check that the robot decides to release.
This compound assertion monitor is used to indicate whether or not handover was successful.

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

global receivedflag0
global receivedflag1
global receivedflag2
receivedflag0=0
receivedflag1=0
receivedflag2=0
globaltime=0
stats = open('assertion_handover_success.txt','a')
fileno = 0





class Flag0(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
	global receivedflag0
	rospy.sleep(0.05)
	if receivedflag0 == 1:
                # Robot is trying to grasp the object.  Start monitoring.
                receivedflag1 = 0
		return 'outcome1'
        else:
		return 'outcome2'
                	
def callback0(data):
	global receivedflag0
        
	if data.data == 1:
                # Robot has decided to grasp.
		receivedflag0 = 1
	else:
		receivedflag0 = 0




class Flag1(smach.State):
    # Check object in hand until gaze pressure and location have all been set correctly by the human:

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','Done'])

    def execute(self, userdata):

	global receivedflag1
	#receivedflag1 = 0
        inHand = check_obj_in_robot_hand()
        if not inHand and receivedflag1==0:
                stats.write('Assertion_handover_success at trace ' + str(fileno) +': False at global time '+ str(time.time()-globaltime) +'\n')
                return 'Done'
	rospy.sleep(0.05)
	if receivedflag1 == 1:
                receivedflag2 = 0
	        return 'outcome1'
        else:
		return 'outcome2'

def callback1(data):
	global receivedflag1
	if data.data==1:
		receivedflag1 = 1
	else:
		receivedflag1 = 0
		stats.write('Assertion_handover_success at trace ' + str(fileno) +': False at global time '+ str(time.time()-globaltime) +'\n')



class Flag2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Done','outcome2'])

    def execute(self, userdata):
	global receivedflag2
	#receivedflag2 = 0
	rospy.sleep(0.05)
	if receivedflag2 == 1:
                # Robot has decided whether to release.
                stats.write('Assertion_handover_success at trace ' + str(fileno) +': True at global time '+ str(time.time()-globaltime) +'\n')
		return 'Done'
        elif receivedflag2 == 2:
		stats.write('Assertion_handover_success at trace ' + str(fileno) +': False at global time '+ str(time.time()-globaltime) +'\n')
                return 'Done'
        else:
		return 'outcome2'
                	

def callback2(data):
	global receivedflag2
        
	if data.data == 1:
                # Robot has decided to release.
		receivedflag2 = 1
	else:
		receivedflag2 = 2
		

	
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
	rospy.init_node('assertion_handover_success', anonymous=True) #Start node first
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
		smach.StateMachine.add('Flag0', Flag0(), 
                transitions={'outcome1':'Flag1','outcome2':'Flag0'})

		#Check obj in robot hand and human sets GPL right
		smach.StateMachine.add('Flag1', Flag1(), 
                transitions={'outcome1':'Flag2','outcome2':'Flag1'})

		#Check robot decides to release
		smach.StateMachine.add('Flag2', Flag2(), 
                transitions={'outcome2':'Flag2'})


	rospy.Subscriber("robot_gripper", Int8, callback0)
	rospy.Subscriber("gpl_is_ok", Int8, callback1)
	rospy.Subscriber("decide", Int8, callback2)

	# Execute SMACH plan
    	outcome = sm.execute()

if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except	rospy.ROSInterruptException:
		pass
