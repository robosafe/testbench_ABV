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
import numpy
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
        global tf_Listener
        global bert2LL
        
        # Get robot hand location:
        tf_Listener.waitForTransform('/left_wrist_flex_link','/world',rospy.Time(),rospy.Duration(5.0))
        (trans,objQ_vic) = tf_Listener.lookupTransform('/world', '/left_wrist_flex_link', rospy.Time(0))
        rob_hand_loc = numpy.matrix(trans)
        rob_hand_loc = rob_hand_loc-(0.0,0.0,0.5) # correction to tf.
        # Get object location::
        obj_loc = bert2LL.get_loc(tf_Listener,'object')
        # Compare:
        offsetVec = (0.0, 0.0, 0.1) # The robot grasps the object roughly 0.1 m above the origin of the object's local coordinate system.
        threshVec = (0.2, 0.2, 0.2) # 20cm in any direction.
        cond = abs(obj_loc+offsetVec-rob_hand_loc)<=threshVec
        #print cond
        if cond.all() and not (obj_loc[0,2]==0.0):
                return True
        else:
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


	#rospy.Subscriber("robot_gripper", Int8, callback1)
	rospy.Subscriber("pressure_e1", Int8, callback1)
	rospy.Subscriber("gpl_is_ok", Int8, callback2)
	
	# Execute SMACH plan
    	outcome = sm.execute()


if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except	rospy.ROSInterruptException:
		pass
