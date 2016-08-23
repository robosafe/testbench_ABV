#!/usr/bin/env python
"""
Implements assertion: if human is within 5cm, robot hand speed is less than 250mm/s
"""
import sys
import rospy
import smach
import smach_ros
import math
import random
import time
from bert2_gazebo.msg import *
from std_msgs.msg import Int8
from bert2_gazebo import ROBOT_LowLevel
import tf
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
import numpy


vel = 0
globaltime=0
monitor_name = 'speed_near_human'

stats = open('assertion_'+monitor_name+'.txt','a')
fileno = 0

class Flag1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
	global receivedflag1
        global stats
        global bert2LL
        global tf_Listener

	rospy.sleep(0.05)
        # Get robot hand location:
        tf_Listener.waitForTransform('/left_wrist_flex_link','/world',rospy.Time(),rospy.Duration(5.0))
        (trans,objQ_vic) = tf_Listener.lookupTransform('/world', '/left_wrist_flex_link', rospy.Time(0))
        rob_hand_loc = numpy.matrix(trans)
        rob_hand_loc = rob_hand_loc-(0.0,0.0,0.5) # correction to tf.
        # Get human hand location::
        hum_hand_loc = bert2LL.get_loc(tf_Listener,'human_hand')
        hum_hand_loc = hum_hand_loc-(0.15,0.0,0.0) # correction to tf.

        print rob_hand_loc
        print hum_hand_loc

        threshVec = (0.1, 0.1, 0.2) # 10cm in any direction, plus 10cm vertical to account for hand widths.
        cond = abs(hum_hand_loc-rob_hand_loc)<=threshVec
        #print cond
        if cond.all() and not (hum_hand_loc[0,2]==0.0):
		return 'outcome1'
        else:
		return 'outcome2'
		

		
class Speed_check1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
        global stats
        global tf_Listener

        
        tStep = 0.1
        tNow = rospy.Time.now()
        tf_Listener.waitForTransform('/left_wrist_flex_link','/world',tNow-rospy.Duration(0.05),rospy.Duration(5.0))
        (trans,objQ_vic) = tf_Listener.lookupTransform('/world', '/left_wrist_flex_link', tNow-rospy.Duration(0.1))
        rob_hand_loc1 = numpy.matrix(trans)
        rospy.sleep(0.1)
        tf_Listener.waitForTransform('/left_wrist_flex_link','/world',tNow+rospy.Duration(tStep-0.05),rospy.Duration(5.0))
        (trans,objQ_vic) = tf_Listener.lookupTransform('/world', '/left_wrist_flex_link', tNow+rospy.Duration(tStep-0.05))
        rob_hand_loc2 = numpy.matrix(trans)
        vel = rob_hand_loc2-rob_hand_loc1
        spd = numpy.linalg.norm(vel)/tStep
        if spd<=0.25:
		stats.write('Assertion_'+monitor_name+' at trace ' + str(fileno) +': True at global time '+ str(time.time()-globaltime) +'\n')
        	return 'outcome1'
        else:
		stats.write('Assertion_'+monitor_name+' at trace ' + str(fileno) +': False at global time '+ str(time.time()-globaltime) +'\n')
		stats.write('speed = '+str(spd)+'\n')
                return 'outcome2'
		
		
def main(number):
	rospy.init_node('assertion_'+monitor_name, anonymous=True) #Start node first
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
		#Receive signal
		smach.StateMachine.add('Flag1', Flag1(), 
                transitions={'outcome1':'Check1','outcome2':'Flag1'})

		#Check
		smach.StateMachine.add('Check1', Speed_check1(), 
                transitions={'outcome1':'Flag1','outcome2':'Done'})


	# Execute SMACH plan
    	outcome = sm.execute()

if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except	rospy.ROSInterruptException:
		pass
