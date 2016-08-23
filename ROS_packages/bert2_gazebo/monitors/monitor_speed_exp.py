#!/usr/bin/env python
"""
Implements assertion: robot hand speed is always less than 250mm/s
"""
import sys
import rospy
import smach
import smach_ros
import math
import random
import time
from std_msgs.msg import Int8
from bert2_gazebo import ROBOT_LowLevel
import tf
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
import numpy

receivedflag1=0
globaltime=0
monitor_name = 'speed'

stats = open('assertion_'+monitor_name+'.txt','a')
fileno = 0
		
class Gazebo_check1(smach.State):
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
        print spd
        if spd<=0.25:
		stats.write('Assertion_'+monitor_name+' at trace ' + str(fileno) +': True at global time '+ str(time.time()-globaltime) +'\n')
        	return 'outcome1'
        else:
		stats.write('Assertion_'+monitor_name+' at trace ' + str(fileno) +': False at global time '+ str(time.time()-globaltime) +'\n')
        	return 'outcome2'
		
		
def main(number):

	rospy.init_node('assertion_'+monitor_name, anonymous=True) #Start node first
	global globaltime
	globaltime = time.time()
	global fileno
	fileno = number
        global tf_Listener
        tf_Listener = tf.TransformListener()


	# Create a SMACH state machine
    	sm = smach.StateMachine(outcomes=['Done'])

   	 # Open the container
   	with sm:

		#Check
		smach.StateMachine.add('Check1', Gazebo_check1(), 
                transitions={'outcome1':'Check1','outcome2':'Done'})

        rospy.sleep(0.1)

	# Execute SMACH plan
    	outcome = sm.execute()

if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except	rospy.ROSInterruptException:
		pass
