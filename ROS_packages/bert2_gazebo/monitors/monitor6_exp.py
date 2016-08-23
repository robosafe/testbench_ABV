#!/usr/bin/env python
"""
Implements assertion: if robot gripper closes, human hand is not within 5cm.

Adapted by David Western from monitor6.py by Dejanira Araiza-Illan, July 2015
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
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from bert2_gazebo import ROBOT_LowLevel
import tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
import numpy as np

receivedflag1=0
globaltime=0
stats = open('assertion6.txt','a')
fileno = 0

class Flag1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
	global receivedflag1
        global stats
	rospy.sleep(0.05)
	if receivedflag1 == 1:
		return 'outcome1'
        else:
		return 'outcome2'
		
def callback1(data):
	global receivedflag1
	if data.data ==1:
		receivedflag1 = 1
	else:
		receivedflag1 = 0
		
class Gazebo_check1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        global stats
        global bert2LL
        global tf_Listener

        # Hardware version  Not ideal in that it depends on the robot's sensing.
        hLoc = bert2LL.get_loc(tf_Listener,'human_hand')
        hLoc = hLoc-(0.15,0.0,0.0) # Correction to tf
        oLoc = bert2LL.get_loc(tf_Listener,'object')
        threshVec = (0.05,0.05,0.05) # 5cm in any direction.
        cond = abs(hLoc-oLoc)>=threshVec
        if cond.all() and not (hLoc[0,2]==0.0 and oLoc[0,2]==0.0):
		stats.write('Assertion 6 at trace ' + str(fileno) +': True at global time '+ str(time.time()-globaltime) +'\n')

        else:
		stats.write('Assertion 6 at trace ' + str(fileno) +': False at global time '+ str(time.time()-globaltime) +'\n')
		stats.write(str(hLoc[0,0])+' '+str(hLoc[0,1])+' '+str(hLoc[0,2])+'\n')
		stats.write(str(hLoc[0,0])+' '+str(hLoc[0,1])+' '+str(hLoc[0,2])+'\n')
    	rospy.sleep(0.1)
	receivedflag1 = 0
	return 'outcome1'
		
		
def main(number):
	rospy.init_node('assertion6', anonymous=True) #Start node first
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
		smach.StateMachine.add('Check1', Gazebo_check1(), 
                transitions={'outcome1':'Flag1'})

	rospy.Subscriber("robot_gripper", Int8, callback1)

	# Execute SMACH plan
    	outcome = sm.execute()

if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except	rospy.ROSInterruptException:
		pass
