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
from gazebo_msgs.srv import GetModelState, GetLinkState
from gazebo_msgs.msg import ModelState, LinkState
from bert2_gazebo import ROBOT_LowLevel
import tf
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
import numpy as np


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
        global getLink
        global getModel
        global haPos

	rospy.sleep(0.05)
        human_state = getModel('human_hand','world')
        humanHaPos = human_state.pose.position

        robot_state = getLink('bert2::left_wrist_flex_link','world')
        robotHaPos = robot_state.link_state.pose.position

	if abs(humanHaPos.x-robotHaPos.x)<0.05 and abs(humanHaPos.y-robotHaPos.y)<0.05 and abs(humanHaPos.z-robotHaPos.z)<0.05:
		return 'outcome1'
        else:
                global vel
                vel = robot_state.link_state.twist.linear
		return 'outcome2'
		

		
class Gazebo_check1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
        global stats
        global vel

	
        spd = ( vel.x**2 + vel.y**2 + vel.z**2 )**0.5
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

        global getLink
        getLink = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)
        global getModel
   	getModel = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        hand_state = getModel('human_hand','world')

	# Create a SMACH state machine
    	sm = smach.StateMachine(outcomes=['Done'])

   	 # Open the container
   	with sm:
		#Receive signal
		smach.StateMachine.add('Flag1', Flag1(), 
                transitions={'outcome1':'Check1','outcome2':'Flag1'})

		#Check
		smach.StateMachine.add('Check1', Gazebo_check1(), 
                transitions={'outcome1':'Flag1','outcome2':'Done'})


	# Execute SMACH plan
    	outcome = sm.execute()

if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except	rospy.ROSInterruptException:
		pass
