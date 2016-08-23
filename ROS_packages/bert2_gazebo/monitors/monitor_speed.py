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
from bert2_gazebo.msg import *
from std_msgs.msg import Int8
from gazebo_msgs.srv import GetModelState, GetLinkState
from gazebo_msgs.msg import ModelState, LinkState
from bert2_gazebo import ROBOT_LowLevel
import tf
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
import numpy as np

receivedflag1=0
checked1=0
checked2=0
t_or_f=0
xx = 0
yy = 0
zz = 0
globaltime=0
monitor_name = 'speed'

stats = open('assertion_'+monitor_name+'.txt','a')
fileno = 0
		
class Gazebo_check1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
        global stats
        global state_sp

        rospy.sleep(0.1)
        state = state_sp('bert2::left_wrist_flex_link','world')
        vel = state.link_state.twist.linear
        spd = ( vel.x**2 + vel.y**2 + vel.z**2 )**0.5
        if spd<=0.25:
		stats.write('Assertion_'+monitor_name+' at trace ' + str(fileno) +': True at global time '+ str(time.time()-globaltime) +'\n')
        	return 'outcome1'
        else:
		stats.write('Assertion_'+monitor_name+' at trace ' + str(fileno) +': False at global time '+ str(time.time()-globaltime) +'\n')
        	return 'outcome2'
		
		
def main(number):
        global state_sp
        state_sp = rospy.ServiceProxy('gazebo/get_link_state', GetLinkState)

	rospy.init_node('assertion_'+monitor_name, anonymous=True) #Start node first
	global globaltime
	globaltime = time.time()
	global fileno
	fileno = number


	# Create a SMACH state machine
    	sm = smach.StateMachine(outcomes=['Done'])

   	 # Open the container
   	with sm:

		#Check
		smach.StateMachine.add('Check1', Gazebo_check1(), 
                transitions={'outcome1':'Check1','outcome2':'Done'})

	# Execute SMACH plan
    	outcome = sm.execute()

if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except	rospy.ROSInterruptException:
		pass
