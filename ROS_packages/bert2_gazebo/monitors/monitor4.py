#!/usr/bin/env python
"""
Implements assertion: if 'flag1==1' then 'flag2==0' checking it has not changed in X time
If GPL is bad, the robot does not release.

Written by Dejanira Araiza-Illan, July 2015
"""

import rospy
import smach
import smach_ros
import math
import random
import time
import sys
from bert2_gazebo.msg import *
from std_msgs.msg import Int8
from geometry_msgs.msg import Point

receivedflag1=0
receivedflag2=0
start = 0
globaltime=0
stats = open('assertion4.txt','a')
fileno = 0

class Flag1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
	global receivedflag1
        global receivedflag2
	rospy.sleep(0.05)
	if receivedflag1 == 1:
                receivedflag2 = 0
		global start
		start=time.time()
		return 'outcome1'
        else:
		return 'outcome2'

def callback1(data):
	global receivedflag1
        print data
	if data.data==0:
		receivedflag1 = 1
	else:
		receivedflag1 = 0
		
class Flag2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
    	global receivedflag1
    	global receivedflag2
	global stats
        rospy.sleep(0.05)
	if receivedflag2 == 1:
		stats.write('Assertion 4 at trace ' + str(fileno) +': False at global time '+ str(time.time()-globaltime) +'\n')
                receivedflag1 = 0
		return 'outcome1'
        else:
        	if (time.time()-start)>5:
			stats.write('Assertion 4 at trace ' + str(fileno) +': True at global time '+ str(time.time()-globaltime) +'\n')
                        receivedflag1 = 0
			return 'outcome1'
		else:
			return 'outcome2'

		
def callback2(data):
	global receivedflag2
	if data.data == 1:
		receivedflag2 = 1
	else:
		receivedflag2 = 0


def main(number):
	rospy.init_node('assertion4', anonymous=True) #Start node first
	global globaltime
	globaltime=time.time()
	global fileno
	fileno = number
	# Create a SMACH state machine
    	sm = smach.StateMachine(outcomes=['Done'])

   	 # Open the container
   	with sm:
		#Receive signal
		smach.StateMachine.add('Flag1', Flag1(), 
                transitions={'outcome1':'Flag2','outcome2':'Flag1'})

		#Receive signal
		smach.StateMachine.add('Flag2', Flag2(), 
                transitions={'outcome1':'Flag1','outcome2':'Flag2'})

	rospy.Subscriber("gpl_is_ok", Int8, callback1)
	rospy.Subscriber("resetpiece", Int8, callback2)

	# Execute SMACH plan
    	outcome = sm.execute()

if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except	rospy.ROSInterruptException:
		pass
