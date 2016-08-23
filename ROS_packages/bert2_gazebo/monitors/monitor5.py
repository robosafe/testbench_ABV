#!/usr/bin/env python
"""
Implements assertion: Eventually, the robot reaches a decision.

Written by Dejanira Araiza-Illan, July 2015
"""

import rospy
import smach
import smach_ros
import sys
import time
from std_msgs.msg import Int8

receivedflag1=0
receivedflag2=0
globaltime=0
stats = open('assertion5.txt','a')
fileno = 0

class Flag1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
	global receivedflag1
	rospy.sleep(0.05)
	if receivedflag1 == 1:
		stats.write('Assertion 5 at trace '+ str(fileno) +': True at global time '+ str(time.time()-globaltime) +'\n')
		return 'outcome1'
	else:
		return 'outcome2'

def callback1(data):
	global receivedflag1
	if data.data==1:
		receivedflag1 = 1
	else:
		receivedflag1 = 0


def main(number):
	rospy.init_node('assertion5', anonymous=True) #Start node first
	global globaltime
	globaltime = time.time()
	global fileno
	fileno = number
	# Create a SMACH state machine
    	sm = smach.StateMachine(outcomes=['Done'])

   	 # Open the container
   	with sm:
		#Receive signal
		smach.StateMachine.add('Flag1', Flag1(), transitions={'outcome1':'Done','outcome2':'Flag1'})

	rospy.Subscriber("done", Int8, callback1)

	# Execute SMACH plan
    	outcome = sm.execute()

if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except	rospy.ROSInterruptException:
		pass
