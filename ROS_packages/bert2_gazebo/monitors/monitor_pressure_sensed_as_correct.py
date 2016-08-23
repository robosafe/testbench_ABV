#!/usr/bin/env python
"""
Implements assertion: pressure is sensed as correct.

Written by David Western, May 2016
"""
import re
import sys
import rospy
import smach
import smach_ros
import time
from std_msgs.msg import Int8
from std_srvs.srv import Empty
from bert2_gazebo.msg import *

global receivedflag1
global senseOK
receivedflag1=0
senseOK = -100
globaltime=0

monitor_name = 'pressure_sensed_as_correct'

stats = open('assertion_'+monitor_name+'.txt','a')
fileno = 0






class Flag1(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
	global receivedflag1
        global senseOK

	rospy.sleep(0.01)

        if receivedflag1 == 1:
                if senseOK == 1:
        		stats.write('Assertion_'+monitor_name+' at trace ' + str(fileno) +': True at global time '+ str(time.time()-globaltime) +'\n')
                        return 'outcome1'
                else:
        		stats.write('Assertion_'+monitor_name+' at trace ' + str(fileno) +': False at global time '+ str(time.time()-globaltime) +'\n')
                        return 'outcome1'
        else:
                return 'outcome2'

def callback1(data):
        global receivedflag1

	receivedflag1 = 1


def callback2(data):
	global senseOK
                
        if data.pressure==1:
		senseOK = 1
        else:
                senseOK = 0

def main(number):
	rospy.init_node('assertion_'+monitor_name, anonymous=True) #Start node first
	global globaltime
	globaltime = time.time()
	global fileno
	fileno = number


	# Create a SMACH state machine
    	sm = smach.StateMachine(outcomes=['Done'])

   	 # Open the container
   	with sm:

		#Wait for decision point
		smach.StateMachine.add('Flag1', Flag1(), 
                transitions={'outcome1':'Done','outcome2':'Flag1'})

	rospy.Subscriber("decide", Int8, callback1)
	rospy.Subscriber("sensors", Sensors, callback2)

	# Execute SMACH plan
    	outcome = sm.execute()

if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except	rospy.ROSInterruptException:
		pass
