#!/usr/bin/env python
"""
Implements assertion: gaze is sensed as correct.

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

receivedflag1=0
senseOK = -100
globaltime=0

monitor_name = 'gaze_sensed_as_correct'

stats = open('assertion_'+monitor_name+'.txt','a')
fileno = 0





def callback1(data):
        if data.gaze==1:
                stats.write('Assertion_'+monitor_name+' at trace ' + str(fileno) +': True at global time '+ str(time.time()-globaltime) +'\n')
        else:
                stats.write('Assertion_'+monitor_name+' at trace ' + str(fileno) +': False at global time '+ str(time.time()-globaltime) +'\n')

def main(number):
	rospy.init_node('assertion_'+monitor_name, anonymous=True) #Start node first
	global globaltime
	globaltime = time.time()
	global fileno
	fileno = number

	rospy.Subscriber("gpl_report", Sensors, callback1)

	rospy.spin()

if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except	rospy.ROSInterruptException:
		pass
