#!/usr/bin/env python
"""
This script coordinates sensing for the handover task.  It is compatible with
the real robot (bert2_gazebo_noSim.launch) or simulation (bert2_gazebo.launch).
It is not launched by either of those launch files, so it must be run separately,
in parallel with robot's high-level control script.

Written by David Western, April 2016.
Based on sensors.py by Dejanira Araiza-Illan.
"""

import rospy
from bert2_gazebo.msg import *
from std_msgs.msg import Int8
from bert2_gazebo import ROBOT_LowLevel
import bert2_interface
import tf

#Global variables
gaze_ok = -100 #initial values that indicate the sensors are undefined
pressure_ok = -100
location_ok = -100
hand_pos_initialised = False
init_hand_pos = {}


#--------------------------------------------------------------------------------
def check_gaze(data):
	global gaze_ok
        global bert2LL
        global tf_Listener

        gaze_ok = bert2LL.check_human_gaze(tf_Listener)


#--------------------------------------------------------------------------------
def check_pressure(data):
	global pressure_ok
        global bert2LL
        global rHands
        global hand_pos_initialised
        global init_hand_pos

        # Check hand pressure:
        #   Initialisation, at start of sensing period only:
        if (hand_pos_initialised == False):
                rospy.sleep(1)
                init_hand_pos = rHands.get_currents('left')
                hand_pos_initialised = True
                # print "Initialised hand pos"
                # print init_hand_pos

        #   Current readings:
        hand_pos = rHands.get_currents('left')
        #   Change from initial readings:
        hand_pos_diff = { k:abs(int(hand_pos[k]) - int(init_hand_pos[k])) for k in hand_pos if k in init_hand_pos }
        changed = [(x>8 and x<4000) for x in hand_pos_diff.values()]
        num_changed = sum(changed)
        if (num_changed>0):
                pressure_ok = 1
        else:
                pressure_ok = 0


#--------------------------------------------------------------------------------
def check_location(data):
	global location_ok
        global bert2LL
        global tf_Listener
        
        location_ok = bert2LL.check_human_hand_loc(tf_Listener)
	

#--------------------------------------------------------------------------------
def sense(data):

	global gaze_ok
	global pressure_ok
	global location_ok

        # Instantiate the objects that serve as the low-level control interface:
        global bert2LL
        bert2LL = ROBOT_LowLevel()
        global tf_Listener
        tf_Listener = tf.TransformListener()
        global rHands 
        rHands = bert2_interface.Hands()

        # Set up publisher to transmit binary values to high-level control:
        pub = rospy.Publisher("sensors", Sensors, queue_size=1,latch=True)

        # Sensing loop:
        rate = rospy.Rate(10)
	while not rospy.is_shutdown():

                # Sense (ignore inputs):
                check_gaze(1)
                check_pressure(1)
                check_location(1)
	
                # Publish:
		pub.publish(gaze_ok,location_ok,pressure_ok)
		#print str(gaze_ok)+','+str(pressure_ok)+','+str(location_ok)

                rate.sleep()


#-------------------------------------------------------------------------------
#---------------------------------------------------------------------------
def main():
	rospy.init_node('sensors', anonymous=True)

        # Don't start sensing until robot's ready to hand over.
        # This is necessary because "pressure" is actually sensed as the robot's
        # finger positions relative to those at the start of sensing.  Hence we start
        # after the robot hand's closed: 
        rospy.Subscriber("robot_signals", Robot, sense)

        rospy.spin()

#--------------------------------------------------------------------------------------
if __name__ == '__main__':
	try:
                main()
	except rospy.ROSInterruptException: #to stop the code when pressing Ctr+c
        	pass


