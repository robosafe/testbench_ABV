#!/usr/bin/env python
"""
This script simulates the events of pressure, in the current sensors located on BERT2's hand. When the piece is grabbed, code triggers a presure change event. When the human holds the piece simultaneously, another event is triggered, expecting another pressure change from the current readings. 

Written by Dejanira Araiza Illan, July 2015. 

Expanded by David Western to work with the actual robot, Sept 2015.
"""

import rospy
from std_msgs.msg import Int8
import bert2_core_msgs.msg


e1 = 0
e2 = 0

def main():
	rospy.init_node('pressure', anonymous=True)

        bert2_gazebo_mode = try_get_isSim()

        rospy.sleep(0.01)
	pub = rospy.Publisher("pressure", Int8, queue_size=1,latch=True)
        
        if bert2_gazebo_mode==1:
            # Running in simulation, rather than actual robot.  Use dummy sensing:

            # These lines were originally inside the loop below, but were
            # moved because it's not necessary to repeatedly register the 
            # subscriber/publisher.
	    rospy.Subscriber("pressure_e1", Int8, check_e1)
	    rospy.sleep(0.01)
	    rospy.Subscriber("pressure_e2", Int8, check_e2)
	    while not rospy.is_shutdown():
	
		if e1 == 1 and e2 ==1: 
			pub.publish(1)
		else:
			pub.publish(0)
		rospy.sleep(0.1)
        else:
            # Running the actual robot:

            # Temp fix, until pressure readings from new hand are working, treat pressure condition
            # as being always satisfied:
            pub.publish(1)
            rosp.sleep(0.1)

	    #rospy.Subscriber("bert2/hands/left_state", bert2_core_msgs.msg.HandState, check_motor_currents)
            
	    #while not rospy.is_shutdown():
	
		#if e1: 
		#	pub.publish(1)
		#else:
		#	pub.publish(0)
		#rospy.sleep(0.1)


def check_motor_currents(data):
        global e1
        # ****** To do:  Check thresholds and choice of motors below ******
        if data.index_finger_prox_current>50 and data.thumb_flex_current>50:
                e1 = 1
        else:
                e1 = 0


	
def check_e1(data):
	global e1
	if data.data == 1:
		e1 = 1
	else:
		e1 = 0

def check_e2(data):
	global e2
	if data.data == 1:
		e2 = 1
	else:
		e2 = 0

def try_get_isSim():

        # Handle ROS's get_param bug (https://github.com/RobotWebTools/rosbridge_suite/issues/103)
        attempt = 0
        while attempt<50:
            try:
                bert2_gazebo_mode = rospy.get_param('is_this_a_simulation')
                print "Success after attempt x ",attempt
                return bert2_gazebo_mode
            except:
                attempt+=1
                rospy.sleep(0.05)
	
	
if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException: #to stop the code when pressing Ctr+c
        	pass
