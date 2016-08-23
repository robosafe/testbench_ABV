#!/usr/bin/env python

"""
This script draws a cylindrical object in Gazebo, and keeps a node with the location of its mass center in x,y,z coordinates (absolute from an origin at the base of the robot. The node is updated via a state machine. 

Written by Dejanira Araiza-Illan, March 2015.
"""

import rospy
import smach
import smach_ros
import random
import os
from bert2_gazebo.msg import *
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int8

def main():
	rospy.init_node('object', anonymous=True)
	#Initial location (reset)
	setmodel = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)	
        rospy.sleep(0.1)
	setmodel(ModelState('object',Pose(Point(0.27,-0.4,1.2),Quaternion(0.0,0.0,0.0,1.0)),Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)),'world'))

	rospy.Subscriber('resetpiece', Int8,reset)

        rospy.spin()	

        
        # Publish object position via ROS topic
    	getmodel = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
	while not rospy.is_shutdown():
    		rospy.sleep(0.01)
	        data = getmodel('object','')
	        piece = rospy.Publisher('piece_location', Point,queue_size=1,latch=True) 
	        piece.publish(data.pose.position.x,data.pose.position.y,data.pose.position.z)
        
		
def reset(data):
	setmodel = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)	
        rospy.sleep(5)
	setmodel(ModelState('object',Pose(Point(0.27,-0.4,1.2),Quaternion(0.0,0.0,0.0,1.0)),Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)),'world'))
    	

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
