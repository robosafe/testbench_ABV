#!/usr/bin/env python

"""
Low-level control of human simulated in Gazebo.

Created by David Western, April 2016.
"""


import rospy
from bert2_gazebo.msg import *
from gazebo_msgs.srv import GetModelState, SetModelState, GetLinkState, ApplyBodyWrench
from gazebo_msgs.msg import ModelState, LinkState
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3, Wrench
import tf
from math import atan2

class HUMAN_LowLevel( object ):

    #-----------------------------------------------------------------------------------------------
    def __init__( self ):
        """
        Constructor.
        """

        rospy.wait_for_service('gazebo/set_model_state')
        self.setmodel = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        rospy.wait_for_service('gazebo/get_model_state')
        self.getmodel = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        
        rospy.wait_for_service('gazebo/apply_body_wrench')
        self.apply_wrench = rospy.ServiceProxy('gazebo/apply_body_wrench', ApplyBodyWrench)



    #-----------------------------------------------------------------------------------------------
    def move_head(self,angle):
        # Set human gaze direction to be either good or bad, depending on whether angle is less than 40.
        # For more detailed implementations, see HUMAN_LowLevel.set_gaze_angles() or HUMAN_LowLevel.look_at_object().

	if angle<=40.0:
		self.setmodel(ModelState('human_head',Pose(Point(1.15,-0.25,1.4),Quaternion(0.0,-0.15,-0.25,1.0)),Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)),'world'))
	else:
		self.setmodel(ModelState('human_head',Pose(Point(1.15,-0.25,1.4),Quaternion(0.0,0.0,0.0,1.0)),Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)),'world'))
	


    #-----------------------------------------------------------------------------------------------
    def set_head_height(self,height):
        # Move the human's head to a specified height (in meters)

	self.setmodel(ModelState('human_head',Pose(Point(1.15,-0.25,height),Quaternion(0.0,0.0,0.0,1.0)),Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)),'world'))
	


    #-----------------------------------------------------------------------------------------------
    def move_hand(self,x,y,z):
	gx = x+0
	gy = y+0
	gz = z+0	
	self.setmodel(ModelState('human_hand',Pose(Point(gx,gy,gz),Quaternion(0.0,0.0,0.0,1.0)),Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)),'world'))


    #-----------------------------------------------------------------------------------------------
    def move_model(self,model_name,x,y,z):
	gx = x+0
	gy = y+0
	gz = z+0	
	self.setmodel(ModelState(model_name,Pose(Point(gx,gy,gz),Quaternion(0.0,0.0,0.0,1.0)),Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)),'world'))
	


    #-----------------------------------------------------------------------------------------------
    def set_hand_distance(self,dist,target):
        # Set hand a specified distance from the target object, measured in meters in the +y direction.

        # Get object state:
        rospy.wait_for_service('gazebo/get_model_state')
        obj_state = self.getmodel(target,"world")
        
        # Shift position:
        loc = obj_state.pose.position
	loc.y = loc.y-dist

        # Set hand state:
	self.setmodel(ModelState('human_hand',Pose(loc,Quaternion(0.0,0.0,0.0,1.0)),Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)),'world'))
	


    #-----------------------------------------------------------------------------------------------
    def set_hand_speed(self,spd):
        # Set hand moving in the +y direction at a specified speed (m/s).
        
        # Get hand state:
        rospy.wait_for_service('gazebo/get_model_state')
        hand_state = self.getmodel("human_hand","world")

        # Modify velocity:
        hand_state.twist.linear = Vector3(0,spd,0)

        # Set hand state:
	self.setmodel(ModelState('human_hand',hand_state.pose,hand_state.twist,'world'))
	
	

    #-----------------------------------------------------------------------------------------------
    def reset_head_hand(self):
	self.setmodel(ModelState('human_head',Pose(Point(1.15,-0.25,1.4),Quaternion(0.0,0.0,0.0,1.0)),Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)),'world'))
	self.setmodel(ModelState('human_hand',Pose(Point(1.15,-0.43,0.73),Quaternion(0.0,0.0,0.0,1.0)),Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)),'world'))

	

    #-----------------------------------------------------------------------------------------------
    def reset_obj(self):
	self.setmodel(ModelState('object',Pose(Point(0.27,-0.4,1.2),Quaternion(0.0,0.0,0.0,1.0)),Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)),'world'))

	

    #-----------------------------------------------------------------------------------------------
    def move_hand_to_object(self):
        
        # Get object location:
        rospy.wait_for_service('gazebo/get_model_state')
        obj_state = self.getmodel("object","world")
        obj_loc = obj_state.pose.position

        # Get hand state:
	rospy.wait_for_service('gazebo/get_model_state')
	hand_state = self.getmodel("human_hand","world")

        # Set hand location appropriately:
        hand_state2 = ModelState("human_hand",hand_state.pose,Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)),'world')
        #hand_state2.model_name = "human_hand"
        #hand_state2.pose = hand_state.pose
        hand_state2.pose.position.x = obj_loc.x
        hand_state2.pose.position.y = obj_loc.y-0.1
        hand_state2.pose.position.z = obj_loc.z-0.1
        rospy.wait_for_service('gazebo/set_model_state')
        self.setmodel(hand_state2)


	

    #-----------------------------------------------------------------------------------------------
    def angles_to_object(self,target):

        # Get target location:
        rospy.wait_for_service('gazebo/get_model_state')
        obj_state = self.getmodel(target,"world")
        obj_loc = obj_state.pose.position

        # Get head location:
        rospy.wait_for_service('gazebo/get_model_state')
        head_state = self.getmodel("human_head","world")
        head_loc = head_state.pose.position

	# Calculate pitch and yaw needed to direct gaze along
        # vector from head to target:
        gazeVec = [obj_loc.x-head_loc.x, obj_loc.y-head_loc.y, obj_loc.z-head_loc.z]
	pitch = atan2(gazeVec[2],(gazeVec[0]**2+gazeVec[1]**2)**0.5)
	yaw = atan2(gazeVec[1],gazeVec[0])+3.14159
        roll = 0

        angles = [roll,pitch,yaw]
        return angles



    #-----------------------------------------------------------------------------------------------
    def set_gaze_angles(self,angles):
        # Make human look in a specified direction (roll, pitch, yaw).

        roll = angles[0]
        pitch = angles[1]
        yaw = angles[2]

        # Get head state:
        rospy.wait_for_service('gazebo/get_model_state')
        head_state = self.getmodel("human_head","world")

	# Apply orientation to head:
        head_state2 = ModelState("human_head",head_state.pose,Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)),'world')
	q = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
	head_state2.pose.orientation = Quaternion(*q)
        rospy.wait_for_service('gazebo/set_model_state')
        self.setmodel(head_state2)




    #-----------------------------------------------------------------------------------------------
    def look_at_object(self,target):

        angles = self.angles_to_object(target)
        
        self.set_gaze_angles(angles)


	
    #-----------------------------------------------------------------------------------------------
    def apply_pressure(self):
        # Pull the object in a standard way (moderate force towards human, moderate torque about 
        # horizontal axis, rolling towards human)

        rospy.wait_for_service('gazebo/apply_body_wrench')
        refPoint = Point(0,0,0)
        force = Vector3(2,0,0)
        torque = Vector3(0,1,0)
        wrench = Wrench(force,torque)
        start_time = rospy.Time(0)
        duration = rospy.Duration(1)
        print "*** Ignore upcoming error message about apply_body_wrench ***"
        success = self.apply_wrench("object::object_link", "human_hand::human_hand_link", refPoint, wrench, start_time, duration)


	
    #-----------------------------------------------------------------------------------------------
    def pull_object(self,Fx,Fy,Fz,Tx,Ty,Tz,dur):
        # Pull the object with a specified force (Newtons, 3D), torque (Nm, 3D) and duration (s)

        rospy.wait_for_service('gazebo/apply_body_wrench')
        force = Vector3(Fx,Fy,Fz)
        torque = Vector3(Tx,Ty,Tz)
        wrench = Wrench(force,torque)
        start_time = rospy.Time(0)
        duration = rospy.Duration(dur)

        # Apply wrench defined in world frame:
        ref_frame = "world"
        refPoint = Point(0,0,0)

        # Apply wrench defined in human_hand frame.  This seems to generate an error message.
        #ref_frame = "human_hand::human_hand_link"
        #refPoint = Point(0,0,0)
        # print "*** Ignore upcoming error message about apply_body_wrench ***"

        success = self.apply_wrench("object::object_link", ref_frame, refPoint, wrench, start_time, duration)
