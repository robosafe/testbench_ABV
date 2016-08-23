#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
"""
Author: Acorn Pooley
Modified by Dejanira Araiza Illan, July 2015.

Format of the vector of joint commands for Gazebo-ROS:
'hipRotor', 'hipFlexor', 'neckFlexor', 'neckRotor', 'leftShoulderFlexor', 'rightShoulderFlexor', 'leftShoulderAbduction',  'leftHumeralRotation', 'leftElbowFlexor','leftWristPronation', 'leftWristAbduction', 'leftWristFlexor'

Format of the joints used for planning:
'hipRotor', 'hipFlexor', 'leftShoulderFlexor', 'leftShoulderAbduction',  'leftHumeralRotation', 'leftElbowFlexor','leftWristPronation', 'leftWristAbduction', 'leftWristFlexor'

"""

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from trajectory_msgs.msg import JointTrajectory

def interface(group_variables):
  
  # Initialize moveit_commander
  moveit_commander.roscpp_initialize(sys.argv)
 
  # Instantiate a RobotCommander object
  robot = moveit_commander.RobotCommander()
  
  # Instantiate a PlanningSceneInterface object
  scene = moveit_commander.PlanningSceneInterface()

  # Instantiate a MoveGroupCommander object
  group = moveit_commander.MoveGroupCommander("body_arm")


  # Planning to a joint-space goal 
  # Get the current set of joint values for the group
  group_variable_values = group.get_current_joint_values() 

  # Plan to the new joint space goal 
  for k,elem in enumerate(group_variables):
  	if elem <1000.0:
  		group_variable_values[k] = group_variables[k] 
  group.set_joint_value_target(group_variable_values)
 

  try:
  	plan = group.plan()
  	the_traj = plan.joint_trajectory.points
  	new_plan=[]
  
  	#Format the plans to be: [joint value,...,joint value] following the format in robot_g.py 
  	for j,traj in enumerate(the_traj):
  		temp_plan = [traj.positions[0],traj.positions[1],0.0,0.0, traj.positions[2],1000.0, traj.positions[3], traj.positions[4], traj.positions[5], traj.positions[6], traj.positions[7], traj.positions[8]]
  		new_plan.append(temp_plan)
  except:
  	new_plan = []

  
  #moveit_commander.roscpp_shutdown()				
  return new_plan

