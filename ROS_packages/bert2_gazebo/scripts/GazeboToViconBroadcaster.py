#! /usr/bin/env python

#-------------------------------------------------------------------------------
# This node broadcasts the tf (spatial transformation) info of specified Gazebo
# objects in a format consist with the bert2 egosphere's YARP-ROS interface.
# This allows control code to run in simulation as in hardware. 
#
# Created by David Western, January 2016.
#-------------------------------------------------------------------------------

# ROS imports
#import roslib
#roslib.load_manifest( 'bert2_egosphere' )
import rospy
import sys
import tf
import numpy
import random
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState

import math

#-------------------------------------------------------------------------------
class GazeboToVicon:
        
    OBJECT_NAME = "rp_tor"
    VICON_FRAME_ID = "vicon"
    TABLE_FRAME_ID = "world"
    OUTPUT_TF_FRAME_ID = "ViconTorus"
    
    # Make table frame the same as the Gazebo world frame.
    VICON_IN_TABLE_SPACE_POS = [ 0.0, 0.0, 0.0 ] 
    #VICON_IN_TABLE_SPACE_POS = [ 0.317, 0.0, 0.0 ] # Value used in the real 
                                                    # vicon broadcaster.
    VICON_IN_TABLE_SPACE_ROT = [ 0.0, 0.0, 0.0 ]    # X, Y, and Z angle
    
    #---------------------------------------------------------------------------
    def __init__( self, args ):  

        self.objectsToTrack = args[1:len(args)-2] # skip arg[0] and last two.
        
        # Connect to ROS
        rospy.init_node( 'ViconObjectBroadcaster', anonymous=True )
        self.transformBroadcaster = tf.TransformBroadcaster()
        
        print "GazeboToVicon is ready to broadcast the following objects from Gazebo:"
        for obj in self.objectsToTrack:
            print "    "+obj
            if obj=='object':
                print
                print "*******************************************************"
                print "* Warning: Occasional errors are being deliberately"
                print "*          added to tracking of 'object'.  For details,"
                print "*          see GazeboToViconBroadcaster.py."
                print "*******************************************************"
                print


    #---------------------------------------------------------------------------
    def broadcast( self ):
        
        for objectName in self.objectsToTrack:
            rospy.wait_for_service('gazebo/get_model_state')
            get_state_sp = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
            obj_state = get_state_sp(objectName,"world")
            obj_loc = obj_state.pose.position
            xyz = (obj_loc.x, obj_loc.y, obj_loc.z)
            Q = [obj_state.pose.orientation.x, obj_state.pose.orientation.y, obj_state.pose.orientation.z,
                                                                         obj_state.pose.orientation.w]

            if objectName=='object':            
            	    no = random.uniform(0, 100)
                    if no<=3.1: 
                        #rospy.loginfo("Fake vicon error on object.")
                        xyz = (0,0,0)
                        Q = (0,0,0,1)
                                 

            #self.transformBroadcaster.sendTransform( ( x, y, z ), 
            #        tf.transformations.quaternion_from_euler( rotX, rotY, rotZ ),
            #        rospy.Time.now(), child="vicon_" + objectName, parent=self.VICON_FRAME_ID )
            self.transformBroadcaster.sendTransform( xyz, Q, rospy.Time.now(), child="vicon_" + objectName, parent=self.VICON_FRAME_ID )
            #if objectName=='object':
            #    print xyz
    
    #---------------------------------------------------------------------------
    def broadcastTableToViconFrame( self ):
        
        # Broadcast the transform from the TABLE_FRAME_ID to VICON_FRAME_ID
        self.transformBroadcaster.sendTransform( self.VICON_IN_TABLE_SPACE_POS, 
            tf.transformations.quaternion_from_euler( 
                self.VICON_IN_TABLE_SPACE_ROT[ 0 ],
                self.VICON_IN_TABLE_SPACE_ROT[ 1 ],
                self.VICON_IN_TABLE_SPACE_ROT[ 2 ] ),
            rospy.Time.now(), child=self.VICON_FRAME_ID, parent=self.TABLE_FRAME_ID )

#-------------------------------------------------------------------------------
if __name__ == "__main__":
    
    try:
        if len(sys.argv) < 2:
            print("usage: rosrun bert2_gazebo GazeboToViconBroadcaster.py NameOf1stObjectToTrack NameOf2ndObjectToTrack ...")
        else:
            broadcaster = GazeboToVicon(sys.argv)
         
            hz = rospy.get_param( "rate", 100 ) # 100hz
            r = rospy.Rate( hz ) 
    
        while not rospy.is_shutdown():
            r.sleep()
            broadcaster.broadcast()
            broadcaster.broadcastTableToViconFrame()
    
    except rospy.ROSInterruptException: #to stop the code when pressing Ctr+c
      	pass
    
