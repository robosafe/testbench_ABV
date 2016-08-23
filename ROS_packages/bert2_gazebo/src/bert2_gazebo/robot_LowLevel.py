#!/usr/bin/env python
"""
This class provides methods for BERT2's low-level control, to be called by a higher level control script.

Assumptions:
  - If running as a Gazebo simulation (see bert2_gazebo.launch for an example)...
        - ... bert2_gazebo_motor_controller must be already running.  
        - ... GazeboToViconBroadcaster must be running with args="human_head human_hand object"

Format of the vector of joint commands for Gazebo-ROS:
'hipRotor', 'hipFlexor', 'neckFlexor', 'neckRotor', 'leftShoulderFlexor', 'rightShoulderFlexor', 'leftShoulderAbduction',  'leftHumeralRotation', 'leftElbowFlexor','leftWristPronation', 'leftWristAbduction', 'leftWristFlexor'

Format of the vector of joint commands for the hand:
'leftThumb1', 'leftThumb2', 'leftIndex1', 'leftIndex2', 'leftMiddle1', 'leftMiddle2', 'leftAnular1', 'leftAnular2', 'leftLittle1', 'leftLittle2'

Written by David Western, March 2016.
"""

import rospy
from std_msgs.msg import Float64
import tf
import numpy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
import bert2_core_msgs.msg

class ROBOT_LowLevel( object ):


    #-----------------------------------------------------------------------------------------------
    def __init__( self ):
        """
        Constructor.
        """

                
        self._IS_SIM = self._try_get_isSim()

        # Standard task-related poses:
        self.PREPARE_POSE = {
                "hip_rotor_joint": 0.2594325425513616,
                "hip_flexor_joint": 0.05,
                "left_shoulder_flex_joint": -0.7182422610321082,
                "left_shoulder_abduction_joint": 0.7853981633974483,
                "left_humeral_rot_joint": 0.030736993456872138,
                "left_elbow_flex_joint": 0.6381096762976632,
                "left_wrist_pronation_joint": 0.0,
                "left_wrist_abduction_joint": -0.05925885896095981,
                "left_wrist_flex_joint": 0.0,
        }

        self.ABOVE_PIECE_POSE = {
                "hip_rotor_joint": 1.2594325425513615,
                "hip_flexor_joint": 0.05,
                "left_shoulder_flex_joint": -0.7182422610321082,
                "left_shoulder_abduction_joint": 0.7853981633974483,
                "left_humeral_rot_joint": 0.030736993456872138,
                "left_elbow_flex_joint": 0.6381096762976632,
                "left_wrist_pronation_joint": 0.0,
                "left_wrist_abduction_joint": -0.05925885896095981,
                "left_wrist_flex_joint": 0.0,
        }

        self.PICKUP_POSE = {
                "hip_rotor_joint": 1.2594325425513615,
                "hip_flexor_joint": 0.05,
                "left_shoulder_flex_joint": -0.68,
                "left_shoulder_abduction_joint": 0.7853981633974483,
                "left_humeral_rot_joint": 0.030736993456872138,
                "left_elbow_flex_joint": 0.6381096762976632,
                "left_wrist_pronation_joint": 0.0,
                "left_wrist_abduction_joint": -0.05925885896095981,
                "left_wrist_flex_joint": 0.0,
        }

        self.SERVE_POSE = {
                "hip_rotor_joint": -0.24056745744863842,
                "hip_flexor_joint": 0.05,
                "left_shoulder_flex_joint": -0.7182422610321082,
                "left_shoulder_abduction_joint": 0.7853981633974483,
                "left_humeral_rot_joint": 0.030736993456872138,
                "left_elbow_flex_joint": 0.6381096762976632,
                "left_wrist_pronation_joint": 0.0,
                "left_wrist_abduction_joint": -0.05925885896095981,
                "left_wrist_flex_joint": 0.0,
        }


        # Mapping of gazebo model names onto names returned by actual Vicon system:
        self.VICON_ALIAS = {
                "/vicon_human_hand": "/vicon_MAN2_hhand",
                "/vicon_human_head": "/vicon_MAN2_hhead",
                "/vicon_object": "/vicon_MAN2b_table_leg_3"
        }



    def _try_get_isSim(self):

        # Handle ROS's get_param bug (https://github.com/RobotWebTools/rosbridge_suite/issues/103)
        attempt = 0
        while attempt<50:
            try:
                bert2_simulator_mode = rospy.get_param('is_this_a_simulation')
                #print "Success after attempt x ",attempt
                return bert2_simulator_mode
            except:
                attempt+=1
                rospy.sleep(0.05)

        # If we're not sure, assume we're running with hardware (since a mistake's more costly in that situation).        
        return 0


    #-----------------------------------------------------------------------------------------------
    def set_robot_joints(self,data):
        # Direct control of robot joints, bypassing the BERT2 body interface or trajectory planning.
        # Alternatively, use bert2_interface.Body.set_joint_positions(positions).
        #        
        # This section is taken from robot_g.py by Dejanira Araiza-Illan.

	hipRotor = rospy.Publisher('/bert2/hip_rotor_joint_posControlr/command',Float64, queue_size=1,latch=True) 
	hipFlexor = rospy.Publisher('/bert2/hip_flexor_joint_posControlr/command',Float64, queue_size=1,latch=True) 
	neckFlexor = rospy.Publisher('/bert2/neck_flexor_joint_posControlr/command',Float64, queue_size=1,latch=True) 
	neckRotor = rospy.Publisher('/bert2/neck_rotor_joint_posControlr/command',Float64, queue_size=1,latch=True) 
	leftShoulderFlexor = rospy.Publisher('/bert2/left_shoulder_flex_joint_posControlr/command', Float64, queue_size=1,latch=True) 
	rightShoulderFlexor = rospy.Publisher('/bert2/right_shoulder_flex_joint_posControlr/command', Float64, queue_size=1,latch=True) 
	leftShoulderAbduction = rospy.Publisher('/bert2/left_shoulder_abduction_joint_posControlr/command', Float64, queue_size=1,latch=True) 
	leftHumeralRotor = rospy.Publisher('/bert2/left_humeral_rot_joint_posControlr/command', Float64, queue_size=1,latch=True)
	leftElbowFlexor = rospy.Publisher('/bert2/left_elbow_flex_joint_posControlr/command', Float64, queue_size=1,latch=True) 
	leftWristPronation = rospy.Publisher('/bert2/left_wrist_pronation_joint_posControlr/command', Float64, queue_size=1,latch=True) 
	leftWristAbduction = rospy.Publisher('/bert2/left_wrist_abduction_joint_posControlr/command', Float64, queue_size=1,latch=True) 
	leftWristFlexor = rospy.Publisher('/bert2/left_wrist_flex_joint_posControlr/command', Float64, queue_size=1,latch=True) 

	if data[0]!=1000.0:
		hipRotor.publish(data[0])
		rospy.sleep(0.01)
	if data[1]!=1000.0:
		hipFlexor.publish(data[1])
		rospy.sleep(0.01)	
	if data[2]!=1000.0:	
		neckFlexor.publish(data[2])
		rospy.sleep(0.01)
	if data[3]!=1000.0:	
		neckRotor.publish(data[3])
		rospy.sleep(0.01)
	if data[4]!=1000.0:
		leftShoulderFlexor.publish(data[4])
		rospy.sleep(0.01)
	if data[5]!=1000.0:
		rightShoulderFlexor.publish(data[5])
		rospy.sleep(0.01)
	if data[6]!=1000.0:
		leftShoulderAbduction.publish(data[6])
		rospy.sleep(0.01)
	if data[7]!=1000.0:
		leftHumeralRotor.publish(data[7])
		rospy.sleep(0.01)
	if data[8]!=1000.0:
		leftElbowFlexor.publish(data[8])
		rospy.sleep(0.01)
	if data[9]!=1000.0:
		leftWristPronation.publish(data[9])
		rospy.sleep(0.01)
	if data[10]!=1000.0:
		leftWristAbduction.publish(data[10])
		rospy.sleep(0.01)
	if data[11]!=1000.0:
		leftWristFlexor.publish(data[11])
		rospy.sleep(0.01)



    #-----------------------------------------------------------------------------------------------
    def get_loc(self,tf_Listener,model_name):
        
        model_name = '/vicon_'+model_name

        if self._IS_SIM==0:
            try:
                model_name = self.VICON_ALIAS[model_name]
            except:
                pass

        # Get object location:
        try:
            tf_Listener.waitForTransform(model_name,'/world',rospy.Time(),rospy.Duration(5.0))
            (trans,objQ_vic) = tf_Listener.lookupTransform('/world', model_name, rospy.Time(0))
        except:
            rospy.signal_shutdown('*** Failed to access vicon data in get_loc() ***')

        loc = numpy.matrix(trans)

        #print model_name, " loc: ", loc
        return loc


# SENSING:
    #-----------------------------------------------------------------------------------------------
    def check_human_hand_loc(self,tf_Listener):

        offsetVec = (0,0,0.1) # Hand expected around 10cm below centre of object.
        threshVec = (0.3,0.3,0.3) # ... to within 30cm in any direction.
        
        obj_loc = self.get_loc(tf_Listener,'object')

        hand_loc = self.get_loc(tf_Listener,'human_hand')
 
        # Compare:
        cond = abs((obj_loc+offsetVec)-hand_loc)<threshVec
        if cond.all():
            return 1
	else:
            return 0



    #-----------------------------------------------------------------------------------------------
    def check_human_gaze(self,tf_Listener):
        
        obj_loc = self.get_loc(tf_Listener,'object')
        
        # Get human head state (gaze):
        model_name = '/vicon_human_head'
        if self._IS_SIM==0:
            try:
                model_name = self.VICON_ALIAS[model_name]
            except:
                pass

	try:
            tf_Listener.waitForTransform(model_name,'/world',rospy.Time(),rospy.Duration(5.0))
            (trans,headQ) = tf_Listener.lookupTransform('/world', model_name, rospy.Time(0))
        except:
            rospy.signal_shutdown('*** Failed to access vicon data in check_human_gaze() ***')

        head_loc = numpy.matrix(trans)
        
        # Extract gaze vector:
        gazeVecQ = numpy.array([-1, 0, 0, 0]) # as a quaternion.
        headQInv = tf.transformations.quaternion_inverse(headQ)
        gazeVecQ = tf.transformations.quaternion_multiply(gazeVecQ,headQInv)
        gazeVecQ = tf.transformations.quaternion_multiply(headQ,gazeVecQ)
        gazeVec = numpy.matrix(gazeVecQ[0:3])

        # Compare object location with gaze cone:
        # First get distance to gaze plane:
        d = numpy.inner( obj_loc-head_loc, gazeVec )
        # Then get intersection point between gaze plane and gaze ray:
        p = head_loc+d*gazeVec
        # Get in-plane distance to object:
        t = numpy.linalg.norm(obj_loc-p)
        # Get radius of gaze cone at intersection point:
        c = 0.83910 # tan(40 degrees) = 0.83910
        initRad = 0.1 # radius of gaze cone at head
        gcRad = initRad+d*c
        #print "gazeVec", gazeVec
        
        if d>0 and t<gcRad:
            return 1
        else:
            #print "gaze error (m)", t, "thresh =", gcRad
            return 0 



    #-----------------------------------------------------------------------------------------------
    def get_gaze_vec(self,tf_Listener):

        # Get human head state (gaze):
        model_name = '/vicon_human_head'

        if self._IS_SIM==0:
            try:
                model_name = self.VICON_ALIAS[model_name]
            except:
                pass

	try:
            tf_Listener.waitForTransform(model_name,'/world',rospy.Time(),rospy.Duration(5.0))
            (trans,headQ) = tf_Listener.lookupTransform('/world', model_name, rospy.Time(0))
        except:
            rospy.signal_shutdown('*** Failed to access vicon data in check_human_gaze() ***')

        # Extract gaze vector:
        gazeVecQ = numpy.array([-1, 0, 0, 0]) # as a quaternion.
        headQInv = tf.transformations.quaternion_inverse(headQ)
        gazeVecQ = tf.transformations.quaternion_multiply(gazeVecQ,headQInv)
        gazeVecQ = tf.transformations.quaternion_multiply(headQ,gazeVecQ)
        
        gazeVec = numpy.matrix(gazeVecQ[0:3])

        #print "gazeVec: ", gazeVec
        return gazeVec



