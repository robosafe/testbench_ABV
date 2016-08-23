
import rospy
import math

import std_msgs.msg
import bert2_core_msgs.msg

#---------------------------------------------------------------------------------------------------
class Face( object ):

    """
    Interface class for the face of the BERT2 robot.
    """

    PARALLEL_GAZE_DISTANCE = 100
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self ):
        """
        Constructor.
        """
        
        # Set up ROS publishers
        ns = '/bert2/'

        self._pub_expression_3d = rospy.Publisher(
            ns + 'face/expression3d',
            bert2_core_msgs.msg.FaceExpression3D, latch=True )
        self._pub_face_on = rospy.Publisher(
            ns + 'face/on',
            std_msgs.msg.Bool, latch=True )
        self._pub_randomness = rospy.Publisher(
            ns + 'face/randomness',
            std_msgs.msg.Float64, latch=True )
        self._pub_expression_8d = rospy.Publisher(
            ns + 'face/expression8d',
            bert2_core_msgs.msg.FaceExpression8D, latch=True )
        self._pub_gaze = rospy.Publisher(
            ns + 'face/gaze',
            bert2_core_msgs.msg.FaceGaze, latch=True )
        self._pub_interpolation_time_ms = rospy.Publisher(
            ns + 'face/interpolation_time_ms',
            std_msgs.msg.Float64, latch=True )
        self._pub_mouth_open = rospy.Publisher(
            ns + 'face/mouth_open',
            std_msgs.msg.Bool, latch=True )
            
    #-----------------------------------------------------------------------------------------------
    def set_expression_3d( self, arousal, valence, stance ):
        """
        Sets an expression in the 3D arousal, valence, stance space. All parameters should be 
        in the range [-1.0,1.0]

        @type arousal: float
        @param arousal: The arousal of the expression
        @type valence: float
        @param valence: The valence of the expression
        @type stance: float
        @param stance: The stance of the expression
        """
        
        msg = bert2_core_msgs.msg.FaceExpression3D( arousal, valence, stance )
        self._pub_expression_3d.publish( msg )
        
    #-----------------------------------------------------------------------------------------------
    def set_face_on( self, face_on ):
        """
        Sets the face display to be on or off

        @type face_on: bool
        @param face_on: determines if the face is on or off
        """
        msg = std_msgs.msg.Bool( face_on )
        self._pub_face_on.publish( msg )
        
    #-----------------------------------------------------------------------------------------------
    def set_randomness( self, randomness ):
        """
        Sets the amount of random movement injected into the expression (from 0 to 100)

        @type randomness: float
        @param randomness: the amount of randomness in the expression
        """
        msg = std_msgs.msg.Float64( randomness )
        self._pub_randomness.publish( msg )
        
    #-----------------------------------------------------------------------------------------------
    def set_expression_8d( self, happy, thinking, angry, disgusted, surprised, afraid, sad, tired ):
        """
        Sets an expression in the 8D happy, thinking, angry, disgusted, surprised, afraid,
        # sad, tired space. All parameters should be in the range [-1.0,1.0]

        @type happy: float
        @param happy: The happiness of the expression
        @type thinking: float
        @param thinking: The thought of the expression
        @type angry: float
        @param angry: The anger of the expression
        @type disgusted: float
        @param disgusted: The disgust of the expression
        @type surprised: float
        @param surprised: The surprise of the expression
        @type afraid: float
        @param afraid: The fear of the expression
        @type sad: float
        @param sad: The sadness of the expression
        @type tired: float
        @param tired: The tiredness of the expression
        """
        
        msg = bert2_core_msgs.msg.FaceExpression8D( 
            happy, thinking, angry, disgusted, surprised, afraid, sad, tired )
        self._pub_expression_8d.publish( msg )
        
    #-----------------------------------------------------------------------------------------------
    def set_gaze( self, pitch, yaw, distance_to_target=PARALLEL_GAZE_DISTANCE ):
        """
        # Sets the direction of the robot's gaze, and can also be used to set the distance from
        # the robot to the object. This changes the angle between the robot's eyes

        @type pitch: float
        @param pitch: Eye pitch in radians (+ve down)
        @type yaw: float
        @param yaw: Eye yaw in radians (+ve left from robot's perspective)
        @type distance_to_target: float
        @param distance_to_target: Not sure the unit of measure. Closer will bring eyes 
                                   together. Set to 100 for parallel gaze
        """
        
        msg = bert2_core_msgs.msg.FaceGaze( pitch, yaw, distance_to_target )
        self._pub_gaze.publish( msg )
        
    #-----------------------------------------------------------------------------------------------
    def set_interpolation_time_ms( self, interpolation_time_ms ):
        """
        # Sets the time that BERT2's face takes to interpolate between expressions

        @type interpolation_time_ms: float
        @param interpolation_time_ms: Time in milliseconds to interpolate between expressions
        """
        msg = std_msgs.msg.Float64( interpolation_time_ms )
        self._pub_interpolation_time_ms.publish( msg )

    #-----------------------------------------------------------------------------------------------
    def set_mouth_open( self, mouth_open ):
        """
        Sets the mouth to be open or closed

        @type mouth_open: bool
        @param mouth_open: determines if the mouth is open or closed
        """
        msg = std_msgs.msg.Bool( mouth_open )
        self._pub_mouth_open.publish( msg )
