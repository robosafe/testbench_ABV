
import rospy
import math
import copy

import std_msgs.msg
import bert2_core_msgs.msg

#---------------------------------------------------------------------------------------------------
CURRENT_SENSOR_NAMES = [
    "palm_pressure",
    "index_finger_prox_current",
    "middle_finger_trigger_current",
    "index_finger_trigger_current",
    "middle_finger_prox_current",
    "ring_finger_current",
    "small_finger_current",
    "thumb_oppose_current",
    "thumb_flex_current"
]

#---------------------------------------------------------------------------------------------------
class HandState:
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self ):
        
        self.is_alive = False
        self.active_command = -1
        self.command_complete = True
        self.current = {}
        
        for current_sensor_name in CURRENT_SENSOR_NAMES:
            self.current[ current_sensor_name ] = 0

    #-----------------------------------------------------------------------------------------------
    def set_from_hand_state_msg( self, msg ):
        
        self.is_alive = msg.is_alive
        self.active_command = msg.active_command
        self.command_complete = msg.command_complete
        
        for current_sensor_name in CURRENT_SENSOR_NAMES:
            self.current[ current_sensor_name ] = getattr( msg, current_sensor_name )
            
#---------------------------------------------------------------------------------------------------
class Hands( object ):

    """
    Interface class for the hands of the BERT2 robot.
    """

    PARALLEL_GAZE_DISTANCE = 100
    
    RESET_HAND_COMMAND = -1
    PREPARE_FOR_GRASP_COMMAND = 0
    GRASP_COMMAND = 1
    RELEASE_COMMAND = 2
    POINT_COMMAND = 3
    FIST_COMMAND = 4
    FLAT_COMMAND = 5
    SENSITIVE_RELEASE_COMMAND = 6
    SENSITIVE_GRASP_COMMAND = 7
    SET_VELOCITY_COMMAND = 9
    
    LEFT_HAND = "left"
    RIGHT_HAND = "right"
    
    #-----------------------------------------------------------------------------------------------
    def __init__( self ):
        """
        Constructor.
        """
        
        # Store hand states in internal state variables
        self.hand_states = {
            self.LEFT_HAND : HandState(),
            self.RIGHT_HAND : HandState(),
        }
        
        # Set up ROS publishers and subscribers
        ns = '/bert2/'

        self._left_hand_state_sub = rospy.Subscriber(
            ns + 'hands/left_state',
            bert2_core_msgs.msg.HandState,
            self._on_left_hand_state_msg,
            queue_size=1,
            tcp_nodelay=True)
        self._right_hand_state_sub = rospy.Subscriber(
            ns + 'hands/right_state',
            bert2_core_msgs.msg.HandState,
            self._on_right_hand_state_msg,
            queue_size=1,
            tcp_nodelay=True)
        
        self._pub_hand_command = rospy.Publisher(
            ns + 'hands/command',
            bert2_core_msgs.msg.HandCommand,
            queue_size=1, latch=False )
    
    #-----------------------------------------------------------------------------------------------
    def _on_left_hand_state_msg( self, msg ):
        
        self.hand_states[ self.LEFT_HAND ].set_from_hand_state_msg( msg )
        
    #-----------------------------------------------------------------------------------------------
    def _on_right_hand_state_msg( self, msg ):
        
        self.hand_states[ self.RIGHT_HAND ].set_from_hand_state_msg( msg )
    
    #-----------------------------------------------------------------------------------------------
    def is_hand_alive( self, hand_name ):
        """
        Tests to see if the given hand is alive

        @type hand_name: str
        @param hand_name: The name of the hand to query
        @rtype: bool
        @return: True if the hand is alive, False otherwise
        """
        
        return self.hand_states[ hand_name ].is_alive
    
    #-----------------------------------------------------------------------------------------------
    def get_active_hand_command( self, hand_name ):
        """
        Gets the active command for the given hand

        @type hand_name: str
        @param hand_name: The name of the hand to query
        @rtype: int
        @return: The id of the active command
        """
        
        return self.hand_states[ hand_name ].active_command
        
    #-----------------------------------------------------------------------------------------------
    def is_command_complete( self, hand_name ):
        """
        Determines if the command for the given hand is complete

        @type hand_name: str
        @param hand_name: The name of the hand to query
        @rtype: bool
        @return: True if the command is complete, False otherwise
        """
        
        return self.hand_states[ hand_name ].command_complete
            
    #-----------------------------------------------------------------------------------------------
    def get_current( self, hand_name, current_sensor_name ):
        """
        Gets the current for a given sensor on a given hand
        
        @type hand_name: str
        @param hand_name: The name of the hand to query
        @type current_sensor_name: str
        @param current_sensor_name: The name of the current sensor to query
        @rtype: int
        @return: The current (0 to 1000mA) measured by the current sensor
        """
        
        return self.hand_states[ hand_name ].current[ current_sensor_name ]
        
    #-----------------------------------------------------------------------------------------------
    def get_currents( self, hand_name ):
        """
        Gets a dictionary of all the current sensor readings on a given hand

        @type hand_name: str
        @param hand_name: The name of the hand to query
        @rtype: dict({str:int})
        @return: An unordered dictionary giveing the current (0 to 1000mA) measured by each current sensor
        """
        
        return copy.deepcopy( self.hand_states[ hand_name ].current )
        
    #-----------------------------------------------------------------------------------------------
    def get_current_sensor_names( self ):
        """
        Gets a list of available current sensors
        @rtype: list(str)
        @return: A list of avialable current sensors
        """
        
        return copy.deepcopy( CURRENT_SENSOR_NAMES )
    
    #-----------------------------------------------------------------------------------------------
    def send_hand_command( self, hand_name, command_id, param_1=0, param_2=0 ):
        """
        Sends an arbitrary command to the given hand

        @type hand_name: str
        @param hand_name: The name of the hand to reset
        @type command_id: int
        @param command_id: The id of the command (see bert2_core_msgs/HandCommand.msg for valid ids)
        @type param_1: int
        @param param_1: First param
        @type param_2: int
        @param param_2: Second param
        """
        msg = bert2_core_msgs.msg.HandCommand()
        if hand_name == self.LEFT_HAND:
            
            msg.left_hand_command = command_id
            msg.left_hand_param_1 = param_1
            msg.left_hand_param_2 = param_2
            msg.right_hand_command = -10        # Hopefully this will leave active commands untouched
            self._pub_hand_command.publish( msg )
            
        elif hand_name == self.RIGHT_HAND:
        
            msg.left_hand_command = -10         # Hopefully this will leave active commands untouched
            msg.right_hand_command = command_id
            msg.right_hand_param_1 = param_1
            msg.right_hand_param_2 = param_2
            self._pub_hand_command.publish( msg )
    
    #-----------------------------------------------------------------------------------------------
    def reset_hand( self, hand_name ):
        """
        Resets the given hand

        @type hand_name: str
        @param hand_name: The name of the hand to reset
        """
        self.send_hand_command( hand_name, bert2_core_msgs.msg.HandCommand.RESET_HAND_COMMAND )
        
    #-----------------------------------------------------------------------------------------------
    def prepare_for_grasp( self, hand_name ):
        """
        Prepares a given hand to grasp

        @type hand_name: str
        @param hand_name: The name of the hand to prepare
        """
        self.send_hand_command( hand_name, bert2_core_msgs.msg.HandCommand.PREPARE_FOR_GRASP_COMMAND ) 
        
    #-----------------------------------------------------------------------------------------------
    def grasp( self, hand_name ):
        """
        Makes a given hand grasp

        @type hand_name: str
        @param hand_name: The name of the hand to use to grasp
        """
        self.send_hand_command( hand_name, bert2_core_msgs.msg.HandCommand.GRASP_COMMAND ) 
        
    #-----------------------------------------------------------------------------------------------
    def release( self, hand_name ):
        """
        Makes a given hand release

        @type hand_name: str
        @param hand_name: The name of the hand to use to release
        """
        self.send_hand_command( hand_name, bert2_core_msgs.msg.HandCommand.RELEASE_COMMAND ) 

    #-----------------------------------------------------------------------------------------------
    def point( self, hand_name ):
        """
        Makes a given hand point

        @type hand_name: str
        @param hand_name: The name of the hand to use to point
        """
        self.send_hand_command( hand_name, bert2_core_msgs.msg.HandCommand.POINT_COMMAND )
        
    #-----------------------------------------------------------------------------------------------
    def form_fist( self, hand_name ):
        """
        Makes a given hand form a fist

        @type hand_name: str
        @param hand_name: The name of the hand to use to form a fist
        """
        self.send_hand_command( hand_name, bert2_core_msgs.msg.HandCommand.FIST_COMMAND ) 
    
    #-----------------------------------------------------------------------------------------------
    def flatten_hand( self, hand_name ):
        """
        lattens a given hand

        @type hand_name: str
        @param hand_name: The name of the hand to flatten
        """
        self.send_hand_command( hand_name, bert2_core_msgs.msg.HandCommand.FLAT_COMMAND ) 
    
    #-----------------------------------------------------------------------------------------------
    def sensitive_grasp( self, hand_name ):
        """
        Makes a given hand grasp sensitively

        @type hand_name: str
        @param hand_name: The name of the hand to use to grasp
        """
        self.send_hand_command( hand_name, bert2_core_msgs.msg.HandCommand.SENSITIVE_GRASP_COMMAND ) 
        
    #-----------------------------------------------------------------------------------------------
    def sensitive_release( self, hand_name ):
        """
        Makes a given hand release sensitively

        @type hand_name: str
        @param hand_name: The name of the hand to use to release
        """
        self.send_hand_command( hand_name, bert2_core_msgs.msg.HandCommand.SENSITIVE_RELEASE_COMMAND ) 
