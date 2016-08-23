
import collections

from copy import deepcopy
import math

import rospy

import sensor_msgs.msg
import std_msgs.msg

import bert2_dataflow
import bert2_core_msgs.msg

#---------------------------------------------------------------------------------------------------
class Body( object ):
    """
    Interface class for the body of the BERT2 robot.
    """

    # Containers
    Point = collections.namedtuple('Point', ['x', 'y', 'z'])
    Quaternion = collections.namedtuple('Quaternion', ['x', 'y', 'z', 'w'])
    
    NEUTRAL_POSE = {
        "hip_rotor_joint": math.radians( -13.7835 ),
        "hip_flexor_joint": math.radians( 0.0 ), #-10.1011 ),
        "neck_flexor_joint": math.radians( -1.5310 ),
        "neck_rotor_joint": math.radians( -0.0051 ),
        "left_shoulder_flex_joint": math.radians( 2.9655 ),
        "left_shoulder_abduction_joint": math.radians( 45.0000 ),
        "left_humeral_rot_joint": math.radians( 1.7611 ),
        "left_elbow_flex_joint": math.radians( -43.6531 ),
        "left_wrist_pronation_joint": math.radians( 65.1627 ),
        "left_wrist_abduction_joint": math.radians( 6.3450 ),
        "left_wrist_flex_joint": math.radians( 1.3932 ),
    }
    
    JOINT_NAMES = [
        "hip_rotor_joint",
        "hip_flexor_joint",
        "neck_flexor_joint",
        "neck_rotor_joint",
        "left_shoulder_flex_joint",
        "left_shoulder_abduction_joint",
        "left_humeral_rot_joint",
        "left_elbow_flex_joint",
        "left_wrist_pronation_joint",
        "left_wrist_abduction_joint",
        "left_wrist_flex_joint"
    ]
    
    # Tolerance for moving to a set joint angle.
    JOINT_ANGLE_TOLERANCE = math.radians( 1.1 )

    #-----------------------------------------------------------------------------------------------
    def __init__( self ):
        """
        Constructor.
        """
        self._joint_angle = dict()
        self._joint_velocity = dict()
        #self._cartesian_pose = dict()
        #self._cartesian_velocity = dict()
        
        # These limits don't seem to have an effect, at least in simulation.
        self._max_joint_speeds = dict()
        #self._default_max_joint_speed = math.radians( 5.0 )
        self._default_max_joint_speed = math.radians( 10.0 )
        #self._default_max_joint_speed = math.radians( 0.0005 )

        ns = '/bert2/'

        self._command_msg = bert2_core_msgs.msg.JointCommand()

        self._pub_joint_cmd = rospy.Publisher(
            ns + 'joint_command',
            bert2_core_msgs.msg.JointCommand,
            queue_size=1,
            tcp_nodelay=True)

        self._pub_joint_cmd_timeout = rospy.Publisher(
            ns + 'joint_command_timeout',
            std_msgs.msg.Float64,
            queue_size=1,
            latch=True)

        #_cartesian_state_sub = rospy.Subscriber(
            #ns + 'endpoint_state',
            #EndpointState,
            #self._on_endpoint_states,
            #queue_size=1,
            #tcp_nodelay=True)

        joint_state_topic = ns + 'joint_states'
        _joint_state_sub = rospy.Subscriber(
            joint_state_topic,
            sensor_msgs.msg.JointState,
            self._on_joint_states,
            queue_size=1,
            tcp_nodelay=True)

	# For dev use: Option to extend timeout when getting initial joint_states.
        #	This was found to be necessary when including stl meshes in the human,
	#	because they slow down the loading process.
        let_it_slow = 1
	if let_it_slow:
		timeout_dur = 10
	else:
		timeout_dur = 1

        err_msg = "body init failed to get current joint_states from {0}".format( joint_state_topic )
        bert2_dataflow.wait_for(lambda: len(self._joint_angle.keys()) > 0, timeout_msg=err_msg, timeout=timeout_dur)

    #-----------------------------------------------------------------------------------------------
    def _on_joint_states(self, msg):
        for idx, name in enumerate(msg.name):
            self._joint_angle[name] = msg.position[idx]
            self._joint_velocity[name] = msg.velocity[idx]

    #-----------------------------------------------------------------------------------------------
    #def _on_endpoint_states(self, msg):
        ## Comments in this private method are for documentation purposes.
        ## _pose = {'position': (x, y, z), 'orientation': (x, y, z, w)}
        #self._cartesian_pose = {
            #'position': self.Point(
                #msg.pose.position.x,
                #msg.pose.position.y,
                #msg.pose.position.z,
            #),
            #'orientation': self.Quaternion(
                #msg.pose.orientation.x,
                #msg.pose.orientation.y,
                #msg.pose.orientation.z,
                #msg.pose.orientation.w,
            #),
        #}
        ## _twist = {'linear': (x, y, z), 'angular': (x, y, z)}
        #self._cartesian_velocity = {
            #'linear': self.Point(
                #msg.twist.linear.x,
                #msg.twist.linear.y,
                #msg.twist.linear.z,
            #),
            #'angular': self.Point(
                #msg.twist.angular.x,
                #msg.twist.angular.y,
                #msg.twist.angular.z,
            #),
        #}
  
    #-----------------------------------------------------------------------------------------------
    def joint_angle(self, joint):
        """
        Return the requested joint angle.

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: angle in radians of individual joint
        """
        return self._joint_angle[joint]

    #-----------------------------------------------------------------------------------------------
    def joint_angles(self):
        """
        Return all joint angles.

        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to angle (rad) Values
        """
        return deepcopy(self._joint_angle)

    #-----------------------------------------------------------------------------------------------
    def joint_velocity(self, joint):
        """
        Return the requested joint velocity.

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: velocity in radians/s of individual joint
        """
        return self._joint_velocity[joint]

    #-----------------------------------------------------------------------------------------------
    def joint_velocities(self):
        """
        Return all joint velocities.

        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to velocity (rad/s) Values
        """
        return deepcopy(self._joint_velocity)

    #def endpoint_pose(self):
        #"""
        #Return Cartesian endpoint pose {position, orientation}.

        #@rtype: dict({str:L{Limb.Point},str:L{Limb.Quaternion}})
        #@return: position and orientation as named tuples in a dict

        #C{pose = {'position': (x, y, z), 'orientation': (x, y, z, w)}}

          #- 'position': Cartesian coordinates x,y,z in
                        #namedtuple L{Limb.Point}
          #- 'orientation': quaternion x,y,z,w in named tuple
                           #L{Limb.Quaternion}
        #"""
        #return deepcopy(self._cartesian_pose)

    #def endpoint_velocity(self):
        #"""
        #Return Cartesian endpoint twist {linear, angular}.

        #@rtype: dict({str:L{Limb.Point},str:L{Limb.Point}})
        #@return: linear and angular velocities as named tuples in a dict

        #C{twist = {'linear': (x, y, z), 'angular': (x, y, z)}}

          #- 'linear': Cartesian velocity in x,y,z directions in
                      #namedtuple L{Limb.Point}
          #- 'angular': Angular velocity around x,y,z axes in named tuple
                       #L{Limb.Point}
        #"""
        #return deepcopy(self._cartesian_velocity)

    #-----------------------------------------------------------------------------------------------
    def set_command_timeout(self, timeout):
        """
        Set the timeout in seconds for the joint controller

        @type timeout: float
        @param timeout: timeout in seconds
        """
        self._pub_joint_cmd_timeout.publish(std_msgs.msg.Float64(timeout))

    #-----------------------------------------------------------------------------------------------
    def set_default_max_joint_speed(self, speed):
        """
        Sets the default maximum joint speed in radians per second. This speed is used for all
        joints unless a specific maximum joint speed is set for a joint using set_max_joint_speed

        @type speed: float
        @param speed: default maximum joint speed in radians per second
        """
        self._default_max_joint_speed = speed
        
    #-----------------------------------------------------------------------------------------------
    def get_default_max_joint_speed(self):
        """
        Gets the default maximum joint speed in radians per second.
        """
        return self._default_max_joint_speed
        
    #-----------------------------------------------------------------------------------------------
    def set_max_joint_speed( self, joint_name, speed ):
        """
        Sets the maximum joint speed in radians per second for a given joint

        @type joint_name: string
        @param joint_name: name of the joint to set the maximum speed for
        @type speed: float
        @param speed: maximum joint speed in radians per second
        """
        self._max_joint_speeds[ joint_name ] = speed
        
    #-----------------------------------------------------------------------------------------------
    def get_default_max_joint_speed( self, joint_name ):
        """
        Gets the maximum joint speed in radians per second for a given joint

        @type joint_name: string
        @param joint_name: name of the joint to set the maximum speed for
        """
        return self._max_joint_speeds[ joint_name ]

    #-----------------------------------------------------------------------------------------------
    def set_joint_positions( self, positions ):
        """
        Commands the joints of this limb to the specified positions.

        @type positions: dict({str:float})
        @param positions: joint_name:angle command
        """
        
        num_positions = len( positions )
        
        self._command_msg.names = []
        self._command_msg.angles = []
        self._command_msg.velocities = []
        self._command_msg.accelerations = []
        
        for joint_name in positions.keys():
            self._command_msg.names.append( joint_name )
            self._command_msg.angles.append( positions[ joint_name ] )
            
            if joint_name in self._max_joint_speeds:
                self._command_msg.velocities.append( self._max_joint_speeds[ joint_name ] )
            else:
                self._command_msg.velocities.append( self._default_max_joint_speed )
            
            self._command_msg.accelerations.append( math.radians( 0.0 ) )
        
        self._pub_joint_cmd.publish(self._command_msg)

    #-----------------------------------------------------------------------------------------------
    def move_to_neutral(self, timeout=15.0):
        """
        Command the joints to a neutral pose

        @type timeout: float
        @param timeout: seconds to wait for move to finish [15]
        """
        return self.move_to_joint_positions(self.NEUTRAL_POSE, timeout)

    #-----------------------------------------------------------------------------------------------
    def move_to_joint_positions(self, positions, timeout=15.0,
                                threshold=JOINT_ANGLE_TOLERANCE):
        """
        (Blocking) Commands the robot to the provided positions.

        Waits until the reported joint state matches that specified.

        This function uses a low-pass filter to smooth the movement.

        @type positions: dict({str:float})
        @param positions: joint_name:angle command
        @type timeout: float
        @param timeout: seconds to wait for move to finish [15]
        @type threshold: float
        @param threshold: position threshold in radians across each joint when
        move is considered successful
        """
        cmd = self.joint_angles()

        def filtered_cmd():
            # First Order Filter - 0.2 Hz Cutoff
            for joint in positions.keys():
                cmd[joint] = 0.012488 * positions[joint] + 0.98751 * cmd[joint]
            return cmd

        def genf(joint, angle):
            def joint_diff():
                return abs(angle - self._joint_angle[joint])
            return joint_diff

        diffs = [genf(j, a) for j, a in positions.items() if
                 j in self._joint_angle]

        self.set_joint_positions(filtered_cmd())
        bert2_dataflow.wait_for(
            lambda: (all(diff() < threshold for diff in diffs)),
            timeout=timeout,
            timeout_msg="bert2 failed to reach commanded joint positions",
            rate=100,
            raise_on_error=True,
            body=lambda: self.set_joint_positions(filtered_cmd())
            )
