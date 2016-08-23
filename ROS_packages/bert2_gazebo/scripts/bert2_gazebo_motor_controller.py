#!/usr/bin/env python
"""
This node performs various functions to allow code controlling a gazebo simulation of the
bert2 robot to behave similarly to the hardware implementation.

Pick up commands from the joint_trajectory_action_server
and distribute them to the individual joint controllers for the
Gazebo model of BERT2.

Created by David Western, March 2015.
"""

import rospy

import bert2_core_msgs.msg
import bert2_interface
import std_msgs.msg
import control_msgs.msg
import sensor_msgs.msg
import moveit_msgs.msg

class BERT2GazeboMotorController:
    def __init__(self):
  
        # Body stuff:

        self.jointPubs = {} # Dictionary of publisher for each joint.
                            # Will be populated as needed.

        joint_command_sub = rospy.Subscriber(
            '/bert2/joint_command',
            bert2_core_msgs.msg.JointCommand,
            self._on_joint_command,
            queue_size=1,
            tcp_nodelay=True)



        # Hand stuff:

        ns = '/bert2/'
        allJoints = ('left_thumb', 'left_thumb2', 'left_index_finger', 'left_index_finger2', 'left_mid_finger',
                  'left_mid_finger2', 'left_anular_finger', 'left_anular_finger2', 'left_little_finger',
                  'left_little_finger2')

        # Choose mode of finger sensing to simulate; two different modes have been
        # implemented in the OpenBionics hand at different times.
        # self.fingSenseMode = 'vnc'; # Finger sensors return position variance over 1 second.
        self.fingSenseMode = 'pos'; # Finger sensors return position.

        if self.fingSenseMode=='vnc':
                self.fingVncStoreSize = 20
        elif self.fingSenseMode=='pos':
                # Set calibration points used to interpolate sensor readings:
                # Positions used in simulator when hand is open/closed:
                self.fingPosSimOpen = dict(zip(allJoints,[0.0]*len(allJoints)))
                self.fingPosSimOpen['left_thumb'] = -1.57
                self.fingPosSimClose = {'left_thumb'   : 0.75,
                                        'left_thumb2'  : 0.1,
                                        'left_index_finger': -0.6,
                                        'left_index_finger2': -0.7,
                                        'left_mid_finger': -0.6,
                                        'left_mid_finger2': -0.7,
                                        'left_anular_finger': -0.6,
                                        'left_anular_finger2': -0.9,
                                        'left_little_finger': -0.6,
                                        'left_little_finger2': -0.9
                                       }
                # Corresponding readings from actual sensors (approx):
                self.fingPosSensOpen = {'left_thumb'   : 3500,
                                        'left_thumb2'  : 0,
                                        'left_index_finger': 1700,
                                        'left_index_finger2': 0,
                                        'left_mid_finger': 1620,
                                        'left_mid_finger2': 0,
                                        'left_anular_finger': 2020,
                                        'left_anular_finger2': 0,
                                        'left_little_finger': 1620,
                                        'left_little_finger2': 0
                                       }
                self.fingPosSensClose = {'left_thumb'   : 8070,
                                        'left_thumb2'  : 0,
                                        'left_index_finger': 7610,
                                        'left_index_finger2': 0,
                                        'left_mid_finger': 7780,
                                        'left_mid_finger2': 0,
                                        'left_anular_finger': 7120,
                                        'left_anular_finger2': 0,
                                        'left_little_finger': 7120,
                                        'left_little_finger2': 0
                                       }
                                       
        else:
                raise ValueError('Unrecognised fingSenseMode')

        self.active_command = bert2_core_msgs.msg.HandCommand.RELEASE_COMMAND
        
        # Establish publisher (actuation) and subscriber (sensing) for each joint:
        self.jPubs = emptyObj()
        self.jSubs = emptyObj()
        self.last_vnc_update = emptyObj()
        self.fingPosStore = emptyObj()
        self.fingSens = emptyObj()
        for j in allJoints:
                if self.fingSenseMode == 'vnc':
                        setattr(self.last_vnc_update, j, 0.0)
                        setattr(self.fingPosStore, j, [0.0]*self.fingVncStoreSize)
                setattr(self.jPubs,j,rospy.Publisher(ns+j+'_flex_joint_posControlr/command', std_msgs.msg.Float64, queue_size=1,latch=True))
                setattr(self.jSubs,j,rospy.Subscriber(ns+j+'_flex_joint_posControlr/state', control_msgs.msg.JointControllerState, self._on_hand_JC_state, callback_args=j, queue_size=1, tcp_nodelay=True))

        hand_command_sub = rospy.Subscriber(ns+'hands/command', bert2_core_msgs.msg.HandCommand, self._on_hand_command, queue_size=1, tcp_nodelay=True)

        self.left_hand_state_pub = rospy.Publisher(ns+'hands/left_state', bert2_core_msgs.msg.HandState, queue_size=1,latch=True)


    #-----------------------------------------------------------------------------------------------
    def _on_joint_command(self,msg):
	
        # Publish a new set point for each joint:
        for idx, joint in enumerate(msg.names):

            # First check that we've set up the publisher:
            if joint not in self.jointPubs:
                self._add_pub(joint)

            self.jointPubs[joint].publish(std_msgs.msg.Float64(msg.angles[idx]))


    #-----------------------------------------------------------------------------------------------
    def _add_pub(self,joint):
        # Add a publisher for this joint:

        # The topic will be inferred from the joint name.  Make sure
        # that the loaded controllers (in joint_controllers.yaml,
        # perhaps?) comply with the assumed naming structure.
        topic = '/bert2/'+joint+'_posControlr/command'
          
        print('Adding topic: '+topic)

        pub = rospy.Publisher(
            topic,
            std_msgs.msg.Float64,
            queue_size=1,
            latch=True)
        
        self.jointPubs[joint] = pub



    #-----------------------------------------------------------------------------------------------
    def _on_hand_command(self,msg):
	
        # Deal with left hand command:

        command = msg.left_hand_command

        if command == bert2_core_msgs.msg.HandCommand.RELEASE_COMMAND:

                self.active_command = bert2_core_msgs.msg.HandCommand.RELEASE_COMMAND

		self.jPubs.left_thumb.publish(-1.57)
		self.jPubs.left_thumb2.publish(0.0)
		self.jPubs.left_index_finger.publish(0.0)
		self.jPubs.left_index_finger2.publish(0.0)
		self.jPubs.left_mid_finger.publish(0.0)
		self.jPubs.left_mid_finger2.publish(0.0)
		self.jPubs.left_anular_finger.publish(0.0)
		self.jPubs.left_anular_finger2.publish(0.0)
		self.jPubs.left_little_finger.publish(0.0)
		self.jPubs.left_little_finger2.publish(0.0)

	elif command == bert2_core_msgs.msg.HandCommand.GRASP_COMMAND:

                self.active_command = bert2_core_msgs.msg.HandCommand.GRASP_COMMAND

		self.jPubs.left_thumb2.publish(0.0)
		self.jPubs.left_thumb.publish(0.5)

		self.jPubs.left_mid_finger2.publish(-0.7)
		self.jPubs.left_index_finger2.publish(-0.7)
		
                rospy.sleep(0.1)

		self.jPubs.left_thumb.publish(0.75)

		self.jPubs.left_index_finger.publish(-1.5)
		self.jPubs.left_mid_finger.publish(-1.5)

		self.jPubs.left_anular_finger2.publish(-1.5)
		self.jPubs.left_little_finger2.publish(-1.5)
		self.jPubs.left_anular_finger.publish(-1.5)
		self.jPubs.left_little_finger.publish(-1.5)

        # TODO: Deal with right hand command (as above).



    #-----------------------------------------------------------------------------------------------
    def _on_hand_JC_state(self,msg,joint):

        if self.fingSenseMode=='vnc':
            # Calculate position variance in each finger over the last 1 second.

            vnc_update_step = 0.05 # 20 Hz update rate

            now = rospy.get_time()
            if ( now-getattr(self.last_vnc_update,joint) > vnc_update_step ):
                # Store new value:
                oldStore = getattr(self.fingPosStore,joint)
                store = oldStore[1:]+[msg.process_value]
                setattr(self.fingPosStore,joint,store)

                # Incremental calculation of position variance, as used in the hand's embedded controller.
                # Based on https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Online_algorithm
                # This bit seems inefficient, but that's how it's done in the
                # embedded controller (for now at least).
                mean = 0.0
                variance = 0.0
                minPeak = float("inf");
                maxPeak = -minPeak;

                for i in range(0,self.fingVncStoreSize-1):
                        # Calculate the variance incrementally
                        delta = store[ i ] - mean;
                        mean = mean + delta/(i + 1.0)
                        variance = variance + delta*(store[ i ] - mean)

                        # Keep track of the minimum and maximum values
                        if ( store[ i ] < minPeak ):
                                minPeak = store[ i ]
        
                        if ( store[ i ] > maxPeak ):
                                maxPeak = store[ i ]
    
                variance = variance*100 / (self.fingVncStoreSize - 1)
                setattr(self.fingSens,joint,variance)

                setattr(self.last_vnc_update,joint,now)

        elif self.fingSenseMode=='pos':
                # Interpolate finger position to match observed output range of embedded sensors:
                sensVal = self.fingPosSensOpen[joint] +    \
                          (msg.process_value-self.fingPosSimOpen[joint]) *    \
                          (self.fingPosSensClose[joint]-self.fingPosSensOpen[joint]) /    \
                           (self.fingPosSimClose[joint]-self.fingPosSimOpen[joint])
                setattr(self.fingSens,joint,sensVal)

        else:
                raise ValueError('Unrecognised fingSenseMode')

    #-----------------------------------------------------------------------------------------------
    def pub_hand_state(self):
        
        msg = bert2_core_msgs.msg.HandState()
        msg.is_alive = True
        msg.active_command = self.active_command
        msg.command_complete = True # Placeholder.
        msg.palm_pressure = 0.0
        msg.index_finger_prox_current = self.fingSens.left_index_finger+self.fingSens.left_index_finger2
        msg.middle_finger_trigger_current = 0.0
        msg.index_finger_trigger_current = 0.0
        msg.middle_finger_prox_current = self.fingSens.left_mid_finger+self.fingSens.left_mid_finger2
        msg.ring_finger_current = self.fingSens.left_anular_finger+self.fingSens.left_anular_finger2
        msg.small_finger_current = self.fingSens.left_little_finger+self.fingSens.left_little_finger2
        msg.thumb_oppose_current = self.fingSens.left_thumb+self.fingSens.left_thumb2
        msg.thumb_flex_current = 0.0

        self.left_hand_state_pub.publish(msg)        





#---------------------------------------------------------------------------------------------------
class emptyObj: pass




#---------------------------------------------------------------------------------------------------
def tempFixes():

    ## Echo /bert2/joint_states to /joint_states, where the move_group node expects to read joint states.
    ## As suggested at https://groups.google.com/a/rethinkrobotics.com/forum/#!msg/brr-users/P890sqFxpBo/7DSF_cOcUC8J
    ## it should be possible to just remap /joint_states to /bert2/joint_states when launching the
    ## move_group node, but that doesn't seem to work at the moment.  This workaround does the trick,
    ## but it means all the joint_states are getting published twice as much as they need to be.
    print( "    Echoing /bert2/joint_states to /joint_states for move_group node... " )
    p = rospy.Publisher(
        '/joint_states',
        sensor_msgs.msg.JointState,
        queue_size=1,
        latch=True)
    s = rospy.Subscriber(
        '/bert2/joint_states',
        sensor_msgs.msg.JointState,
        p.publish,
        queue_size=1)
    """
    collnPub = rospy.Publisher(
        '/collision_object',
        moveit_msgs.msg.CollisionObject)
    cylinder_object = moveit_msgs.msg.CollisionObject
    """


#---------------------------------------------------------------------------------------------------
def main():
    
    print( "Initializing node BERT2_gazebo_motor_controller... " )
    rospy.init_node('BERT2_gazebo_motor_controller')

    print( "Implementing some temporary fixes.  If you're looking to improve performance, you might start here.")
    tempFixes()

    print( "Initializing BERT2_gazebo_motor_controller..." )                              
    BGMC = BERT2GazeboMotorController()
    # Don't let the hand state publishing start before BGMC's run a full cycle.
    rospy.sleep(3)

    print( "Running. Ctrl-c to quit" )
    #rospy.spin()
    rate = rospy.Rate(20) # 20hz to match embedded hand controller.
    tick = 0
    while not rospy.is_shutdown():
        BGMC.pub_hand_state()
        rate.sleep()

#---------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
