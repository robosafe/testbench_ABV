#!/usr/bin/env python

"""
BERT2 Joint Trajectory Controller
    This is a cut down and hacked around version of the joint tracjectory controller
    provided by Rethink Robotics for the Baxter robot
"""

import argparse

import rospy

from dynamic_reconfigure.server import Server

from bert2_interface.cfg import (
    PositionJointTrajectoryActionServerConfig,
)
from joint_trajectory_action.joint_trajectory_action import (
    JointTrajectoryActionServer,
)

#---------------------------------------------------------------------------------------------------
def start_server( rate, mode ):
    
    print( "Initializing node... " )
    rospy.init_node( "{0}_joint_trajectory_action_server".format( mode ) )
    print( "Initializing joint trajectory action server..." )

    if mode == "velocity":
        raise Exception( "Velocity mode not supported yet" )
        #dyn_cfg_srv = Server(VelocityJointTrajectoryActionServerConfig,
        #                     lambda config, level: config)
    else:
        dyn_cfg_srv = Server( PositionJointTrajectoryActionServerConfig,
                              lambda config, level: config )
                              
    jtas = JointTrajectoryActionServer( dyn_cfg_srv, rate, mode )

    def cleanup():
        jtas.clean_shutdown()

    rospy.on_shutdown( cleanup )
    print( "Running. Ctrl-c to quit" )
    rospy.spin()

#---------------------------------------------------------------------------------------------------
def main():
    arg_fmt = argparse.ArgumentDefaultsHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt)
    parser.add_argument(
        "-r", "--rate", dest="rate", default=100.0,
        type=float, help="trajectory control rate (Hz)"
    )
    #parser.add_argument(
    #    "-m", "--mode", default='position', choices=['position', 'velocity'],
    #    help="control mode for trajectory execution"
    #)
    mode = "position"
    
    args = parser.parse_args(rospy.myargv()[1:])
    start_server( args.rate, mode )

#---------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
