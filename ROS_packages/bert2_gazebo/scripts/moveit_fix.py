#!/usr/bin/env python

import rospy
import sensor_msgs.msg

def main():
  
    rospy.init_node('Moveit_fixes')

    ## Mapping /bert2/joint_states to /joint_states, where the move_group node expects to read joint states.
    ## As suggested at https://groups.google.com/a/rethinkrobotics.com/forum/#!msg/brr-users/P890sqFxpBo/7DSF_cOcUC8J

    p = rospy.Publisher('/joint_states', sensor_msgs.msg.JointState, queue_size=1, latch=True)
    s = rospy.Subscriber('/bert2/joint_states', sensor_msgs.msg.JointState, p.publish, queue_size=3)


    rospy.spin()

#---------------------------------------------------------------------------------------------------
if __name__ == "__main__":
	try:
    		main()
    	except rospy.ROSInterruptException:
		pass
