#!/usr/bin/env python

"""
This script converts a list of high-level commands in a *.txt file into a state machine. This state machine is the human element in the simulator. The state types available are: send signal, receive signal, positioning (setting gaze, pressure and location), and timing out. """

"""
Written by Dejanira Araiza-Illan, February 2015
v2 by David Western, April 2016
"""
import sys
import rospy
import smach
import smach_ros
import re
import os
import math
import random
from bert2_gazebo.msg import *
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from std_msgs.msg import Int8
from bert2_gazebo import HUMAN_LowLevel

instructions=[]
data=[]
reception=0
x=0.0
y=0.0
z=0.0
global count_timeout
count_timeout=0

xx = random.randint(0, 100000) #Specify amount of seeds available
print 'GPL Seed: '+ str(xx)
random.seed(xx)

#--------------------------------------------------------------------------------------------------------
class Send(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'],input_keys=['c_in'],output_keys=['c_out'])

    def execute(self, userdata):
        h_signaling = rospy.Publisher('human_signals', Human, queue_size=1, latch=True) 
	if data[userdata.c_in] == '10': #activateRobot signal
		rospy.sleep(2)	
		h_signaling.publish(1, 0) 
		rospy.sleep(0.2)
	elif data[userdata.c_in] == '01': #humanIsReady signal
		rospy.sleep(2)		
		h_signaling.publish(0, 1) 
		rospy.sleep(0.2)	
	userdata.c_out=userdata.c_in+1
	return 'outcome1'

#---------------------------------------------------------------------------------------------------------
class Receive(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3'],input_keys=['c_in'],output_keys=['c_out'])

    def execute(self, userdata):
	global reception
        global count_timeout
	reception = 0
	rospy.sleep(0.2)
	rospy.Subscriber("robot_signals", Robot, callback)
	if reception == 1:
		userdata.c_out=userdata.c_in+1
		return 'outcome1'
        elif count_timeout<500:
        	count_timeout += 1
		userdata.c_out=userdata.c_in
		return 'outcome2'
	else:
		userdata.c_out=userdata.c_in+1
		print 'Timed out in loop'
		return 'outcome3'

def callback(data):
	global reception
	if data.informedHuman == 1:
		reception = 1

#--------------------------------------------------------------------------------------------------------
class Positioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'],input_keys=['c_in'],output_keys=['c_out'])

    def execute(self, userdata):
	global countpos
	temp=data[userdata.c_in]
	if temp == 'G1': #Set values for "good gaze"
		one_n_mapping('g',1,0)
	elif temp == 'G0': #Set values for "bad gaze"
		one_n_mapping('g',0,0)
	elif temp == 'P1':  #Set values for "good pressure"
		one_n_mapping('p',1,0)
	elif temp == 'P0': #Set values for "bad pressure"
		one_n_mapping('p',0,0)
	elif temp == 'L1': #Set values for "good location"
		one_n_mapping('l',1,0)
	elif temp == 'L0': #Set values for "bad location"
		one_n_mapping('l',0,0)
	userdata.c_out=userdata.c_in+1
        return 'outcome1'
        

def one_n_mapping(gpl,g_or_b,s_or_c):
	if gpl == 'g':
		if s_or_c == 1:
			offset = sample_corner_case(0.2,0.1,0.5)
			distance = sample_corner_case(0.6,0.5,0.8)
			angle = sample_corner_case(10.0,0,50)
		else:
			if g_or_b == 1:
				offset = sample_variable(0.1,0.2)
				distance = sample_variable(0.5,0.6)
				angle = sample_variable(-5.0,5.0)
			else:
				offset = sample_variable(0.2,0.5)
				distance = sample_variable(0.6,0.8)
				angle = sample_variable(40.0,50.0)
		pub = rospy.Publisher('gaze', Gaze, queue_size=1,latch=True)
		pub.publish(offset,distance,angle)
                head_height = 1.5+offset
                humanLL.set_head_height(head_height)
		rospy.sleep(0.2)
                angles = humanLL.angles_to_object("object")
                new_angles = (angles[0],angles[1],angles[2]+angle*3.14159/180)  # Add random offset to yaw angle
                humanLL.set_gaze_angles(new_angles)
	elif gpl == 'p':
		if g_or_b == 1:
			pub = rospy.Publisher("pressure_e2", Int8, queue_size=1,latch=True)
			pub.publish(1)
			rospy.sleep(0.2)
                        Fx = sample_variable(1.0,15.0)
                        print "Fx",Fx
                        Fy = 0.0
                        Fz = sample_variable(1.0,10.0)
                        print "Fz",Fz
                        Tx = 0.0 
                        Ty = 0.0
                        Tz = 0.0
                        dur = 3.0
                        humanLL.pull_object(Fx,Fy,Fz,Tx,Ty,Tz,dur)
		else:
			pub = rospy.Publisher("pressure_e2", Int8, queue_size=1,latch=True)
			pub.publish(0)
			rospy.sleep(0.2)
	elif gpl == 'l':
		if s_or_c == 1:
			rospy.sleep(0.5)
			rospy.Subscriber("piece_location", Point,check_point1) 
			rospy.sleep(0.2)
		else:
			if g_or_b == 1:
                                humanLL.move_hand_to_object()
                                hand_speed = 0.0
                                humanLL.set_hand_speed(hand_speed)				
			else:
                                humanLL.set_hand_distance(0.5+sample_variable(0.0,0.5),'object')
                                hand_speed = 0.0
                                humanLL.set_hand_speed(hand_speed)
		pub = rospy.Publisher('location',Location, queue_size=1,latch=True)
		pub.publish(x,y,z)
		rospy.sleep(0.2)

		

def sample_variable(min_value, max_value): #Sample within the range with uniform distribution
	return random.uniform(min_value, max_value)


def sample_corner_case(value,min_value,max_value): #Sample preferably near a specific value
	y = min_value-1
	while (y<min_value or y>max_value):
		y = random.gauss(value, value*0.2)
	return y
	
def check_point1(data):
	global x
	global y
	global z
	x = data.x+0.05
	y = data.y-0.1
	z = data.z-0.05
	
def check_point3(data):
	global x
	global y
	global z
	x = data.x+0.5+sample_variable(0.0,0.5)
	y = data.y-0.5-sample_variable(0.0,0.5)
	z = data.z+0.5+sample_variable(0.0,0.5)


#------------------------------------------------------------------------------------------
class Engaged(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'],input_keys=['c_in'],output_keys=['c_out'])

    def execute(self, userdata):
	if data[userdata.c_in]=='000': #Disengaged
		one_n_mapping('g',0,0)
		one_n_mapping('p',0,0)
		one_n_mapping('l',0,0)
	userdata.c_out=userdata.c_in+1
        return 'outcome1'
        
#------------------------------------------------------------------------------------------
class Delay(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'],input_keys=['c_in'],output_keys=['c_out'])

    def execute(self, userdata):
	rospy.sleep(0.05*int(userdata.c_in))
	userdata.c_out=userdata.c_in+1
	return 'outcome1'
	
#-----------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------------
def main(name_file):
    rospy.init_node('human', anonymous=True)
    
    global humanLL
    humanLL = HUMAN_LowLevel()

    #Reset conditions in Gazebo
    humanLL.reset_head_hand()

    sm = smach.StateMachine(outcomes=['end'])
    sm.userdata.sm_c = 0

    with sm:
	#Create machine by reading instruction list
	global instructions
	global data
	for num,command in enumerate(open(os.getcwd()+'/tests/stimuli/'+name_file+'.txt','r')): #ADD READING FILE
		if re.search("send_signal",command): #if the command is to send a signal
			if re.search("activateRobot",command):
				data.append('10')
				instructions.append('send')
			elif re.search("humanIsReady",command):
				data.append('01')
				instructions.append('send')
		elif re.search("set_param",command): #if the command is to set the value of a variable
			if re.search("h_onTask=1",command):
				data.append('111') 
				instructions.append('engaged')
			elif re.search("h_gazeOk=1",command):
				instructions.append('positioning')
				data.append('G1')
			elif re.search("h_pressureOk=1",command):
				instructions.append('positioning')
				data.append('P1')
			elif re.search("h_locationOk=1",command):
				instructions.append('positioning')
				data.append('L1')
			elif re.search("h_gazeOk=0",command):
				instructions.append('positioning')
				data.append('G0')
			elif re.search("h_pressureOk=0",command):
				instructions.append('positioning')
				data.append('P0')
			elif re.search("h_locationOk=0",command):
				instructions.append('positioning')
				data.append('L0')
			elif re.search("h_onTask=0",command): #timeout
				instructions.append('engaged')
				data.append('000')
			elif re.search("time=",command):
				instructions.append('delay')
				delay_count = re.split("time=",command)
				data.append(delay_count[1])

		elif re.search("receive_signal",command): #if the command is to receive a signal
			if not(re.search("setv",command)):
				instructions.append('receive')
				data.append('')
		else:
			instructions.append('')

	for i in range(len(instructions)-1):
		if instructions[i] == 'send':
			if instructions[i+1]== 'send':
				smach.StateMachine.add('Send'+str(i), Send(), 
		                transitions={'outcome1':'Send'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
	       		elif instructions[i+1]== 'receive':
				smach.StateMachine.add('Send'+str(i), Send(), 
		                transitions={'outcome1':'Receive'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
			elif instructions[i+1]== 'positioning':
				smach.StateMachine.add('Send'+str(i), Send(), 
		                transitions={'outcome1':'Positioning'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
			elif instructions[i+1]== 'engaged':
				smach.StateMachine.add('Send'+str(i), Send(), 
		                transitions={'outcome1':'Engaged'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
		        elif instructions[i+1]== 'delay':
				smach.StateMachine.add('Send'+str(i), Send(), 
		                transitions={'outcome1':'Delay'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
		if instructions[i] == 'receive':
			if instructions[i+1]== 'send':
				smach.StateMachine.add('Receive'+str(i), Receive(), 
		                transitions={'outcome1':'Send'+str(i+1),'outcome2':'Receive'+str(i),'outcome3':'end'},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
	       		elif instructions[i+1]== 'receive':
				smach.StateMachine.add('Receive'+str(i), Receive(), 
		                transitions={'outcome1':'Receive'+str(i+1),'outcome2':'Receive'+str(i),'outcome3':'end'},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
			elif instructions[i+1]== 'positioning':
				smach.StateMachine.add('Receive'+str(i), Receive(), 
		                transitions={'outcome1':'Positioning'+str(i+1),'outcome2':'Receive'+str(i),'outcome3':'end'},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
			elif instructions[i+1]== 'engaged':
				smach.StateMachine.add('Receive'+str(i), Receive(), 
		                transitions={'outcome1':'Engaged'+str(i+1),'outcome2':'Receive'+str(i),'outcome3':'end'},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
		        elif instructions[i+1]== 'delay':
				smach.StateMachine.add('Receive'+str(i), Receive(), 
		                transitions={'outcome1':'Delay'+str(i+1),'outcome2':'Receive'+str(i),'outcome3':'end'},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
		if instructions[i] == 'positioning':
			if instructions[i+1]== 'send':
				smach.StateMachine.add('Positioning'+str(i), Positioning(), 
		                transitions={'outcome1':'Send'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
	       		elif instructions[i+1]== 'receive':
				smach.StateMachine.add('Positioning'+str(i), Positioning(), 
		                transitions={'outcome1':'Receive'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
			elif instructions[i+1]== 'positioning':
				smach.StateMachine.add('Positioning'+str(i), Positioning(), 
		                transitions={'outcome1':'Positioning'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
			elif instructions[i+1]== 'engaged':
				smach.StateMachine.add('Positioning'+str(i), Positioning(), 
		                transitions={'outcome1':'Engaged'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
		        elif instructions[i+1]== 'delay':
				smach.StateMachine.add('Positioning'+str(i), Positioning(), 
		                transitions={'outcome1':'Delay'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
		if instructions[i] == 'engaged':
			if instructions[i+1]== 'send':
				smach.StateMachine.add('Engaged'+str(i), Engaged(), 
		                transitions={'outcome1':'Send'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
	       		elif instructions[i+1]== 'receive':
				smach.StateMachine.add('Engaged'+str(i), Engaged(), 
		                transitions={'outcome1':'Receive'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
			elif instructions[i+1]== 'positioning':
				smach.StateMachine.add('Engaged'+str(i), Engaged(), 
		                transitions={'outcome1':'Positioning'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
			elif instructions[i+1]== 'engaged':
				smach.StateMachine.add('Engaged'+str(i), Engaged(),  
		                transitions={'outcome1':'Engaged'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
		        elif instructions[i+1]== 'delay':
				smach.StateMachine.add('Engaged'+str(i), Engaged(),  
		                transitions={'outcome1':'Delay'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
		if instructions[i] == 'delay':
			if instructions[i+1]== 'send':
				smach.StateMachine.add('Delay'+str(i), Delay(), 
		                transitions={'outcome1':'Send'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
	       		elif instructions[i+1]== 'receive':
				smach.StateMachine.add('Delay'+str(i), Delay(), 
		                transitions={'outcome1':'Receive'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
			elif instructions[i+1]== 'positioning':
				smach.StateMachine.add('Delay'+str(i), Delay(),
		                transitions={'outcome1':'Positioning'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
			elif instructions[i+1]== 'engaged':
				smach.StateMachine.add('Delay'+str(i), Delay(),
		                transitions={'outcome1':'Engaged'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
		        elif instructions[i+1]== 'delay':
				smach.StateMachine.add('Delay'+str(i), Delay(),
		                transitions={'outcome1':'Delay'+str(i+1)},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
	if instructions[len(instructions)-1] == 'send':
		smach.StateMachine.add('Send'+str(len(instructions)-1), Send(), 
		transitions={'outcome1':'end'},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
	elif instructions[len(instructions)-1] == 'receive':
		smach.StateMachine.add('Receive'+str(len(instructions)-1), Receive(), 
		transitions={'outcome1':'end','outcome2':'Receive'+str(len(instructions)-1),'outcome3':'end'},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
	elif instructions[len(instructions)-1] == 'positioning':
		smach.StateMachine.add('Positioning'+str(len(instructions)-1), Positioning(), 
		transitions={'outcome1':'end'},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
	elif instructions[len(instructions)-1] == 'engaged':
		smach.StateMachine.add('Engaged'+str(len(instructions)-1), Engaged(), 
		transitions={'outcome1':'end'},remapping={'c_in':'sm_c', 'c_out':'sm_c'})
	elif instructions[len(instructions)-1] == 'delay':
		smach.StateMachine.add('Delay'+str(len(instructions)-1), Delay(), 
		transitions={'outcome1':'end'},remapping={'c_in':'sm_c', 'c_out':'sm_c'})

 
    outcome = sm.execute()


#------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
	try:
		main(sys.argv[1])
	except rospy.ROSInterruptException: #to stop the code when pressing Ctr+c
        	pass

