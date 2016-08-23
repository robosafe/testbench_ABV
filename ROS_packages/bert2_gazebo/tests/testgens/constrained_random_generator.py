#!/usr/bin/env python
"""
This script generates a number of random high-level stimulus, into files. The first sequence of actions (constraint) is enforced for each file. An alphabet is needed for the high-level stimulus part. Ranges are needed for variables that are not boolean. 

n -- number of tests (e.g., n = 1 gives 1 test)
s_vs_c -- sampling the variables (0) or sampling around constraints (1)

Written by Dejanira Araiza-Illan, July 2015. 
"""

import random
import sys

#Alphabet for high-level stimulus
#Syntax: [hl actions] [Options for hl1] .. [Options for hln]
list_hl_actions = [['send_signal', 'receive_signal', 'set_param'],['activateRobot', 'humanIsReady','setv'],['informHumanOfHandoverStart', 'setv'], ['h_onTask', 'h_gazeOk', 'setg', 'h_undefGaze', 'h_pressureOk', 'setp', 'h_undefPress', 'h_locationOk', 'setl', 'h_undefLoc','time']]

boolean_variables = ['h_onTask', 'h_gazeOk', 'setg', 'h_undefGaze', 'h_pressureOk', 'setp', 'h_undefPress', 'h_locationOk', 'setl', 'h_undefLoc']
ranges_var = [['time', 1.0,100.0,40.0]]

def main(n,s_vs_c):
	for i in range (0,int(n)): #Number of tests required
		#Open files to write
		f = open('stimulus_10'+str(i+1)+'.txt', 'w')
		f.write('send_signal\tactivateRobot\n')
		f.write('set_param\ttime=40\n')
		f.write('receive_signal\tinformHumanOfHandoverStart\n')
		f.write('send_signal\thumanIsReady\n')
		f.write('set_param\ttime=10\n')
		f.write('send_signal\tsetv\n')
		x = random.randint(0, 100000) #Specify amount of seeds available
		print 'Seed '+ str(i+1)+':'+ str(x)
		random.seed(x)
		k = random.randint(1, 10) #Specify length of random traces
		sample_actions(k,list_hl_actions,ranges_var,boolean_variables,s_vs_c,f) 
	
# --------------------------------------------------------------------
def sample_variable(min_value, max_value): #Sample within the range with uniform distribution
	return random.uniform(min_value, max_value)

#----------------------------------------------------------------------
def sample_corner_case(value,min_value,max_value): #Sample preferably near a specific value
	y = min_value-1
	while (y<min_value or y>max_value):
		y = random.gauss(value, value*0.2)
	return y

def sample_actions(no_actions, alphabet,variables_w_range,boolean_var,s_or_c,f):
	the_actions = alphabet[0]
	the_param = alphabet[1:len(alphabet)]
	for j in range(0,no_actions):
		select = random.randint(0, len(the_actions)-1)
		curr_param = the_param[select]
		ch_parameter = curr_param[random.randint(0,len(curr_param)-1)]
		for ll, element in enumerate(variables_w_range):
			if ch_parameter == element[0]:
				if s_or_c == 1:
					ch_parameter = ch_parameter+ '=' +str(int(sample_corner_case(element[3],element[1],element[2])))
				else:
					ch_parameter = ch_parameter+ '=' +str(int(sample_variable(element[1],element[2])))
		for ll, element in enumerate(boolean_var):
			if ch_parameter == element:
				b_value = random.randint(0,1)
				if b_value == 0:
					ch_parameter = ch_parameter + '=1'
				else:
					ch_parameter = ch_parameter + '=0'
		f.write(the_actions[select] +'\t'+ ch_parameter+'\n')
	

	
if __name__ == '__main__':
    main(sys.argv[1],sys.argv[2]) #Arguments: number of trace files, random parameters from min-max ranges or sampling close to a boundary value

