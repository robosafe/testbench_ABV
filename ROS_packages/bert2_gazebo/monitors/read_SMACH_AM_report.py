#!/usr/bin/env python

"""
This script reads a report generated by an SMACH assertion monitor to count the
number of tests for which the monitor has passed or failed.

Written by David Western, August 2015
"""

import os
import re

#traces_expected = ['10'+str(x+1) for x in xrange(100)]
traces_expected = [str(x+1) for x in xrange(100)]
#traces_expected = ['103', '1025', '1044', '1054', '1076', '1088', '1093']
#traces_expected = ['3', '25', '44', '54', '76', '88', '93']
#num_traces_expected = 55
#traces_expected = [str(x+1) for x in xrange(num_traces_expected)]

# reportFiles = ['assertion2.txt','assertion3.txt','assertion4.txt','assertion5.txt','assertion6.txt','assertion_collision_speed.txt','assertion_collision_speed_human.txt','assertion_collision_speed_self.txt']
#reportFiles = ['assertion_no_accidental_drop.txt']
#reportFiles = ['assertion_no_accidental_drop.txt','assertion_handover_success.txt','assertion_no_false_pos_gaze.txt','assertion_no_false_neg_gaze.txt','assertion_no_false_pos_press.txt','assertion_no_false_neg_press.txt','assertion_no_false_pos_loc.txt','assertion_no_false_neg_loc.txt']
reportFiles = ['assertion2.txt','assertion3.txt','assertion4.txt','assertion5.txt','assertion6.txt','assertion_collision_speed.txt','assertion_collision_speed_human.txt','assertion_collision_speed_self.txt','assertion_no_accidental_drop.txt','assertion_handover_success.txt','assertion_gaze_sensed_as_correct.txt','assertion_pressure_sensed_as_correct.txt','assertion_location_sensed_as_correct.txt']

fOut = open('AM_report_summary.txt','a')

for reportFile in reportFiles:
    traces_checked = []
    passes = 0
    fails = 0
    print
    print
    print
    print reportFile
    print os.getcwd()+'/'+reportFile
    fOut.write("\n\n\n"+reportFile+"\n"+os.getcwd()+"/"+reportFile+"\n")
    for i, line in enumerate(open(os.getcwd()+'/'+reportFile, 'r')):
        for match in re.finditer("Assertion",line):
	        trace_no = re.split("at trace ", line)
                trace_no = re.split(": ",trace_no[1])
                trace_no = trace_no[0]
                """if trace_no=='31':
                        print line
                        print i
                """
                if trace_no not in traces_expected:
                        continue

                if trace_no not in traces_checked:
                        traces_checked.append(trace_no)
                        failFound = 0
                        trueFound = 0

                if not (re.search("False",line)==None):
                    if failFound==0:
                        failFound = 1
                        fails += 1
                        if trueFound==1:
                                # Cancel the 'pass' counted for this trace:
                                passes -= 1
                elif not (re.search("True",line)==None):
                    if trueFound==0 and failFound==0:
                        trueFound = 1
                        passes += 1
                else:
                        print "Neither True nor False found at line",i
                        print line
                        fOut.write("Neither True nor False found at line"+i+"\n")
                        fOut.write(line+"\n")
                        
                             
        traces_missing = list(set(traces_expected)-set(traces_checked))

    print "passes:",passes
    print "fails:",fails
    print "traces missing:",traces_missing
    print "# of traces missing:",len(traces_missing)
    print "traces checked:",traces_checked
    fOut.write("passes: "+str(passes)+"\n")
    fOut.write("fails: "+str(fails)+"\n")
    fOut.write("traces missing: "+str(traces_missing)+"\n")
    fOut.write("# of traces missing: "+str(len(traces_missing))+"\n")
    fOut.write("traces checked: "+str(traces_checked)+"\n")




