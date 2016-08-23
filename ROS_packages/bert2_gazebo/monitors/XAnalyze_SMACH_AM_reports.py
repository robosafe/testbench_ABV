#!/usr/bin/env python

"""
This code cross analyzes assertion monitor reports, to allow examination of results and coverage.  For example, it can list all traces for which monitorA returned True and monitorB returned false.  It will also list all tests for which no result was returned on a particular monitor, i.e. the assertion was not covered by that test, assuming the monitor was working properly.

From the terminal, cd to the folder containing the reports.  Then run

python YOUR/PATH/TO/THIS/MODULE/XAnalyze_SMACH_AM_reports.py nTests _monitorA True _monitorB False

nTests is the number of tests to look for.  It is assumed that the tests start at 1.
'_monitorA' and '_monitorB' may be replaced with whatever text follows 'assertion' in the file name of the report files of interest.

Written by David Western, June 2016
"""

import os
import re

def analyze(args):

  num_traces_expected = int(args[1])
  traces_expected = [str(x+1) for x in xrange(num_traces_expected)]
  #traces_expected = ['10'+str(x+1) for x in xrange(num_traces_expected)] # Searching tests from a different start point.
  #traces_expected = ['3', '25', '44', '54', '76', '88', '93'] # Searching a specific set of tests.
  #num_traces_expected = len(traces_expected)

  props = args[2::2]
  propVals = args[3::2]

  print props
  print propVals
  print

  #fOut = open('AM_analyze_out.txt','a')

  traces_satisfying = [set() for _ in xrange(20)]

  for k in xrange(len(props)):
    traces_checked = []

    traces_satisfying[k] = set()
    
    reportFile = 'assertion'+props[k]+'.txt'

    for i, line in enumerate(open(os.getcwd()+'/'+reportFile, 'r')):
        failFound = 0
        trueFound = 0
        for match in re.finditer("Assertion",line):
	        trace_no = re.split("at trace ", line)
                trace_no = re.split(": ",trace_no[1])
                trace_no = trace_no[0]

                if trace_no not in traces_expected:
                        continue

                if trace_no not in traces_checked:
                        traces_checked.append(trace_no)
                        failFound = 0
                        trueFound = 0

                if not (re.search("False",line)==None):
                    if failFound==0:
                        failFound = 1
                elif not (re.search("True",line)==None):
                    if trueFound==0 and failFound==0:
                        trueFound = 1
                else:
                        print "Neither True nor False found at line",i
                        print line
                        fOut.write("Neither True nor False found at line "+i+" in "+reportFile+".\n")
                        fOut.write("See-> "+line+"\n")
            
    
        if (failFound==1 and propVals[k]=='False') or (failFound==0 and trueFound==1 and propVals[k]=='True'):
                traces_satisfying[k].add(trace_no)
                               
                             
    traces_missing = list(set(traces_expected)-set(traces_checked))

    pctTracesMissing = 100*len(traces_missing)/float(num_traces_expected)
    print len(traces_missing),"(",pctTracesMissing,"percent) traces missing for",props[k],":",sorted([int(x) for x in traces_missing])
    #fOut.write("Traces missing for "+props[k]+":"+str(traces_missing)+"\n")

    if k==0:
        satSet = traces_satisfying[k]
    else:
        satSet = satSet.intersection(traces_satisfying[k])

  sortSatSet = sorted([int(x) for x in satSet])
  print
  print "Set satisfying all specified properties:"
  print sortSatSet
  print "# in set:",len(sortSatSet)
  print "satisfaction rate:",len(sortSatSet)/float(num_traces_expected)

if __name__ == "__main__":
    import sys
    analyze(sys.argv)

