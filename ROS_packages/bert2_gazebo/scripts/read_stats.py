#!/usr/bin/env python

"""
This script reads stats.txt to count the number of tests achieving each level of coverage.

Written by David Western, August 2015
"""

import os
import re

covLevels = dict()

for i, line in enumerate(open(os.getcwd()+'/stats.txt', 'r')):
        for match in re.finditer("Covered percentage:",line):
		pctCov = re.split("Covered percentage:", line)
		pctCov = re.split("%",pctCov[1])
                pctCov = pctCov[0]
                if pctCov in covLevels:
                        covLevels[pctCov] += 1
                else:
                        covLevels[pctCov] = 1

print covLevels
