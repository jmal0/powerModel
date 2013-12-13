#!/usr/bin/env python
from numpy import *

timeStep = .005
V = .3
posStep = V*timeStep# .3rad/s * .001s/step = .003rad/step

# arms2Horizontal.traj
f = open('sr.traj', 'w')
f.write("RSR LSR\n")

i = 0
while(i > -1.57):
	f.write(str(i) + " " + str(-i) + "\n")
	i -= posStep

while(i < 0):
	f.write(str(i) + " " + str(-i) + "\n")
	i += posStep

f.close()

'''
f = open('armsSinusoidal.traj', 'w')
f.write("RSR LSR\n")

# pos = .5cos(t) +/- .5
A = .5
omega = 1
i = 0
for i in xrange(3000):
	rsr = A*cos(omega*i*timeStep)-.5
	lsr = -A*cos(omega*i*timeStep)+.5
	f.write(str(rsr) + " " + str(lsr) + "\n")
'''