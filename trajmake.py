#!/usr/bin/env python
from numpy import sin

N = .005/.001
timeStep = .001*N
V = .3
posStep = V*timeStep# .3rad/s * .001s/step = .003rad/step

''' arms2Horizontal.traj
f = open('arms2Horizontal.traj', 'w')
f.write("RSP LSP\n")

i = 0
while(i > -1.57):
	f.write(str(i) + " " + str(i) + "\n")
	i -= posStep

f.close()
'''

f = open('armsSinusoidal.traj', 'w')
f.write("RSR LSR\n")

# pos = .5sin(t)
A = .5
omega = 1
i = 0
for i in xrange(1000):
	rsr = -A*sin(omega*i*timeStep)
	lsr = A*sin(omega*i*timeStep)
	f.write(str(rsr) + " " + str(lsr) + "\n")