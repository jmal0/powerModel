#!/usr/bin/env python

import sys
from numpy import dot

# Populate lookup dictionaries from reference text file
const = dict() # Contains joint names and its motor's torque constant
ratio = dict() # Contains joint names and its motor's gear ratio
voltage = dict() # Contains joint names and the voltage they run off of
mutable = dict() # Contains joint names and a boolean variable indicatin if that joint's position can be set
altname = dict() # Contains trajectory joint names and the corresponding name that openhubo recognizes
totals = dict() # Adds each motor's torque readings; Average is taken and written to file
num = 5 # Number of timesteps before writing to file

f = open('jointPower.txt','r')
lines = f.readlines();
del(lines[0])
for line in lines:
    vals = line.split()
    ratio[vals[0]] = float(vals[1])
    const[vals[0]] = float(vals[2])
    voltage[vals[0]] = float(vals[3])
    mutable[vals[0]] = vals[4] == 'True'
    altname[vals[5]] = vals[0]
    totals[vals[0]] = 0.0
f.close()

# Calculate battery usage
def calcBatteryUsage(robot, step, q):
    usage = 0
    [power, torque] =  getPowerUsage(robot, step, q)
    for use in power:
        usage += use # old was 2892
    return usage + .000074317, torque # Add in battery drain by idle current draw

# Converts motor torque to current and calculates battery drain by each motor
def getPowerUsage(robot, step, q):
    power = [0]*len(const)
    torque = dict()
    j = 0
    for jointName in const:
        t = totals[jointName]/float(num)
        torque[jointName] = abs(t)
        i = abs(t)/const[jointName]
        if(jointName == "RSR"):
            q.write(str(t) + " ")
        totals[jointName] = 0.0
        power[j] = voltage[jointName]*i*step/3600.0
        j += 1
    return power, torque

# Add each motor's torque output
def addTorques(robot):
    physics = robot.GetEnv().GetPhysicsEngine()
    for jointName in const:
        totals[jointName] += getTorque(robot, jointName, physics)

# Calculate motor torque with physics engine
def getTorque(robot, jointName, physics):
    joint = robot.GetJoint(jointName)
    [force, torque] = physics.GetJointForceTorque(joint)
    return dot(torque, joint.GetAxis())/ratio[jointName]
