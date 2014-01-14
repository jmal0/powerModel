#!/usr/bin/env python

import sys
from motorJoint import *

class PowerModel:
    def __init__(self, robot, num):
        self.motors = [] # List of motors
        self.totals = dict() # Adds each motor's torque readings; Average is taken and written to file
        self.num = 5 # Number of timesteps before writing to file
        self.names = []

        f = open('jointPower.txt','r')
        lines = f.readlines();
        del(lines[0])
        for line in lines:
            vals = line.split()
            if(vals[4] == "True"):
                motor = MotorJoint(robot, vals[0], float(vals[2]), float(vals[1]), float(vals[6]), float(vals[3]))
                self.names.append(vals[0])
                self.motors.append(motor)
                self.totals[motor] = 0.0
        f.close()

    def calcPowerUsage(self, step, q):
        usage = 0
        for motor in self.motors:
            torque = abs(self.totals[motor]/self.num)
            current = motor.getCurrent(torque)
            voltage = motor.getVoltage(current)
            if(motor.getName() == "RSP"):
                i = voltage*current/48
                print i
                q.write(str(i) + " ")
            self.totals[motor] = 0.0
            usage += voltage*current*step/3600.0
        return usage + .0148634*step # Add in battery drain by idle current draw

    # Add each motor's torque output
    def addTorques(self):
        for motor in self.motors:
            self.totals[motor] += motor.getTorque()

    def getMotor(self, jointName):
        for motor in self.motors:
            if(motor.name == jointName):
                return motor
        return None