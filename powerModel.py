#!/usr/bin/env python

import sys
from motorJoint import *

class PowerModel:
    def __init__(self, robot, num, q):
        self.motors = [] # List of motors
        self.num = 5 # Number of timesteps to add torque before calculating power 
        self.totals = dict() # Adds each motor's torque readings; Average is taken and used to calculate power
        self.mutableNames = [] # List of joints that can be set
        self.q = q

        f = open('/etc/hubo-ach/jointPower.txt','r')
        lines = f.readlines();
        del(lines[0])
        for line in lines:
            vals = line.split()

            motor = MotorJoint(robot, vals[0], vals[1], float(vals[3]), float(vals[4]), float(vals[5]), float(vals[6]))
            self.motors.append(motor)
            self.totals[motor] = 0.0

            if(vals[2] == "True"):
                self.mutableNames.append(vals[0])
        f.close()

    def calcPowerUsage(self, step):
        usage = 0
        for motor in self.motors:
            torque = abs(self.totals[motor]/self.num)
            current = motor.getCurrent(torque)
            voltage = motor.getVoltage(current)
            self.totals[motor] = 0.0
            if motor.maestroName == "RSR":
                self.q.write(str(voltage*current*step/3600.0))
            usage += voltage*current*step/3600.0
        return usage + .0148634*step # Add in battery drain by idle current draw

    # Add each motor's torque output
    def addTorques(self):
        for motor in self.motors:
            self.totals[motor] += motor.getTorque()

    # Get motor name corresponding to valid maestro name
    def getMotor(self, jointName):
        for motor in self.motors:
            if(motor.openHuboName == jointName):
                return motor
        return None

    # Get openHubo name corresponding to a valid maestro name
    def getName(self, jointName):
        for motor in self.motors:
            if motor.maestroName == jointName:
                if motor.openHuboName in self.mutableNames:
                    return motor.openHuboName
        return None