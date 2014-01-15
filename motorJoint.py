#!/usr/bin/env python
from numpy import dot, sqrt

class MotorJoint:
	def __init__(self, robot, ohName, mName, gearRatio, torqueConstant, noLoadCurrent, resistance, vel = .3):
		self.physics = robot.GetEnv().GetPhysicsEngine()
		self.openHuboName = ohName
		self.maestroName = mName
		self.joint = robot.GetJoint(self.openHuboName)
		self.G = gearRatio
		self.Kt = torqueConstant
		self.R = resistance
		self.nlc = noLoadCurrent
		self.interpolationVel = vel

	def getTorque(self):
		[force, torque] = self.physics.GetJointForceTorque(self.joint)
		#return dot(torque, self.joint.GetAxis())/self.G
		return torque/self.G

	def getVelocity(self):
		return self.joint.GetVelocities()[0]*self.G

	def getCurrent(self, t):
		return sqrt(t[0]**2+t[1]**2+t[2]**2)/self.Kt

	def getVoltage(self, i):
		return abs(self.getVelocity())*self.Kt + (i+self.nlc)*self.R

	def setInterpolationVelocity(self, v):
		if v > 0:
			self.interpolationVel = v