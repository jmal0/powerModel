#!/usr/bin/env python
from numpy import dot, sqrt

class MotorJoint:
	def __init__(self, robot, jointName, motorConstant, gearRatio, resistance, noLoadCurrent, vel = .3):
		self.physics = robot.GetEnv().GetPhysicsEngine()
		self.name = jointName
		self.joint = robot.GetJoint(jointName)
		self.G = gearRatio
		self.K = motorConstant
		self.R = resistance
		self.nlc = noLoadCurrent
		self.interpolationVel = vel

	def getName(self):
		return self.name

	def getTorque(self):
		[force, torque] = self.physics.GetJointForceTorque(self.joint)
		#return dot(torque, self.joint.GetAxis())/self.G
		return torque/self.G

	def getVelocity(self):
		return self.joint.GetVelocities()[0]*self.G

	def getCurrent(self, t):
		return sqrt(t[0]**2+t[1]**2+t[2]**2)/self.K

	def getVoltage(self, i):
		return abs(self.getVelocity())*self.K + (i+self.nlc)*self.R

	def setInterpolationVelocity(self, v):
		if v > 0:
			self.interpolationVel = v

	def getInterpolationVelocity(self):
		return self.interpolationVel