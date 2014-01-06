#!/usr/bin/env python
from numpy import dot

class MotorJoint:
	def __init__(self, robot, motorConstant, gearRatio, resistance, jointName, vel = .3):
		self.physics = robot.GetEnv().GetPhysicsEngine()
		self.name = jointName
		self.joint = robot.GetJoint(jointName)
		self.G = gearRatio
		self.K = motorConstant
		self.R = resistance
		self.interpolationVel = vel

	def getName(self):
		return self.name

	def getTorque(self):
		[force, torque] = self.physics.GetJointForceTorque(self.joint)
		return dot(torque, self.joint.GetAxis())/self.G

	def getVelocity(self):
		return self.joint.GetVelocities()[0]*self.G

	def getCurrent(self, t):
		return t/self.K

	def getVoltage(self, t):
		return abs(self.getVelocity())*self.K + t*self.R

	def setInterpolationVelocity(self, v):
		if v > 0:
			self.interpolationVel = v

	def getInterpolationVelocity(self):
		return self.interpolationVel