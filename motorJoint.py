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
		self.power = 0.0
		self.currentLog = []
		self.torqueLog = []
		self.velocityLog = []
		self.positionLog = []

	def getTorque(self):
		[force, torque] = self.physics.GetJointForceTorque(self.joint)
		#return dot(torque, self.joint.GetAxis())/self.G
		return torque/self.G

	def getVelocity(self):
		v = self.joint.GetVelocities()[0]*self.G
		self.velocityLog.append(v)
		return v

	def getCurrent(self, t):
		t = sqrt(t[0]**2+t[1]**2+t[2]**2)
		self.torqueLog.append(abs(t))
		return t/self.Kt

	def getVoltage(self, i):
		return abs(self.getVelocity())*self.Kt + (i+self.nlc)*self.R

	def setInterpolationVelocity(self, v):
		if v > 0:
			self.interpolationVel = v

	def getPower(self, step, t):
		i = self.getCurrent(t)
		p = i*self.getVoltage(i)*step/3600.0
		self.power += p
		self.currentLog.append(p/54.0)
		self.positionLog.append(self.joint.GetValue(0))
		return p
