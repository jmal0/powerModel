#!/usr/bin/env python
from openravepy import *
from openhubo import *
import hubo_ach
import sys
import time
import numpy
import cmd
import readline, glob
COMMANDS = ['runTrajectory', 'getPosition', 'setPosition', 'stepSimulation', 'getVelocity', 'setVelocity']

from powerModel import * # Import for power computing and joint properties

# Hubo-ach status Logger to track computer and sim time and rate
class StatusLogger:
    """Simple and efficient status updater for the main loop"""
    def __init__(self,skipcount=100,init_time=0.0):
        self.t_last=init_time
        self.skip=skipcount
        self.count=0
        self.rate=0.0

    def tick(self):
        self.count+=1
        if self.count>=self.skip:
            self.show()

    def show(self):
        ideal_time = hubo_ach.HUBO_LOOP_PERIOD*self.count
        t=time.time()
        actual_time = t-self.t_last
        log('Sim time: {:.3f}, Actual time: {:.3f}, RT rate: {:.3f}% T= {:.6f}'.format(ideal_time,actual_time,ideal_time/actual_time*100,TIMESTEP))
        self.t_last=t
        self.count=0

    def zero(self):
        self.t_last = time.time()
        self.count = 0

def log(string,level=4):
    raveLog(string,level)

def runTrajectory(fileName):
    # Check to see if file exists and is a trajectory file
    try:
        f = open(fileName, 'r')
    except IOError:
        print "File does not exist"
        return

    if(not(fileName[len(fileName) - 5:] == ".traj")):
        print("Not a valid trajectory file")
        return

    print("\nLoadingTrajectory")

    lines = f.readlines()
    trajJointNames = lines[0].split()
    jointNames = []
    positions = dict()

    # Check if all joints are valid
    for jointName in trajJointNames:
        fileJointName = jointName
        jointName = power.getName(jointName)
        if jointName is None:
            print 'Warning: Joint %s not recognized or cannot be set' % fileJointName
        jointNames.append(jointName)
        positions[jointName] = []

    # Load joint positions into arrays
    del(lines[0])
    i = 0
    for line in lines:
        vals = line.split()
        j = 0
        for jointName in jointNames:
            positions[jointName].append(float(vals[j]))
            j += 1
        i += 1

    print("Trajectory file loaded") 
    
    print("\nStarting trajectory")
    usage = 0.0
    N = int(numpy.ceil(hubo_ach.HUBO_LOOP_PERIOD/TIMESTEP)) # Simulation runs faster than hubo-ach
    statusLogger.zero() # Restart timer
    for i in xrange(len(lines)):
        posei = Pose(robot, ctrl)
        pose = posei

        # Approximate positions to send in between trajectory values
        for j in xrange(1, N+1):
            for jointName in jointNames:
                if jointName is not None:
                    pose[jointName] = float(j)/N*(positions[jointName][i]-posei[jointName]) + posei[jointName]
            pose.send()
            env.StepSimulation(TIMESTEP)
            power.addTorques()

        statusLogger.tick()
        usage += power.calcPowerUsage(TIMESTEP*N)
        q.write(" " + str(pose["RSR"]) + "\n")

    print("\nTrajectory completed\nPower used: %.6fWh" % usage)

# Continues the simulation without moving the robot
def stepSimulation(sleepTime):
    try:
        t = float(sleepTime)
    except:
        print("Argument of sleep not understood")
        return
    
    pose = Pose(robot, ctrl)##
    usage = 0.0
    N = int(numpy.ceil(hubo_ach.HUBO_LOOP_PERIOD/TIMESTEP))
    statusLogger.zero() # Restart timer
    for i in xrange(int(t/TIMESTEP/N)):
        for j in xrange(N):
            env.StepSimulation(TIMESTEP)
            power.addTorques()
        statusLogger.tick()
        usage += power.calcPowerUsage(TIMESTEP*N)
        q.write(str(pose["RSP"]) + "\n")

    print("\nPower used: %.6fWh" % usage)

def getVelocity(jointName):
    jointName = power.getName(jointName)
    if jointName is not None:
        print jointName + " vel = " +"%.4f" % power.getMotor(jointName).interpolationVel
    else:
        print "Invalid joint name"

def setVelocity(jointName, vel):
    jointName = power.getName(jointName)
    if jointName is not None:
        try:
            vel = float(vel)
            power.getMotor(jointName).setInterpolationVelocity(vel)
        except:
            print "Argument of setVelocity not understood"
    else:
        print "Invalid joint name"

def getPosition(jointName):
    # Check to see if jointName is valid
    jointName = power.getName(jointName)
    if jointName is not None:
        pose = Pose(robot, ctrl)
        print(jointName + " pos = " + "%.4f" % pose[jointName])
    else:
        print "Invalid joint name"

def setPosition(jointName, position):
    # Check to see if jointName is valid
    jointName = power.getName(jointName)
    if(jointName is not None):
        try:
            # Check to see if position is a decimal number
            position = float(position)
        except:
            print "Argument of setPosition not understood"
            return

        pose = Pose(robot, ctrl)
        jointPos = pose[jointName]
        step = power.getMotor(jointName).interpolationVel*TIMESTEP # Change in osition per timestep
        usage = 0
        count = 0
        N = int(numpy.ceil(hubo_ach.HUBO_LOOP_PERIOD/TIMESTEP))
        statusLogger.zero() # Restart timer

        if(jointPos > position):
            step *= -1
        jointPos += step
        # While joint is not near desired position yet
        while(True):
            for i in xrange(N):
                if abs(jointPos - position) >= abs(step):
                    pose[jointName] = jointPos
                    pose.send()
                    env.StepSimulation(TIMESTEP)
                    power.addTorques()
                    jointPos += step
                else:
                    break
            else: # So outer loop continues
                usage += power.calcPowerUsage(TIMESTEP*N)
                statusLogger.tick()
                continue
            break # So inner loop breaks out of outer loop
        
        # Joint is near desired position, set position to desired position
        pose[jointName] = position
        env.StepSimulation(TIMESTEP)

        print("Power used: %.6fWh" % usage)

    else:
        print "Invalid joint name"

# Tab completion for trajectory file paths and commands   
def complete(text, state):
    for cmd in COMMANDS:
        if cmd.startswith(text):
            if not state:
                return cmd
            else:
                state -=1
    return (glob.glob(text+'*')+[None])[state]

if __name__ == '__main__':
    # Initialize tab completion
    readline.set_completer_delims(' \t\n;')
    readline.parse_and_bind("tab: complete")
    readline.set_completer(complete)

    # Setup simulation options
    [env, options] = setup('qtcoin', True)
    env.SetDebugLevel(4)
    time.sleep(.25)
    options.physics = True
    options.stop = True
    options.ghost = True
    
    # Start OpenHubo
    [robot, ctrl, ind, ghost, recorder] = load_scene(env, options.robotfile, options.scenefile, options.stop, options.physics, options.ghost)
    env.StopSimulation()
    env.StepSimulation(.0005)
    TIMESTEP = .001
    statusLogger = StatusLogger(100, time.time())
    
    # Initialize power model, motors
    ##
    q = open('trajectories/armTorque.txt', 'w')
    ##
    power = PowerModel(robot, 5, q) ##

    print "\nMaestro OpenHubo script for modelling power usage"
    print "\tCommands: runTrajectory stepSimulation getPosition setPosition getVelocity setVelocity\n"

    # Continuously ask for command
    print "Enter command or press Ctrl+C to exit"
    while(True):
        try:
            userInput = raw_input("> ")
            words = userInput.split()

            if(len(words) == 0):
                continue
                    
            if(words[0] == "runTrajectory"):
                if(len(words) == 2):
                    runTrajectory(words[1])
                    continue
                else:
                    print "Runs given trajectory file and calculates power use\nUsage: runTrajectory FILEPATH"
                    continue

            if(words[0] == "stepSimulation"):
                if(len(words) == 2):
                    stepSimulation(words[1])
                    continue
                else:
                    print "Runs the simulation without moving the robot and calculates power use\nUsage: stepSimulation TIME"
                    continue
            
            if(words[0] == "getPosition"):
                if(len(words) == 2):
                    getPosition(words[1])
                    continue
                else:
                    print "Returns the position of a joint\nUsage: getPosition JOINTNAME"
                    continue

            if(words[0] == "setPosition"):
                if(len(words) == 3):
                    setPosition(words[1], words[2])
                else:
                    print "Moves joint to specified position\nUsage: setPosition JOINTNAME POSITION"

            if(words[0] == "getVelocity"):
                if(len(words) == 2):
                    getVelocity(words[1])
                else:
                    print "Returns interpolation velocity of a joint\nUsage: getVelocity JOINTNAME"

            if(words[0] == "setVelocity"):
                if(len(words) == 3):
                    setVelocity(words[1], words[2])
                else:
                    print "Sets interpolation velocity of a joint\nUsage: setVelocity JOINTNAME VELOCITY"

        # Exit
        except KeyboardInterrupt:
            print("\nExiting")
            sys.exit()
