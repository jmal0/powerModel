#!/usr/bin/env python
from openravepy import *
from openhubo import *
import hubo_ach
import sys
import time
import numpy
import cmd
import readline, glob
COMMANDS = ['runTrajectory', 'getPosition', 'setPosition', 'stepSimulation']

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
        log('Sim time: {:.3f}, Actual time: {:.3f}, RT rate: {:.3f}% T= {:.6f}'.format(ideal_time,actual_time,ideal_time/actual_time*100,TIMESTEP)) #
        self.t_last=t
        self.count=0

    def zero(self):
        self.t_last = time.time()
        self.count = 0

def log(string,level=4):
    raveLog(string,level)

def paint(torque):
    for jointName in torque:
        link = robot.GetLink("Body_" + jointName)
        if(not(link == None)):
            for geom in link.GetGeometries():
                geom.SetDiffuseColor([min(torque[jointName], .5)/.5,.5-min(torque[jointName], .5)/.1,0])

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
        if not(jointName in mutable):
            if not(jointName in altname):
                print 'Warning: Joint %s not recognized' % jointName
            else:
                jointName = altname[jointName]
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
                if(jointName in mutable and mutable[jointName]):
                    pose[jointName] = float(j)/N*(positions[jointName][i]-posei[jointName]) + posei[jointName]
            pose.send()
            env.StepSimulation(TIMESTEP)
            addTorques(robot)

        statusLogger.tick()
        [use, torque] = calcBatteryUsage(robot, TIMESTEP*N, q)
        #paint(torque)
        usage+= use
        q.write(str(pose["RSR"]) + "\n")

    print("\nTrajectory completed\nPower used: %.6fWh" % usage)

def stepSimulation(sleepTime):
    try:
        t = float(sleepTime)
    except:
        print("Argument of sleep not understood")
        return
    
    pose = Pose(robot, ctrl)##
    usage = 0.0
    N = int(numpy.ceil(hubo_ach.HUBO_LOOP_PERIOD/TIMESTEP)) # Simulation runs faster than hubo-ach
    statusLogger.zero() # Restart timer
    for i in xrange(int(t/TIMESTEP/N)):
        for j in xrange(N):
            env.StepSimulation(TIMESTEP)
            addTorques(robot)
        statusLogger.tick()
        [use, torque] = calcBatteryUsage(robot, TIMESTEP*N, q)
        #paint(torque)
        usage+= use
        q.write(str(pose["RSR"]) + "\n")

    print("\nPower used: %.6fWh" % usage)

def getPosition(jointName):
    if jointName in mutable:
        pose = Pose(robot, ctrl)
        print(jointName + ": " + "%.4f" % pose[jointName])

def setPosition(jointName, position):
    return 0

# Tab completion for trajectory file paths    
def complete(text, state):
    for cmd in COMMANDS:
        if cmd.startswith(text):
            if not state:
                return cmd
            else:
                state -=1
    return (glob.glob(text+'*')+[None])[state]

if __name__ == '__main__':
    readline.set_completer_delims(' \t\n;')
    readline.parse_and_bind("tab: complete")
    readline.set_completer(complete)

    # Setup simulation options
    [env, options] = setup('qtcoin', True)
    env.SetDebugLevel(4)
    time.sleep(.25)
    options.physics = True
    options.stop = True
    
    # Start OpenHubo
    [robot, ctrl, ind, ghost, recorder] = load_scene(env, options.robotfile, options.scenefile, options.stop, options.physics, options.ghost)
    env.StopSimulation()
    env.StepSimulation(.0005)

    print "\nMaestro OpenHubo script for modelling power usage of trajectories"
    print "\tCommands: runTrajectory setPosition getPosition stepSimulation\n"

    ##
    q = open('trajectories/armTorque.txt', 'w')
    ##

    # Main loop
    TIMESTEP = .001
    statusLogger = StatusLogger(100, time.time())
    while(True):
        print("Enter command or press Ctrl+C to exit")
        
        # Continuously ask for command
        while(True):
            try:
                userInput = raw_input("> ")
                words = userInput.split()

                if(len(words) == 0):
                    continue
                    
                if(words[0] == "runTrajectory"):
                    if(len(words) == 2):
                        runTrajectory(words[1])
                        break
                    else:
                        print "Runs given trajectory file and calculates power use\nUsage: runTrajectory FILEPATH"
                        continue

                if(words[0] == "stepSimulation"):
                    if(len(words) == 2):
                        stepSimulation(words[1])
                        break
                    else:
                        print "Runs the simulation without moving the robot and calculates power use\nUsage: stepSimulation TIME"
                        continue
            
                if(words[0] == "getPosition"):
                    if(len(words) == 2):
                        getPosition(words[1])
                        break
                    else:
                        print "Returns the position of a joint\nUsage: getPosition JOINTNAME"
                        continue

                if(words[0] == "setPosition"):
                    if(len(words) == 2):
                        setPosition(words[1], words[2])
                        break
                    else:
                        print "Moves joint to specified position\nUsage: setPosition JOINTNAME VALUE"

            # Exit
            except KeyboardInterrupt:
                print("\nExiting")
                sys.exit()

