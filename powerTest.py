#!/usr/bin/env python
from openravepy import *
from openhubo import *
import hubo_ach
import sys
import time
import numpy

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

def loadTrajectory(fileName):
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

    return positions, jointNames, len(lines)

def runTrajectory(positions, jointNames, iterations):
    print("\nStarting trajectory")
    
    usage = 0.0
    N = int(numpy.ceil(hubo_ach.HUBO_LOOP_PERIOD/TIMESTEP))
    statusLogger.zero()
    for i in xrange(iterations):
        posei = Pose(robot, ctrl)
        pose = posei
        for j in xrange(1, N+1):
            for jointName in jointNames:
                if(jointName in mutable and mutable[jointName]):
                    pose[jointName] = float(j)/N*(positions[jointName][i]-posei[jointName]) + posei[jointName]
            pose.send()
            env.StepSimulation(TIMESTEP)
            addTorques(robot)

        statusLogger.tick()
        usage += calcBatteryUsage(robot, TIMESTEP*N)

    print("\nTrajectory completed\nPower used: %.6fWh" % usage)
    

if __name__ == '__main__':
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


    # Main loop
    TIMESTEP = .001
    statusLogger = StatusLogger(100, time.time())
    while(True):
        #Open trajectory file input by user. If input is null, the script will exit
        while(True):
            try:
                print("Enter trajectory file to run or press Ctrl+C to exit")
                fileName = raw_input("> ")
                if(fileName == ""):
                    sys.exit()
                f = open(fileName, 'r')
                break
            except IOError:
                # File not found, ask user to re-enter file name
                print("File does not exist")
        
        # Load trajectory
        [positions, jointNames, iterations] = loadTrajectory(fileName)

        # Run trajectory
        runTrajectory(positions, jointNames, iterations)