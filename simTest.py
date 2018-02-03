import vrep
import numpy as np
import math
from RRT_DP import findPathDP
import time

## Orientation: need only change gamma

def runSimulation():

    # close any open connections
    vrep.simxFinish(-1)
    # Connect to the V-REP continuous server
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5)

    if clientID != -1: # if we connected successfully
        print ('Connected to remote API server')

    # --------------------- Setup the simulation

    vrep.simxSynchronous(clientID,True)

    objectName = "bubbleRob"

    joint_names = ['bubbleRob_rightMotor', 'bubbleRob_leftMotor']
    # joint target velocities discussed below
    joint_target_velocities = np.ones(len(joint_names)) * 0.0

    # get the handles for each joint and set up streaming
    joint_handles = [vrep.simxGetObjectHandle(clientID,
        name, vrep.simx_opmode_blocking)[1] for name in joint_names]
    _, objectHandle = vrep.simxGetObjectHandle(clientID, objectName, vrep.simx_opmode_blocking)

    print(objectName)
    print(objectHandle)
    posZ = 0.025

    #errorCode = vrep.simxSetObjectPosition(clientID, objectHandle, -1, [posX, posY, posZ], vrep.simx_opmode_oneshot)
    #orientation = vrep.simxGetObjectOrientation(clientID, objectHandle, -1, vrep.simx_opmode_streaming)
    #print(errorCode)
    #print(orientation)

    path = findPathDP("P3.json")
    print(len(path))
    startNode = path[0]
    startVel = startNode.vel
    gamma = velToAng(startVel)
    print("gamma")
    print(gamma)


    errorCode = vrep.simxSetObjectPosition(clientID, objectHandle, -1, [startNode.x/10, startNode.y/10, posZ], vrep.simx_opmode_oneshot_wait)
    errorCode = vrep.simxSetObjectOrientation(clientID, objectHandle, -1, [0, 0, gamma],vrep.simx_opmode_oneshot_wait)
    #errorCode = vrep.simxSetObjectPosition(clientID, objectHandle, -1, [path[len(path)-1].x/10, path[len(path)-1].y/10, posZ], vrep.simx_opmode_oneshot)


    # get handle for target and set up streaming
    _, target_handle = vrep.simxGetObjectHandle(clientID,
                    'bubbleRob', vrep.simx_opmode_blocking)

    dt = 1
    vrep.simxSetFloatingParameter(clientID,
            vrep.sim_floatparam_simulation_time_step,
            dt, # specify a simulation time step
            vrep.simx_opmode_oneshot)



    # --------------------- Start the simulation

    # start our simulation in lockstep with our code
    vrep.simxStartSimulation(clientID,
            vrep.simx_opmode_blocking)

    count = 0



    while count < len(path): # run for 1 simulated second

        #posX += 0.005
        #posY += 0.005
        currentNode = path[count]
        gamma = velToAng(currentNode.vel)
        print(currentNode.vel)
        print(gamma)
        #print(currentNode.x/100)


        errorCode = vrep.simxSetObjectPosition(clientID, objectHandle, -1, [currentNode.x/10, currentNode.y/10, posZ], vrep.simx_opmode_oneshot)
        errorCode = vrep.simxSetObjectOrientation(clientID, objectHandle, -1, [0, 0, gamma], vrep.simx_opmode_oneshot)

        # move simulation ahead one time step
        vrep.simxSynchronousTrigger(clientID)
        count += dt


    # Reset the orientation
    #errorCode = vrep.simxSetObjectOrientation(clientID, objectHandle, -1, [0, 0, gamma], vrep.simx_opmode_oneshot)


    #Pause simulation
    vrep.simxPauseSimulation(clientID, vrep.simx_opmode_oneshot)
    time.sleep(5)

    # stop our simulation
    vrep.simxStopSimulation(clientID,
            vrep.simx_opmode_blocking)



    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)

def velToAng(vel):
    """Takes a velocity and computes the angle between the vector and the x-axis in degrees"""
    radians = math.acos(abs(vel[0])/np.linalg.norm(vel))
    #degrees = round(math.degrees(radians), 5)
    #print(np.linalg.norm(vel))
    #print(radians)
    #print(degrees)
    if vel[0] < 0 < vel[1]:
        radians = math.pi - radians
        #degrees += 90
    elif vel[1] < 0 < vel[0]:
        radians = 0 - radians
        #degrees += 270
    elif vel[0] < 0 and vel[1] < 0:
        radians = radians - math.pi
        #degrees += 180
    #if vel[0] < 0 or vel[1] < 0:
    #    radians = -radians
    #return degrees
    return radians

runSimulation()

#print(velToAng([0.5, 0.5]))
#print(velToAng([0.5, -0.5]))
#print(velToAng([-0.5, 0.5]))
#print(velToAng([-0.5, -0.5]))