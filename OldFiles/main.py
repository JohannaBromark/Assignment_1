import vrep
import numpy as np
from queue import PriorityQueue

grid = np.full((2, 2), True)
V_MAX = 1
V_GOAL = np.array([0.9, -0.2])

class Node(x, y):
    self.x = x
    self.y = y


def parseJson():
    #todo
    return 0 

def planPath(start, goal):

    q = PriorityQueue()
    q.put(start)
    visited = set()
    visited.add(start)
    cameFrom = {}
    gScore = {}
    gScore[start] = 0
    fScore = {}
    fScore[start] = heuristic(start, goal)
    nodesInQueue = {}
    nodesInQueue[start] = True

    while not q.empty():
        current = q.get()
        nodesInQueue[current] = False
        if current == goal:
            return reconstruct_path(cameFrom, current)
        visited.add(current)
        for neighbor in generateNeighbors(current):
            if neighbor in visited:
                continue
            if neighbor not in nodesInQueue:
                q.put(neighbor)
                nodesInQueue[neighbor] = True
            
    
    
##    q = PriorityQueue()
##    difference = goal_pars - start_pars
##    q.put((np.sqrt(difference[0] ** 2 + difference[1] ** 2), difference[2], difference[3], np.array(start_pars)))
##    current_pars = start_pars
##    while np.any(current_pars != goal_pars):
##        current_pars = q.get()
##        print(current_pars)
##        for combination in generateCombinations(current_pars[2:4]):
##            comb_pos = current_pars[0:2] + combination * dt
##            q.put((np.sqrt((goal_pars[0] - comb_pos[0])**2 + (goal_pars[0] - comb_pos[0])**2), goal_pars[2] - combination[0], goal_pars[3] - combination[1], np.concatenate(comb_pos, combination)))
        
    #return path

def generateCombinations(pars):
    v_values = [V_MAX, 0, -V_MAX, V_GOAL[0], V_GOAL[1]]
    combinations = []
    for v_x in v_values:
        for v_y in v_values:
            combinations.append(pars + np.array([v_x, v_y]))
    
    return combinations

def kinematicPoint(path):
    return 0
    
def runSimulation():

    # close any open connections
    vrep.simxFinish(-1)
    # Connect to the V-REP continuous server
    clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5) 
     
    if clientID != -1: # if we connected successfully
        print ('Connected to remote API server')

    # --------------------- Setup the simulation 
     
    vrep.simxSynchronous(clientID,True)

    joint_names = ['bubbleRob_rightMotor', 'bubbleRob_leftMotor']
    # joint target velocities discussed below
    joint_target_velocities = np.ones(len(joint_names)) * 50.0
     
    # get the handles for each joint and set up streaming
    joint_handles = [vrep.simxGetObjectHandle(clientID,
        name, vrep.simx_opmode_blocking)[1] for name in joint_names]
     
    # get handle for target and set up streaming
    _, target_handle = vrep.simxGetObjectHandle(clientID,
                    'bubbleRob', vrep.simx_opmode_blocking)

    dt = .01
    vrep.simxSetFloatingParameter(clientID,
            vrep.sim_floatparam_simulation_time_step,
            dt, # specify a simulation time step
            vrep.simx_opmode_oneshot)
     
    # --------------------- Start the simulation
     
    # start our simulation in lockstep with our code
    vrep.simxStartSimulation(clientID,
            vrep.simx_opmode_blocking)

    count = 0

    while count < 1: # run for 1 simulated second

        for i in range(2):
        
            vrep.simxSetJointTargetVelocity(clientID,
                            joint_handles[i],
                            joint_target_velocities[i], # target velocity
                            vrep.simx_opmode_blocking)
         
        # move simulation ahead one time step
        vrep.simxSynchronousTrigger(clientID)
        count += dt

    # stop our simulation
    vrep.simxStopSimulation(clientID,
            vrep.simx_opmode_blocking)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)

planPath(np.array([0, 0, 1, 1]), np.array([2, 2, 1, 1]))
