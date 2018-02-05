import numpy as np
import matplotlib.pyplot as plt
import math
import Map


# Code for differential drive

V_MAX = 1.1
DT = 0.1
A_MAX = 1.3
TOLERANCE = V_MAX * DT #/ 10
MAX_EDGE = V_MAX * DT# + ((A_MAX*DT)**2)/2
FILEPATH = "P1.json"
OMEGA_MAX = 1.3

class Node():

    def __init__(self, pos, orientation, omega):
        self.pos = pos
        self.name = str(self.pos)
        self.parent = None
        self.children = [] # only used to plot the graph
        self.distance = 0 # Needed only to keep track of the distance to the "competition" or something
        self.orientation = orientation
        self.omega = omega # Only used for plotting the graph

    def dist(self, otherNode):
        return np.linalg.norm(otherNode.pos - self.pos)

def findNewNodeDP(nearestNeighbor, randomNode):

    delta = randomNode.pos - nearestNeighbor.pos
    a = np.array([A_MAX / math.sqrt(1 + (delta_y / delta_x) ** 2), A_MAX / math.sqrt(1 + (delta_x / delta_y) ** 2)]) # Set the acceleration towards the sampled position

    timeOfMaxVel = (V_MAX ** 2 - 2 * np.dot(nearestNeighbor.vel, a) - np.linalg.norm(nearestNeighbor.vel)) / np.linalg.norm(a)
    if timeOfMaxVel < DT:
        pass
    
    return 0

def findNewNodeDD(nearestNeighbor, randomNode):

    x_0 = nearestNeighbor.pos[0]
    y_0 = nearestNeighbor.pos[1]
    theta_0 = nearestNeighbor.orientation

    #print(x_0, y_0, np.degrees(theta_0))
    #print(randomNode.pos[0], randomNode.pos[1])

    angle = np.angle(complex(randomNode.pos[0] - x_0, randomNode.pos[1] - y_0))
    if angle > np.pi:
        angle -= (2 * np.pi)

    #print(np.degrees(angle))
    
    angle_difference = angle - nearestNeighbor.orientation
    omega = angle_difference / DT
    if abs(omega) > OMEGA_MAX:
        omega *= (OMEGA_MAX / abs(omega))
    omega += 0.00001
    #print(omega)

    new_theta = theta_0 + omega * DT
    new_x = x_0 + (V_MAX / omega) * (np.sin(new_theta) - np.sin(theta_0))
    new_y = y_0 - (V_MAX / omega) * (np.cos(new_theta) - np.cos(theta_0))
    #print(new_x)

    return Node(np.array([new_x, new_y]), new_theta, omega)

#def ValueBaseTSTS(pos, orientation):
    

def RRT(start, goal):

    K = 10000
    tree = []
    treeAccs = {}

    newNode = start
    tree.append(start)
    maxSpeed = 0



    for k in range(K):
        # Bias towards the goal, every fifth k
        if k % 5 == 0:
            randomPos = goal.pos
        else:
            randomPos = np.multiply(np.random.random(2), np.array([2, 2]))

        randNode = Node(randomPos, 0, 1)

        if not mapp.isBlocked(randNode):
            nearestNode = nearestNeighbor(randNode, tree)

            #print(randNode.pos)
            #print(nearestNode.pos)

            ##MAX_EDGE kan nu inte använda max_vel, utan måste använda den faktiska hastigheten som beror på accelerationen.
            #ratio = MAX_EDGE / nearestNode.dist(randNode)
            #print(ratio)
            #print("randNode: ", randNode.pos, randNode.vel)
            #print("nearestNode: ", nearestNode.pos, nearestNode.vel)
            newNode = findNewNodeDD(nearestNode, randNode)# Node(nearestNode.x + ratio * (randomX - nearestNode.x),
                                                          # nearestNode.y + ratio * (randomY - nearestNode.y))

            #print(type(newNode.pos))
            #print("newNode: ", newNode.pos, newNode.orientation)

            #Plots the tree
##            for i in range(len(tree)-1):
##                node = tree[i]
##                for child in node.children:
##                    plt.plot([node.pos[0], child.pos[0]], [node.pos[1], child.pos[1]])
##            plt.show()
##
##            newNode.distance = nearestNode.distance + newNode.dist(nearestNode)

            # computes the speed, only for curiosity
            speed = newNode.dist(nearestNode)/DT
            if speed > maxSpeed:
                maxSpeed = speed
                #print(newNode.pos, nearestNode.pos)

            newNode.parent = nearestNode
            nearestNode.children.append(newNode)
            tree.append(newNode)

        #print(newNode.dist(goal))
        
        if newNode.dist(goal) <= TOLERANCE:
            print("breakar")
            print("k: " + str(k))
            print("speed: "+str(maxSpeed))
            #return newNode
            return tree

    print("k: "+str(k))
    return startNode


def nearestNeighbor(randNode, tree):
    bestNode = tree[0]
    bestDistance = randNode.dist(tree[0])

    for node in tree:
        distance = randNode.dist(node)
        if distance < bestDistance:
            bestDistance = distance
            bestNode = node
    #Vid hinder, kontrollera om det är hinder ivägen, i sådana fall skrota randNode, returnera null eller nåt?
    return bestNode

mapp = Map.Map(FILEPATH)
startNode = Node(np.array([0.1, 0.1]), np.pi / 4, 1)
goalNode = Node(np.array([2, 2]), 5 * np.pi / 4, 1)

print("TOLERANCE: ", TOLERANCE)
tree = RRT(startNode, goalNode)

path = []
currentNode = tree[len(tree)-1]
while not currentNode == startNode:
    #print(str(currentNode.name))
    path.append(currentNode.name)
    currentNode = currentNode.parent

path.append(startNode.name)
path.reverse()

#print(path)

print("Distance: "+str(tree[len(tree)-1].distance))


#Plots the tree
for i in range(len(tree)-1):
    node = tree[i]
    for child in node.children:
        time_series = np.linspace(0, DT, 10)
        x_array = node.pos[0] + (V_MAX / child.omega) * (np.sin(node.orientation + child.omega * time_series) - np.sin(node.orientation))
        y_array = node.pos[1] - (V_MAX / child.omega) * (np.cos(node.orientation + child.omega * time_series) - np.cos(node.orientation))
        #print(child.omega)
        #print(x_array[9], child.pos[0], node.pos[0] + (V_MAX / child.omega) * (np.sin(child.orientation + child.omega * DT) - np.sin(child.orientation)))
        #print(time_series)
        plt.plot(x_array, y_array)

plt.scatter(startNode.pos[0], startNode.pos[1], c = "g")
plt.scatter(goalNode.pos[0], goalNode.pos[1], c = "r")

plt.show()
