import numpy as np
import matplotlib.pyplot as plt
import math
import Map


# Code for dynamic point

V_MAX = 1.1
DT = 0.1
A_MAX = 1.3
TOLERANCE = V_MAX * DT #/ 10
MAX_EDGE = V_MAX * DT# + ((A_MAX*DT)**2)/2
FILEPATH = "P1.json"

class Node():

    def __init__(self, pos, vel):
        self.pos = np.array(pos)
        self.name = str(self.pos)
        self.parent = None
        self.children = [] # only used to plot the graph
        self.distance = 0 # Needed only to keep track of the distance to the "competition" or something
        # Need to keep track of the velocity, part of the state space
        self.vel = np.array(vel)


    def dist(self, otherNode):
        return np.linalg.norm(otherNode.pos - self.pos)

def findNewNodeDP(nearestNeighbor, randomNode):

    delta = randomNode.pos - nearestNeighbor.pos
    a = np.array([A_MAX / math.sqrt(1 + (delta_y / delta_x) ** 2), A_MAX / math.sqrt(1 + (delta_x / delta_y) ** 2) # Set the acceleration towards the sampled position

    timeOfMaxVel = (V_MAX ** 2 - 2 * np.dot(nearestNeighbor.vel, a) - np.linalg.norm(nearestNeighbor.vel)) / np.linalg.norm(a)
    if timeOfMaxVel < DT:
                  
    
    return 0

def findNewNodeDD(nearestNeighbor, randomNode):
    

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
            randomPos = np.multiply(np.random.random(2), mapp.dims)

        randNode = Node(randomPos)

        if not mapp.isBlocked(randNode):
            nearestNode = nearestNeighbor(randNode, tree)

            #print(nearestNode.pos)

            ##MAX_EDGE kan nu inte använda max_vel, utan måste använda den faktiska hastigheten som beror på accelerationen.
            #ratio = MAX_EDGE / nearestNode.dist(randNode)
            #print(ratio)
            newNode = findNewNodeDP(nearestNode, randNode)# Node(nearestNode.x + ratio * (randomX - nearestNode.x),
                                                          # nearestNode.y + ratio * (randomY - nearestNode.y))

            #print(newNode.pos)

            newNode.distance = nearestNode.distance + newNode.dist(nearestNode)

            # computes the speed, only for curiosity
            speed = newNode.dist(nearestNode)/DT
            if speed > maxSpeed:
                maxSpeed = speed
                print(newNode.pos, nearestNode.pos)

            newNode.parent = nearestNode
            nearestNode.children.append(newNode)
            tree.append(newNode)
        
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
startNode = Node(np.array([1, 2]), mapp.vel_start)
goalNode = Node(np.array([10, 20]), mapp.vel_goal)

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
        plt.plot([node.pos[0], child.pos[0]], [node.pos[1], child.pos[1]])

plt.scatter(startNode.pos[0], startNode.pos[1], c = "g")
plt.scatter(goalNode.pos[0], goalNode.pos[1], c = "r")

plt.show()
