import numpy as np
import matplotlib.pyplot as plt
import math
import Map


# Code for kinematic car

V_MAX = 1.1
DT = 0.1
A_MAX = 1.3
TOLERANCE = 2 * V_MAX * DT #/ 10
MAX_EDGE = V_MAX * DT# + ((A_MAX*DT)**2)/2
FILEPATH = "P1.json"
OMEGA_MAX = 1.3
L = 2
PHI_MAX = 0.6
LOCAL_PLANNER_TOLERANCE = (2 * L) / np.tan(PHI_MAX)

class Node():

    def __init__(self, pos, orientation):
        self.pos = pos
        self.name = str(self.pos)
        self.parent = None
        self.children = [] # only used to plot the graph
        self.distance = 0 # Needed only to keep track of the distance to the "competition" or something
        self.orientation = orientation


    def dist(self, otherNode):
        return np.linalg.norm(otherNode.pos - self.pos)


def findNewNodeKC(nearestNeighbor, randomNode):

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
    phi = np.arctan((angle_difference * L) / (V_MAX * DT))
    if abs(phi) > PHI_MAX:
        phi *= (PHI_MAX / abs(phi))

    new_theta = theta_0 + (V_MAX / L) * np.tan(phi) * DT
    new_x = x_0 + (L / np.tan(phi)) * (np.sin(new_theta) - np.sin(theta_0))
    new_y = y_0 - (L / np.tan(phi)) * (np.cos(new_theta) - np.cos(theta_0))

    return Node(np.array([new_x, new_y]), new_theta)

def localPlannerKC(nearestNode, tree):

    angle = goalNode.orientation #np.angle(complex(goalNode.pos[0] - x_0, goalNode.pos[1] - y_0))
    print(np.degrees(angle))
    rotation = 1
    if angle > np.pi:
        angle -= (2 * np.pi)
        rotation = -1
        

    angle_difference = angle - nearestNode.orientation
    #print("angle_difference: ", np.degrees(angle_difference))
    t_aligned = (L * abs(angle_difference)) / (V_MAX * np.tan(PHI_MAX))
    print(t_aligned)

    t = 0
    phi = PHI_MAX

    print("nearestNode.dist(goalNode): ", nearestNode.dist(goalNode), "TOLERANCE: ", TOLERANCE)
    
    while nearestNode.dist(goalNode) >= TOLERANCE:

        x_0 = nearestNode.pos[0]
        y_0 = nearestNode.pos[1]
        theta_0 = nearestNode.orientation
        
        print(x_0, y_0, np.degrees(theta_0))

        new_theta = theta_0 + (V_MAX / L) * np.tan(phi) * DT
        new_x = x_0 + (L / (np.tan(phi) + 0.00001)) * (np.sin(new_theta) - np.sin(theta_0))
        new_y = y_0 - (L / (np.tan(phi) + 0.00001)) * (np.cos(new_theta) - np.cos(theta_0))

        if t > t_aligned:
            print("GOIN STRAIGHT")
            phi = 0
            new_x = x_0 + V_MAX * np.cos(theta_0) * DT
            new_y = y_0 + V_MAX * np.sin(theta_0) * DT

        newNode = Node(np.array([new_x, new_y]), new_theta)
        newNode.parent = nearestNode
        nearestNode.children.append(newNode)
        tree.append(newNode)
        nearestNode = newNode

        print("wallawallawalla", newNode.pos, newNode.orientation)

        t += DT
        #print(t)

    print(nearestNode.pos, nearestNode.orientation)
    
    return tree

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

        randNode = Node(randomPos, 0)

        if not mapp.isBlocked(randNode):
            nearestNode = nearestNeighbor(randNode, tree)

            #print(randNode.pos)
            #print(nearestNode.pos)

            ##MAX_EDGE kan nu inte använda max_vel, utan måste använda den faktiska hastigheten som beror på accelerationen.
            #ratio = MAX_EDGE / nearestNode.dist(randNode)
            #print(ratio)
            #print("randNode: ", randNode.pos, randNode.vel)
            #print("nearestNode: ", nearestNode.pos, nearestNode.vel)
            newNode = findNewNodeKC(nearestNode, randNode)# Node(nearestNode.x + ratio * (randomX - nearestNode.x),
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
        
        if newNode.dist(goal) <= LOCAL_PLANNER_TOLERANCE:
            print("breakar")
            print("k: " + str(k))
            print("speed: "+str(maxSpeed))
            #return newNode
            print(newNode.pos, np.degrees(newNode.orientation))
            return nearestNode, tree

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
startNode = Node(np.array([0.1, 0.1]), np.pi / 4)
goalNode = Node(np.array([10, 10]), 5 * np.pi / 4)

print("TOLERANCE: ", TOLERANCE)
nn, tree = RRT(startNode, goalNode)
tree = localPlannerKC(nn, tree)

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
