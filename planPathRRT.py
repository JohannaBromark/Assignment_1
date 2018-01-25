import numpy as np
import matplotlib.pyplot as plt
import math
import Map

V_MAX = 1.1
DT = 0.1
TOLERANCE = V_MAX * DT / 10
MAX_EDGE = V_MAX * DT
FILEPATH = "P1.json"

class Node():

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

    def dist(self, otherNode):
        return math.sqrt((otherNode.x - self.x) ** 2 + (otherNode.y - self.y) ** 2)

def RRT(start, goal):

    tree = []
    newNode = start
    tree.append(start)
    while not (newNode.dist(goal) < TOLERANCE):
        print(str(newNode.x) + ", " + str(newNode.y))

        randomX = np.random.random() * mapp.width
        randomY = np.random.random() * mapp.height
        randNode = Node(randomX, randomY)
        
        if not mapp.isBlocked(randNode):
            nearestNode = nearestNeighbor(randNode, tree)

            #Go from nearest node in vel, find new node
            ratio = MAX_EDGE / nearestNode.dist(randNode)
            newNode = Node(nearestNode.x + ratio * (randomX - nearestNode.x),
                           nearestNode.y + ratio * (randomY - nearestNode.y))
            
            newNode.parent = nearestNode
            tree.append(newNode)
            
    return newNode


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
startNode = Node(1, 1)
goalNode = Node(2, 3)

currentNode = RRT(startNode, goalNode)
pathx = np.array()
pathy = np.array()
while not currentNode.parent == None:
    print(str(currentNode.parent.x) + ", " + str(currentNode.parent.y))
    currentNode = currentNode.parent
    np.append(pathx, currentNode.x)
    np.append(pathy, currentNode.y)

plt.plot(pathx, pathy)
