import numpy as np
import matplotlib.pyplot as plt
import math
import Map

# Working code for Kinematic point

V_MAX = 1.1
DT = 0.1
TOLERANCE = V_MAX * DT #/ 10
MAX_EDGE = V_MAX * DT
FILEPATH = "P2.json"

class Node():

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.XY = [self.x, self.y]
        self.name = str(self.x)+","+str(self.y)
        self.parent = None
        self.children = [] # only used to plot the graph
        self.distance = 0 # Needed only to keep track of the distance to the "competition" or something


    def dist(self, otherNode):
        return math.sqrt((otherNode.x - self.x) ** 2 + (otherNode.y - self.y) ** 2)

def RRT(start, goal):

    K = 10000
    tree = []
    newNode = start
    tree.append(start)
    maxSpeed = 0

    for k in range(K):
        if k%5 == 0:
            randomX = goal.x
            randomY = goal.y
        else:
            randomX = np.random.random() * mapp.width
            randomY = np.random.random() * mapp.height

        randNode = Node(randomX, randomY)

        if mapp.isOK(randNode):
            nearestNode = nearestNeighbor(randNode, tree)

            ratio = MAX_EDGE / nearestNode.dist(randNode)
            newNode = Node(nearestNode.x + ratio * (randomX - nearestNode.x),
                           nearestNode.y + ratio * (randomY - nearestNode.y))

            newNode.distance = nearestNode.distance + newNode.dist(nearestNode)

            # computes the speed, only for curiosity
            speed = newNode.dist(nearestNode)/DT
            if speed > maxSpeed:
                maxSpeed = speed

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
startNode = Node(1, 2)
goalNode = Node(10, 15)

tree = RRT(startNode, goalNode)

path = []
currentNode = tree[len(tree)-1]
while not currentNode == startNode:
    #print(str(currentNode.name))
    path.append(currentNode)
    currentNode = currentNode.parent

path.append(startNode)
path.reverse()



lastNode = path[len(path)-1]
print("sista Noden ut: "+ lastNode.name)

#Lägger till slutpunkten, om inte den går att nås, lägg till mellannoder
while path[len(path)-1].dist(goalNode)> V_MAX * DT:
    #Lägger till en punkt emellan
    ratio = MAX_EDGE / lastNode.dist(goalNode)
    lastNode = Node(lastNode.x + ratio * (goalNode.x - lastNode.x),
                   lastNode.y + ratio * (goalNode.y - lastNode.y))
    path.append(lastNode)
else:
    lastNode = goalNode
    path.append(lastNode)

for node in path:
    print(node.name)

"""
* Måste sätta hastighet på noderna, så att sista har rätt hastighet
"""



#print(path)

print("Distance: "+str(tree[len(tree)-1].distance))



#Plots the tree
for i in range(len(tree)-1):
    node = tree[i]
    for child in node.children:
        plt.plot([node.x, child.x], [node.y, child.y])

plt.scatter(startNode.x, startNode.y, c = "g")
plt.scatter(goalNode.x, goalNode.y, c = "r")

plt.show()


def plotObstacles(obstacles):
    