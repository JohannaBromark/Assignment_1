import numpy as np
import math
from Map import Map
from plotFunctions import *
from obstacleCheck import isObstacleBetween
from commonFunctions import *

class Node():

    def __init__(self, x, y, vel=np.zeros((1, 2))):
        self.x = x
        self.y = y
        self.pos = np.array([self.x, self.y])
        self.name = str(self.x)+","+str(self.y)
        self.parent = None
        self.children = [] # only used to plot the graph
        self.distance = 0 # Needed only to keep track of the distance to the "competition" or something
        # Need to keep track of the velocity, part of the state space
        self.vel = np.array(vel)

    def dist(self, otherNode):
        # Need to take into account the velocity as well
        return math.sqrt((otherNode.x - self.x) ** 2 + (otherNode.y - self.y) ** 2)

def RRT(start, goal, theMap):

    K = 15000
    tree = []
    tree.append(start)
    newNode = start

    #Used for curiosity
    maxSpeed = 0

    for k in range(K):
        # 10 seems to be best for P3 or 100?
        if k % 100 == 0:
            randomX = goal.x
            randomY = goal.y
        else:
            randomX = np.random.random() * theMap.width
            randomY = np.random.random() * theMap.height
        #
        randNode = Node(randomX, randomY)

        while not theMap.isOK(randNode):
            randomX = np.random.random() * theMap.width
            randomY = np.random.random() * theMap.height
            randNode = Node(randomX, randomY)

        nearestNode = nearestNeighbor(randNode, tree)

        newNode = nextStateKP(nearestNode, randNode, theMap.v_max, theMap.dt)

        if theMap.isOK(newNode):
            newNode.distance = nearestNode.distance + newNode.dist(nearestNode)

            # computes the speed, only for curiosity
            speed = newNode.dist(nearestNode) / theMap.dt
            if speed > maxSpeed:
                maxSpeed = speed

            newNode.parent = nearestNode
            nearestNode.children.append(newNode)
            tree.append(newNode)

        if newNode.dist(goal) < 7:
            if not isObstacleBetween(newNode, goal, theMap.obstacles):
                print("går in i local")
                finalPath = finalSample(newNode, goal, theMap.v_max, theMap.dt)
                for node in finalPath:
                    tree.append(node)
                path = makePath(tree[len(tree)-1], start)
                print("Constraints hold")
                print(checkVel(path, theMap.v_max, theMap.dt))
                print("The error in distance")
                print(tree[len(tree)-1].dist(goal))
                print("The error in velocity")
                print(np.linalg.norm(tree[len(tree)-1].vel-goal.vel))
                return tree, path
            else:
                continue
        #print(k)

    print("k: " + str(k))
    return tree, []

def nearestNeighbor(randNode, tree):
    bestNode = tree[0]
    bestDistance = randNode.dist(tree[0])

    for node in tree:
        distance = randNode.dist(node)
        if distance < bestDistance:
            bestDistance = distance
            bestNode = node
    return bestNode

def nextStateKP(nearestNode, randomNode, vMax, dt):
    vel = (randomNode.XY-nearestNode.XY)/dt
    velNorm = np.linalg.norm(vel)
    if velNorm > vMax:
        vel /= velNorm
    return Node(nearestNode.x + vel[0] * dt, nearestNode.y + vel[1] * dt, vel)

def finalSample(node, goal, vMax, dt):
    path = []
    currentNode = node
    while currentNode.dist(goal) > vMax * dt:
        newNode = nextStateKP(currentNode, goal, vMax, dt)
        currentNode.children.append(newNode)
        newNode.parent = currentNode
        newNode.distance = currentNode.distance + newNode.dist(currentNode)
        path.append(newNode)
        currentNode = newNode
    lastNode = Node(goal.x, goal.y, goal.vel)
    path[len(path)-1].children.append(lastNode)
    lastNode.parent = path[len(path)-1]
    lastNode.distance = lastNode.parent.distance + lastNode.dist(lastNode.parent)
    path.append(lastNode)
    return path

def findPathKP(theMap):
    #theMap = Map(filePath)

    start = Node(theMap.start[0], theMap.start[1], theMap.vel_start)
    goal = Node(theMap.goal[0], theMap.goal[1], theMap.vel_goal)

    tree, path = RRT(start, goal, theMap)

    print("Total distance travelled:")
    print(tree[len(tree)-1].distance)
    print("Total time:")
    print(len(path)* theMap.dt)

    plotMap(theMap.bounding_polygon, theMap.obstacles)
    plotTree(tree[0])
    plotPath(path[len(path)-1])

    plt.plot(start.XY[0], start.XY[1], "o", c = "g" )
    plt.plot(goal.XY[0], goal.XY[1], "o", c = "r" )

    plt.show()
    return path

if __name__ == "__main__":
    findPathKP(Map("P1.json"))