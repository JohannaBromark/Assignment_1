import numpy as np
import math
from Map2 import Map
from plotFunctions import *


class Node():

    def __init__(self, x, y, vel=np.zeros((1, 2))):
        self.x = x
        self.y = y
        self.XY = np.array([self.x, self.y])
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

    errorMargin = theMap.dt * theMap.v_max

    for k in range(K):
        #randomX = goal.x
        #randomY = goal.y
        if k % 3 == 0:
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

        newNode = nextStateDP(nearestNode, randNode, theMap.v_max, theMap.a_max, theMap.dt)

        if theMap.isOK(newNode):
            newNode.distance = nearestNode.distance + newNode.dist(nearestNode)

            # computes the speed, only for curiosity
            speed = newNode.dist(nearestNode) / theMap.dt
            if speed > maxSpeed:
                maxSpeed = speed

            newNode.parent = nearestNode
            nearestNode.children.append(newNode)
            tree.append(newNode)

        if newNode.dist(goal):
            finalPath = finalSample(newNode, goal)


        if newNode.dist(goal) <= errorMargin:
            print("breakar")
            print("k: " + str(k))
            print("speed: " + str(maxSpeed))
            # return newNode
            return tree
        print(k)

    print("k: " + str(k))
    return tree  # , sampledPoint
    #

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

def nextStateDP(nearestNode, randomNode, vMax, aMax, dt):
    """"Finds the acceleration needed and goes the the node"""
    # acceleation, a vector with ax and ay
    ax = 2 * (randomNode.x - nearestNode.x - nearestNode.vel[0] * dt) / (dt ** 2)
    ay = 2 * (randomNode.y - nearestNode.y - nearestNode.vel[1] * dt) / (dt ** 2)
    a = np.array([ax, ay])

    # Normalize acceleration so it is not larger than A_MAX
    if np.linalg.norm(a) > aMax:
        a /= np.linalg.norm(a) * aMax

    # Position with the new node with accepted acceleration
    tempNewx = nearestNode.x + nearestNode.vel[0] * dt + (a[0] * dt ** 2) / 2
    tempNewy = nearestNode.y + nearestNode.vel[1] * dt + (a[1] * dt ** 2) / 2

    velx = (tempNewx - nearestNode.x) / dt
    vely = (tempNewy - nearestNode.y) / dt

    vel = np.array([velx, vely])
    if np.linalg.norm(vel):
        vel /= np.linalg.norm(vel) * vMax

    newx = nearestNode.x + vel[0] * dt
    newy = nearestNode.y + vel[1] * dt

    return Node(newx, newy, vel)



def main(filePath):
    theMap = Map(filePath)

    start = Node(theMap.start[0], theMap.start[1], theMap.vel_start)
    goal = Node(theMap.goal[0], theMap.goal[1], theMap.vel_goal)

    tree = RRT(start, goal, theMap)

    plotObstacles(theMap.bounding_polygon, theMap.obstacles)
    plotTree(tree[0])

    plt.plot(start.XY[0], start.XY[1], "o", c = "g" )
    plt.plot(goal.XY[0], goal.XY[1], "o", c = "r" )

    plt.show()


if __name__ == "__main__":
    main("P1.json")