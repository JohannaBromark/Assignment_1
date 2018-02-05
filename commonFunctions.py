import numpy as np
import math


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
        self.vel = vel


    def dist(self, otherNode):
        # Need to take into account the velocity as well
        return math.sqrt((otherNode.x - self.x) ** 2 + (otherNode.y - self.y) ** 2)


def makePath(lastNode, startNode):
    path = []
    node = lastNode
    while node != startNode:
        path.append(node)
        node = node.parent
    path.append(node)
    path.reverse()
    return path

def checkVelAcc(path, vMax, aMax, dt):
    for i in range(len(path)-1):
        node1 = path[1]
        node2 = path[2]
        vel = np.linalg.norm((node1.pos - node2.pos)/dt)
        acc = np.linalg.norm((node1.vel - node2.vel)/dt)
        if round(vel, 10) > vMax or round(acc, 10) > aMax:
            print(vel)
            print(acc)
            return False
    return True

def checkVel(path, vMax, dt):
    for i in range(len(path)-1):
        node1 = path[1]
        node2 = path[2]
        vel = np.linalg.norm((node1.pos - node2.pos)/dt)
        if round(vel, 10) > vMax :
            print(vel)
            return False
    return True

def savePath(filename, path):
    file = open(filename, "w")
    file.write(str(len(path))+"\n")
    for node in path:
        file.write(str(node.x)+"\n")
        file.write(str(node.y) + "\n")
        file.write(str(node.vel[0]) + "\n")
        file.write(str(node.vel[1]) + "\n")
        file.write(str(node.distance) + "\n")
    file.close()

def readPath(filename):
    path = []
    file = open(filename, "r")
    numNodes = int(file.readline())
    for i in range(numNodes):
        x = float(file.readline())
        y = float(file.readline())
        velx = float(file.readline())
        vely = float(file.readline())
        distance = float(file.readline())
        node = Node(x, y, np.array([velx, vely]))
        if len(path) != 0:
            node.parent = path[i-1]
        node.distance = distance
        path.append(node)
    file.close()
    return path

