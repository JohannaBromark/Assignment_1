import numpy as np
import math
from Map2 import Map
from plotFunctions import *
from obstacleCheck import isObstacleBetween


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
        self.vel = vel


    def dist(self, otherNode):
        # Need to take into account the velocity as well
        return math.sqrt((otherNode.x - self.x) ** 2 + (otherNode.y - self.y) ** 2)

def RRT(start, goal, theMap):
    # Har fått rätt på 6 av 6 på P3
    K = 15000
    tree = []
    tree.append(start)
    newNode = start

    errorMargin = theMap.dt * theMap.v_max

    for k in range(K):
        # 3 works for P1, 100 pretty good for P2 and P3
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

        newNode = nextStateDP(nearestNode, randNode, theMap.v_max, theMap.a_max, theMap.dt)

        if theMap.isOK(newNode):
            newNode.distance = nearestNode.distance + newNode.dist(nearestNode)
            newNode.parent = nearestNode
            nearestNode.children.append(newNode)
            tree.append(newNode)

            if newNode.dist(goal) < 7:
                if not isObstacleBetween(newNode, goal, theMap.obstacles):
                    finalPath = finalSample(newNode, goal, theMap.v_max, theMap.a_max, theMap.dt)
                    if np.linalg.norm(goal.vel - finalPath[len(finalPath)-1].vel) > errorMargin:
                        #print("hastigheten är fel, moving on")
                        for node in finalPath:
                            node.children.clear()
                        continue
                    else:
                        for node in finalPath:
                            tree.append(node)
                            path = makePath(tree[len(tree)-1], start)
                        print("Constraints hold")
                        print(checkVelAcc(path, theMap.v_max, theMap.a_max, theMap.dt))
                        print("The error in distance")
                        print(tree[len(tree)-1].dist(goal))
                        print("The error in velocity")
                        print(np.linalg.norm(tree[len(tree)-1].vel-goal.vel))

                        return tree, path
                else:
                    continue
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

def nextStateDP(nodeFrom, nodeTo, vMax, aMax, dt):
    acc = (nodeTo.XY - nodeFrom.XY - np.multiply(nodeFrom.vel, dt))/dt**2

    if np.linalg.norm(acc) > aMax:
        acc = acc/np.linalg.norm(acc) * aMax

    velocity = nodeFrom.vel + np.multiply(acc, dt)

    if np.linalg.norm(velocity) > vMax:
        velocity = velocity/np.linalg.norm(velocity) * vMax

    newNode = Node(nodeFrom.x + velocity[0] * dt, nodeFrom.y + velocity[1] * dt, velocity)

    return newNode

def finalSample(node, goal, vMax, aMax, dt):
    """The nodes in the goal path has the absolut value of the goal velocity"""
    margin = vMax * dt
    startPath = []
    goalPath = []

    currentNode = node
    currentGoal = Node(goal.x, goal.y, np.multiply(goal.vel, (-1)))

    # Computes a trajectory from the goal to follow
    while currentGoal.dist(currentNode) > margin * 30: # Vet att den kommer köras när avståndet är max 7
        goalPath.append(currentGoal)
        prevGoal = currentGoal
        currentGoal = nextStateDP(prevGoal, currentNode, np.linalg.norm(goal.vel), aMax, dt)
        # Needed to plot the tree later, not needed for the final path
        prevGoal.children.append(currentGoal)

    # Adds the last node to the path, goalPath not needed for the final path only for reference
    goalPath.append(currentGoal)

    while currentNode.dist(currentGoal) > margin:
        prevNode = currentNode
        currentNode = nextStateDP(currentNode, currentGoal, vMax, aMax, dt)
        ### Måste ev. kontrollera så denna inte är i ett hinder ###
        currentNode.distance = prevNode.distance + currentNode.dist(prevNode)
        prevNode.children.append(currentNode)
        currentNode.parent = prevNode
        startPath.append(currentNode)

    goalPath.reverse()

    lastPath = followCurveMod(startPath[len(startPath)-1], goalPath, vMax, aMax, dt)

    while lastPath[len(lastPath)-1].dist(goal) > margin:
        # # # kanske ska fortsätta försöka få sista hastigheten här
        distance = lastPath[len(lastPath)-1].dist(goal)
        lastNode = lastPath[len(lastPath)-1]
        lastStep = Node(lastNode.x + lastNode.vel[0]*dt, lastNode.y + lastNode.vel[1]*dt, lastNode.vel)
        if lastStep.dist(goal) > distance:
            print("FAILIURE")
            break
        lastNode.children.append(lastStep)
        lastPath.append(lastStep)

    for node in lastPath:
        # # # Kolla så inte det är dublett av nod där listorna möts
        prevNode = startPath[len(startPath)-1]
        node.distance = prevNode.distance + node.dist(prevNode)
        startPath.append(node)

    return startPath

def followCurveMod(startNode, curve, vMax, aMax, dt):
    """Tries to follow both the path and the velocity"""
    thePath = []
    thePath.append(startNode)
    currentNode = startNode

    for step in range(len(curve)):
        wantVel = np.multiply(curve[step].vel, (-1))
        velToPos = np.array([(curve[step].x - currentNode.x)/dt, (curve[step].y - currentNode.y )/dt])

        # Prioritizes end velocity, so scales down the velocity to position
        totVel = wantVel + velToPos * 0.05

        acc = (totVel - currentNode.vel) / dt

        if np.linalg.norm(acc) > aMax:
            acc /= np.linalg.norm(acc)
            totVel = currentNode.vel + np.multiply(acc, dt)

        if np.linalg.norm(totVel) > vMax:
            totVel /= np.linalg.norm(totVel)

        #if np.linalg.norm((totVel - currentNode.vel)/dt) > aMax:
        #    print("ACCELERATIONEN")

        nextNode = Node(currentNode.x + totVel[0] * dt, currentNode.y + totVel[1] * dt, totVel)
        currentNode.children.append(nextNode)
        nextNode.parent = currentNode
        currentNode = nextNode
        thePath.append(currentNode)

    return thePath

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
        vel = np.linalg.norm((node1.XY - node2.XY)/dt)
        acc = np.linalg.norm((node1.vel - node2.vel)/dt)
        if round(vel, 10) > vMax or round(acc, 10) > aMax:
            print(vel)
            print(acc)
            return False
    return True

def main(filePath):
    theMap = Map(filePath)

    start = Node(theMap.start[0], theMap.start[1], theMap.vel_start)
    goal = Node(theMap.goal[0], theMap.goal[1], theMap.vel_goal)

    tree, path = RRT(start, goal, theMap)

    plotMap(theMap.bounding_polygon, theMap.obstacles)
    plotTree(tree[0])
    plotPath(tree[len(tree)-1])

    print("Total distance travelled:")
    print(tree[len(tree)-1].distance)
    print("Total time:")
    print(len(path)* theMap.dt)

    plt.plot(start.XY[0], start.XY[1], "o", c = "g" )
    plt.plot(goal.XY[0], goal.XY[1], "o", c = "r" )

    plt.show()


if __name__ == "__main__":
    main("P1.json")
