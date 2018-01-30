import numpy as np
import matplotlib.pyplot as plt
import math
import Map

# Code for dynamic point

V_MAX = 1.1
DT = 0.1
A_MAX = 1.3
TOLERANCE = V_MAX * DT #/ 10
MAX_EDGE = V_MAX * DT + ((A_MAX*DT)**2)/2
FILEPATH = "P1.json"

class Node():

    def __init__(self, x, y, vel = [0,0]):
        self.x = x
        self.y = y
        self.coord = np.array([self.x, self.y])
        self.name = str(self.x)+","+str(self.y)
        self.parent = None
        self.children = [] # only used to plot the graph
        self.distance = 0 # Needed only to keep track of the distance to the "competition" or something
        # Need to keep track of the velocity, part of the state space
        self.vel = np.array(vel)


    def dist(self, otherNode):
        # Need to take into account the velocity as well
        return math.sqrt((otherNode.x - self.x) ** 2 + (otherNode.y - self.y) ** 2) #+
                         #(otherNode.vel[0]-self.vel[0])**2 + (otherNode.vel[1] - self.vel[1])**2)


def findNewNodeDP(nearestNeghbor, randomNode):
    """"Finds the acceleration needed and goes the the node"""
    # acceleation, a vector with ax and ay
    ax = 2*(randomNode.x - nearestNeghbor.x - nearestNeghbor.vel[0]* DT)/(DT**2)
    ay = 2*(randomNode.y - nearestNeghbor.y - nearestNeghbor.vel[1]* DT)/(DT**2)
    aTemp = np.array([ax, ay])
    # Normalize acceleration so it is not larger than A_MAX
    a = aTemp/np.linalg.norm(aTemp) * A_MAX

    # calculates the velocity from the acceleration
    #velx = nearestNeghbor.vel[0] + a[0]*DT
    #vely = nearestNeghbor.vel[1] + a[1]*DT
    #velTemp = np.array([velx, vely])
    #normalize it so it is not too fast
    #vel = velTemp/np.linalg.norm(velTemp) * V_MAX
    #print(math.sqrt(vel[0]**2 + vel[1]**2))

    # Position with the new
    tempNewx = nearestNeghbor.x + nearestNeghbor.vel[0]*DT + (a[0]*DT**2)/2
    tempNewy = nearestNeghbor.y + nearestNeghbor.vel[1]*DT + (a[1]*DT**2)/2

    velx = (tempNewx - nearestNeghbor.x)/DT
    vely = (tempNewy - nearestNeghbor.y)/DT

    velTemp = np.array([velx, vely])
    vel = velTemp/np.linalg.norm(velTemp) * V_MAX

    newx = nearestNeghbor.x + vel[0]*DT
    newy = nearestNeghbor.y + vel[1]*DT

    #print(newx)
    #print(newy)

    return Node(newx, newy, vel)

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

def RRT(start, goal):

    K = 1000
    tree = []

    newNode = start
    tree.append(start)
    maxSpeed = 0

    #goalRadius = timeToAdjustVel(goal.vel) * V_MAX / 2

    for k in range(K):
        # Bias towards the goal, every fifth k
        if k % 5 == 0:
            randomX = goal.x
            randomY = goal.y
        else:
            randomX = np.random.random() * mapp.width
            randomY = np.random.random() * mapp.height

        randNode = Node(randomX, randomY)

        if not mapp.isBlocked(randNode):
            nearestNode = nearestNeighbor(randNode, tree)

            ##MAX_EDGE kan nu inte använda max_vel, utan måste använda den faktiska hastigheten som beror på accelerationen.
            #ratio = MAX_EDGE / nearestNode.dist(randNode)

            newNode = findNewNodeDP(nearestNode, randNode)
            newNode.distance = nearestNode.distance + newNode.dist(nearestNode)

            # computes the speed, only for curiosity
            speed = math.sqrt((newNode.x - nearestNode.x)**2 + (newNode.y - nearestNode.y)**2)/DT
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
    print("vel: "+str(speed))
    # Returnerar tree, men om den inte hittat någon path blir det fel nu.
    return tree

# Funktion som kanske inte behöver användas, eller kan användas för att räkna ut målområde
def timeToAdjustVel(goalVel, vel):
    numDT = 0
    #print(np.multiply(goalVel, (-1)))
    #vel = np.multiply(goalVel, (-1))
    #vel = vel/np.linalg.norm(vel) * V_MAX
    #print(vel)
    while vel[0] != goalVel[0] or vel[1] != goalVel[1]:
        numDT += 1
        acc = np.array([(goalVel[0] - vel[0])/DT, (goalVel[1] - vel[1])/DT])

        if np.linalg.norm(acc) > A_MAX:
            acc = acc/np.linalg.norm(acc) * A_MAX
        vel[0] += acc[0] * DT
        vel[1] += acc[1] * DT
        if np.linalg.norm(vel) > V_MAX:
            vel = vel/np.linalg.norm(vel) * V_MAX
    return numDT

def localSearch(nearestNode, goal):
    """Funkar inte eftersom det blir svårt att koppla ihop noderna utan att accelerationen eller hsstigheten blir för stor"""
    currentGoal = Node(goal.x, goal.y, np.multiply(goal.vel, (-1)))
    currentNear = Node(nearestNode.x, nearestNode.y, nearestNode.vel)

    firstPathFromGoal = findLocalPath(currentGoal, currentNear, nearestNode.dist(goal)*2)
    firstPathFromNode = findLocalPath(currentNear, currentGoal, nearestNode.dist(goal)*2)

    newGoal = firstPathFromGoal[len(firstPathFromGoal)-1]
    newNode = firstPathFromNode[len(firstPathFromNode)-1]

    #secondPathFromGoal = findLocalPath(newGoal, newNode, 20)
    #secondPathFromNode = findLocalPath(newNode, newGoal, 20)

    #newGoal = secondPathFromGoal[len(firstPathFromGoal)-1]
    #newNode = secondPathFromNode[len(secondPathFromNode)-1]

    lastPath = []
    #lastPath = findLocalPath(firstPathFromNode[len(firstPathFromNode)//4], firstPathFromGoal[len(firstPathFromGoal)//4], 1)

    #middleNode = lastPath[len(lastPath)-1]
    #middleGoal = firstPathFromGoal[len(firstPathFromGoal)-1]

    #print(middleNode.name)
    #print(middleGoal.name)

    #Måste kolla så att det är godkänd hastighet och acceleration mellan middleNode och middleGoal


    finalPath = []
    for node in firstPathFromNode:
        finalPath.append(node)
    for node in lastPath:
        finalPath.append(node)
    i = len(firstPathFromGoal)-1
    #while i >= 0:
    #    finalPath.append(firstPathFromGoal[i])
    #    i-=1

    # Different returns depending on what you want
    #return finalPath
    #eturn firstPathFromGoal, firstPathFromNode, lastPath
    return firstPathFromNode, lastPath

def findLocalPath(node, goal, factor):
    pathFromGoal = []
    maxVel = 0
    maxAcc = 0

    while goal.dist(node) > TOLERANCE*factor:
        pathFromGoal.append(node)
        accFromNode = (goal.coord - node.coord - np.multiply(node.vel, DT))/DT**2

        if np.linalg.norm(accFromNode) > A_MAX:
            accFromNode = accFromNode/np.linalg.norm(accFromNode) * A_MAX

        velFromNode = node.vel + np.multiply(accFromNode, DT)

        if np.linalg.norm(velFromNode) > V_MAX:
            velFromNode = velFromNode/np.linalg.norm(velFromNode) * V_MAX
        prevNode = node

        node = Node(prevNode.x + velFromNode[0] * DT, prevNode.y + velFromNode[1] * DT, velFromNode)


        prevNode.children.append(node)

        # Test för accelerationen och hastigheten
        testVel = node.dist(prevNode) / DT
        if testVel > maxVel:
            maxVel = testVel
        testAcc = (node.vel - prevNode.vel)/DT

        if np.linalg.norm(testAcc) > maxAcc:
            maxAcc = np.linalg.norm(testAcc)

    print(maxVel)
    print(maxAcc)
    return pathFromGoal


def extendPoint(point):
    vel = np.multiply(point.vel, (-1))
    newPoint = Node(point.x + 3* vel[0]* DT, point.y + 3*vel[1] * DT, point.vel)
    return newPoint

def lastBit(point, goal):
    path = []
    path.append(point)
    while point.dist(goal) > TOLERANCE:
        #print(point.dist(goal))
        point = Node(point.x + point.vel[0]*DT, point.y + point.vel[1]*DT, point.vel)
        path.append(point)
    return path

def followCurve(startNode, curve):
    """"Follows the curve by keeping the same velocity but negative"""
    thePath = []
    thePath.append(startNode)
    currentNode = startNode
    thePath.append(startNode)
    for dt in range(len(curve)):
        #print("hastighet innan")
        #print(currentNode.vel)
        vel = np.multiply(curve[dt].vel, (-1))
        #print("hastighet efter")
        #print(vel)
        acc = np.multiply((vel - currentNode.vel), 1/DT)
        if np.linalg.norm(acc) > A_MAX:
            acc /= np.linalg.norm(acc)
            vel = currentNode.vel + np.multiply(acc, DT)
            print(dt)
            print(np.linalg.norm(vel - startNode.vel)/DT)
        acc2 = np.multiply((vel - currentNode.vel), 1/DT)
        print(np.linalg.norm(acc2))
        if np.linalg.norm(acc2) > A_MAX:
            print("ACCELERATIONEN!!!")

        nextNode = Node(currentNode.x + vel[0]*DT, currentNode.y + vel[1]*DT, vel)
        currentNode.children.append(nextNode)
        currentNode = nextNode
        thePath.append(currentNode)

    return thePath


def goalSearch(start, goal):
    startPath = []
    goalPath = []

    currentNode = start
    currentGoal = Node(goal.x, goal.y, np.multiply(goal.vel, (-1)))


    #while start.dist(currentGoal) > 2:
    #    startPath.append(currentNode)
    #    goalPath.append(currentGoal)
    #    prevNode = currentNode
    #    prevGoal = currentGoal
    #    currentNode = findNextNode(currentNode, currentGoal)
    #    currentGoal = findNextNode(currentGoal, currentNode)
    #    prevNode.children.append(currentNode)
    #    prevGoal.children.append(currentGoal)

    while currentNode.dist(currentGoal) > TOLERANCE*20: #currentNode.x != currentGoal.x or currentNode.y != currentGoal.y:
    #    #print("Current: "+str(currentNode.name))
    #    #print("Goal: " + currentGoal.name)
        startPath.append(currentNode)
        goalPath.append(currentGoal)
        prevNode = currentNode
        prevGoal = currentGoal
        currentNode = findNextNode(currentNode, currentGoal)
        currentGoal = findNextNode(currentGoal, currentNode)
        prevNode.children.append(currentNode)
        prevGoal.children.append(currentGoal)

    startPath.append(currentNode)
    goalPath.append(currentGoal)

    while currentNode.dist(currentGoal) > TOLERANCE:
        prevNode = currentNode
        currentNode = findNextNode(currentNode, currentGoal)
        prevNode.children.append(currentNode)
        startPath.append(currentNode)

    goalPath.reverse()

    lastPath = followCurve(startPath[len(startPath)-1], goalPath)


    distance = lastPath[len(lastPath)-1].dist(goal)

    print("avstånd innan")
    print(distance)
    while lastPath[len(lastPath)-1].dist(goal) > TOLERANCE:
        distance = lastPath[len(lastPath)-1].dist(goal)
        lastNode = lastPath[len(lastPath)-1]
        lastStep = Node(lastNode.x + lastNode.vel[0]*DT, lastNode.y + lastNode.vel[1]*DT, lastNode.vel)
        if lastStep.dist(goal) > distance:
            print("FAILIURE")
            break
        lastNode.children.append(lastStep)
        lastPath.append(lastStep)


    print("Avstånd :(")
    print(lastPath[len(lastPath)-1].dist(goal))

    for node in startPath:
        print(node.name)

    return startPath, lastPath, goalPath


def findNextNode(nodeFrom, nodeTo):
    acc = (nodeTo.coord - nodeFrom.coord - np.multiply(nodeFrom.vel, DT))/DT**2

    if np.linalg.norm(acc) > A_MAX:
        acc = acc/np.linalg.norm(acc) * A_MAX

    velocity = nodeFrom.vel + np.multiply(acc, DT)
    #print("velocity: "+str(np.linalg.norm(velocity)))

    if np.linalg.norm(velocity) > V_MAX:
        velocity = velocity/np.linalg.norm(velocity) * V_MAX

    newNode = Node(nodeFrom.x + velocity[0] * DT, nodeFrom.y + velocity[1] * DT, velocity)
    accBetween = newNode.dist(nodeFrom)/(DT**2)
    #print("Avstånd")
    #print(newNode.dist(nodeFrom))
    #print("Hastighet")
    #print(np.linalg.norm(newNode.vel))
    #print(newNode.dist(nodeFrom)/DT)
    #print("Acc vector")
    #print(np.linalg.norm((velocity-nodeFrom.vel)/DT))
    #print((np.linalg.norm(newNode.vel)**2 - np.linalg.norm(nodeFrom.vel)**2)/(2*newNode.dist(nodeFrom)))
    #print(accBetween)
    #print("Acc: " + str(np.linalg.norm(accBetween)))

    return newNode



mapp = Map.Map(FILEPATH)
startVel = mapp.vel_start
goalVel = mapp.vel_goal

#startNode = Node(1, 2, [startVel[0], startVel[1]])
#goalNode = Node(10, 15, [goalVel[0], goalVel[1]])


startNode = Node(1, 2, [0, 1.1])
goalNode = Node(1, 5, [-0.5, 0.5])

#startNode = Node(1, 2, [-0.7, -0.7])
#goalNode = Node(10, 15, [-0.5, 0.5])


#findNewNodeDP(startNode, goalNode)

#extendPoint(startNode, goalNode)

#print(timeToAdjustVel(goalVel, np.multiply(goalVel, (-1))) * V_MAX / 2)
#print(timeToAdjustVel(goalVel, (goalVel/np.linalg.norm(goalVel) * V_MAX)))


treeNode, treeGoal, treeMiddle = goalSearch(startNode, goalNode)

findNextNode(startNode, goalNode)
testPoint = extendPoint(goalNode)
print(testPoint.vel)
print(goalNode.vel)
path = lastBit(testPoint, goalNode)


#tree = RRT(startNode, goalNode)
#treeGoal, treeNode = localSearch(startNode, goalNode)
#treeGoal, treeNode, treeMiddle = localSearch(startNode, goalNode)
#tree = localSearch(startNode, goalNode)

#
#path = []
#currentNode = tree[len(tree)-1]
#while not currentNode == startNode:
#    #print(str(currentNode.name))
#    path.append(currentNode.name)
#    currentNode = currentNode.parent
#
#path.append(startNode.name)
#path.reverse()
#
#print(path)

#print("Distance: "+str(tree[len(tree)-1].distance))

#Plots the tree
#for i in range(len(tree)-1):
#    node = tree[i]
#    for child in node.children:
#        plt.plot([node.x, child.x], [node.y, child.y])
#
for i in range(len(treeGoal)-1):
    node = treeGoal[i]
    for child in node.children:
        plt.plot([node.x, child.x], [node.y, child.y], c = "r")
#
for i in range(len(treeNode)-1):
    node = treeNode[i]
    for child in node.children:
        plt.plot([node.x, child.x], [node.y, child.y], c = "b")
#
for i in range(len(treeMiddle)-1):
    node = treeMiddle[i]
    for child in node.children:
        plt.plot([node.x, child.x], [node.y, child.y], c = "g")
#
#print(testPoint.name)
plt.scatter(startNode.x, startNode.y, c = "g")
plt.scatter(goalNode.x, goalNode.y, c = "r")
#plt.scatter(testPoint.x, testPoint.y, c= "b")
#for node in path:
#    plt.scatter(node.x, node.y, c= "c")


plt.show()
