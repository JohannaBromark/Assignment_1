import numpy as np
import matplotlib.pyplot as plt
import math
import Map

#from scipy.interpolate import CubicSpline

# Code for dynamic point

V_MAX = 1.1
DT = 0.1
A_MAX = 1.3
TOLERANCE = V_MAX * DT #/ 10
MAX_EDGE = V_MAX * DT + ((A_MAX*DT)**2)/2
FILEPATH = "P1.json"

class Node():

    def __init__(self, x, y, vel = np.zeros((1,2))):
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

def RRT(start, goal, mapp):

    K = 10000
    tree = []

    newNode = start
    tree.append(start)
    maxSpeed = 0

    #goalRadius = timeToAdjustVel(goal.vel) * V_MAX / 2

    for k in range(K):
        #randomX = goal.x
        #randomY = goal.y
        # Bias towards the goal, every fifth k
        if k % 3 == 0:
            #print("Samplar goal")
            randomX = goal.x
            randomY = goal.y
        else:
            randomX = np.random.random() * mapp.width
            randomY = np.random.random() * mapp.height
        #
        randNode = Node(randomX, randomY)

        #if not mapp.isBlocked(randNode):
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
        print(k)

    print("k: "+str(k))
    print("vel: "+str(speed))
    # Returnerar tree, men om den inte hittat någon path blir det fel nu.
    return tree

# Funktion som kanske inte behöver användas, eller kan användas för att räkna ut målområde
#def timeToAdjustVel(goalVel, vel):
#    numDT = 0
#    #print(np.multiply(goalVel, (-1)))
#    #vel = np.multiply(goalVel, (-1))
#    #vel = vel/np.linalg.norm(vel) * V_MAX
#    #print(vel)
#    while vel[0] != goalVel[0] or vel[1] != goalVel[1]:
#        numDT += 1
#        acc = np.array([(goalVel[0] - vel[0])/DT, (goalVel[1] - vel[1])/DT])
#
#        if np.linalg.norm(acc) > A_MAX:
#            acc = acc/np.linalg.norm(acc) * A_MAX
#        vel[0] += acc[0] * DT
#        vel[1] += acc[1] * DT
#        if np.linalg.norm(vel) > V_MAX:
#            vel = vel/np.linalg.norm(vel) * V_MAX
#    return numDT
#
#def localSearch(nearestNode, goal):
#    """Funkar inte eftersom det blir svårt att koppla ihop noderna utan att accelerationen eller hsstigheten blir för stor"""
#    currentGoal = Node(goal.x, goal.y, np.multiply(goal.vel, (-1)))
#    currentNear = Node(nearestNode.x, nearestNode.y, nearestNode.vel)
#
#    firstPathFromGoal = findLocalPath(currentGoal, currentNear, nearestNode.dist(goal)*2)
#    firstPathFromNode = findLocalPath(currentNear, currentGoal, nearestNode.dist(goal)*2)
#
#    newGoal = firstPathFromGoal[len(firstPathFromGoal)-1]
#    newNode = firstPathFromNode[len(firstPathFromNode)-1]
#
#    #secondPathFromGoal = findLocalPath(newGoal, newNode, 20)
#    #secondPathFromNode = findLocalPath(newNode, newGoal, 20)
#
#    #newGoal = secondPathFromGoal[len(firstPathFromGoal)-1]
#    #newNode = secondPathFromNode[len(secondPathFromNode)-1]
#
#    lastPath = []
#    #lastPath = findLocalPath(firstPathFromNode[len(firstPathFromNode)//4], firstPathFromGoal[len(firstPathFromGoal)//4], 1)
#
#    #middleNode = lastPath[len(lastPath)-1]
#    #middleGoal = firstPathFromGoal[len(firstPathFromGoal)-1]
#
#    #print(middleNode.name)
#    #print(middleGoal.name)
#
#    #Måste kolla så att det är godkänd hastighet och acceleration mellan middleNode och middleGoal
#
#
#    finalPath = []
#    for node in firstPathFromNode:
#        finalPath.append(node)
#    for node in lastPath:
#        finalPath.append(node)
#    i = len(firstPathFromGoal)-1
#    #while i >= 0:
#    #    finalPath.append(firstPathFromGoal[i])
#    #    i-=1
#
#    # Different returns depending on what you want
#    #return finalPath
#    #eturn firstPathFromGoal, firstPathFromNode, lastPath
#    return firstPathFromNode, lastPath
#
#def findLocalPath(node, goal, factor):
#    pathFromGoal = []
#    maxVel = 0
#    maxAcc = 0
#
#    while goal.dist(node) > TOLERANCE*factor:
#        pathFromGoal.append(node)
#        accFromNode = (goal.coord - node.coord - np.multiply(node.vel, DT))/DT**2
#
#        if np.linalg.norm(accFromNode) > A_MAX:
#            accFromNode = accFromNode/np.linalg.norm(accFromNode) * A_MAX
#
#        velFromNode = node.vel + np.multiply(accFromNode, DT)
#
#        if np.linalg.norm(velFromNode) > V_MAX:
#            velFromNode = velFromNode/np.linalg.norm(velFromNode) * V_MAX
#        prevNode = node
#
#        node = Node(prevNode.x + velFromNode[0] * DT, prevNode.y + velFromNode[1] * DT, velFromNode)
#
#
#        prevNode.children.append(node)
#
#        # Test för accelerationen och hastigheten
#        testVel = node.dist(prevNode) / DT
#        if testVel > maxVel:
#            maxVel = testVel
#        testAcc = (node.vel - prevNode.vel)/DT
#
#        if np.linalg.norm(testAcc) > maxAcc:
#            maxAcc = np.linalg.norm(testAcc)
#
#    print(maxVel)
#    print(maxAcc)
#    return pathFromGoal
#
#
#def extendPoints(startPoint, endPoint):
#    vel = np.multiply(endPoint.vel, (-1))
#    newPointGoal = Node(endPoint.x + 3* vel[0]* DT, endPoint.y + 3*vel[1] * DT, endPoint.vel)
#    newPointStart = Node(startPoint.x + startPoint.vel[0]*DT, startPoint.y + startPoint.vel[1]*DT, startPoint.vel)
#    return newPointStart, newPointGoal
#
#def lastBit(point, goal):
#    path = []
#    path.append(point)
#    while point.dist(goal) > TOLERANCE:
#        #print(point.dist(goal))
#        point = Node(point.x + point.vel[0]*DT, point.y + point.vel[1]*DT, point.vel)
#        path.append(point)
#    return path
#
#def followCurve(startNode, curve):
#    """"Follows the curve by keeping the same velocity but negative"""
#    thePath = []
#    thePath.append(startNode)
#    currentNode = startNode
#    thePath.append(startNode)
#    for dt in range(len(curve)):
#        #print("hastighet innan")
#        #print(currentNode.vel)
#        vel = np.multiply(curve[dt].vel, (-1))
#        #print("hastighet efter")
#        #print(vel)
#        acc = np.multiply((vel - currentNode.vel), 1/DT)
#        if np.linalg.norm(acc) > A_MAX:
#            acc /= np.linalg.norm(acc)
#            vel = currentNode.vel + np.multiply(acc, DT)
#            print(dt)
#            print(np.linalg.norm(vel - startNode.vel)/DT)
#        acc2 = np.multiply((vel - currentNode.vel), 1/DT)
#        print(np.linalg.norm(acc2))
#        if np.linalg.norm(acc2) > A_MAX:
#            print("ACCELERATIONEN!!!")
#
#        nextNode = Node(currentNode.x + vel[0]*DT, currentNode.y + vel[1]*DT, vel)
#        currentNode.children.append(nextNode)
#        currentNode = nextNode
#        thePath.append(currentNode)
#
#    return thePath
#
def followCurveMod(startNode, curve):
    thePath = []
    thePath.append(startNode)
    currentNode = startNode

    for step in range(len(curve)):
        wantVel = np.multiply(curve[step].vel, (-1))
        print("jämförnod")
        print(curve[step].name)
        print("nuvarande nod")
        print(currentNode.name)
        velToPos = np.array([(curve[step].x - currentNode.x)/DT, (curve[step].y - currentNode.y )/DT])

        print("målhastighet")
        print(wantVel)
        print("Hastighet till punkt")
        print(velToPos)

        totVel = wantVel + velToPos * 0.1

        acc = (totVel - currentNode.vel) / DT

        if np.linalg.norm(acc) > A_MAX:
            acc /= np.linalg.norm(acc)
            totVel = currentNode.vel + np.multiply(acc, DT)

        if np.linalg.norm(totVel) > V_MAX:
            totVel /= np.linalg.norm(totVel)

        if np.linalg.norm((totVel - currentNode.vel)/DT) > A_MAX:
            print("ACCELERATIONEN")


        nextNode = Node(currentNode.x + totVel[0] * DT, currentNode.y + totVel[1] * DT, totVel)
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

    while currentNode.dist(currentGoal) > TOLERANCE*30: #currentNode.x != currentGoal.x or currentNode.y != currentGoal.y:
    #    #print("Current: "+str(currentNode.name))
    #    #print("Goal: " + currentGoal.name)
        #startPath.append(currentNode)
        goalPath.append(currentGoal)
        #revNode = currentNode
        prevGoal = currentGoal
        #currentNode = findNextNode(currentNode, currentGoal)
        currentGoal = findNextNode(currentGoal, currentNode)
        #prevNode.children.append(currentNode)
        prevGoal.children.append(currentGoal)




    startPath.append(currentNode)
    goalPath.append(currentGoal)
    for i in range(7):
        prevGoal = currentGoal
        currentGoal = Node(currentGoal.x + currentGoal.vel[0]*DT, currentGoal.y + currentGoal.vel[1] * DT, currentGoal.vel)
        prevGoal.children.append(currentGoal)
        goalPath.append(currentGoal)

    while currentNode.dist(currentGoal) > TOLERANCE:
        print("Avstånd")
        print(currentNode.dist(currentGoal))
        prevNode = currentNode
        print("Nod innan")
        print(currentNode.name)
        currentNode = findNextNode(currentNode, currentGoal)
        print("node efter")
        print(currentNode.name)
        prevNode.children.append(currentNode)
        startPath.append(currentNode)

    goalPath.reverse()

    lastPath = followCurveMod(startPath[len(startPath)-1], goalPath)


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

    print("Sista hastigheten")
    print(lastPath[len(lastPath)-1].vel)
    print("målhastighet")
    print(goal.vel)

    return startPath, lastPath, goalPath
#
#
def findNextNode(nodeFrom, nodeTo):
    acc = (nodeTo.XY - nodeFrom.XY - np.multiply(nodeFrom.vel, DT))/DT**2

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

#def findNextNodeMod(nodeFrom, nodeTo):
#    #print("FindNextNodeMOD")
#    wantVel = np.multiply(nodeTo.vel, (-1))
#    #print("jämförnod")
#    #print(nodeTo.name)
#    #print("nuvarande nod")
#    #print(nodeFrom.name)
#    velToPos = np.array([(nodeTo.x - nodeFrom.x) / DT, (nodeTo.y - nodeFrom.y) / DT])
#    """
#    print("målhastighet")
#    print(wantVel)
#    print("Hastighet till punkt")
#    print(velToPos)
#    """
#    totVel = wantVel + velToPos * 0.5
#    print("hastighet")
#    print(np.linalg.norm(totVel))
#
#    acc = (totVel - nodeFrom.vel) / DT
#
#    if np.linalg.norm(acc) > A_MAX:
#        acc /= np.linalg.norm(acc)
#        totVel = nodeFrom.vel + np.multiply(acc, DT)
#
#    if np.linalg.norm(totVel) > V_MAX:
#        totVel /= np.linalg.norm(totVel)
#
#    if np.linalg.norm((totVel - nodeFrom.vel) / DT) > A_MAX:
#        print("ACCELERATIONEN")
#
#    nextNode = Node(nodeFrom.x + totVel[0] * DT, nodeFrom.y + totVel[1] * DT, totVel)
#    return nextNode

def main():
    mapp = Map.Map(FILEPATH)
    startVel = mapp.vel_start
    goalVel = mapp.vel_goal

    #startNode = Node(1, 2, [startVel[0], startVel[1]])
    #goalNode = Node(10, 15, [goalVel[0], goalVel[1]])


    startNode = Node(1, 2, [-0.1, 0.9])
    goalNode = Node(2, 6, [-0.1, 0.5])
    print(startNode.dist(goalNode))

    #newStart, newGoal = extendPoints(startNode, goalNode)
    #x = [startNode.x, newStart.x, goalNode.x, newGoal.x]
    #y = [startNode.y, newStart.y, goalNode.y, newGoal.y]

    #for node in x:
    #    print(node)

    #pointsX = np.arange(startNode.x, goalNode.x, np.linalg.norm(goalNode.vel)*DT)

    #isThisPath = CubicSpline(x, y)

    #startNode = Node(1, 2, [-0.7, -0.7])
    #goalNode = Node(10, 15, [-0.5, 0.5])


    #findNewNodeDP(startNode, goalNode)

    #extendPoint(startNode, goalNode)

    #print(timeToAdjustVel(goalVel, np.multiply(goalVel, (-1))) * V_MAX / 2)
    #print(timeToAdjustVel(goalVel, (goalVel/np.linalg.norm(goalVel) * V_MAX)))


    treeNode, treeGoal, treeMiddle = goalSearch(startNode, goalNode)

    #findNextNode(startNode, goalNode)
    #testPoint = extendPoints(goalNode)
    #print(testPoint.vel)
    #print(goalNode.vel)
    #path = lastBit(testPoint, goalNode)


    #tree = RRT(startNode, goalNode, mapp)
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

    #print(testPoint.name)
    #plt.plot(x, y, "o")
    #plt.plot(pointsX, isThisPath(pointsX))

    plt.scatter(startNode.x, startNode.y, c = "g")
    plt.scatter(goalNode.x, goalNode.y, c = "r")
    #plt.scatter(testPoint.x, testPoint.y, c= "b")
    #for node in path:
    #    plt.scatter(node.x, node.y, c= "c")


    plt.show()

if __name__ == "__main__":
    main()