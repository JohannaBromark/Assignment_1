import numpy as np
import matplotlib.pyplot as plt
import math
import Map
import plotFunctions

# Code for kinematic car

class Node():

    def __init__(self, pos, orientation, phi):
        self.pos = pos
        self.name = str(self.pos)
        self.parent = None
        self.children = [] # only used to plot the graph
        self.distance = 0 # Needed only to keep track of the distance to the "competition" or something
        self.orientation = orientation
        self.x = pos[0]
        self.y = pos[1]
        self.vel = []
        self.phi = phi


    def dist(self, otherNode):
        return np.linalg.norm(otherNode.pos - self.pos)


def findNewNodeKC(nearestNeighbor, randomNode):

    x_0 = nearestNeighbor.pos[0]
    y_0 = nearestNeighbor.pos[1]
    theta_0 = nearestNeighbor.orientation

    ##print(x_0, y_0, np.degrees(theta_0))
    ##print(randomNode.pos[0], randomNode.pos[1])

    angle = np.angle(complex(randomNode.pos[0] - x_0, randomNode.pos[1] - y_0))
    if angle > np.pi:
        angle -= (2 * np.pi)

    ##print(np.degrees(angle))
    
    angle_difference = angle - nearestNeighbor.orientation
    phi = np.arctan((angle_difference * L) / (V_MAX * DT))
    if abs(phi) > PHI_MAX:
        phi *= (PHI_MAX / abs(phi))

    new_theta = theta_0 + (V_MAX / L) * np.tan(phi) * DT
    new_x = x_0 + (L / np.tan(phi)) * (np.sin(new_theta) - np.sin(theta_0))
    new_y = y_0 - (L / np.tan(phi)) * (np.cos(new_theta) - np.cos(theta_0))

    newNode = Node(np.array([new_x, new_y]), new_theta, phi)

    return newNode

def localPlannerKC(nearestNode, tree, mapp):

    #RSR
    x1 = nearestNode.pos[0]
    x2 = mapp.goal[0]
    y1 = nearestNode.pos[1]
    y2 = mapp.goal[1]
    theta1 = nearestNode.orientation
    theta2 = goalNode.orientation

    print(x1, y1, theta1)

    angle = goalNode.orientation #np.angle(complex(goalNode.pos[0] - x_0, goalNode.pos[1] - y_0))
    #print(np.degrees(angle))

    pc1 = np.array([x1 - R_MIN * np.cos(theta1 - np.pi / 2), y1 - R_MIN * np.sin(theta1 - np.pi / 2)])
    pc2 = np.array([x2 + R_MIN * np.cos(theta2 - np.pi / 2), y2 + R_MIN * np.sin(theta2 - np.pi / 2)])
    tangents = getTangents(x1, x2, R_MIN, y1, y2, R_MIN)
    print(pc1, pc2)
    print(tangents)
    RSR = tangents[0]

    print(RSR)

    #Do the first "R" part
    #print(1)
    arcLength1 = arcLength(pc1, nearestNode.pos, RSR[0:2], R_MIN, "left")
    print("arcLength1: ", arcLength1)
    print(2 * np.pi * R_MIN)
    t = 0
    phi = PHI_MAX
    while t < (arcLength1 / V_MAX):
        new_theta = theta1 + (V_MAX / L) * np.tan(phi) * t
        new_x = x1 + (L / np.tan(phi)) * (np.sin(new_theta) - np.sin(theta1))
        new_y = y1 - (L / np.tan(phi)) * (np.cos(new_theta) - np.cos(theta1))
        
        newNode = Node(np.array([new_x, new_y]), new_theta, phi)
        newNode.parent = nearestNode
        nearestNode.children.append(newNode)
        tree.append(newNode)
        nearestNode = newNode

        t += DT

    x1 = nearestNode.pos[0]
    y1 = nearestNode.pos[1]
    theta1 = nearestNode.orientation

    #return tree

    #Do the "S" part
    #print(2)
    t = 0
    length = np.linalg.norm(np.array(RSR[2:4]) - np.array(RSR[0:2]))
    while t < length / V_MAX:
        new_x = V_MAX * np.cos(theta1) * t
        new_y = V_MAX * np.sin(theta1) * t

        newNode = Node(np.array([new_x, new_y]), theta1, 0)
        newNode.parent = nearestNode
        nearestNode.children.append(newNode)
        tree.append(newNode)
        nearestNode = newNode
        
        t += DT

    #Do the second "R" part
    #print(3)
    arcLength2 = arcLength(pc2, RSR[0:2], goalNode.pos, R_MIN, "right")
    t = 0
    phi = -PHI_MAX
    while t < (arcLength1 / V_MAX):
        new_theta = theta1 + (V_MAX / L) * np.tan(phi) * t
        new_x = x1 + (L / np.tan(phi)) * (np.sin(new_theta) - np.sin(theta1))
        new_y = y1 - (L / np.tan(phi)) * (np.cos(new_theta) - np.cos(theta1))
        
        newNode = Node(np.array([new_x, new_y]), new_theta, phi)
        newNode.parent = nearestNode
        nearestNode.children.append(newNode)
        tree.append(newNode)
        nearestNode = newNode

        t += DT
        
    return tree

def getTangents(x1, y1, r1, x2, y2, r2):
    tangents = np.zeros((4, 4))
    d_sq = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)
    if d_sq <= (r1 - r2) * (r1 - r2):
        return tangents

    d = np.sqrt(d_sq)
    vx = (x2 - x1) / d
    vy = (y2 - y1) / d

    i = 0
    for sign1 in range(1, -3, -2):
        c = (r1 - sign1 * r2) / d

        if (c * c) > 1:
            continue

        h = np.sqrt(max(0, 1 - c * c))
        for sign2 in range(1, -3, -2):
            nx = vx * c - sign2 * h * vy
            ny = vy * c + sign2 * h * vx

            tangents[i, 0] = x1 + r1 * nx
            tangents[i, 1] = y1 + r1 * ny
            tangents[i, 2] = x2 + sign1 * r2 * nx
            tangents[i, 3] = y2 + sign1 * r2 * ny
            i += 1
        

    return tangents

def arcLength(p1, p2, p3, r, d):
    V1 = p2 - p1
    V2 = p3 - p1
    theta = np.arctan2(V2[1], V2[0]) - np.arctan2(V1[1], V1[0])

    if theta < 0 and d == "left":
        theta += 2 * np.pi
    elif theta > 0 and d == "right":
        theta -= 2 * np.pi
    return abs(theta * r)

#def ValueBaseTSTS(pos, orientation):
    

def RRT(start, goal, mapp):

    K = 10000
    tree = []
    treeAccs = {}

    newNode = start
    tree.append(start)
    maxSpeed = 0



    for k in range(K):
        # Bias towards the goal, every fifth k
        #print(k)
        if k % 100 == 0:
            randomPos = goal.pos
            randNode = Node(randomPos, 0, 1)
        else:
            while True:
                randomPos = np.multiply(np.random.random(2), np.array([mapp.width, mapp.height]))
                randNode = Node(randomPos, 0, 1)
                if mapp.isOK(randNode):
                    break

            nearestNode = nearestNeighbor(randNode, tree)

            newNode = findNewNodeKC(nearestNode, randNode)

            #Plots the tree
##            for i in range(len(tree)-1):
##                node = tree[i]
##                for child in node.children:
##                    plt.plot([node.pos[0], child.pos[0]], [node.pos[1], child.pos[1]])
##            plt.show()
##
##            newNode.distance = nearestNode.distance + newNode.dist(nearestNode)

            # computes the speed, only for curiosity

            if mapp.isOK(newNode):
                speed = newNode.dist(nearestNode)/DT
                if speed > maxSpeed:
                    maxSpeed = speed

                #print(newNode.pos, nearestNode.pos)

                newNode.parent = nearestNode
                nearestNode.children.append(newNode)
                tree.append(newNode)

        print(newNode.dist(goal))
        
        if newNode.dist(goal) <= LOCAL_PLANNER_TOLERANCE:
            #print("breakar")
            #print("k: " + str(k))
            #print("speed: "+str(maxSpeed))
            #return newNode
            #print(newNode.pos, np.degrees(newNode.orientation))
            return nearestNode, tree

    #print("k: "+str(k))
    return tree


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

def findPathKC(mapp):

    goalNode = Node(np.array(mapp.goal), np.angle(complex(mapp.vel_goal[0], mapp.vel_goal[1])), 0)
    global goalNode
    startNode = Node(np.array(mapp.start), np.angle(complex(mapp.vel_start[0], mapp.vel_goal[1])), 0)
    global startNode
##    startNode = Node(np.array([0.1, 0.1]), np.pi / 4, 1)
##    goalNode = Node(np.array([10, 10]), 5 * np.pi / 4, 1)  
    L = mapp.length
    global L
    DT = mapp.dt
    global DT
    OMEGA_MAX = mapp.omega_max
    global OMEGA_MAX
    PHI_MAX = mapp.phi_max
    global PHI_MAX
    V_MAX = mapp.v_max
    global V_MAX
    TOLERANCE = V_MAX * DT
    global TOLERANCE
    R_MIN = L / np.tan(PHI_MAX)
    global R_MIN
    LOCAL_PLANNER_TOLERANCE = 2 * R_MIN
    global LOCAL_PLANNER_TOLERANCE
    #print(L, DT, OMEGA_MAX, PHI_MAX, V_MAX, TOLERANCE, R_MIN)

    nn, tree = RRT(startNode, goalNode, mapp)
    tree = localPlannerKC(nn, tree, mapp)
    
    path = []
    currentNode = tree[len(tree)-1]
    while not currentNode == startNode:
        ##print(str(currentNode.name))
        path.append(currentNode)
        currentNode = currentNode.parent

    path.append(startNode)
    path.reverse()

    ##print(path)

    #print("Distance: "+str(tree[len(tree)-1].distance))


    #Plots the tree
    for i in range(len(tree)-1):
        node = tree[i]
        for child in node.children:
            time_series = np.linspace(0, DT, 10)
            if node.phi == 0:
                x_array = node.pos[0] + V_MAX * np.cos(node.orientation) * time_series
                y_array = node.pos[1] + V_MAX * np.sin(node.orientation) * time_series
            else:
                x_array = node.pos[0] + (L / np.tan(child.phi)) * (np.sin(node.orientation + (V_MAX / L) * np.tan(child.phi) * time_series) - np.sin(node.orientation))
                y_array = node.pos[1] - (L / np.tan(child.phi)) * (np.cos(node.orientation + (V_MAX / L) * np.tan(child.phi) * time_series) - np.cos(node.orientation))
            ##print(child.omega)
            ##print(x_array[9], child.pos[0], node.pos[0] + (V_MAX / child.omega) * (np.sin(child.orientation + child.omega * DT) - np.sin(child.orientation)))
            ##print(time_series)
            plt.plot(x_array, y_array)

    plt.scatter(startNode.pos[0], startNode.pos[1], c = "g")
    plt.scatter(goalNode.pos[0], goalNode.pos[1], c = "r")

    #print(len(path) * DT)

    #plotFunctions.plotMap(mapp.bounding_polygon, mapp.obstacles)

    plt.show()

    return path
