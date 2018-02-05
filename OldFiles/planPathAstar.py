import heapq
import numpy as np
import math
import json
import matplotlib.pyplot as plt
import time


grid = np.full((2, 2), True)
V_MAX = 1
V_GOAL = np.array([0.9, -0.2])

class Node(object):

    def __init__(self, x, y):
        self.x = x
        self.y = y

        # Behövs ej verkar det som
        self.name = str(self.x)+","+str(self.y)
        self.gScore = float("inf")
        self.fScore = float("inf")
        self.parent = None

    def setGscore(self, gvalue):
        self.gScore = gvalue

    def setFscore(self, fValue):
        self.fScore = fValue

    def getXY(self):
        return self.x, self.y

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __ne__(self, other):
        return self.x != other.x or self.y != other.y

    def __lt__(self, other):
        return self.fScore < other.fScore

    def setParent(self, parent):
        self.parent = parent

def coordFromVel(node, vel, dt):
    x,y = node.getXY()

    newX = x + dt*vel[0]
    newY = y + dt*vel[1]

    return newX, newY

def reconstruct_path(cameFrom, current):
    """Returns a string with the reconstructed path"""
    #current = current[1]
    if current.name in cameFrom:
        return reconstruct_path(cameFrom, cameFrom[current.name]) +" --> "+ current.name
    return current.name


def reconstructPath(cameFrom, current, path):
    """Returns a matrix with the reconstructed path. Each step is a row"""
    #current = current[1]
    if current.name in cameFrom:
        reconstructPath(cameFrom, cameFrom[current.name], path)
        string = current.name.split(",")

        path.append([int(string[0]), int(string[1])])
        return path
    string = current.name.split(",")
    return path.append([int(string[0]), int(string[1])])

def getNeighbor(current):
    x, y = current.getXY()
    limit = 6000
    neighbors = []
    if x < limit:
        neighbors.append(str(x+1)+","+str(y))
    if x > 0:
        neighbors.append(str(x-1)+","+str(y))
    if y < limit:
        neighbors.append(str(x)+","+str(y+1))
    if y > 0:
        neighbors.append(str(x)+","+str(y-1))

    if x > 0 and y > 0:
        neighbors.append(str(x-1)+","+str(y-1))
    if x > 0 and y < limit:
        neighbors.append(str(x-1)+","+str(y+1))
    if x < limit and y > 0:
        neighbors.append(str(x+1)+","+str(y-1))
    if x < limit and y < limit:
        neighbors.append(str(x+1)+","+str(y+1))

    return neighbors

def planPath(startCoord, goalCoord):
    start = Node(int(startCoord.split(",")[0]), int(startCoord.split(",")[1]))
    goal = Node(int(goalCoord.split(",")[0]), int(goalCoord.split(",")[1]))

    #start = grid[int(startCoord.split(",")[0])+10][int(startCoord.split(",")[1])+10]
    #goal = grid[int(goalCoord.split(",")[0])+10][int(goalCoord.split(",")[1])+10]

    queue = []
    visited = set()
    cameFrom = {}

    start.setGscore(0)
    start.setFscore(heuristic(start, goal))
    queue.append(start)
    neighbors = {}
    neighbors[start.name] = start

    i = 0
    while queue:
        i += 1
        #print(i)
        heapq.heapify(queue)
        current = heapq.heappop(queue)

        if current.name == goalCoord:
            #print("Hittad!")
            #return reconstruct_path(cameFrom, goal)

            path = []
            reconstructPath(cameFrom, goal, path)
            return path

            #return current

        visited.add(current.name)

        print("Nuvarande "+current.name)

        for neighborName in getNeighbor(current):
            #print("Grannamn "+neighborName)
            if neighborName in neighbors:
                #print("Granne är i dictionaryt")
                neighbor = neighbors[neighborName]

            else:
                #print("granne är inte i dictionary")
                coordStrings = neighborName.split(",")
                neighbor = Node(int(coordStrings[0]), int(coordStrings[1]))
                #print("namn: "+neighbor.name)
                neighbors[neighborName] = neighbor

            score = current.gScore + heuristic(current, neighbor)

            if neighbor in queue and neighbor.gScore > score:
                queue.remove(neighbor)

            if neighbor.name in visited and neighbor.gScore > score:
                visited.remove(neighbor.name)

            if neighbor not in queue and neighbor.name not in visited:

                #neighbor.setGscore(current.gScore + heuristic(current, neighbor))

                neighbor.setGscore(current.gScore + 1)
                neighbor.setFscore(neighbor.gScore + heuristic(neighbor, goal))
                queue.append(neighbor)
                cameFrom[neighbor.name] = current
                #neighbor.setParent(current)

            #print(queue)

def heuristic(nodeFrom, nodeTo):
    """Takes two nodes and computes the distance between them"""

    x1, y1 = nodeFrom.getXY()
    x2, y2 = nodeTo.getXY()

    return math.sqrt(((x1 - x2)**2 + (y1-y2)**2))

def printList(list):
    for i in list:
        print(i.name+" "+str(i.fScore))

def generateGrid():
    xLimit = 600
    yLimit = 600
    grid = []
    for i in range(xLimit+1):
        column = []
        for j in range(yLimit+1):
            column.append(Node(i,j))
        grid.append(column)
    return grid

def getPath(goal, path):
    if (goal.parent == None):
        return

def main():
    start = "1,1"
    goal = "500,100"
    #start_time = time.time()
    #grid = generateGrid()
    #end_time_grid = time.time()

    start_timePatt = time.time()

    pathInArr = planPath(start, goal)



    end_timePath = time.time()

    nod = Node(1,1)
    x,y = coordFromVel(nod, [1.1, 0.7], 0.1)

    #print(end_time_grid-start_time)
    print(start_timePatt-end_timePath)

    """
    goal = Node(480, 370)
    start = Node(0,0)
    print(heuristic(Node(148, 137), goal))
    print(heuristic(Node(82, 65), goal))
    print(heuristic(Node(148, 137), start))
    print(heuristic(Node(82, 65), start))
    """

    #print(x)
    #print(y)


    print(pathInArr)
    #print(len(pathInArr))
    array = np.asarray(pathInArr)

    plt.scatter(array[:,0], array[:,1])
    plt.show()


#def velFunc(neighbors):
#
#
#    fScore = float("inf")
#
#    generateNextNode()
#
#    for neighbor in neighbors:
#





if __name__ == "__main__":
    main()