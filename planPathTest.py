import heapq
import numpy as np
import math
import matplotlib.pyplot as plt


grid = np.full((2, 2), True)
V_MAX = 1
V_GOAL = np.array([0.9, -0.2])

class Node(object):

    def __init__(self, x, y):
        self.x = x -10
        self.y = y -10

        # Behövs ej verkar det som
        self.name = str(self.x)+","+str(self.y)

    def setGscore(self, gvalue):
        self.gScore = gvalue

    def setFscore(self, fValue):
        self.fScore = fValue

    def getXY(self):
        return self.x+10, self.y+10

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __ne__(self, other):
        return self.x != other.x or self.y != other.y

    def __lt__(self, other):
        return self.fScore < other.fScore

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
        path.append(current.name.split(","))
        return path
    return path.append(current.name.split(","))


def getNeighbor(grid, current):
    x, y = current.getXY()

    neighbors = []
    if x < len(grid)-1:
        neighbors.append(grid[x+1][y])
    if x > 0:
        neighbors.append(grid[x-1][y])
    if y < len(grid[0])-1:
        neighbors.append(grid[x][y+1])
    if y > 0:
        neighbors.append(grid[x][y-1])

    return neighbors


def planPath(grid, startCoord, goalCoord):
    start = grid[int(startCoord.split(",")[0])+10][int(startCoord.split(",")[1])+10]
    goal = grid[int(goalCoord.split(",")[0])+10][int(goalCoord.split(",")[1])+10]

    queue = []
    visited = set()
    cameFrom = {}

    start.setGscore(0)
    start.setFscore(heuristic(start, goal))
    queue.append(start)

    while queue:
        heapq.heapify(queue)
        current = heapq.heappop(queue)

        if current.name == goalCoord:
            #return reconstruct_path(cameFrom, goal)
            path = []
            reconstructPath(cameFrom, goal, path)
            return path

        visited.add(current.name)

        for neighbor in getNeighbor(grid, current):

            score = current.gScore + heuristic(current, neighbor)

            if neighbor in queue and neighbor.gScore > score:
                queue.remove(neighbor)

            if neighbor.name in visited and neighbor.gScore > score:
                queue.remove(neighbor)

            if neighbor not in queue and neighbor.name not in visited:
                neighbor.setGscore(current.gScore + heuristic(current, neighbor))
                neighbor.setFscore(neighbor.gScore + heuristic(neighbor, goal))
                queue.append(neighbor)
                cameFrom[neighbor.name] = current

def heuristic(nodeFrom, nodeTo):
    """Takes two nodes and computes the distance between them"""

    x1,y1 = nodeFrom.getXY()
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

def main():
    start = "-10,-10"
    goal = "500,9"

    grid = generateGrid()
    pathInArr = planPath(grid, start, goal)

    nod = Node(1,1)
    x,y = coordFromVel(nod, [1.1, 0.7], 0.1)
    #print(x)
    #print(y)


    print(pathInArr)
    #print(len(pathInArr))
    #array = np.asarray(pathInArr)



if __name__ == "__main__":
    main()