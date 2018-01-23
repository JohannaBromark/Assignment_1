import math
from main import Node

def heuristic(nodeFrom, nodeTo):
    """Takes two nodes and computes the distance between them"""
    return  math.sqrt(math.exp(nodeFrom.x - nodeTo.x) + math.exp(nodeFrom.y-nodeTo.y))

def generateNeighbors(current):
    """Takes a node and generates its neighbors as nodes"""
    neighbors = []
    x = current.x
    y = current.y
    neighbors.append(Node(x, y+1))
    neighbors.append(Node(x, y-1))
    neighbors.append(Node(x+1, y))
    neighbors.append(Node(x-1), y)
    return neighbors

