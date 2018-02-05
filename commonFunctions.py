import numpy as np

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
