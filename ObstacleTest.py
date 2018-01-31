# Hitta Xmin, Xmax, Ymin, Ymax, för alla hinder. Om en punkt har en koordinat större än max eller mindre en min, är den inte i punkten.
# lägg till hitbox till hindret, räkna ut det när hindrena läses in eller nåt
# Ray casting for andra punkter

def makeBox(obstacles):
    maxMinBoxes = []
    for obstacle in obstacles:
        maxMin = findMaxMin(obstacle)
        maxMinBoxes.append(maxMin)
    return maxMinBoxes

def findMaxMin(obstacle):
    Xmin = float("inf")
    Ymin = float("inf")
    Xmax = -float("inf")
    Ymax = -float("inf")

    for corner in obstacle:
        if corner[0] < Xmin:
            Xmin = corner[0]
        if corner[0] > Xmax:
            Xmax = corner[0]
        if corner[1] < Ymin:
            Ymin = corner[1]
        if corner[1] > Ymax:
            Ymax = corner[1]
    return [Xmin, Xmax, Ymin, Ymax]

def checkIfHitBox(point, hitBoxes):
    posObstacles = []
    index = 0
    for box in hitBoxes:
        if point.x > box[0] and point.x <box[1] and point.y > box[2] and point.y < box[3]:
            posObstacles.append(index)
        index += 1
    return posObstacles


def nodeOK(node, obstacles):
    posObsacles = checkIfHitBox(node, obstacles)
    if len(posObsacles) > 0:
        # There is a possibility that the point is within an obstacle
        for obstacle in posObsacles:
            rayCastCheck(node, obstacle)


def rayCastCheck(node, obstacle):
    """"Obstacle walls 1-2, 2-3, 3-4, 4-1. Takes a point outside of the box
        Two lines intersect if (p1, q1, p2) and (p1, q1, q2) have different orientation AND
        (p2, q2, p1) and (p2, q2, q1) have different orientation. There is a special case not implemented yet.
        p1 = node to test, q1 = point outside box. p2 = corner in obstacle, q2 = connected corner
    """
    numIntersect = 0
    #Take point from outside of the box, now it is a line between the node and that point
    # x = minX -1, y = maxY, outside of the box
    outsideNode = [hitBox[0] - 1, hitBox[3]]

    for i in range(len(obstacle)):
        orientation1 = getOrientation(node.XY, outsideNode, obstacle[i])
        orientation2 = getOrientation(node.XY, outsideNode, obstacle[(i+1) % len(obstacle)])
        orientation3 = getOrientation(obstacle[i], obstacle[(i+1)%len(obstacle)], node.XY)
        orientation4 = getOrientation(obstacle[i], obstacle[(i+1)%len(obstacle)], outsideNode)
        if orientation1 != orientation2 and orientation3 != orientation4:
            numIntersect += 1

    if numIntersect%2 != 0:
        return False
    else:
        return True


def getOrientation(node1, node2, node3):
    """"Computes the orientation of the points. 0 = colinear, 1 = Clockwise, -1 = counterclockwise"""
    value = (node2[1] - node1[1]) * (node3[0] - node2[0]) - (node2[0] - node1[0]) * (node3[1] - node2[1])
    if value == 0:
        return 0
    return value / abs(value)





