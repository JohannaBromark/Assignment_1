from planPathRRTDP import Node
from Map import *
from obstacleCheck import *

def main():
    nodeGreen = Node(3, 5)
    nodeRed = Node(6, 3)
    nodeFalse = Node(4.8, 4)
    nodeOut = Node(-10, -10)
    nodeObs = Node(20, 12)
    nodeBetw = Node(4.9, 7.8)

    map = Map("P1.json")

    obstacles = [[[5, 3], [7, 5], [7, 8], [4, 7]], [[4,8], [5,8], [5,9], [4,9]]]
    hitboxes = makeBoxes(obstacles)


    #print("Grön")
    #print(len(checkIfHitBox(nodeGreen, hitboxes)))

    #rint("Röd")
    #print(len(checkIfHitBox(nodeRed, hitboxes)))

    #print(len(checkIfHitBox(nodeFalse, hitboxes)))

    #print(isBlocked(nodeRed, obstacles, hitboxes))

    #print(isBlocked(nodeFalse, obstacles, hitboxes))

    #print(map.isOK(nodeFalse))
    print(map.isOK(nodeOut))

    print(map.isOK(nodeObs))

    #print(isBlocked(nodeBetw, obstacles, hitboxes))

    #print(isCornerOnLine([21, 11], [19, 13], [20, 12]))


if __name__ == "__main__":
    main()