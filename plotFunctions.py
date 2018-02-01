import matplotlib.pyplot as plt
import queue


def plotObstacles(polygon, obstacles):
    def plotCorner(obstacle):
        for cor in range(len(obstacle)):
            c1 = obstacle[cor]
            c2 = obstacle[(cor + 1) % len(obstacle)]
            plt.plot([c1[0], c2[0]], [c1[1], c2[1]])

    plotCorner(polygon)
    for obstacle in obstacles:
        plotCorner(obstacle)

def plotTree(startNode):
     q = queue.Queue()
     q.put(startNode)
     while not q.empty():
         currentNode = q.get()
         for child in currentNode.children:
             q.put(child)
             plt.plot([currentNode.x, child.x], [currentNode.y, child.y])

