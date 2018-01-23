def heuristic(nodeFrom, nodeTo):
    return  math.sqrt(math.exp(nodeFrom.x - nodeTo.x) + math.exp(nodeFrom.y-nodeTo.y))