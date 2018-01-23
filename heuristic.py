def heuristic(nodeFrom, nodeTo):
    """Takes two nodes and computes the distance between them"""
    return  math.sqrt(math.exp(nodeFrom.x - nodeTo.x) + math.exp(nodeFrom.y-nodeTo.y))