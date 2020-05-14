"""
A* path finding algorithm implementation
"""

import numpy as np
from functools import reduce
import matplotlib.pyplot as plt
import numpy as np
from common import Simulation
import common, math, time

# Utility class to handle nodes more easily
class Node:
    OBSTACLE = 1
    FREE = 0

    def __init__(self, pos, value):
        self.value = value
        self.pos = pos
        self.parent = None
        self.H = 0
        self.G = 0

    def __repr__(self):
        return 'Node: ' + self.pos.__repr__()


# Return all the children (neighbors) of a specific node
def children(pos,grid):
    a, b = math.floor(pos.pos[0]/under_sampling_factor), math.floor(pos.pos[1]/under_sampling_factor)
    c = [];
    for d in [(x,y) for x in range(a-1,a+2) for y in range(b-1,b+2)]:
        if d[0] < len(grid) and d[0] >= 0 and d[1] < len(grid[0]) and d[1] >= 0 and grid[d[0]][d[1]].value != Node.OBSTACLE:
            c.append(grid[d[0]][d[1]])
    return c


# Return the path computed by the A* optimized algorithm from the start and goal points
def astar(start, goal, grid, display=False):
    openset = set()
    closedset = set()

    current = start
    openset.add(current)

    i=0
    while openset:
        i = i+1

        current = min(openset, key=lambda o:o.G + o.H)

        discovered_neighbors = len(openset) + len(closedset)
        if current == goal:
            path = []
            while current.parent:
                path.append(list(current.pos))
                current = current.parent
            path.append(list(current.pos))
            return (path[::-1], i, discovered_neighbors)

        openset.remove(current)
        closedset.add(current)

        # Loop through the node's children/siblings
        # From start
        for node in children(current, grid):
            # If it is already in the closed set, skip it
            if node in closedset:
                continue

            if node in openset:
                # Check if we beat the G score
                new_g = current.G + common.dist(current, node)
                if node.G > new_g:
                    # If so, update the node to have a new parent
                    node.G = new_g
                    node.parent = current
            else:
                # If it isn't in the open set, calculate the G and H score for the node
                node.G = current.G + common.dist(current, node)
                node.H = common.dist(node, goal)
                node.parent = current
                openset.add(node)

        if display:
            Simulation.draw_astar(start, goal, grid, openset, under_sampling_factor, point=current.pos)
            # time.sleep(1/60)

    raise ValueError('No Path Found')

# The fine resolution grid is under sampled to reduce its resolution
# Bigger values give faster algorithm but less efficient algorithm
under_sampling_factor = 4

def preprocess_astar(start, goal, obstacles, width, height):
    grid = [[Node((x,y), Node.OBSTACLE if common.is_in_obstacle((x,y), obstacles) else Node.FREE)
            for y in range(0, height, under_sampling_factor)]
            for x in range(0, width, under_sampling_factor)]
    start = grid[math.floor(start[0]/under_sampling_factor)][math.floor(start[1]/under_sampling_factor)]
    goal = grid[math.floor(goal.center[0]/under_sampling_factor)][math.floor(goal.center[1]/under_sampling_factor)]
    return [start, goal, grid]
