"""
Algorithm for RRT* path finding algorithm
"""

import numpy as np
from functools import reduce
import math
import matplotlib.pyplot as plt
import sys, time
import numpy as np
from utils import Node
import utils
import world


# Radius of closeness for rerouting a node
REROUTING_RADIUS = 100
END_RADIUS = 50
MAX_ITERATIONS = 1000

# Main algorithm for the RRT* logic
def rrt_star(start, goal, obstacles):
    nodes = []
    nodes.append(Node(start, None))

    i=0
    while i < MAX_ITERATIONS:
        i = i+1

        if i%10 == 0:
            rand = [goal[0], goal[1]]
        else:
            rand = world.random_position()

        if utils.is_in_obstacle(rand, obstacles):
            continue

        # Search nearest node to the rand point
        nearest = nodes[0]
        for node in nodes:
            if node.pos == rand:
                nearest = None
                break
            if utils.dist(node, rand) < utils.dist(nearest, rand):
                nearest = node
        if not nearest:
            continue

        # Search the neighbors of the nearest node
        neighbors = []
        for node in nodes:
            if utils.dist(node, nearest) < REROUTING_RADIUS and not utils.line_in_obstacles(node.pos, rand, obstacles):
                neighbors.append(node)

        # Select best possible neighbor
        nearest = None
        min_cost = math.inf
        for node in neighbors:
            if node.cost < min_cost:
                nearest = node
                min_cost = node.cost

        if not nearest:
            continue

        rand = Node(rand, nearest)
        nodes.append(rand)

        # Rewiring of the tree
        for node in neighbors:
            if rand.cost + utils.dist(rand, node) < node.cost:
                node.parent = rand
                node.cost = rand.cost + utils.dist(rand, node)
        center = [goal[0] + goal[2] / 2, goal[1] + goal[3] / 2]
        if utils.dist(rand, center, sqrt=True) < END_RADIUS:
            path, path_len = build_path(Node(center, rand))
            return (path, nodes, i, path_len)

    raise ValueError('No Path Found')


def build_path(current):
    path = []
    path_len = 0
    while current:
        path.append(list(current.pos))
        if current.parent:
            path_len += utils.dist(current, current.parent, sqrt=True)
        current = current.parent
    return path[::-1], int(path_len)
