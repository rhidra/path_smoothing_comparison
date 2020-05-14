import numpy as np
from functools import reduce
import math, pygame
import matplotlib.pyplot as plt
import sys, time
import numpy as np
from common import Constants, Node, Simulation
import common as common
import world


# Radius of closeness for rerouting a node
RADIUS = 100


def rrt_star(start, goal, obstacles, display=False):
    nodes = []
    nodes.append(Node(start, None))

    i=0
    while i < Constants.MAX_ITERATIONS:
        i = i+1

        if i%10 == 0:
            rand = goal.topleft
        else:
            rand = world.random_position()

        if common.is_in_obstacle(rand, obstacles):
            continue

        # Search nearest node to the rand point
        nearest = nodes[0]
        for node in nodes:
            if node.pos == rand:
                nearest = None
                break
            if common.dist(node, rand) < common.dist(nearest, rand):
                nearest = node
        if not nearest:
            continue

        # Search the neighbors of the nearest node
        neighbors = []
        for node in nodes:
            if common.dist(node, nearest) < RADIUS and not common.line_in_obstacles(node.pos, rand, obstacles):
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
            if rand.cost + common.dist(rand, node) < node.cost:
                node.parent = rand
                node.cost = rand.cost + common.dist(rand, node)

        if common.dist(rand, goal.center, sqrt=True) < Constants.EPSILON:
            path, path_len = build_path(Node(goal.center, rand))
            return (path, nodes, i, path_len)

        if display:
            Simulation.draw(start, goal, obstacles, nodes, point=rand.pos if isinstance(rand, Node) else rand)
            # time.sleep(1/24)

    raise ValueError('No Path Found')


def build_path(current):
    path = []
    path_len = 0
    while current:
        path.append(list(current.pos))
        if current.parent:
            path_len += common.dist(current, current.parent, sqrt=True)
        current = current.parent
    return path[::-1], int(path_len)
