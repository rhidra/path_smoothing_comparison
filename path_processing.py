import numpy as np
import copy
import common
from common import Simulation
import time

# Add points in a paths
# Used by RRT*
def over_sampling(path, max_length=5):
    new = []
    for i in range(len(path) - 1):
        a, b = path[i], path[i+1]
        new.append(a)
        last = a
        norm = common.dist(a, b, sqrt=True)
        dir_x = (b[0]-a[0]) * max_length / norm
        dir_y = (b[1]-a[1]) * max_length / norm
        while common.dist(last, b) > max_length * max_length:
            last = [last[0] + dir_x, last[1] + dir_y]
            new.append(last)
    new.append(path[-1])
    return new

# Remove unnecessary nodes
def filter(path, obstacles, display=False):
    new = [path[0]]
    i = 0
    while i < len(path):
        for j, target in reversed(list(enumerate(path))[i:]):
            if not common.line_in_obstacles(path[i], target, obstacles):
                new.append(target)
                i = j
                break
        if display:
            Simulation.draw_paths(None, None, obstacles, path, (0,255,0), new, (0,0,255))
            time.sleep(1/10)
        i += 1
    return new
