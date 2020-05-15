from world import generate_world, random_position, World
from rrt_star import rrt_star
from random_walk import random_walk
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import eval, path_processing


def generate_random_walk(N=10):
    steps = int(interp1d([1, 100], [10, 2000])(N))
    start = [0, 0]
    path = random_walk(start, speed=10, steps=steps)
    return [path, []]


def generate_rrt(N=10):
    World(N)
    [start, goal, obstacles] = generate_world()
    path, _, _, _ = rrt_star(start, goal, obstacles)
    path = path_processing.over_sampling(path, max_length=200)
    return [path, obstacles]
