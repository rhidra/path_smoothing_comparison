from world import generate_world, random_position, WIDTH, HEIGHT
import path_processing
from rrt_star import rrt_star
from random_walk import random_walk


def generate_random_walk():
    start = list(random_position())
    path = random_walk(start, speed=20, steps=50)
    return [path, []]


def generate_rrt():
    [start, goal, obstacles] = generate_world()
    path, _, _, _ = rrt_star(start, goal, obstacles)
    path = path_processing.over_sampling(path, 200)
    return [path, obstacles]
