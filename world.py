import pygame
import numpy as np
from common import Constants
import common as common

# WIDTH = 960
# HEIGHT = 720
WIDTH = 10000
HEIGHT = 20000

GOAL_WIDTH = 10
GOAL_HEIGHT = 10

MAX_OBSTACLES = 1000
MIN_OBSTACLES = 200
MAX_OBS_WIDTH = 500
MAX_OBS_HEIGHT = 500


def random_position():
    return (np.random.randint(WIDTH), np.random.randint(HEIGHT))


def random_area(width=MAX_OBS_WIDTH, height=MAX_OBS_HEIGHT, random_size=True):
    pos = random_position()
    if random_size:
        return pygame.Rect(pos[0], pos[1], np.random.randint(width)+10, np.random.randint(height)+10)
    else:
        return pygame.Rect(pos[0], pos[1], width, height)


def generate_obstacles(start, goal, num_obstacles=None):
    obstacles = []
    if num_obstacles:
        n_obs = num_obstacles
    else:
        n_obs = np.random.randint(MAX_OBSTACLES - MIN_OBSTACLES) + MIN_OBSTACLES
    while len(obstacles) < n_obs:
        o = random_area()
        if not o.colliderect(goal) and not o.collidepoint(start):
            obstacles.append(o)
    return obstacles


def generate_world(num_obstacles=None):
    # start = (Constants.WIDTH/2., Constants.HEIGHT/2.)
    # goal = common.random_area(width=Constants.GOAL_WIDTH, height=Constants.GOAL_HEIGHT, random_size=False)
    start = (4, 4)
    goal = pygame.Rect(WIDTH - GOAL_WIDTH*2, HEIGHT - GOAL_HEIGHT*2, GOAL_WIDTH, GOAL_HEIGHT)
    while goal.collidepoint(start[0], start[1]):
        goal = random_area(width=GOAL_WIDTH, height=GOAL_HEIGHT, random_size=False)

    obstacles = generate_obstacles(start, goal, num_obstacles=num_obstacles)

    return [start, goal, obstacles]
