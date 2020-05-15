import numpy as np
import utils
from scipy.interpolate import interp1d

GOAL_WIDTH = 10
GOAL_HEIGHT = 10

class World:
    def __init__(self, N):
        World.width = int(interp1d([1,100], [200, 10000])(N))
        World.height = int(interp1d([1,100], [200, 10000])(N))
        World.max_obs = int(interp1d([1, 100], [100, 140])(N))
        World.min_obs = int(interp1d([1, 100], [50, 60])(N))
        World.max_obs_width = int(interp1d([1, 100], [10, 1000])(N))
        World.max_obs_height = int(interp1d([1, 100], [10, 1000])(N))


def random_position():
    return (np.random.randint(World.width), np.random.randint(World.height))


def random_area(width, height, random_size=True):
    pos = random_position()
    if random_size:
        return [*pos, np.random.randint(width) + 10, np.random.randint(height) + 10]
    else:
        return [*pos, width, height]


def generate_obstacles(start, goal, num_obstacles=None):
    obstacles = []
    if num_obstacles:
        n_obs = num_obstacles
    else:
        n_obs = np.random.randint(World.max_obs - World.min_obs) + World.min_obs
    while len(obstacles) < n_obs:
        o = random_area(World.max_obs_width, World.max_obs_width)
        if not utils.box_in_box(goal, o) and not utils.point_in_box(start, o):
            obstacles.append(o)
    return obstacles


def generate_world(num_obstacles=None):
    # start = (Constants.WIDTH/2., Constants.HEIGHT/2.)
    # goal = utils.random_area(width=Constants.GOAL_WIDTH, height=Constants.GOAL_HEIGHT, random_size=False)
    start = [4, 4]
    goal = [World.width - GOAL_WIDTH*2, World.height - GOAL_HEIGHT*2, GOAL_WIDTH, GOAL_HEIGHT]

    obstacles = generate_obstacles(start, goal, num_obstacles=num_obstacles)

    return [start, goal, obstacles]
