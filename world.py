import numpy as np
import utils

WIDTH = 960
HEIGHT = 720
# WIDTH = 10000
# HEIGHT = 20000

GOAL_WIDTH = 10
GOAL_HEIGHT = 10

MAX_OBSTACLES = 100
MIN_OBSTACLES = 50
MAX_OBS_WIDTH = 50
MAX_OBS_HEIGHT = 50


def random_position():
    return (np.random.randint(WIDTH), np.random.randint(HEIGHT))


def random_area(width=MAX_OBS_WIDTH, height=MAX_OBS_HEIGHT, random_size=True):
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
        n_obs = np.random.randint(MAX_OBSTACLES - MIN_OBSTACLES) + MIN_OBSTACLES
    while len(obstacles) < n_obs:
        o = random_area()
        if not utils.box_in_box(goal, o) and not utils.point_in_box(start, o):
            obstacles.append(o)
    return obstacles


def generate_world(num_obstacles=None):
    # start = (Constants.WIDTH/2., Constants.HEIGHT/2.)
    # goal = utils.random_area(width=Constants.GOAL_WIDTH, height=Constants.GOAL_HEIGHT, random_size=False)
    start = [4, 4]
    goal = [WIDTH - GOAL_WIDTH*2, HEIGHT - GOAL_HEIGHT*2, GOAL_WIDTH, GOAL_HEIGHT]

    obstacles = generate_obstacles(start, goal, num_obstacles=num_obstacles)

    return [start, goal, obstacles]
