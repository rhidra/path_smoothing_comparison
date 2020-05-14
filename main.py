from world import generate_world, random_position, WIDTH, HEIGHT
from common import Simulation
from rrt_star import rrt_star
from astar import astar, preprocess_astar
from random_walk import random_walk
import path_processing, smoothing
import eval
import pygame
import sys
import time

def main():
    Simulation(WIDTH, HEIGHT)
    start = list(random_position())
    path = random_walk(start, 20, 100, WIDTH, HEIGHT)
    Simulation.draw_paths(start, None, [], path, (0,255,0))
    # time.sleep(1)
    path_smooth1 = smoothing.kalman_randomwalk(path)
    Simulation.draw_paths(start, None, [], path, (0,255,0), path_smooth1, (0,255,255))
    time.sleep(1)
    path_smooth2 = smoothing.bezier_divided(path)
    Simulation.draw_paths(start, None, [], path, (0,255,0), path_smooth1, (0,255,255), path_smooth2, (255,255,0))
    time.sleep(1)

    eval.eval(path, path_smooth1, path_smooth2)

    while True:
        for e in pygame.event.get():
            if e.type == pygame.locals.QUIT or (e.type == pygame.locals.KEYUP and e.key == pygame.locals.K_ESCAPE):
                sys.exit("Stopping...")


def rrt():
    # World generation
    [start, goal, obstacles] = generate_world()
    Simulation(WIDTH, HEIGHT)
    Simulation.draw(start, goal, obstacles)

    # RRT*
    path_rrt, _, _, _ = rrt_star(start, goal, obstacles, display=True)
    Simulation.draw_paths(start, goal, obstacles, path_rrt, (0,255,0))
    time.sleep(1)
    path_rrt = path_processing.over_sampling(path_rrt, 200)
    Simulation.draw_paths(start, goal, obstacles, path_rrt, (0,255,0))
    time.sleep(1)
    path_rrt_kalman = smoothing.kalman_randomwalk(path_rrt)
    Simulation.draw_paths(start, goal, obstacles, path_rrt, (0,255,0), path_rrt_kalman, (0,255,255))
    time.sleep(1)
    path_rrt_bezier = smoothing.bezier(path_rrt)
    Simulation.draw_paths(start, goal, obstacles, path_rrt, (0,255,0), path_rrt_kalman, (0,255,255), path_rrt_bezier, (255,255,0))

    eval.eval(path_rrt, path_rrt_kalman, path_rrt_bezier)

    while True:
        for e in pygame.event.get():
            if e.type == pygame.locals.QUIT or (e.type == pygame.locals.KEYUP and e.key == pygame.locals.K_ESCAPE):
                sys.exit("Stopping...")

def astar():
    # World generation
    [start, goal, obstacles] = generate_world()
    Simulation(WIDTH, HEIGHT)
    Simulation.draw(start, goal, obstacles)

    # A*
    args = preprocess_astar(start, goal, obstacles, WIDTH, HEIGHT)
    path_astar, _, _ = astar(*args, display=True)
    Simulation.draw_paths(start, goal, obstacles, path_astar, (0,255,0))
    time.sleep(1)
    path_astar = path_processing.filter(path_astar, obstacles)
    path_astar = path_processing.over_sampling(path_astar, 200)
    Simulation.draw_paths(start, goal, obstacles, path_astar, (0,255,0))

    path_kalman = smoothing.kalman_randomwalk(path_astar)
    Simulation.draw_paths(start, goal, obstacles, path_astar, (0,255,0), path_kalman, (0,255,255))
    time.sleep(1)

    path_bezier = smoothing.bezier(path_astar)
    Simulation.draw_paths(start, goal, obstacles, path_astar, (0,255,0), path_kalman, (0,255,255), path_bezier, (255,255,0))

    eval.eval(path_astar, path_kalman, path_bezier)

    while True:
        for e in pygame.event.get():
            if e.type == pygame.locals.QUIT or (e.type == pygame.locals.KEYUP and e.key == pygame.locals.K_ESCAPE):
                sys.exit("Stopping...")


if __name__ == '__main__':
    main()
