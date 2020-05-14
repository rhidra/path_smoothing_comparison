from world import generate_world, random_position, WIDTH, HEIGHT
from rrt_star import rrt_star
from astar import astar, preprocess_astar
from random_walk import random_walk
import path_processing, smoothing
import eval
import sys
import time


def main():
    start = list(random_position())
    path = random_walk(start, speed=20, steps=1000)
    path_kalman = smoothing.kalman_randomwalk(path)
    path_bezier = smoothing.bezier_divided(path)

    eval.draw_paths(None, None, [], path, 'Raw path', path_kalman, 'Kalman filter path', path_bezier, 'Bezier curve')
    eval.eval(path, 'Raw path', path_kalman, 'Kalman filter curve', path_bezier, 'Bezier curve splitted')


def rrt():
    [start, goal, obstacles] = generate_world()
    path_rrt, _, _, _ = rrt_star(start, goal, obstacles)
    path_rrt = path_processing.over_sampling(path_rrt, 200)
    path_rrt_kalman = smoothing.kalman_randomwalk(path_rrt)
    path_rrt_bezier = smoothing.bezier(path_rrt, steps=len(path_rrt)*3)

    eval.draw_paths(start, goal, obstacles, path_rrt, 'Raw path', path_rrt_kalman, 'Kalman filter curve', path_rrt_bezier, 'Bezier curve')
    eval.eval(path_rrt, 'Raw path', path_rrt_kalman, 'Kalman filter curve', path_rrt_bezier, 'Bezier curve')


if __name__ == '__main__':
    main()
