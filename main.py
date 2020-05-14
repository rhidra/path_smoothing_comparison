from time import time as now
import smoothing, eval, sys, path_generator


def eval_time(algo):
    for size in []:
        start_time = now()
        smooth = algo(path)
        end_time = now()
    duration = end_time - start_time


def random_walk_once():
    path, _ = path_generator.generate_random_walk()
    path_kalman = smoothing.kalman(path)
    path_bezier = smoothing.bezier_divided(path)

    eval.draw_paths([], path, 'Raw path', path_kalman, 'Kalman filter path', path_bezier, 'Bezier curve')
    eval.eval(path, 'Raw path', path_kalman, 'Kalman filter curve', path_bezier, 'Bezier curve splitted')


def rrt_once():
    path, obstacles = path_generator.generate_rrt()
    path_kalman = smoothing.kalman(path)
    path_bezier = smoothing.bezier(path, steps=len(path)*3)

    eval.draw_paths(obstacles, path, 'Raw path', path_kalman, 'Kalman filter curve', path_bezier, 'Bezier curve')
    eval.eval(path, 'Raw path', path_kalman, 'Kalman filter curve', path_bezier, 'Bezier curve')


if __name__ == '__main__':
    # random_walk_once()
    rrt_once()
