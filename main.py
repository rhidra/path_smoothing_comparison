help="""\
Main file for the path smoothing comparison.
You need python 3, numpy, matplotlib and scipy installed.
To execute, run with python 3:

    # Run a random walk simulation
    python main.py random

    # Run a RRT* simulation
    python main.py rrt

    # Run multiple instances of the algorithms to compare time performances
    python main.py time

For more information about running the program, read the file 'readme.md'
For more information about the algorithms, read the paper 'paper.pdf'
"""
from time import time as now
import numpy as np
import sys
import smoothing, eval, sys, path_generator

MEAN_ITER = 10
N_LIST = range(1, 100, 10)
def eval_time(algo, path_gen=path_generator.generate_random_walk, iter=MEAN_ITER):
    data = np.zeros((len(N_LIST), iter))
    path_len = np.zeros((len(N_LIST), iter))
    for i, N in enumerate(N_LIST):
        print('[%d/%d] N = %d' % (i+1, len(N_LIST), N))
        j = 0
        while j < iter:
            try:
                path, _ = path_gen(N=N)
                start_time = now()
                smooth = algo(path)
                end_time = now()
                data[i, j] = end_time - start_time
                path_len[i, j] = len(path)
                j += 1
            except ValueError:
                pass
    return [np.mean(data, axis=1), np.std(data, axis=1), np.mean(path_len, axis=1)]


def compare_time():
    print('*** Evaluation of KFS in Random Walk ***')
    kalman_rw = eval_time(smoothing.kalman, path_gen=path_generator.generate_random_walk)
    print('*** Evaluation of Piecewise Bézier Curve in Random Walk ***')
    bezier_rw = eval_time(smoothing.bezier_divided, path_gen=path_generator.generate_random_walk)

    print('*** Evaluation of KFS in RRT* ***')
    kalman_rrt = eval_time(smoothing.kalman, path_gen=path_generator.generate_rrt)
    print('*** Evaluation of Bézier Curve in RRT* ***')
    bezier_rrt = eval_time(smoothing.bezier, path_gen=path_generator.generate_rrt)
    print('*** Evaluation of Piecewise Bézier Curve in RRT* ***')
    bezier_sp_rrt = eval_time(smoothing.bezier_divided, path_gen=path_generator.generate_rrt)

    eval.display_time(['Random walk path generator', kalman_rw, 'Kalman Filter Smoothing', bezier_rw, 'Piecewise Bézier Curve'],
                    ['RRT path generator', kalman_rrt, 'Kalman filter smoothing', bezier_sp_rrt, 'Piecewise Bézier curve', bezier_rrt, 'Bézier curve'])



def random_walk():
    path, _ = path_generator.generate_random_walk(N=2)
    path_kalman = smoothing.kalman(path)
    path_bezier = smoothing.bezier_divided(path)

    eval.eval([], path, 'Raw path', path_kalman, 'Kalman Filter Smoothing', path_bezier, 'Piecewise Bézier Curve')


def rrt():
    path, obstacles = path_generator.generate_rrt(N=2)
    path_kalman = smoothing.kalman(path)
    path_bezier = smoothing.bezier(path)
    path_bezier_sp = smoothing.bezier_divided(path)

    eval.eval(obstacles, path, 'Raw path', path_kalman, 'Kalman Filter Smoothing', path_bezier, 'Bézier Curve', path_bezier_sp, 'Piecewise Bézier Curve')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        exit(help)
    elif sys.argv[1] == 'random':
        random_walk()
    elif sys.argv[1] == 'rrt':
        rrt()
    elif sys.argv[1] == 'time':
        compare_time()
    else:
        exit(help)
