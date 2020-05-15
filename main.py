from time import time as now
import smoothing, eval, sys, path_generator

MEAN_ITER = 10

N_LIST = range(1, 100, 10)

def eval_time(algo, path_gen=path_generator.random_walk, iter=MEAN_ITER):
    means = np.zeros(len(N_LIST))

    for i, N in enumerate(N_LIST):
        for _ in range(iter):
            path = path_gen(N=N)
            start_time = now()
            smooth = algo(path)
            end_time = now()
            means[i] += (end_time - start_time) / iter
    return means


def compare_time():
    pass


def random_walk_once():
    path, _ = path_generator.generate_random_walk(N=2)
    path_kalman = smoothing.kalman(path)
    path_bezier = smoothing.bezier_divided(path)

    eval.eval([], path, 'Raw path', path_kalman, 'Kalman filter curve', path_bezier, 'Bezier curve splitted')


def rrt_once():
    path, obstacles = path_generator.generate_rrt(N=2)
    path_kalman = smoothing.kalman(path)
    path_bezier = smoothing.bezier(path, steps=len(path)*3)

    eval.eval(obstacles, path, 'Raw path', path_kalman, 'Kalman filter curve', path_bezier, 'Bezier curve')


if __name__ == '__main__':
    random_walk_once()
    # rrt_once()
    # compare_time()
