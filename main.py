from time import time as now
import numpy as np
import smoothing, eval, sys, path_generator

MEAN_ITER = 20

N_LIST = range(1, 100, 5)

def eval_time(algo, path_gen=path_generator.generate_random_walk, iter=MEAN_ITER):
    data = np.zeros((len(N_LIST), iter))
    path_len = np.zeros((len(N_LIST), iter))
    print('**************')
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
    kalman_rw = eval_time(smoothing.kalman, path_gen=path_generator.generate_random_walk)
    bezier_rw = eval_time(smoothing.bezier_divided, path_gen=path_generator.generate_random_walk)

    # kalman_rrt = eval_time(smoothing.kalman, path_gen=path_generator.generate_rrt)
    # bezier_rrt = eval_time(smoothing.bezier, path_gen=path_generator.generate_rrt)
    eval.display_time(['Random walk path generator', kalman_rw, 'Kalman filter smoothing', bezier_rw, 'Bezier splitted curve'])
                    #['RRT path generator', kalman_rrt, 'Kalman filter smoothing', bezier_rrt, 'Bezier curve'])



def random_walk_once():
    path, _ = path_generator.generate_random_walk(N=2)
    path_kalman = smoothing.kalman(path)
    path_bezier = smoothing.bezier_divided(path)

    eval.eval([], path, 'Raw path', path_kalman, 'Kalman filter curve', path_bezier, 'Bezier curve splitted')


def rrt_once():
    path, obstacles = path_generator.generate_rrt(N=2)
    path_kalman = smoothing.kalman(path)
    path_bezier = smoothing.bezier(path)

    eval.eval(obstacles, path, 'Raw path', path_kalman, 'Kalman filter curve', path_bezier, 'Bezier curve')


if __name__ == '__main__':
    # random_walk_once()
    # rrt_once()
    compare_time()
