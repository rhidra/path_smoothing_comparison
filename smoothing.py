import copy
import numpy as np
import time, math
import scipy.special
from scipy.integrate import odeint
from numpy.linalg import inv


# Bezier curve interpolation
# Ref: Bezier curve based smooth path planning for mobile robot, Song et al., 2011
def bezier_divided(path):
    new = []

    # Divide the curve in small group of 4 points
    # According to [6] of the ref, the Bezier curve is ideal if it is separated
    # by groups of 4 points
    for i in range(0, len(path)-3, 3):
        a, b, c, d = path[i], path[i+1], path[i+2], path[i+3]
        for l in np.arange(0,1,.1):
            x = (1-l)**3 * a[0] + 3*l*(1-l)**2 * b[0] + 3*l**2*(1-l) * c[0] + l**3 * d[0]
            y = (1-l)**3 * a[1] + 3*l*(1-l)**2 * b[1] + 3*l**2*(1-l) * c[1] + l**3 * d[1]
            new.append([x,y])
        new.append(d)
    if len(path)%4 != 0:
        new = new + path[-(len(path)%4):]
    return new


def bezier(path, steps=None):
    if steps is None:
        steps = len(path)*3
    new = []
    n = len(path)
    for l in np.arange(0,1,1/steps):
        x = 0
        y = 0
        for i, p in enumerate(path):
            factor = scipy.special.comb(n, i+1) * l**(i+1) * (1-l)**(n-i-1)
            x += factor * p[0]
            y += factor * p[1]
        new.append([x,y])
    new.append(path[-1])
    return new


# Kalman smoothing with a model of a random walk
# x_1(t+1) = x_1(t) + v_1(t)
# x_2(t+1) = x_2(t) + v_2(t) with v = [v_1 v_2] ~ N(0, 1)
#
# y_1(t) = x_1(t) + e_1(t)
# y_2(t) = x_2(t) + e_2(t) with e = [e_1 e_2] ~ N(0, 1)
def kalman(path):
    new = []

    # Noise estimation
    Q = np.eye(2)
    R = np.eye(2)

    # System model
    F = np.eye(2)
    H = np.eye(2)

    x_estim = np.array([path[0][0], path[0][1]]).reshape(2,1)
    P_estim = np.zeros((2,2))

    for i, y in enumerate(path):
        y = np.array(y).reshape(2,1) + np.random.rand(2,1)

        # Time update (t|t-1)
        x_predict = F.dot(x_estim)
        P_predict = F.dot(P_estim).dot(F.T) + Q

        # Measurements update (t|t)
        K = P_predict.dot(H.T).dot( inv(H.dot(P_predict).dot(H.T) + R))
        x_estim = x_predict + K.dot(y - H.dot(x_predict))
        P_estim = P_predict - K.dot(H.dot(P_predict).dot(H.T) + R).dot(K.T)

        new.append([x_estim[0,0], x_estim[1,0]])
    return new
