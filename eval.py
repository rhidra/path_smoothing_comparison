import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist

def eval(path, smooth, smooth2=None):
    path = np.array(path)
    smooth = np.array(smooth)
    if smooth2 is not None:
        smooth2 = np.array(smooth2)

    grid = plt.GridSpec(2, 2)
    plt.subplot(grid[0,0])
    d = np.diag(cdist(path, smooth))
    plt.plot(d, label='Raw-Kalman')
    if smooth2 is not None:
        d2 = np.diag(cdist(path, smooth[::int(len(smooth) / len(path))]))
        plt.plot(d2, label='Raw-Bezier')
    plt.title('Distance between paths')
    plt.legend()

    plt.subplot(grid[1,0])
    path_angles = path_angle(path)
    plt.plot(np.linspace(0,1,len(path_angles)), path_angles, label='Raw path')
    smooth_angles = path_angle(smooth)
    plt.plot(np.linspace(0,1,len(smooth_angles)), smooth_angles, label='Kalman filtered path')
    if smooth2 is not None:
        smooth_angles2 = path_angle(smooth2)
        plt.plot(np.linspace(0,1,len(smooth_angles2)), smooth_angles2, label='Bezier curve path')
    plt.title('Rotation angle along the path')
    plt.legend()

    plt.subplot(grid[:,1])
    plt.boxplot([path_angles, smooth_angles, smooth_angles2],
                labels=['Raw data', 'Kalman filtered path', 'Bezier curve path'])
    plt.ylabel('Angle (°)')
    plt.title('Mean rotation angle')

    plt.show()

def path_angle(path):
    # Compute the angle of rotation along a path
    angles = np.zeros(len(path))
    for i in range(1, len(path)-1):
        a, b, c = path[i-1], path[i], path[i+1]
        vec1, vec2 = b - a, c - b
        angles[i] = angle_between(vec1, vec2)
    return angles

def unit_vector(vector):
    # Returns the unit vector of the vector
    return vector / np.linalg.norm(vector) if np.linalg.norm(vector) != 0 else vector

def angle_between(v1, v2):
    # Returns the angle in radians between vectors 'v1' and 'v2'
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)) * 180 / np.pi
