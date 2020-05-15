import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.spatial.distance import cdist

def eval(obstacles, *args):
    paths = []
    labels = []
    for path, label in zip(args[0::2], args[1::2]):
        paths.append(np.array(path))
        labels.append(label)
    path_length = int(np.sqrt(np.sum(np.diff(paths[0], axis=1)**2, axis=0))[0])

    # Distance between paths
    grid = plt.GridSpec(2, 4)
    plt.subplot(grid[0,0])
    for path, label in zip(paths[1:], labels[1:]):
        d = np.diag(cdist(paths[0], path[::int(len(path) / len(paths[0]))]))
        plt.plot(d, label=label)
    plt.title('Distance between paths')
    plt.legend()

    # Rotation along paths
    plt.subplot(grid[1,0])
    angles = [path_angle(p) for p in paths]
    for angle, label in zip(angles, labels):
        plt.plot(np.linspace(0,1,len(angle)), angle, label=label)
    plt.title('Rotation angle along the path')
    plt.legend()

    # Stats about the rotation
    plt.subplot(grid[:,1])
    plt.boxplot(angles, labels=labels)
    plt.ylabel('Angle (°)')
    plt.title('Mean rotation angle (length={})'.format(path_length))

    # Display the world
    ax = plt.subplot(grid[:,2:4])
    for o in obstacles:
        rect = patches.Rectangle((o[0], o[1]), o[2], o[3], color=(1,0,0))
        ax.add_patch(rect)
    for path, label in zip(paths, labels):
        plt.plot(*zip(*path), '-o', label=label)
    plt.legend()

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
