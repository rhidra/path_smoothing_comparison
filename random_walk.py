"""
Algorithm of random_walk path generation
"""

import numpy as np

def random_walk(start, speed, steps):
    path = [start]
    pos = np.array(start)

    for i in range(steps):
        pos = np.add(pos, (speed * np.random.randint(1,5)/2) * np.random.randint(-1, 2, size=2))
        path.append(list(pos))
    return path
