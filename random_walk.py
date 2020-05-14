import numpy as np

def random_walk(start, speed, steps, width, height):
    path = [start]
    pos = np.array(start)

    for i in range(steps):
        pos = np.add(pos, (speed * np.random.randint(1,5)/2) * np.random.randint(-1, 2, size=2))
        pos = np.clip(pos, [0,0], [width-1, height-2])
        path.append(list(pos))
    return path
