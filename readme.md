# Robot Path Smoothing through Kalman filtering and Bézier Curves

## How to run

You need python 3, with Numpy, Matplotlib and Scipy installed.
With pip3, run the following command to install those libraries:

```shell script
pip install numpy matplotlib scipy
```

To launch the program, run one of the following command:

```shell script
# Run a random walk simulation
python main.py random

# Run a RRT* simulation
python main.py rrt

# Run multiple instances of the algorithms to compare time performances
python main.py time
```

## Files

- `main.py` : Run the core logic of the comparison between each algorithm
- `world.py` : Generate the world used by the RRT*
- `random_walk.py` : Algorithm of random_walk path generation
- `rrt_star.py` : Algorithm for RRT* path finding algorithm
- `path_generator.py` : Generate a path with the random walk model of the RRT*
- `path_processing.py` : Utility functions to modify a path (eg: path pruning)
- `smoothing.py` : Smoothing algorithms (Bézier, Piecewise Bézier and KFS)
- `eval.py` : Display stats and graphs to evaluate the performances of each algorithm
- `utils.py` : Utility functions (collision detection, path planning, ...)

## Paper

For more information about the algorithms, read `paper.pdf`.
