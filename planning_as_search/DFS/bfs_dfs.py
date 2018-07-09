# Import numpy and Enum
import numpy as np
import queue
from enum import Enum


# Define a start and goal location
start = (0, 0)
goal = (4, 4)
# Define your grid-based state space of obstacles and free space
grid = np.array([[0, 1, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 1, 0, 1, 0, 0],
                    [0, 0, 0, 1, 1, 0],
                    [0, 0, 0, 1, 0, 0]])

class Action(Enum):

    UP = (-1, 0)
    LEFT = (0, -1)
    DOWN = (1, 0)
    RIGHT = (0, 1)

    def __str__(self):
        if self == self.UP:
            return "^"
        if self == self.LEFT:
            return "<"
        if self == self.DOWN:
            return "v"
        if self == self.RIGHT:
            return ">"


def valid_actions(grid, current_position):

    valid = [Action.DOWN, Action.LEFT, Action.RIGHT, Action.UP]
    n,m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_position

    if x - 1 < 0 and grid[x - 1, y] == 1:
        valid.remove(Action.UP)
    if x + 1 > n and grid[x + 1, y] == 1:
        valid.remove(Action.DOWN)
    if y - 1 < 0 and grid[x, y - 1] == 1:
        valid.remove(Action.LEFT)
    if y + 1 > m and grid[x, y + 1] == 1:
        valid.remove(Action.RIGHT)

    return valid


def visualise_path(grid, path, start):
    x,y = start

    start_grid = np.zeros(np.shape(grid), dtype=np.str)
    start_grid[:] = ' '
    start_grid[grid[:] == 1] = 'O'

    for a in path:
        val_x, val_y = a.value
        start_grid[x,y] = str(a)
        x,y = (x + val_x, y + val_y)
    start_grid[x,y] = 'G'
    start_grid[start[0], start[1]] = 'S'
    return start_grid