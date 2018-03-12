
# coding: utf-8

# # Uniform Cost Search

# In this exercise you'll implement extend breadth-first search by
# incorporating a cost for each action.

# In[ ]:
import numpy as np
from enum import Enum
from queue import PriorityQueue


# https://wiki.python.org/moin/TimeComplexity gives a solid overview of Python
# data structures and their time complexity.

# * [`Enum`](https://docs.python.org/3/library/enum.html#module-enum) is used
# to represent possible actions on the grid.

# In[ ]:
class Action(Enum):
    """
    An action is represented by a 3 element tuple.
    
    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """
    LEFT = (0, -1, 1)
    RIGHT = (0, 1, 1)
    UP = (-1, 0, 1)
    DOWN = (1, 0, 1)
    
    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'
    
    @property
    def cost(self):
        return self.value[2]
    
    @property
    def delta(self):
        return (self.value[0], self.value[1])
            
    
def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN]
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node
    
    # check if the node is off the grid or
    # it's an obstacle
    
    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid.remove(Action.UP)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid.remove(Action.DOWN)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid.remove(Action.LEFT)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid.remove(Action.RIGHT)
        
    return valid

def visualize_path(grid, path, start):
    sgrid = np.zeros(np.shape(grid), dtype=np.str)
    sgrid[:] = ' '
    sgrid[grid[:] == 1] = 'O'
    
    pos = start
    
    for a in path:
        da = a.value
        sgrid[pos[0], pos[1]] = str(a)
        pos = (pos[0] + da[0], pos[1] + da[1])
    sgrid[pos[0], pos[1]] = 'G'
    sgrid[start[0], start[1]] = 'S'  
    return sgrid


# ### Cost Search
#
# In this section you will extend the breadth-first search algorithm by
# incorporating a cost for each action.  Your task is to compute the lowest
# cost path.  Does this change the data structures you should use?
#
# You will need to implement the remaining `TODOs`.

# In[ ]:
def uniform_cost(grid, start, goal):

    # TODO: Initialize the starting variables
    path = []
    queue = PriorityQueue()
    visited = set()
    queue.put(start)
    
    branch = {}
    found = False
    
    while not queue.empty():
        # TODO: Remove the first element from the queue
        current_node = queue.get(start)
        # TODO: Check if the current vertex corresponds to the goal state
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # TODO: determine the next_node using the action delta
                da = action.value
                next_node = (current_node[0] + da[0], current_node[1])
                # TODO: compute the new cost
                new_cost = da[2]
                
                # TODO: Check if the new vertex has not been visited before.
                # If the node has not been visited you will need to
                # 1.  Mark it as visited
                # 2.  Add it to the queue
                if next_node not in visited:
                    visited.add(next_node)
                    queue.put(next_node)                  
                    
                    branch[next_node] = (new_cost, current_node, action)
             
    path = []
    path_cost = 0
    if found:
        
        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][2])
            n = branch[n][1]
        path.append(branch[n][2])
            
    return path[::-1], path_cost


# ### Executing the search
#
# Run `uniform_cost()` and reference the grid to see if the path makes sense.

# In[ ]:
start = (0, 0)
goal = (4, 4)

grid = np.array([[0, 1, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 0, 1, 0],
    [0, 0, 0, 1, 0, 0],])


# In[ ]:
path, path_cost = uniform_cost(grid, start, goal)
print(path_cost, path)


# In[ ]:


# S -> start, G -> goal, O -> obstacle
visualize_path(grid, path, start)


# [Solution](/notebooks/Cost-Solution.ipynb)

