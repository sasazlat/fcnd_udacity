
# coding: utf-8

# # Breadth-First Search

# In this exercise you'll implement the breadth-first search algorithm.

# In[1]:


# Import numpy and Enum
import numpy as np
import queue
from enum import Enum
import BFS


# https://wiki.python.org/moin/TimeComplexity gives a solid overview of Python
# data structures and their time complexity.

# [`Enum`](https://docs.python.org/3/library/enum.html#module-enum) is used to
# represent possible actions on the grid.

# In[3]:


# Define a start and goal location
start = (0, 0)
goal = (4, 4)
# Define your grid-based state space of obstacles and free space
grid = np.array([[0, 1, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 1, 0, 1, 0, 0],
                    [0, 0, 0, 1, 1, 0],
                    [0, 0, 0, 1, 0, 0]])

# In[4]:


# Define your action set using Enum()
class Action(Enum): 
    LEFT = (0, -1)
    RIGHT = (0, 1)
    UP = (-1, 0)
    DOWN = (1, 0)
    
    def __str__(self):
        if self == self.LEFT:
            return '<'
        elif self == self.RIGHT:
            return '>'
        elif self == self.UP:
            return '^'
        elif self == self.DOWN:
            return 'v'      


# Define a function that returns a list of valid actions from the current node
def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN]
    #print(grid.shape)
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


def create_adjacenty_list(grid):
    result = {}
    n,m = grid.shape
    for y in range(m):
        for x in range(n):
            k = (x, y)
            valid = valid_actions(grid, k)
            result[k] = set()
            for v in valid:
                xun = (k[0] + (v.value)[0], k[1] + (v.value)[1])
                result[k].add(xun)
    return result

gridsa = create_adjacenty_list(grid)


# Define a function to visualize the path
def visualize_path(grid, path, start):
    """
    Given a grid, path and start position
    return visual of the path to the goal.
    
    'S' -> start 
    'G' -> goal
    'O' -> obstacle
    ' ' -> empty
    """
    # Define a grid of string characters for visualization
    sgrid = np.zeros(np.shape(grid), dtype=np.str)
    sgrid[:] = ' '
    sgrid[grid[:] == 1] = 'O'
    
    pos = start
    # Fill in the string grid
    for a in path:
        da = a.value
        sgrid[pos[0], pos[1]] = str(a)
        pos = (pos[0] + da[0], pos[1] + da[1])
    sgrid[pos[0], pos[1]] = 'G'
    sgrid[start[0], start[1]] = 'S'  
    return sgrid


# ### Breadth-First Algorithm
#
# In this section you will implement breadth-first search.  The main body of
# the function is already given.  You will need to implement the remaining
# `TODOs`.

# In[7]:



def depth_first(graph, start, goal):

    visited, stack = set(), [start]
    while stack:
        vertex = stack.pop()
        if vertex == goal:
            break
        if vertex not in visited:
            visited.add(vertex)
            stack.extend(graph[vertex] - visited)
    return visited

def dfs_paths(graph, start, goal):
    stack = [(start, [start])]
    while stack:
        (vertex, path) = stack.pop()
        for next in graph[vertex] - set(path):
            if next == goal:
                yield path + [next]
            else:
                stack.append((next, path + [next]))

path = list(dfs_paths(gridsa, start, goal))
shorthest_path = len(path[0])
idx = 0
for p in path:
    lenp = len(p)
    if lenp < shorthest_path:
        shorthest_path = lenp
        idx = path.index(p)
print (path[idx])

# ### Executing the search
#
# Run `breadth_first()` and reference the grid to see if the path makes sense.

# In[8]:
path_dfs = depth_first(gridsa, start, goal)
print(path_dfs)

path_bfs = BFS.breadth_first(grid, start, goal)
print(path_bfs)


# In[9]:


# S -> start, G -> goal, O -> obstacle
#visualize_path(grid, path, start)


# Check out our solution [here!](/notebooks/BFS-Solution.ipynb)

