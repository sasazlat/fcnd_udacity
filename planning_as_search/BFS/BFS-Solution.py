
# coding: utf-8

# # Breadth-First Search

# In this exercise you'll implement the breadth first search algorithm.

# In[1]:


from queue import Queue 
import numpy as np
from enum import Enum


# https://wiki.python.org/moin/TimeComplexity gives a solid overview of Python data structures and their time complexity.

# * [`Enum`](https://docs.python.org/3/library/enum.html#module-enum) is used to represent possible actions on the grid.
# * A [`Queue`](https://docs.python.org/3/library/queue.html) is used to cycle through the positions since it has O(1) complexity enqueue and dequeue times.

# In[18]:


start = (0, 0)
goal = (4, 4)
grid = np.array([
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 1, 0, 0],
    [0, 0, 0, 1, 1, 0],
    [0, 0, 0, 1, 0, 0],
])


# In[19]:


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
            
    
def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid = [Action.UP, Action.LEFT, Action.RIGHT, Action.DOWN]
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node
    
    # check if the node is off the grid or
    # it's an obstacle
    
    if x - 1 < 0 or grid[x-1, y] == 1:
        valid.remove(Action.UP)
    if x + 1 > n or grid[x+1, y] == 1:
        valid.remove(Action.DOWN)
    if y - 1 < 0 or grid[x, y-1] == 1:
        valid.remove(Action.LEFT)
    if y + 1 > m or grid[x, y+1] == 1:
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


# ### Breadth-First Algorithm
# 
# In this section you will implement breadth-first search. The main body of the function is already given. You will need to implement the remaining `TODOs`.

# In[20]:


def breadth_first(grid, start, goal):

    # TODO: Initialize the starting variables
    path = []
    queue = Queue()
    queue.put(start)
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        # TODO: Remove the first element from the queue
        current_node = queue.get()

        # TODO: Check if the current vertex corresponds to the goal state
        # and set `found` to True if that's the case.
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            # Get the new vertexes connected to the current vertex 
            for a in valid_actions(grid, current_node):
                # delta of performing the action
                da = a.value
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                
                # TODO: Check if the new vertex has not been visited before.
                # If the node has not been visited you will need to
                # 1. Mark it as visited
                # 2. Add it to the queue
                if next_node not in visited:                
                    visited.add(next_node)               
                    queue.put(next_node)
                    
                    branch[next_node] = (current_node, a)
             
    path = []
    if found:
        # retrace steps
        path = []
        n = goal
        while branch[n][0] != start:
            path.append(branch[n][1])
            n = branch[n][0]
        path.append(branch[n][1])
            
    return path[::-1]


# ### Executing the search
# 
# Run `breath_first()` and reference the grid to see if the path makes sense.

# In[21]:


path = breadth_first(grid, start, goal)
print(path)


# In[22]:


# S -> start, G -> goal, O -> obstacle
visualize_path(grid, path, start)

