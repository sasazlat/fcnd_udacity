
# coding: utf-8

# # Breadth-First Search

# In this exercise you'll implement the breadth-first search algorithm.

# In[1]:


# Import numpy and Enum
import numpy as np
import queue
from enum import Enum


# https://wiki.python.org/moin/TimeComplexity gives a solid overview of Python
# data structures and their time complexity.

# [`Enum`](https://docs.python.org/3/library/enum.html#module-enum) is used to
# represent possible actions on the grid.

# In[3]:


# Define a start and goal location
start = (0, 0)
goal = (4, 4)
# Define your grid-based state space of obstacles and free space
grid = np.array([   [0, 1, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 1, 0, 1, 0, 0],
                    [0, 0, 0, 1, 1, 0],
                    [0, 0, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 1, 0, 1, 0, 0],
                    [0, 0, 0, 1, 1, 0],
                    [0, 0, 0, 0, 0, 0]
                    ])


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
    #print (grid.shape)
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
def bfs(grid, start, goal):
    # Below:
        # "queue" is meant to contain your partial paths
        # "visited" is meant to contain your visited cells
    # TODO: Replace the None values for "queue" and "visited" with data
    # structure types
    path = []
    q = queue.Queue() # TODO: Choose a data structure type for your partial paths
    q.put(start)
    visited = set() # TODO: Choose a data structure type for your visited list
    
    branch = {}
    found = False
    
    # Run loop while queue is not empty
    while not q.empty(): # e.g, replace True with queue.empty() if using a Queue:
        # TODO: Replace "None" to remove the first element from the queue
        current_node = q.get()
        visited.add(current_node)
        # TODO: Replace "False" to check if the current vertex corresponds to
        # the goal state
        if current_node == goal: 
            print('Found a path.')
            found = True
            break
        #if current_node in visited:
        #    continue
        else:
            # Get the new vertexes connected to the current vertex
            actions = valid_actions(grid, current_node)
            for a in actions:
                # delta of performing the action
                #print (a.name)
                da = a.value
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                # TODO: Check if the new vertex has not been visited before.
                # If the node has not been visited you will need to
                # 1.  Mark it as visited
                # 2.  Add it to the queue
                if next_node not in visited:
                    visited.add(next_node)             
                    q.put(next_node)
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

def dfs(grid, start, goal):
    # Below:
        # "queue" is meant to contain your partial paths
        # "visited" is meant to contain your visited cells
    # TODO: Replace the None values for "queue" and "visited" with data
    # structure types
    path = []
    stack = [] # TODO: Choose a data structure type for your partial paths
    stack.append(start)
    visited = set(start) # TODO: Choose a data structure type for your visited list
    
    branch = {}
    found = False
    
    # Run loop while queue is not empty
    while stack: # e.g, replace True with queue.empty() if using a Queue:
        # TODO: Replace "None" to remove the first element from the queue
        current_node = stack.pop()
        visited.add(current_node)
        # TODO: Replace "False" to check if the current vertex corresponds to
        # the goal state
        if current_node == goal: 
            print('Found a path.')
            found = True
            break
        #if current_node in visited:
        #    continue
        else:
            # Get the new vertexes connected to the current vertex
            actions = valid_actions(grid, current_node)
            for a in actions:
                # delta of performing the action
                #print (a.name)
                da = a.value
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                # TODO: Check if the new vertex has not been visited before.
                # If the node has not been visited you will need to
                # 1.  Mark it as visited
                # 2.  Add it to the queue
                if next_node not in visited:
                    visited.add(next_node)             
                    stack.append(next_node)
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
# Run `breadth_first()` and reference the grid to see if the path makes sense.

# In[8]:
path_bfs = bfs(grid, start, goal)
print(path_bfs)

path_dfs = dfs(grid, start, goal)
print(path_dfs)



# In[9]:


# S -> start, G -> goal, O -> obstacle
visualize_path(grid, path_bfs, start)

visualize_path(grid, path_dfs, start)


# Check out our solution [here!](/notebooks/BFS-Solution.ipynb)

