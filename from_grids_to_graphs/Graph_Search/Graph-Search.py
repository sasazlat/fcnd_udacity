
# coding: utf-8

# ## Finding Your Way In The City (Graph Edition)
# In this notebook your attention will shift from grids to graphs.  At least
# for search ...
#
# Using Voronoi graphs and the medial axis transform we can find paths which
# maximize safety from obstacles.  In addition, graph representation allows
# further optimizations and more succinct queries.

# In[ ]:


# OK this might look a little ugly but...
# need to import the latest version of networkx
# This occassionally fails, so if the next cell
# doesn't show that you're using networkx 2.1
# please "restart and clear output" from "Kernel" menu
# above and try again.
import sys
#get_ipython().system('{sys.executable} -m pip install -I networkx==2.1')
import pkg_resources
pkg_resources.require("networkx==2.1")
import networkx as nx


# In[ ]:

nx.__version__


# In[1]:

import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid_and_edges
import numpy.linalg as LA
#get_ipython().run_line_magic('matplotlib', 'inline')


# In[ ]:

plt.rcParams['figure.figsize'] = 12, 12


# In[ ]:


# This is the same obstacle data from the previous lesson.
filename = 'Graph_Search/colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)


# Starting and goal positions in *(north, east)*.

# In[ ]:

start_ne = (25,  100)
goal_ne = (750., 370.)


# In[ ]:


# Static drone altitude (metres)
drone_altitude = 5
safety_distance = 3


# In[ ]:


# This is now the routine using Voronoi
grid, edges = create_grid_and_edges(data, drone_altitude, safety_distance)
print(len(edges))


# Plot the edges on top of the grid along with start and goal locations.

# In[ ]:


# equivalent to
# plt.imshow(np.flip(grid, 0))
plt.imshow(grid, origin='lower', cmap='Greys') 

for e in edges:
    p1 = e[0]
    p2 = e[1]
    plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-')

    
plt.plot(start_ne[1], start_ne[0], 'rx')
plt.plot(goal_ne[1], goal_ne[0], 'rx')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()


# We now have a graph, well at least visually.  The next step is to use the
# [`networkx`](https://networkx.github.io) to create the graph.  **NetworkX**
# is a popular library handling anything and everything related to graph data
# structures and algorithms.
#
# **NOTE:** In the initial import above it was imported with the `nx` alias.
#
# You're encouraged to read the documentation but here's a super quick tour:
#
# 1.  Create a graph:
#
# ```
# G = nx.Graph()
# ```
#
# 2.  Add an edge:
#
# ```
# p1 = (10, 2.2)
# p2 = (50, 40)
# G = nx.add_edge(p1, p2)
# ```
#
# 3 Add an edge with a weight:
#
# ```
# p1 = (10, 2.2)
# p2 = (50, 40)
# dist = LA.norm(np.array(p2) - np.array(p1))
# G = nx.add_edge(p1, p2, weight=dist)
# ```

# In[ ]:


# TODO: create the graph with the weight of the edges
# set to the Euclidean distance between the points


# You need a method to search the graph, and you'll adapt A* in order to do
# this.  The notable differences being the actions are now the outgoing edges
# and the cost of an action is that weight of that edge.

# In[ ]:

from queue import PriorityQueue

def heuristic(n1, n2):
    #TODO: define a heuristic
    return 0

###### THIS IS YOUR OLD GRID-BASED A* IMPLEMENTATION #######
###### With a few minor modifications it can work with graphs!  ####
#TODO: modify A* to work with a graph
def a_star(graph, heuristic, start, goal):
    
    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:        
            print('Found a path.')
            found = True
            break
            
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                cost = action.cost
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                new_cost = current_cost + cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    queue.put((new_cost, next_node))
                    
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


# ### Solution
# 
# This solution consists of two parts:
# 
# 1. Find the closest point in the graph to our current location, same thing for the goal location.
# 2. Compute the path from the two points in the graph using the A* algorithm.
# 3. Feel free to use any of the path pruning techniques to make the path even smaller! 
# 4. Plot it up to see the results!

# ### TODO: Write your solution!

# [our solution](/notebooks/Graph-Search-Solution.ipynb)
