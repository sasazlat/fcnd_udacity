
# coding: utf-8

# ## Medial Axis
#

# In[1]:
import numpy as np
import matplotlib.pyplot as plt
from grid import create_grid
from skimage.morphology import medial_axis
from skimage.util import invert
from planning import a_star
#get_ipython().run_line_magic('matplotlib', 'inline')


# In[2]:
plt.rcParams['figure.figsize'] = 12, 12


# In[3]:


# This is the same obstacle data from the previous lesson.
filename = 'medial_axis/colliders.csv'
data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
print(data)


# Starting and goal positions in *(north, east)*.

# In[4]:
start_ne = (40,  510)
goal_ne = (700, 790)


# In[5]:


# Static drone altitude (meters)
drone_altitude = 5
safety_distance = 2


# In[6]:
grid = create_grid(data, drone_altitude, safety_distance)
invert_grid = invert(grid)
skeleton = medial_axis(invert_grid)


# Plot the edges on top of the grid along with start and goal locations.

# In[7]:


# equivalent to
# plt.imshow(np.flip(grid, 0))
plt.imshow(grid, origin='lower')
plt.imshow(skeleton, cmap='Greys', origin='lower', alpha=0.7)
    
plt.plot(start_ne[1], start_ne[0], 'rx')
plt.plot(goal_ne[1], goal_ne[0], 'rx')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()


# In[8]:
# TODO: Your start and goal location defined above
# will not necessarily be on the skeleton so you
# must first identify the nearest cell on the
# skeleton to start and goal
def find_start_goal(skel, start, goal):
    # TODO: find start and goal on skeleton
    # Some useful functions might be:
        # np.nonzero()
        # np.transpose()
        # np.linalg.norm()
        # np.argmin()
    non_zero = np.nonzero(skel)
    skel_cells = np.transpose(non_zero)
    start_min_dist = np.linalg.norm(np.subtract(start, skel_cells), axis=1).argmin()
    near_start = skel_cells[start_min_dist]
    goal_min_dist = np.linalg.norm(np.subtract(goal, skel_cells), axis=1).argmin()
    near_goal = skel_cells[goal_min_dist]
    
    return near_start, near_goal

def find_start_goal_slack(skel, start, goal):
    coords = np.transpose(np.nonzero(skel))
    dist = lambda curr, dest: np.linalg.norm(np.subtract(curr, dest))
    starters = [dist(p, start) for p in coords]
    enders = [dist(p, goal) for p in coords]
    near_start = tuple(coords[np.argmin(starters)])
    near_goal = tuple(coords[np.argmin(enders)])
    return near_start, near_goal

skel_start, skel_goal = find_start_goal(skeleton, start_ne, goal_ne)

print(start_ne, goal_ne)
print(skel_start, skel_goal)


# In[9]:
def heuristic_func(position, goal_position):
    return np.linalg.norm(np.subtract(position, goal_position))


# In[11]:


# Run A* on the skeleton
path, cost = a_star(invert(skeleton).astype(np.int), heuristic_func, tuple(skel_start), tuple(skel_goal))
print("Path length = {0}, path cost = {1}".format(len(path), cost))


# In[12]:


# Compare to regular A* on the grid
path2, cost2 = a_star(grid, heuristic_func, start_ne, goal_ne)
print("Path length = {0}, path cost = {1}".format(len(path2), cost2))


# In[14]:
plt.imshow(grid, origin='lower')
plt.imshow(skeleton, cmap='Greys', origin='lower', alpha=0.7)
    
plt.plot(start_ne[1], start_ne[0], 'rx')
plt.plot(goal_ne[1], goal_ne[0], 'rx')

plt.plot(skel_start[1], skel_start[0], 'gx')
plt.plot(skel_goal[1], skel_goal[0], 'gx')

plt.xlabel('EAST')
plt.ylabel('NORTH')
#plt.show()
curr_pos = skel_start
actual_path = []
actual_path2 = []
for action in path:
    curr_pos = (curr_pos[0] + action.delta[0], curr_pos[1] + action.delta[1])
    #curr_pos = (curr_pos[0] + action.delta[0], curr_pos[1] + action.delta[1])
    actual_path.append(curr_pos)
    actual_path2.append(curr_pos)

pp = np.array(actual_path)
pp2 = np.array(actual_path2)

#pp = np.array(path)
plt.plot(pp[:, 1], pp[:, 0], 'g')
#pp2 = np.array(path2)
plt.plot(pp2[:, 1], pp2[:, 0], 'r')

plt.xlabel('EAST')
plt.ylabel('NORTH')
plt.show()

