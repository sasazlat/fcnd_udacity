
# coding: utf-8

# ## Bresenham

# In[3]:
import numpy as np
import matplotlib.pyplot as plt
#get_ipython().run_line_magic('matplotlib', 'inline')
plt.rcParams['figure.figsize'] = 12, 12


# Your task is to implement the bresenham function given two points $p_1$ and
# $p_2$ as inputs.  The function should return the list of grid cells required
# to draw the line.
#
# What conditions would warrant a move by 1 unit along the x-axis?  What about
# along the y-axis?
#
# The idea is to move based on whether the next $y$ value will be above or
# below the line from $p_1$ to $p_2$.  We can keep track of the current line
# value, effectively $f(x)$ where $f$ is the line equation by incrementing a
# counter variable by the slope $m$ whenever we move in the x-axis.
#
# The condition is then (in pseudocode):
#
# ```
# if f(x+1) > y + 1:
#     y += 1
# else:
#     x += 1
# ```
#
# So, if moving along the y-axis results in a y value that is below the line,
# then move along the y-axis, otherwise, move in the x-axis.
#
# But what about the case where `f(x+1) == y+1`?  This will be the result of
# every test case when the line slope `m = 1`.  In this case you have a choice
# to make:
# * Only identify cells that as "in collision" when the line actually passes
# through those cells (less conservative obstacle avoidance)
# * When the line passes directly through a corner of grid cells, identify all
# cells that share that corner as "in collision" (more conservative obstacle
# avoidance).
#
# These two options look like this for a line from (0, 0) to (5, 5):
#
# ![comparison](./bresenham_comparison.png)
#
# Try coding up both!  In the event that you've padded obstacles in your grid
# map with a sufficient safety margin, you can likely get away with the less
# conservative approach (or computer graphics based Bresenham implementation in
# the Python package shown below).
#

# In[7]:
def low_bres(p1, p2, swapped=False):
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    ystep = 1
    if dy < 0:
        ystep = -1
        dy = -dy
    xstep = 1
    if dx < 0:
        xstep = -1
        dx = -dx
    d = dy - dx
    i = x1
    j = y1
    cells = []
    while i <= x2:
        cells.append([i,j])
        if d < 0:
            i += xstep
            d += dy
        elif d == 0:
            # uncomment these two lines for conservative approach
            #cells.append([i+1, j])
            #cells.append([i, j+1])
            d += dy
            i += xstep  
            d -= dx
            j += ystep
        else:
            d -= dx
            j += ystep  

            #plt.plot([p1[0], p2[0]], [p1[1], p2[1]])
            #for q in cells:
            #    plt.plot([q[0], q[0] + 1], [q[1], q[1]], 'k')
            #    plt.plot([q[0], q[0] + 1], [q[1] + 1, q[1] + 1], 'k')
            #    plt.plot([q[0], q[0]], [q[1],q[1] + 1], 'k')
            #    plt.plot([q[0] + 1, q[0] + 1], [q[1], q[1] + 1], 'k')

            #plt.grid()
            #plt.axis('equal')
            #plt.xlabel("X")
            #plt.ylabel("Y")
            #plt.title("Integer based Bresenham algorithm")
            #plt.show()
     
    return cells

def up_bres(p1,p2):
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1
    xstep = 1
    if dx < 0:
        xstep = -1
        dx = -dx
    d = 2 * dx - dy
    i = x1
    cells = []
    for j in range(y1, y2 + 1):
        cells.append([i,j])
        if d > 0:
            i = i + xstep
            d = d - 2 * dy
        d = d + 2 * dx
        #plt.plot([p1[0], p2[0]], [p1[1], p2[1]])
        #for q in cells:
        #    plt.plot([q[0], q[0] + 1], [q[1], q[1]], 'k')
        #    plt.plot([q[0], q[0] + 1], [q[1] + 1, q[1] + 1], 'k')
        #    plt.plot([q[0], q[0]], [q[1],q[1] + 1], 'k')
        #    plt.plot([q[0] + 1, q[0] + 1], [q[1], q[1] + 1], 'k')

        #plt.grid()
        #plt.axis('equal')
        #plt.xlabel("X")
        #plt.ylabel("Y")
        #plt.title("Integer based Bresenham algorithm")
        #plt.show()

    return cells

def bres(p1, p2):

    # Setup initial conditions
    x1, y1 = p1
    x2, y2 = p2
    dx = x2 - x1
    dy = y2 - y1

    # Determine how steep the line is
    is_not_steep = abs(dy) < abs(dx)

    # Swap start and end points if necessary and store swap state
    swapped = False

    if is_not_steep:
        if x1 < x2:
            return low_bres(p1,p2)
        else:
            return low_bres(p2,p1)
    else:
        if y1 < y2:
            return up_bres(p1,p2)
        else:
            return up_bres(p2,p1)


def bresa(start, end):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
 
    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    error = dx // 2
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points


# Plotting the line with the cells which it crosses.

# In[10]:
p1 = (5, 0)
p2 = (7, -4)

cells = bres(p1, p2)

cells1 = bresa(p1, p2)
# print(cells)
plt.plot([p1[0], p2[0]], [p1[1], p2[1]])


for q in cells:
    plt.plot([q[0], q[0] + 1], [q[1], q[1]], 'k')
    plt.plot([q[0], q[0] + 1], [q[1] + 1, q[1] + 1], 'k')
    plt.plot([q[0], q[0]], [q[1],q[1] + 1], 'k')
    plt.plot([q[0] + 1, q[0] + 1], [q[1], q[1] + 1], 'k')

plt.grid()
plt.axis('equal')
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Integer based Bresenham algorithm")
plt.show()


plt.plot([p1[0], p2[0]], [p1[1], p2[1]])
for q in cells1:
    plt.plot([q[0], q[0] + 1], [q[1], q[1]], 'k')
    plt.plot([q[0], q[0] + 1], [q[1] + 1, q[1] + 1], 'k')
    plt.plot([q[0], q[0]], [q[1],q[1] + 1], 'k')
    plt.plot([q[0] + 1, q[0] + 1], [q[1], q[1] + 1], 'k')

plt.grid()
plt.axis('equal')
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Integer based Bresenham 1 algorithm")
plt.show()


# ### Python Bresenham Package
# For comparison let's have a look at the Python Bresenham package!
#
# First we need to install it:

# In[ ]:
from bresenham import bresenham


# Next we can run the same experiment as above and plot it up.

# In[ ]:


# Note: you can run this for any (x1, y1, x2, y2)
line = (5, 0, 7, -4)

cells = list(bresenham(line[0], line[1], line[2], line[3]))
print(cells)

plt.plot([line[0], line[2]], [line[1], line[3]])


for q in cells:
    plt.plot([q[0], q[0] + 1], [q[1], q[1]], 'k')
    plt.plot([q[0], q[0] + 1], [q[1] + 1, q[1] + 1], 'k')
    plt.plot([q[0], q[0]], [q[1],q[1] + 1], 'k')
    plt.plot([q[0] + 1, q[0] + 1], [q[1], q[1] + 1], 'k')

plt.grid()
plt.axis('equal')
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Python package Bresenham algorithm")
plt.show()

