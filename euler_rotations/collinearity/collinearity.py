
# coding: utf-8

# ## Collinearity Check
# Collinearity for any three points can be determined easily by taking the
# determinant of a matrix containing the points.

# In[ ]:


# Define Points (feel free to change these)
# By default these will be cast as int64 arrays
import numpy as np
p1 = np.array([1, 2])
p2 = np.array([2, 3])
p3 = np.array([3, 4])


# ### General Case
#
# Define a function to determine collinearity using the `np.linalg.det()`
# function.  Introduce the `epsilon` threshold to allow a tolerance for
# collinearity.  If the determinant is less than `epsilon` then the points are
# collinear.
#

# In[ ]:

def add_z(point):
    return np.append(point, [1.])

def collinearity_float(p1, p2, p3, epsilon=1e-2): 
    collinear = False
    # TODO: Add a third dimension of z=1 to each point
    p1 = add_z(p1)
    p2 = add_z(p2)
    p3 = add_z(p3)
    # TODO: Create the matrix out of three points
    mat = np.vstack((p1,p2,p3))
    # TODO: Calculate the determinant of the matrix.
    det = np.linalg.det(mat)
    # TODO: Set collinear to True if the determinant is less than epsilon
    collinear = det < epsilon

    return collinear


# ### Integer Case
# Define a function to take three points and test for collinearity by
# evaluating the determinant using the simplified version for the 2D case:
#
# $ det = x_1(y_2-y_3) + x_2(y_3-y_1) + x_3(y_1-y_2)$

# In[ ]:

def collinearity_int(p1, p2, p3): 
    collinear = False
    # TODO: Calculate the determinant of the matrix using integer arithmetic
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    det = x1*(y1-y3) + x2*(y3-y1) + x3*(y1-y2)
    # TODO: Set collinear to True if the determinant is equal to zero

    collinear = det == 0
    return collinear


# ### Test it and time it

# In[ ]:

import time
t1 = time.time()
collinear = collinearity_float(p1, p2, p3)
t_3D = time.time() - t1
print(t_3D)

t1 = time.time()
collinear = collinearity_int(p1, p2, p3)
t_2D = time.time() - t1
print(t_2D)
#if t_2D == 0:
#    t_2D = 0.1
print(t_3D - t_2D)


# ### Which one is faster ???

# Check the solution [here](/notebooks/Collinearity-Solution.ipynb).
