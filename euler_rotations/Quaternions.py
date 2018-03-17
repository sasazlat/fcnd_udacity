
# coding: utf-8

# ## Quaternions
#

# In the following exercise you'll implement functions to convert between Euler
# angles and quaternion representations.  It's useful to be able to easily
# navigate back and forth between these representations because of their
# relative strengths.  Quaternions are better for calculations, while Euler
# angles are far more intuitive.
#
# Some messages coming from your drone in simulation (or in the real world)
# will represent orientation data as a quaternion, while others use Euler
# angles.  So it's a good idea to be able to seamlessly handle both.
#
# The [`udacidrone` API
# imlementation](https://github.com/udacity/udacidrone/blob/master/udacidrone/connection/message_types.py#L189-L284)
# that you're using for the projects in this program already has these
# conversions implemented under the hood so that's a great place to start if
# you aren't sure how to complete this exercise!

# In[ ]:

import numpy as np

def euler_to_quaternion(angles):
    roll = angles[0]
    pitch = angles[1]
    yaw = angles[2]
    
    # TODO: complete the conversion
    # and return a numpy array of
    # 4 elements representing a quaternion [a, b, c, d]




def quaternion_to_euler(quaternion):
    a = quaternion[0]
    b = quaternion[1]
    c = quaternion[2]
    d = quaternion[3]
    
    # TODO: complete the conversion
    # and return a numpy array of
    # 3 element representing the euler angles [roll, pitch, yaw]


# Test the conversion.

# In[ ]:

euler = np.array([np.deg2rad(90), np.deg2rad(30), np.deg2rad(0)])

q = euler_to_quaternion(euler) # should be [ 0.683 0.683 0.183 -0.183]
print(q)

assert np.array_equal(euler, quaternion_to_euler(q))


# Here's our [solution](/notebooks/Quaternions-Solution.ipynb)!
