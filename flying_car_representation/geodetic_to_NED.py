
# coding: utf-8

# ### Geodetic to NED

# In[1]:


# First import the utm and numpy packages
import utm
import numpy


# To convert a GPS position (_longitude_, _latitude_, _altitude_) to a local
# position (_north_, _east_, _down_) you need to define a global home position
# as the origin of your NED coordinate frame.  In general this might be the
# position your vehicle is in when the motors are armed, or some other home
# base position.  You first task is to define a function to convert from global
# position to a local position using the `utm`.  To do this fill in the TODO's
# below!

# In[2]:

def global_to_local(global_position, global_home):
    
    # TODO: Get easting and northing of global_home
    (long_home, lat_home, alt_home) = global_home
    (easting_home, northing_home, zone_home, zone_home_letter) = utm.from_latlon(lat_home, long_home)
    # TODO: Get easting and northing of global_position
    (long_global, lat_global, alt_global) = global_position
    (easting_global, northing_global, zone_global, zone_global_letter) = utm.from_latlon(lat_global, long_global)
    # TODO: Create local_position from global and home positions
    local_position = numpy.array([northing_global - northing_home, easting_global - easting_home, -(alt_global - alt_home)])
    
    return local_position


# ### NED to Geodetic
# Now try converting a local position (_north_, _east_, _down_) relative to the
# home position to a global position (_long_, _lat_, _up_).

# In[3]:

def local_to_global(local_position, global_home):
    
    # TODO: get easting, northing, zone letter and number of global_home
    (long_home, lat_home, alt_home) = global_home
    (easting_home, northing_home, zone_home, zone_home_letter) = utm.from_latlon(lat_home, long_home)
    # TODO: get (lat, lon) from local_position and converted global_home
    n, e, D = local_position
    g = (easting_home + e, northing_home + n, zone_home, zone_home_letter)
    # TODO: Create global_position of (lat, lon, alt)
    (lat, long) = utm.to_latlon(g[0], g[1], zone_home, zone_home_letter)
                               
    global_position = numpy.array([long, lat, -(D - alt_home)])
    
    return global_position


# As an example, we will use two sets of global coordinates.  One
# **```geodetic_home_coordinates```** serving as a local origin for NED frame
# and the second **```geodetic_current_coordinates```** which we will be
# expressed in NED coordinates relative to the first one.

# In[6]:

numpy.set_printoptions(precision=2)

geodetic_current_coordinates = [-122.079465, 37.393037, 30]
geodetic_home_coordinates = [-122.108432, 37.400154, 20]

local_coordinates_NED = global_to_local(geodetic_current_coordinates, geodetic_home_coordinates)

print(local_coordinates_NED)
# Should print [ -764.96 2571.59 -10.  ]


# In this example, we will do the reverse transform by obtaining the global
# coordinated by knowing NED coordinates relative to the other global
# coordinates.

# In[5]:

numpy.set_printoptions(precision=6)
NED_coordinates = [25.21, 128.07, -30.]

# convert back to global coordinates
geodetic_current_coordinates = local_to_global(NED_coordinates, geodetic_home_coordinates)

print(geodetic_current_coordinates)
# Should print [-122.106982   37.40037    50.      ]


# [solution](/notebooks/Geodetic%20to%20NED_solution.ipynb)
