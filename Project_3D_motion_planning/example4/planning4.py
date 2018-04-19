from enum import Enum
from queue import PriorityQueue
import numpy as np
from math import sqrt
import re
# If you want to use the prebuilt bresenham method
# Import the Bresenham package
from bresenham import bresenham


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.
    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    NORTH_EAST = (-1, 1, sqrt(2))
    NORTH_WEST = (-1, -1, sqrt(2))
    SOUTH_WEST = (1, -1, sqrt(2))
    SOUTH_EAST = (1, 1, sqrt(2))

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
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)

    return valid_actions


def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = branch[current_node][0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost



def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))

def read_home(filepath):
    """
    Read the first line of the csv file
    extract lat0 and lon0 as floating point values
    """
    # Read first line
    with open(filepath) as f:
        first_line = f.readline()
#     print(first_line)
    # Regular expression match
    ret_groups = re.match('^lat0 (.*), lon0 (.*)$', first_line)
#     print(ret_groups.group(1))
    lat0 = float(ret_groups.group(1))
    lon0 = float(ret_groups.group(2))
#     print(type(lon0))
    
    return lat0, lon0

# Define a simple function to add a z coordinate of 1
def point(p):
    return np.array([p[0], p[1], 1.])

def collinearity_float(p1, p2, p3, epsilon=1e-2): 
    collinear = False
    # TODO: Add a third dimension of z=1 to each point
    p1_new = point(p1)
    p2_new = point(p2)
    p3_new = point(p3)
    
    # TODO: Create the matrix out of three points
    mat = np.vstack((p1_new, p2_new, p3_new))
    
    # TODO: Calculate the determinant of the matrix. 
    det = np.linalg.det(mat)
    
    # TODO: Set collinear to True if the determinant is less than epsilon
    collinear = False
    if np.abs(det) < epsilon:
        collinear = True

    return collinear

def collinearity_int(p1, p2, p3): 
    collinear = False
    # TODO: Calculate the determinant of the matrix using integer arithmetic 
    det = p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1])
    
    # TODO: Set collinear to True if the determinant is equal to zero
    if 0 == det:
        collinear = True

    return collinear

def bres(p1, p2): 
    """
    Note this solution requires `x1` < `x2` and `y1` < `y2`.
    """
    x1, y1 = p1
    x2, y2 = p2
    cells = []
    
    print("p1 is", p1)
    print("p2 is", p2)
    
    # TODO: Determine valid grid cells
    dy = y2 - y1
    dx = x2 - x1
    # y = m*x + b
    # -> dy = m*dx
    # -> m = dy/dx
    m = dy/dx
#     print("m is", m)
    
    # Initial value
    x = x1
    y = y1
    
    # Initial cell value
    x_cell = x1
    y_cell = y1
    
    # Initial line equation result
    f = m
    
    while x < x2 and y < y2:
        cells.append([x_cell, y_cell])
#         print("y is", y)
        
        if f > (y + 1):
            y += 1
            y_cell += 1
#             print("Increase y")
#             print("y is", y)
        elif f == (y + 1):
            if 0: # Set it to 1 for more conservative approach
                cells.append([x_cell + 1, y_cell])
                cells.append([x_cell, y_cell + 1])
            x += 1
            x_cell += 1
            y += 1
            y_cell += 1
            f += m
        else:
            x += 1
            x_cell += 1
            f += m
#             print("Increase x")
#             print("x is", x)
        
        
    return np.array(cells)

def bres_check(grid, list_points):
    points = []
    edges = []

    idx_start = 0
    idx_end = 1
    
    # Start from the first point until the last but not least point
    while idx_end <= (len(list_points) - 1):
        starting_point = list_points[idx_start]
        ending_point = list_points[idx_end]
        
#         cells_bres = bres(starting_point, ending_point)
        cells_bres = list(bresenham(int(starting_point[0]), int(starting_point[1]), int(ending_point[0]), int(ending_point[1])))
#         print("cells_bres is ", cells_bres)
        in_collision = False
        for cell in cells_bres:
            # First check if we're off the map
            if np.amin(cell) < 0 or cell[0] >= grid.shape[0] or cell[1] >= grid.shape[1]:
                in_collision = True
                break
            # Next check if we're in collision
            if 1 == grid[cell[0], cell[1]]:
                in_collision = True
                break
        
        # Then you can test each pair p1 and p2 for collision using Bresenham
        # (need to convert to integer if using prebuilt Python package)
        # If the edge does not hit an obstacle
        # add it to the list
        if not in_collision:
            if (len(list_points) - 1) == idx_end:
                points.append(starting_point)
                points.append(ending_point)
            idx_end += 1
        else:
            points.append(starting_point)
            if (len(list_points) - 1) == idx_end:
                points.append(starting_point)
                points.append(list_points[idx_end - 1])
                points.append(ending_point)
            idx_start = idx_end - 1
            idx_end += 1
        
    return points, edges

def prune_path(grid, path):
    pruned_path = [p for p in path]
    # TODO: prune the path!
    i = 0
    while i < len(pruned_path) - 2:
        p1 = point(pruned_path[i])
        p2 = point(pruned_path[i + 1])
        p3 = point(pruned_path[i + 2])
        
        # Check if 3 points are in a line
        if collinearity_float(p1, p2, p3):
            # Remove the 2nd point from the path
            pruned_path.remove(pruned_path[i+1])
        else:
            # Move to the next 3 points
            i += 1
    
    print("pruned_path before is", pruned_path)
    pruned_path = bres_check(grid, pruned_path)[0]
    print("pruned_path after is", pruned_path)
    
    return pruned_path