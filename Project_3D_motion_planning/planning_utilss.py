import networkx as nx
import numpy as np

from enum import Enum
from queue import PriorityQueue
from shapely.geometry import Polygon, Point, LineString
from sklearn.neighbors import KDTree
from bresenham import bresenham


class Poly:

    def __init__(self, coords, height):
        self._polygon = Polygon(coords)
        self._height = height

    @property
    def height(self):
        return self._height

    @property
    def coords(self):
        return list(self._polygon.exterior.coords)[:-1]

    @property
    def area(self):
        return self._polygon.area

    @property
    def center(self):
        return (self._polygon.centroid.x, self._polygon.centroid.y)

    def contains(self, point):
        point = Point(point)
        return self._polygon.contains(point)

    def crosses(self, other):
        return self._polygon.crosses(other)

def extract_polygons(data):
    polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        obstacle = [north - d_north, north + d_north, east - d_east, east + d_east]
        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], obstacle[3]),
                   (obstacle[1], obstacle[2])]

        height = alt + d_alt

        p = Poly(corners, height)
        polygons.append(p)

    return polygons

class Sampler:

    def __init__(self, data):
        self._polygons = extract_polygons(data)
        self._xmin = np.min(data[:, 0] - data[:, 3])
        self._xmax = np.max(data[:, 0] + data[:, 3])

        self._ymin = np.min(data[:, 1] - data[:, 4])
        self._ymax = np.max(data[:, 1] + data[:, 4])

        self._zmin = 0
        # limit z-axis
        self._zmax = 20

        centers = np.array([p.center for p in self._polygons])
        self._tree = KDTree(centers, metric='euclidean')

    def sample(self, num_samples):
        """Implemented with a k-d tree for efficiency."""
        xvals = np.random.uniform(self._xmin, self._xmax, num_samples)
        yvals = np.random.uniform(self._ymin, self._ymax, num_samples)
        zvals = np.random.uniform(self._zmin, self._zmax, num_samples)
        samples = list(zip(xvals, yvals, zvals))

        pts = []
        for s in samples:
            _, idx = self._tree.query(np.array([s[0], s[1]]).reshape(1, -1))
            p = self._polygons[int(idx)]
            if not p.contains(s) or p.height < s[2]:
                pts.append(s)
        return pts

    @property
    def polygons(self):
        return self._polygons

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
            obstacle = [int(np.clip(north - d_north - safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size - 1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size - 1)),]
            grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1

    return grid, int(north_min), int(east_min)

def create_grid_and_edges(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    along with Voronoi graph edges given obstacle data and the
    drone's altitude.
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
    # Initialize an empty list for Voronoi points
    points = []
    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [int(np.clip(north - d_north - safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size - 1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size - 1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size - 1)),]
            grid[obstacle[0]:obstacle[1] + 1, obstacle[2]:obstacle[3] + 1] = 1

            # add center of obstacles to points list
            points.append([north - north_min, east - east_min])

    # TODO: create a voronoi graph based on
    # location of obstacle centres
    graph = Voronoi(points)

    # TODO: check each edge from graph.ridge_vertices for collision
    edges = []
    for v in graph.ridge_vertices:
        p1 = graph.vertices[v[0]]
        p2 = graph.vertices[v[1]]
        
        cells = list(bresenham(int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1])))
        hit = False

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                hit = True
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                hit = True
                break

        # If the edge does not hit on obstacle
        # add it to the list
        if not hit:
            # array to tuple for future graph creation step)
            p1 = (p1[0], p1[1])
            p2 = (p2[0], p2[1])
            edges.append((p1, p2))

    return grid, edges

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

def a_star_graph(graph, heuristic, start, goal):
    """Modified A* to work with NetworkX graphs."""
    
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
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    queue.put((new_cost, next_node))
                    
                    branch[next_node] = (new_cost, current_node)
             
    path = []
    path_cost = 0
    if found:
        
        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
            
    return path[::-1], path_cost

def closest_point_graph(graph, current_point):
    """
    Compute the closest point in the `graph`
    to the `current_point`.
    """
    closest_point = None
    dist = 100000
    for p in graph.nodes:
        d = LA.norm(np.array(p) - np.array(current_point))
        if d < dist:
            closest_point = p
            dist = d
    return closest_point

def collinearity_int(p1, p2, p3):
    collinear = False
    collinear = p1[0] * (p2[1] - p3[1]) + p2[0] * (p3[1] - p1[1]) + p3[0] * (p1[1] - p2[1]) == 0

def prune_path_bresenham(grid, path):
   """
   Use the Bresenham module to trim uneeded waypoints from path
   """
   pruned_path = [p for p in path]

   i = 0
   while i < len(pruned_path) - 2:
       p1 = pruned_path[i]
       p2 = pruned_path[i + 1]
       p3 = pruned_path[i + 2]
       
       # if the line between p1 and p2 doesn't hit an obstacle
       # remove the 2nd point.
       # The 3rd point now becomes the 2nd point
       # and the check is redone with a new third point
       # on the next iteration.
       br = list(bresenham(p1[0], p1[1], p3[0], p3[1]))
       #br = bresa(p1, p3)

       if all((grid[pp] == 0) for pp in br):
           # Something subtle here but we can mutate
           # `pruned_path` freely because the length
           # of the list is checked on every iteration.
           pruned_path.remove(p2)
       else:
           i += 1

   return pruned_path

def bres(start, end):
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
    d = dy - dx
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    j = y1
    i = x1
    cells = []
    while i < x2 and j < y2:
        coord = (j, i) if is_steep else (i, j)
        cells.append(coord)
        d -= abs(dy)
        if d < 0:
            #j += ystep
            d += dy
            i += 1
        elif d == 0:
            #uncomment these two lines for conservative approach
            #cells.append((i+1, j))
            #cells.append([i, j+1])
            d += dy
            d -= dx
            j += ystep
            i += 1
        else:
            d -= dx
            j += ystep
    # Reverse the list if the coordinates were swapped
    if swapped:
        cells.reverse()
    return cells

def bresa(p1, p2):
    """
    This solution requires no condition on points p1 and p2.
    """
    xi, yi = p1
    x2, y2 = p2
    cells = [(xi, yi)]

    # Slope is calculated once only if it's possible
    if xi != x2:
        m = (y2 - yi) / (x2 - xi)
        delta_y = m
        x1 = xi
        y1 = yi + 0.5 - m * 0.5

        if x1 < x2 and yi <= y2:
            x = x1 + 1
            y = yi
            inc_y = y1 + delta_y
            while x < x2 + 1 or y < y2:
                if inc_y > y + 1:
                    y += 1
                elif inc_y == y + 1:
                    y += 1
                    x += 1
                    delta_y += m
                else:
                    x += 1
                    delta_y += m
                cells.append((x - 1, y))
                inc_y = y1 + delta_y

        elif x1 < x2 and yi >= y2:
            x = x1 + 1
            y = yi
            inc_y = y1 + delta_y
            while x < x2 + 1 or y > y2:
                if inc_y < y:
                    y -= 1
                elif inc_y == y:
                    y -= 1
                    x += 1
                    delta_y += m
                else:
                    x += 1
                    delta_y += m
                cells.append((x - 1, y))
                inc_y = y1 + delta_y

        elif x1 > x2 and yi > y2:
            x = x1
            y = yi
            inc_y = y1
            delta_y = 0
            while x > x2 or y > y2:
                if inc_y < y:
                    y -= 1
                elif inc_y == y:
                    y -= 1
                    x -= 1
                    delta_y -= m
                else:
                    x -= 1
                    delta_y -= m
                cells.append((x, y))
                inc_y = y1 + delta_y

        elif x1 > x2 and yi <= y2:
            x = x1
            y = yi
            inc_y = y1
            delta_y = 0
            while x > x2 or y < y2:
                if inc_y > y + 1:
                    y += 1
                elif inc_y == y + 1:
                    y += 1
                    x -= 1
                    delta_y -= m
                else:
                    x -= 1
                    delta_y -= m
                cells.append((x, y))
                inc_y = y1 + delta_y

    else:
        y = yi
        if y2 >= yi:
            while y < y2 + 1:
                cells.append((xi, y))
                y += 1
        else:
            while y > y2:
                cells.append((xi, y - 1))
                y -= 1

    return np.array(cells)

def heuristic(p1,p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))