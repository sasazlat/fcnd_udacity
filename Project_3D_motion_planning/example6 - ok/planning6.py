from enum import Enum
from queue import PriorityQueue
import numpy as np
from scipy.spatial import Voronoi
from shapely.geometry import Polygon, Point, LineString


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


def extract_polygons(data, safety_distance):
    polygons = []
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]

        obstacle = [north - d_north - safety_distance, north + d_north + safety_distance,
                    east - d_east - safety_distance, east + d_east + safety_distance]
        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], obstacle[3]),
                   (obstacle[1], obstacle[2])]

        # TODO: Compute the height of the polygon
        height = alt + d_alt + safety_distance

        p = Poly(corners, height)
        polygons.append(p)

    return polygons


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
    north_size = int(np.ceil((north_max - north_min + 1)))
    east_size = int(np.ceil((east_max - east_min + 1)))

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

    # create a voronoi graph based on location of obstacle centres
    graph = Voronoi(points)
    polygons = extract_polygons(data, safety_distance)

    # check each edge from graph.ridge_vertices for collision
    edges = []
    print("start building edges")
    print(len(graph.ridge_vertices))

    for v in graph.ridge_vertices:

        p1 = tuple(graph.vertices[v[0]])
        p2 = tuple(graph.vertices[v[1]])
        # If any of the vertices is out of grid then skip
        if np.amin(p1) < 0 or np.amin(p2) < 0 or p1[0] >= grid.shape[0] or p1[1] >= grid.shape[1] or p2[0] >= grid.shape[0] or p2[1] >= grid.shape[1]:
            continue

        safe = True
        cells = bres((int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])))

        # Test each pair p1 and p2 for collision using Bresenham
        # If the edge does not hit an obstacle
        # add it to the list

        for c in cells:
            # First check if we're off the map
            if np.amin(c) < 0 or c[0] >= grid.shape[0] or c[1] >= grid.shape[1]:
                safe = False
                break
            # Next check if we're in collision
            if grid[c[0], c[1]] == 1:
                safe = False
                break
        if safe:
            edges.append((p1, p2))
    print("done building edges")
    return grid, edges, polygons, int(north_min), int(east_min)


# Action class with diagonal motions.
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
    NORTH_WEST = (-1, -1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    SOUTH_EAST = (1, 1, np.sqrt(2))

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

    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)

    return valid_actions


def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))


def a_star_grid(grid, heuristic_func, start, goal):
    """
    Given a grid and heuristic function returns
    the lowest cost path from start to goal.
    """

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((heuristic_func(start, goal), start))
    visited = set()
    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]
        visited.add(current_node)

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                da = action.delta
                cost = action.cost
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                cost_so_far = current_cost + cost - heuristic_func(current_node, goal)
                new_cost = cost_so_far + heuristic_func(next_node, goal)
                if next_node not in visited:
                    if next_node in branch:
                        if cost_so_far < branch[next_node][0]:
                            queue.put((new_cost,next_node))
                            branch[next_node] = (cost_so_far, current_node, action)
                    else:
                        queue.put((new_cost,next_node))
                        branch[next_node] = (cost_so_far, current_node, action)

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


# A* implementation for graph search
def a_star_graph(graph, heuristic, start, goal):
    """Modified A* to work with NetworkX graphs."""

    queue = PriorityQueue()
    queue.put((heuristic(start, goal), start))
    visited = set()

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]
        visited.add(current_node)

        if current_node == goal:
            print('Found a path.')
            found = True
            break

        else:
            for next_node in graph.adj[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                # Remove the heuristic cost from the current cost
                cost_so_far = (current_cost - heuristic(current_node, goal)) + cost
                new_cost = cost_so_far + heuristic(next_node, goal)
                if next_node not in visited:
                    # Check if we found a less expensive path than the old one
                    if next_node in branch:
                        if cost_so_far < branch[next_node][0]:
                            queue.put((new_cost, next_node))
                            branch[next_node] = (cost_so_far, current_node)
                    else:
                        queue.put((new_cost, next_node))
                        branch[next_node] = (cost_so_far, current_node)

    path = []
    path_cost = 0
    if found:
        # retrace steps
        path = []
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost


def bres(p1, p2):
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


# Uses Bresenham algorithm to check if two points are safe to connect in 2D
# grid
def is_safe(p1, p2, grid):
    safe = True
    cells = bres(p1, p2)
    for c in cells:
        if grid[c[0], c[1]] == 1:
            safe = False
            break
    return safe


# Uses polygon representation of obstacles to check if two points in 3D can be
# connected
def can_connect(p1, p2, polygons):
    line = LineString([p1, p2])
    slope = (abs(p2[2] - p1[2])) / np.linalg.norm([p1[0] - p2[0], p1[1] - p2[1]])
    z = min(p2[2], p1[2])
    x, y = ((p1[0], p1[1]), (p2[0], p2[1]))[z in p2]
    for polygon in polygons:
        if polygon.crosses(line):
            point = polygon.center
            height_line = z + slope * np.linalg.norm(np.array([point[0],point[1]]) - np.array([x, y]))
            if height_line < polygon.height:
                return False
    return True


# This will prune the path using Bresenham algorithm in 2D grid
def prune_path_2d(path, grid):
    i = 1
    pruned_path = [path[0]]
    x1, y1 = path[0][0], path[0][1]
    p1 = (x1, y1)
    while i < len(path):
        x2, y2 = path[i][0], path[i][1]
        p2 = (x2, y2)
        if is_safe((int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), grid):
            prev_safe_path = p2
            i += 1
        else:
            pruned_path.append(prev_safe_path)
            x1, y1 = prev_safe_path[0], prev_safe_path[1]
            p1 = (x1, y1)
    pruned_path.append(prev_safe_path)
    return pruned_path


# This will prune the path in 3D
def prune_path_3d(path, polygon):
    i = 1
    pruned_path = [path[0]]
    x1, y1, z1 = path[0][0], path[0][1], path[0][2]
    p1 = (x1, y1, z1)
    while i < len(path):
        x2, y2, z2 = path[i][0], path[i][1], path[i][2]
        p2 = (x2, y2, z2)
        if can_connect((int(p1[0]), int(p1[1]), int(p1[2])), (int(p2[0]), int(p2[1]), int(p2[2])), polygon):
            prev_safe_path = p2
            i += 1
        else:
            pruned_path.append(list(prev_safe_path))
            x1, y1, z1 = prev_safe_path[0], prev_safe_path[1], prev_safe_path[2]
            p1 = (x1, y1, z1)
    pruned_path.append(list(prev_safe_path))
    return pruned_path


# Return the closest node on graph to the given point in 2D grid
def find_close_point(graph, point):
    closest_pt = None
    shortest_dis = 10000000
    for n in graph.nodes:
        dis = np.linalg.norm(np.array(n) - np.array(point))
        if dis < shortest_dis:
            closest_pt = n
            shortest_dis = dis
    return closest_pt