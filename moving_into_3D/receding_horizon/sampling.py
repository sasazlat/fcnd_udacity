import numpy as np
from sklearn.neighbors import KDTree
from shapely.geometry import Polygon, Point, LineString
import networkx as nx

class Poly:

    def __init__(self, coords, height):
        self._polygon = Polygon(coords)
        self._height = height

    @property
    def height(self):
        return self._height

    @property
    def polygon(self):
        return self._polygon

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
        corners = [(obstacle[0], obstacle[2]), (obstacle[0], obstacle[3]), (obstacle[1], obstacle[3]), (obstacle[1], obstacle[2])]
        
        # TODO: Compute the height of the polygon
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
        # Record maximum polygon dimension in the xy plane
        # multiply by 2 since given sizes are half widths
        # This is still rather clunky but will allow us to
        # cut down the number of polygons we compare with by a lot.
        self._max_poly_xy = 2 * np.max((data[:, 3], data[:, 4]))
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
            in_collision = False
            idxs = list(self._tree.query_radius(np.array([s[0], s[1]]).reshape(1, -1), r=self._max_poly_xy)[0])
            if len(idxs) > 0:
                for ind in idxs: 
                    p = self._polygons[int(ind)]
                    if p.contains(s) and p.height >= s[2]:
                        in_collision = True
            if not in_collision:
                pts.append(s)
                
        return pts

    @property
    def polygons(self):
        return self._polygons



# In[29]:
def can_connect(n1, n2, polygons):
    l = LineString([n1, n2])
    for p in polygons:
        if p.crosses(l) and p.height >= min(n1[2], n2[2]):
            return False
    return True

def create_graph(nodes, k, polygons):
    g = nx.Graph()
    tree = KDTree(nodes)
    for n1 in nodes:
        # for each node connect try to connect to k nearest nodes
        idxs = tree.query([n1], k, return_distance=False)[0]

        for idx in idxs:
            n2 = nodes[idx]
            if n2 == n1:
                continue
                
            if can_connect(n1, n2, polygons):
                g.add_edge(n1, n2, weight=1)
    return g