import argparse
import time
import msgpack
from enum import Enum, auto
import networkx as nx
import csv

import numpy as np

from planning6 import a_star_graph, heuristic, create_grid_and_edges, find_close_point, \
    prune_path_2d, prune_path_3d
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, goal):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        # Set the global goal to the value passed by user
        self.global_goal = goal

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            local = self.local_position
            local[2] = -1 * local[2]
            # Making a 3D Deadband
            if len(self.waypoints) > 0:
                if np.linalg.norm(self.target_position[0:3] - local[0:3]) < 5.0:
                    self.waypoint_transition()
            else:
                if np.linalg.norm(self.target_position[0:3] - local[0:3]) < 1.0:
                    if np.linalg.norm(self.local_velocity[0:2]) < 0.075:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_goal[2] < 0.1:
                self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        print(self.target_position[2])
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2],
                          self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):

        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TAKEOFF_ALTITUDE = 5
        SAFETY_DISTANCE = 3

        # Set the takeoff altitude
        self.target_position[2] = TAKEOFF_ALTITUDE

        # Read lon0, lat0 from colliders into floating point values
        with open('colliders.csv', newline='') as f:
            reader = csv.reader(f)
            b = next(reader)
        home_lat = float(b[0].split(" ")[1])
        home_long = float(b[1].split(" ")[2])
        print("home longitude = {0},  home latitude = {1}".format(home_long, home_lat))

        # Set home position to (lon0, lat0, 0)
        self.set_home_position(home_long, home_lat, 0)

        #  Retrieve current global position
        global_start = self.global_position

        # Convert to current local position using global_to_local()
        local_start = global_to_local(global_start, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         local_start))
        # Start position
        start_altitude = -1 * local_start[2] + TAKEOFF_ALTITUDE
        print("start altitude = ", start_altitude)
        start = [local_start[0], local_start[1], start_altitude]

        # Goal position
        goal = global_to_local(self.global_goal, self.global_home)

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around
        # obstacles
        grid, edges, polygons, north_offset, east_offset = create_grid_and_edges(data, min(start[2], goal[2]),
                                                                                 SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # Convert start position to current position on grid
        grid_start = (start[0] - north_offset, start[1] - east_offset)

        # Convert goal position to grid location
        grid_goal = (goal[0] - north_offset, goal[1] - east_offset)

        print('Local Start and Goal: ', grid_start, grid_goal)

        # Build a 2D graph
        G = nx.Graph()
        for e in edges:
            p1 = e[0]
            p2 = e[1]
            dist = np.linalg.norm(np.array(p2) - np.array(p1))
            G.add_edge(p1, p2, weight=dist)

        # Find closest nodes to start and goal on graph
        start_node = find_close_point(G, grid_start)
        goal_node = find_close_point(G, grid_goal)

        # Run A* to find a path from start to goal
        t0 = time.time()
        path, path_cost = a_star_graph(G, heuristic, start_node, goal_node)
        print("Time to find the path = ", time.time() - t0, "seconds")
        print("Length of path = ", len(path))
        print("Cost of path = ", path_cost)

        # Prune the path in 2D using Bresenham algorithm
        t0 = time.time()
        smooth_path = prune_path_2d(path, grid)
        print("Time to prune the path= ", time.time() - t0, "seconds")
        smooth_path.append(grid_goal)  # Add goal the list of path
        print("Length of pruned path = ", len(smooth_path))

        # Find the inc/dec for the required in altitude for each consecutive
        # waypoint
        alt_inc = ((self.global_goal[2] + SAFETY_DISTANCE) - start[2]) / (len(smooth_path) - 2)

        # Convert smooth_path to 3D waypoints
        temp_path = [[int(smooth_path[i][0] + north_offset), int(smooth_path[i][1] + east_offset),
                      int(start_altitude + i * alt_inc)] for i in range(len(smooth_path) - 1)]

        # Add the goal point to the 3D list with altitude equal to previous
        # waypoint.
        # So that drone can move from final waypoint to goal by moving at same
        # altitude.
        temp_path.append([int(smooth_path[-1][0] + north_offset), int(smooth_path[-1][1] + east_offset),
                          temp_path[-1][2]])

        smooth_path = temp_path

        # Prune the path in 3D using 2.5D Map representation
        t0 = time.time()
        final_path = prune_path_3d(smooth_path, polygons)
        print("Time to prune the 3D path= ", time.time() - t0, "seconds")
        print("Length of pruned 3D path = ", len(final_path))

        # Convert path to waypoints
        waypoints = [[final_path[i][0], final_path[i][1], final_path[i][2], 0] for i in range(len(final_path))]

        # Set self.waypoints
        self.waypoints = waypoints

        # Sends waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)

    global_goal_longitude = -122.400830
    global_goal_latitude = 37.793612
    global_goal_altitude = 33
    # global goal position
    goal = [global_goal_longitude, global_goal_latitude, global_goal_altitude]

    drone = MotionPlanning(conn, goal)
    time.sleep(1)

    drone.start()