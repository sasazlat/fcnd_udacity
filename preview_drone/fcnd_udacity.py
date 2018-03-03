from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
import time


conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=True)
drone = Drone(conn)
drone.start()

drone.take_control()
drone.arm()

drone.set_home_position(drone.global_position[0],
                        drone.global_position[1],
                        drone.global_position[2])


drone.takeoff(3)

square = [(10,0,5,0),
          (10,10,5,0),
          (0,10,5,0),
          (0,0,5,0)]

for sc in square:
    drone.cmd_position(*sc)
    time.sleep(3)


drone.takeoff(23)

drone.cmd_position(5,0,3,0)