from udacidrone.frame_utils import *


def read_lat_lon(filename):
    with open(filename, 'r') as f:
        lat_lon = f.readline()
        lat_lon = [y for x in lat_lon.split(", ") for y in x.split(" ")]
        lat = float(lat_lon[1])
        lon = float(lat_lon[3])
        return lat, lon


def calulate_grid_goal(lon, lat, alt, grid_centre, global_home):
    local_pos = global_to_local((lon, lat, alt), global_home)
    return local_position_to_grid(local_pos, grid_centre)


def local_position_to_grid(local_pos, grid_centre):
    return (
        grid_centre[0] + int(local_pos[0]),
        grid_centre[1] + int(local_pos[1])
    )
