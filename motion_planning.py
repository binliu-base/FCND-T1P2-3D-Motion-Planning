import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, read_global_home, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global

from skimage.morphology import medial_axis
from skimage.util import invert


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, goal_position_g=None):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL
        self.goal_position_g = goal_position_g

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)


    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
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
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

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
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        filename = "colliders.csv"
        lon0, lat0 = read_global_home(filename)
        # print('Global Home Lat: {0}, Lon: {1}'.format(lat0, lon0))
        
        # set home position to (lat0, lon0, 0)
        self.set_home_position(lon0, lat0, 0)

        # retrieve current global position
        local_position_g = [self._longitude, self._latitude, self._altitude]
        # print('current global position: longitude={}, latitude={}'.format(self._longitude, self._latitude))        
             
        # convert to current local position using global_to_local()
        self._north, self._east, self._down =  global_to_local(local_position_g, self.global_home)
        print('global home {}, global position {}, local position {}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)        
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)        
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))

        # convert start position to current position rather than map center
        # Set goal as some arbitrary position on the grid
        # Define starting point on the grid (this is just grid center)
        start_position_l = self.local_position[:2]
        grid_start = (int(np.ceil(-north_offset + start_position_l[0])), int(np.ceil(-east_offset + start_position_l[1])))        

        # adapt to set goal as latitude / longitude position and convert
        print("goal_position_g: {}".format(goal_position_g))
        goal_position_l = global_to_local(goal_position_g, self.global_home)
        grid_goal = (int(np.ceil(-north_offset + goal_position_l[0])), int(np.ceil(-east_offset + goal_position_l[1])))        
        print('Local Start and Goal: ', grid_start, grid_goal)

        # Run A* to find a path from start to goal
        #Compute the lowest cost path with a_star
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print('Length of raw path: {}'.format(len(path)))        
        # print(f'Path cost: {cost}')
        
        #prune path to minimize number of waypoints
        pruned_path = prune_path(path)
        print('Length of pruned path: {}'.format(len(pruned_path)))

        # TODO (if you're feeling ambitious): Try a different approach altogether!

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]
        print('Waypoint count : {}'.format(len(waypoints)))        

        # Set self.waypoints
        self.waypoints = waypoints
        # send waypoints to sim
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
    parser.add_argument('--goal_lat', type=str, default='37.80067575', help="goal latitude, i.e. '37.80067575'")    
    parser.add_argument('--goal_lon', type=str, default='-122.3962475', help="goal longitude, i.e. '-122.3962475'")
    parser.add_argument('--goal_alt', type=str, default='0.', help="altitude of goal position, i.e. '0.'")    
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    goal_position_g= (float(args.goal_lon), float(args.goal_lat), float(args.goal_alt))
    drone = MotionPlanning(conn, goal_position_g=goal_position_g)
    time.sleep(1)

    drone.start()