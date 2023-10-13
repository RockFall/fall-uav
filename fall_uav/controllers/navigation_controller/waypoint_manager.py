# This class is solely responsible for the control of waypoints
# Pathfinding algorithms may be implemented here in the future

# Lets change the waypoint array to be a queue

class WaypointManager:
    def __init__(self):
        self.waypoints = []
        self.current_waypoint_index = 0

    def set_waypoints(self, waypoints):
        self.clear_waypoints()
        self.waypoints = waypoints

    def append_waypoint(self, waypoint):
        self.waypoints.append(waypoint)

    def append_waypoints(self, waypoints):
        self.waypoints.extend(waypoints)

    def clear_waypoints(self):
        self.waypoints = []
        self.current_waypoint_index = 0

    def get_waypoints(self):
        return self.waypoints
    
    def get_current_waypoint_index(self):
        return self.current_waypoint_index

    def next_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
            self.current_waypoint_index += 1
            return waypoint
        else:
            return None
        
    def has_more_waypoints(self):
        return self.current_waypoint_index < len(self.waypoints)

# TODO: Tratamento adequado de erros
# TODO: Adicionar compatibilidade adicional com ROS e MAVROS conforme necessário.