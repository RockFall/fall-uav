# This class acts as the central class for all drone-relted operations
# Current capabilities:
# - Arming
# - Takeoff
# - Landing
# - Move to
# To do:
# - Scan for QR codes
# - Scan for plataforms
# - Battery Status

from .motion_controller import MotionController
from ..robotics_api import RoboticsAPI


class DroneController:
    """
    This class acts as the central class for all drone-relted operations
    
    It uses the RoboticsAPI class to abstract the system used (ROS, MRS, etc)
    """

    def __init__(self, system='ros', log=[]):
        self.robotics_api = RoboticsAPI(system_type=system)
        self.movement_controller = MotionController(log=log, system=system, robotics_api=self.robotics_api)
        self.system = system
        self.log = log

    def sleep(self, seconds):
        """
        The sleep function pauses the execution of the program for a specified number of seconds.
        
        :param seconds: The "seconds" parameter is the number of seconds that the code should pause or
        sleep for
        """
        self.robotics_api.sleep(seconds)

    def arm(self):
        """
        The arm function arms the drone. This is a prerequisite for takeoff.
        """
        self.robotics_api.arm()

    def set_flight_mode(self, mode):
        """
        The set_flight_mode function sets the flight mode of the drone.
        It can be:
        - GUIDED
        - ...
        """
        self.robotics_api.set_flight_mode(mode)

    def takeoff(self):
        self.robotics_api.takeoff()
        
    def land(self):
        self.robotics_api.land()

    def move_to(self, x, y, z, w):
        self.movement_controller.move_to(x, y, z, w)

    def move_to_waypoints(self, waypoints):
        self.movement_controller.move_to_waypoints(waypoints)

    def centralize(self):
        central_position = [[4.0, 0.0, 2.0, 1.57]]  # Placeholder central position [x, y, z, w]
        print("Centralizing the drone...")
        self.movement_controller.move_to_waypoints(central_position)

    def scan_QR(self):
        # Placeholder for scanning QR codes. This needs actual integration with camera systems.
        print("QR code scan...")
        self.robotics_api.sleep(3)  # Assuming 3 seconds for scanning. Adjust this based on real timings.
        print("You need to integrate with an actual QR code recognition system.")

    
    def slow_descent(self, min_height):
        self.movement_controller.slow_descent(min_height)
    
        
    def Scan(self, pose):
        #points = self.scan.scanBaseSrvCallback(pose)
        pass


