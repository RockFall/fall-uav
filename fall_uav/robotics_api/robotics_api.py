from .ros.ros_api import ROS_API

class RoboticsAPI:
    def __init__(self, system_type, namespace='/uav1', pose_update=None, nav_data_update=None):
        self.system_type = system_type
        if system_type == 'ros':
            self.ros_api = ROS_API(system_type, namespace)
        # Other systems may be added here in the future

    def arm(self):
        if self.system_type == 'ros':
            return self.ros_api.arm()
    
    def disarm(self):
        if self.system_type == 'ros':
            return self.ros_api.disarm()
        
    def set_flight_mode(self, mode):
        if self.system_type == 'ros':
            return self.ros_api.set_flight_mode(mode)

    def takeoff(self):
        if self.system_type == 'ros':
            return self.ros_api.takeoff()
    
    def land(self):
        if self.system_type == 'ros':
            return self.ros_api.land()
        
    def sleep(self, seconds):
        if self.system_type == 'ros':
            return self.ros_api.sleep(seconds)

    def go_to(self, x, y, z, w):
        if self.system_type == 'ros':
            return self.ros_api.go_to(x, y, z, w)
        
    def publish_velocity(self, x, y, z, w):
        if self.system_type == 'ros':
            return self.ros_api.publish_velocity(x, y, z, w)
        
    def update_pose(self):
        if self.system_type == 'ros':
            return self.ros_api.update_pose()
        
    def is_flying(self):
        if self.system_type == 'ros':
            return self.ros_api.is_flying()
