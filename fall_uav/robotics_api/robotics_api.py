from .ros.ros_api import ROSInterface

class RoboticsAPI:
    def __init__(self, system_type, namespace='/uav1'):
        self.system_type = system_type
        if system_type == 'ros':
            self.ros_interface = ROSInterface(system_type, namespace)
        # Other systems may be added here in the future

    def arm(self):
        if self.system_type == 'ros':
            return self.ros_interface.arm()
    
    def disarm(self):
        if self.system_type == 'ros':
            return self.ros_interface.disarm()

    def takeoff(self):
        if self.system_type == 'ros':
            return self.ros_interface.takeoff()
    
    def land(self):
        if self.system_type == 'ros':
            return self.ros_interface.land()

    def go_to(self, x, y, z, w):
        if self.system_type == 'ros':
            return self.ros_interface.go_to(x, y, z, w)
        
    def publish_velocity(self, x, y, z, w):
        if self.system_type == 'ros':
            return self.ros_interface.publish_velocity(x, y, z, w)
        
    def update_pose(self):
        if self.system_type == 'ros':
            return self.ros_interface.update_pose()
