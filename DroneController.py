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

import rospy
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_srvs.srv import Trigger

from mavros_msgs.srv import CommandTOL

from MovementController import MovementController

class DroneController:
    def __init__(self, system='mavros', log=[]):
        rospy.init_node('drone_controller', anonymous=True)
        self.movement_controller = MovementController(log=log, system=system)
        self.system = system

        self.log = log

    def sleep(self, seconds):
        rospy.sleep(seconds)

    def setup(self):
        if self.system == 'mavros':
            self.arm()
            self.set_flight_mode('GUIDED')

    def arm(self):
        rospy.wait_for_service('/uav1/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)
            response = arm_service(True)
            if response.success:
                rospy.loginfo("Drone armado com sucesso!")
            else:
                rospy.logwarn("Falha ao armar o drone")
        except rospy.ServiceException as e:
            rospy.logerr("Erro ao chamar o serviço de armamento: %s" % e)

    def set_flight_mode(mode):
        rospy.wait_for_service('/uav1/mavros/set_mode')
        try:
            set_mode_service = rospy.ServiceProxy('/uav1/mavros/set_mode', SetMode)
            response = set_mode_service(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo(f"Modo de voo alterado para '{mode}' com sucesso!")
            else:
                rospy.logwarn("Falha ao alterar o modo de voo")
        except rospy.ServiceException as e:
            rospy.logerr("Erro ao chamar o serviço de alteração de modo de voo: %s" % e)

    def takeoff(self):
        TARGET_HEIGHT_PICTURES = 3.0
        try:
            print("\n----------Tentando decolar----------")

            if self.system == "uav_manager":
                rospy.wait_for_service('/uav1/uav_manager/takeoff')
                takeoff_service = rospy.ServiceProxy('/uav1/uav_manager/takeoff', Trigger)
                response = takeoff_service.call()  # Altitude de decolagem em metros
            elif self.system == "mavros":
                rospy.wait_for_service('/uav1/mavros/cmd/takeoff')
                takeoff_service = rospy.ServiceProxy('/uav1/mavros/cmd/takeoff', CommandTOL)
                response = takeoff_service(altitude= TARGET_HEIGHT_PICTURES)
            else:
                raise Exception("Sistema de controle de voo não reconhecido (escolha mavros ou uav_control)")
            print('Takeoff response:', response)
            if not response.success:
                print("Provavelmente o drone já está no ar\n")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr("Falha na decolagem: %s", str(e))
            return False
        
    def land(self):
        try:
            print("\n----------Pousando----------")
            if self.system == "uav_manager":
                rospy.wait_for_service('/uav1/uav_manager/land')
                landing_service = rospy.ServiceProxy('/uav1/uav_manager/land', Trigger)
                response = landing_service.call()
            elif self.system == "mavros":
                rospy.wait_for_service('/uav1/mavros/cmd/land')
                landing_service = rospy.ServiceProxy('/uav1/mavros/cmd/land', CommandTOL)
                response = landing_service(altitude=0.0)
            else:
                raise Exception("Sistema de controle de voo não reconhecido (escolha mavros ou uav_control)")
            print('Landing response:', response)
            if not response.success:
                print("Provavelmente o drone já está no chão\n")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr("Falha no pouso: %s", str(e))
            return False

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
        rospy.sleep(3)  # Assuming 3 seconds for scanning. Adjust this based on real timings.
        print("You need to integrate with an actual QR code recognition system.")

    
    def slow_descent(self, min_height):
        self.movement_controller.slow_descent(min_height)
    
        
    def Scan(self, pose):
        #points = self.scan.scanBaseSrvCallback(pose)
        pass


