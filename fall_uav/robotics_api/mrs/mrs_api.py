import rospy
from geometry_msgs.msg import PoseStamped
from uav_msgs.msg import VelocityReferenceStamped
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from std_srvs.srv import SetString
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from uav_msgs.srv import Vec4
from uav_msgs.srv import GetOdometry
from uav_msgs.srv import GetPosition
from uav_msgs.srv import GetOrientation
from uav_msgs.srv import GetLinearVelocity
from uav_msgs.srv import GetAngularVelocity
from uav_msgs.srv import GetLinearAcceleration
from uav_msgs.srv import GetAngularAcceleration
from uav_msgs.srv import GetPose
from uav_msgs.srv import GetYaw
from uav_msgs.srv import GetQuaternion
from uav_msgs.srv import GetRoll
from uav_msgs.srv import GetPitch
from uav_msgs.srv import GetEuler
from uav_msgs.srv import GetString

class MRS_API:
    def __init__(self, system_type, namespace='/uav1'):
        rospy.init_node('ros_api', anonymous=True)

        self.system_type = system_type
        self.namespace = namespace
        self.pose = PoseStamped()
        self.velocity = VelocityReferenceStamped()
        self.pose_subscriber = rospy.Subscriber(namespace + '/odometry/odom_main', PoseStamped, self.pose_callback)
        self.velocity_subscriber = rospy.Subscriber(namespace + '/control_manager/reference/velocity', VelocityReferenceStamped, self.velocity_callback)
        self.pose_publisher = rospy.Publisher(namespace + '/control_manager/reference/pose', PoseStamped, queue_size=10)
        self.velocity_publisher = rospy.Publisher(namespace + '/control_manager/reference/velocity', VelocityReferenceStamped, queue_size=10)
        self.arm_service = rospy.ServiceProxy(namespace + '/control_manager/arm', Trigger)
        self.disarm_service = rospy.ServiceProxy(namespace + '/control_manager/disarm', Trigger)
        self.takeoff_service = rospy.ServiceProxy(namespace + '/control_manager/takeoff', Trigger)
        self.land_service = rospy.ServiceProxy(namespace + '/control_manager/land', Trigger)
        self.go_to_service = rospy.ServiceProxy(namespace + '/control_manager/goto', Vec4)
        self.publish_velocity_service = rospy.ServiceProxy(namespace + '/control_manager/publish_velocity', Vec4)
        self.set_flight_mode_service = rospy.ServiceProxy(namespace + '/control_manager/set_flight_mode', SetString)
        self.get_flight_mode_service = rospy.ServiceProxy(namespace + '/control_manager/get_flight_mode', GetString)
        self.get_odometry_service = rospy.ServiceProxy(namespace + '/odometry/get_odometry', GetOdometry)
        self.get_position_service = rospy.ServiceProxy(namespace + '/odometry/get_position', GetPosition)
        self.get_orientation_service = rospy.ServiceProxy(namespace + '/odometry/get_orientation', GetOrientation)
        self.get_linear_velocity_service = rospy.ServiceProxy(namespace + '/odometry/get_linear_velocity', GetLinearVelocity)
        self.get_angular_velocity_service = rospy.ServiceProxy(namespace + '/odometry/get_angular_velocity', GetAngularVelocity)
        self.get_linear_acceleration_service = rospy.ServiceProxy(namespace + '/odometry/get_linear_acceleration', GetLinearAcceleration)
        self.get_angular_acceleration_service = rospy.ServiceProxy(namespace + '/odometry/get_angular_acceleration', GetAngularAcceleration)
        self.get_pose_service = rospy.ServiceProxy(namespace + '/odometry/get_pose', GetPose)
        self.get_yaw_service = rospy.ServiceProxy(namespace + '/odometry/get_yaw', GetYaw)
        self.get_quaternion_service = rospy.ServiceProxy(namespace + '/odometry/get_quaternion', GetQuaternion)
        self.get_roll_service = rospy.ServiceProxy(namespace + '/odometry/get_roll', GetRoll)
        self.get_pitch_service = rospy.ServiceProxy(namespace + '/odometry/get_pitch', GetPitch)
        self.get_euler_service = rospy.ServiceProxy(namespace + '/odometry/get_euler', GetEuler)
        # ...

    def arm(self):
        rospy.wait_for_service(self.namespace + '/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy(self.namespace + '/mavros/cmd/arming', CommandBool)
            response = arm_service(True)
            if response.success:
                rospy.loginfo("Drone armado com sucesso!")
            else:
                rospy.logwarn("Falha ao armar o drone")
        except rospy.ServiceException as e:
            rospy.logerr("Erro ao chamar o serviço de armamento: %s" % e)

    def disarm(self):
        rospy.wait_for_service(self.namespace + '/mavros/cmd/arming')
        try:
            disarm_service = rospy.ServiceProxy(self.namespace + '/mavros/cmd/arming', CommandBool)
            response = disarm_service(False)
            if response.success:
                rospy.loginfo("Drone desarmado com sucesso!")
            else:
                rospy.logwarn("Falha ao desarmar o drone")
        except rospy.ServiceException as e:
            rospy.logerr("Erro ao chamar o serviço de desarmamento: %s" % e)

    def set_flight_mode(self, mode):
        rospy.wait_for_service(self.namespace + '/mavros/set_mode')
        try:
            set_mode_service = rospy.ServiceProxy(self.namespace + '/mavros/set_mode', SetMode)
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
            # TODO: Definir o uso de uav_manager vs mavros
            if self.system == "uav_manager":
                rospy.wait_for_service(self.namespace + '/uav_manager/takeoff')
                takeoff_service = rospy.ServiceProxy(self.namespace + '/uav_manager/takeoff', Trigger)
                response = takeoff_service.call()  # Altitude de decolagem em metros
            elif self.system == "mavros": # FIXME: Trocar para 'ros' para funcionar
                rospy.wait_for_service(self.namespace + '/mavros/cmd/takeoff')
                takeoff_service = rospy.ServiceProxy(self.namespace + '/mavros/cmd/takeoff', CommandTOL)
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
            # TODO: Definir o uso de uav_manager vs mavros
            if self.system == "uav_manager":
                rospy.wait_for_service(self.namespace + '/uav_manager/land')
                landing_service = rospy.ServiceProxy(self.namespace + '/uav_manager/land', Trigger)
                response = landing_service.call()
            elif self.system == "mavros": # FIXME: Trocar para 'ros' para funcionar
                rospy.wait_for_service(self.namespace + '/mavros/cmd/land')
                landing_service = rospy.ServiceProxy(self.namespace + '/mavros/cmd/land', CommandTOL)
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