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

class ROS_API:
    def __init__(self, system_type, namespace='/uav1', realPoseCallback=None, navDataCallback=None):
        rospy.init_node('ros_api', anonymous=True)

        # Context
        self.system_type = system_type
        self.namespace = namespace

        self.pose = PoseStamped()
        self.velocity = VelocityReferenceStamped()

        # Services and topics paths
        self.ARM_path = namespace + '/mavros/cmd/arming'
        self.SET_MODE_path = namespace + '/mavros/set_mode'
        self.TAKEOFF_path = namespace + '/mavros/cmd/takeoff'
        self.LAND_path = namespace + '/mavros/cmd/land'
        self.STATE_path = namespace + '/mavros/state'
        self.POSE_path = namespace + '/mavros/local_position/pose'
        self.VELOCITY_path = namespace + '/mavros/setpoint_velocity/cmd_vel'
        # ...

        # Subscribers
        self.pose_subscriber = rospy.Subscriber(self.POSE_path, PoseStamped, realPoseCallback)
        self.odometry_subscriber = rospy.Subscriber(self.POSE_path, PoseStamped, navDataCallback)

        self.flying = False

    def is_flying(self):
        return self.flying

    def arm(self):
        rospy.wait_for_service(self.ARM_SERVICE_path)
        try:
            arm_service = rospy.ServiceProxy(self.ARM_SERVICE_path, CommandBool)
            response = arm_service(True)
            if response.success:
                rospy.loginfo("Drone armado com sucesso!")
            else:
                rospy.logwarn("Falha ao armar o drone")
        except rospy.ServiceException as e:
            rospy.logerr("Erro ao chamar o serviço de armamento: %s" % e)

    def disarm(self):
        rospy.wait_for_service(self.ARM_SERVICE_path)
        try:
            disarm_service = rospy.ServiceProxy(self.ARM_SERVICE_path, CommandBool)
            response = disarm_service(False)
            if response.success:
                rospy.loginfo("Drone desarmado com sucesso!")
            else:
                rospy.logwarn("Falha ao desarmar o drone")
        except rospy.ServiceException as e:
            rospy.logerr("Erro ao chamar o serviço de desarmamento: %s" % e)

    def set_flight_mode(self, mode):
        rospy.wait_for_service(self.SET_MODE_path)
        try:
            set_mode_service = rospy.ServiceProxy(self.SET_MODE_path, SetMode)
            response = set_mode_service(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo(f"Modo de voo alterado para '{mode}' com sucesso!")
            else:
                rospy.logwarn("Falha ao alterar o modo de voo")
        except rospy.ServiceException as e:
            rospy.logerr("Erro ao chamar o serviço de alteração de modo de voo: %s" % e)

    def takeoff(self, altitude_passed=3.0):
        if self.flying:
            rospy.logwarn("Takeoff FAILED: O drone já está no ar")
            return False
        try:
            print("\n----------Tentando decolar----------")
            rospy.wait_for_service(self.TAKEOFF_path)
            takeoff_service = rospy.ServiceProxy(self.TAKEOFF_path, CommandTOL)
            response = takeoff_service(altitude=altitude_passed, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
            print('Takeoff response:', response)
            if not response.success:
                print("Provavelmente o drone já está no ar mas a variável self.flying estava incorreta\n")
            self.flying = True
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Falha na decolagem: %s", str(e))
            return False
        
    def land(self):
        if not self.flying:
            rospy.logwarn("Land FAILED: O drone já está no chão")
            return False
        try:
            print("\n----------Pousando----------")
            rospy.wait_for_service(self.LAND_path)
            landing_service = rospy.ServiceProxy(self.LAND_path, CommandTOL)
            response = landing_service(altitude=0.0)
            print('Landing response:', response)
            if not response.success:
                print("Provavelmente o drone já está no chão\n")
            self.flying = False
            return True
        except rospy.ServiceException as e:
            rospy.logerr("Falha no pouso: %s", str(e))
            return False