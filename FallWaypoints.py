import rospy
from geometry_msgs.msg import Pose, PoseStamped
from mrs_msgs.srv import Vec4

class FallWaypoints:
    def __init__(self, namespace='/uav1', rate=10, type='mavros', dst_to_target_threshold=0.1):
        rospy.init_node('fall_waypoints', anonymous=True)
        self.namespace = namespace
        self.rate = rospy.Rate(rate)
        self.pose = PoseStamped()
        self.waypoints = []
        self.current_waypoint_index = 0
        self.type = type
        self.stopped = False
        self.dst_to_target_threshold = dst_to_target_threshold
        
        # Publishers and Subscribers
        self.pose_pub = rospy.Publisher(f"{self.namespace}/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/uav1/mavros/local_position/pose',
                                                PoseStamped, self._update_pose)

    def set_waypoints(self, waypoints):
        
        self.waypoints = waypoints

    def append_waypoint(self, waypoint):
        self.waypoints.append(waypoint)

    def append_waypoints(self, waypoints):
        self.waypoints.extend(waypoints)

    def clear_waypoints(self):
        self.waypoints = []
        self.current_waypoint_index = 0

    def start_movement(self):
        target_waypoint = self.waypoints[self.current_waypoint_index]
        print('   Indo para', target_waypoint)
        last_distance_to_target = 0

        self._move_to(target_waypoint)

        while not rospy.is_shutdown() and self.current_waypoint_index < len(self.waypoints) and not self.stopped:
            current_x, current_y, current_z = self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z
            #distance_to_target = ((target_waypoint[0] - current_x) ** 2 + (target_waypoint[1] - current_y) ** 2 + (target_waypoint[2] - current_z) ** 2) ** 0.5

            if self._reached_target(target_waypoint):
                self.current_waypoint_index += 1
                if self.current_waypoint_index >= len(self.waypoints):
                    break
                print('   Indo para o próximo:', target_waypoint)
                target_waypoint = self.waypoints[self.current_waypoint_index]
                self._move_to(target_waypoint)
                continue

            # if last distance to target is greater than current distance to target, 
            # then the drone has passed the target and must return to it
            #if last_distance_to_target > distance_to_target:
            #    self._move_to(target_waypoint)

            #last_distance_to_target = distance_to_target
            
            rospy.sleep(1.0)
        print('Fim do movimento!')

    def _move_to(self, target):
        if self.type == 'goto':
            rospy.wait_for_service('/uav1/control_manager/goto')
            goto_service = rospy.ServiceProxy('/uav1/control_manager/goto', Vec4)
            response = goto_service.call(target)  # Altitude de decolagem em metros
            print('/> GOTO Response:', response)
    
    def _reached_target(self, target):
        current_x, current_y, current_z = self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z
        distance_to_target = ((target[0] - current_x) ** 2 + (target[1] - current_y) ** 2 + (target[2] - current_z) ** 2) ** 0.5
        z_position_hacked = self.pose.pose.position.z + 0.8
        print('target[2]:', target[2], '   z_pos:', z_position_hacked)
        distance_to_target_hacked = ((target[0] - current_x) ** 2 + (target[1] - current_y) ** 2 + (target[2] - z_position_hacked) ** 2) ** 0.5
        threshold = self.dst_to_target_threshold
        print(">> Distance to target:      ", distance_to_target)
        print(">> Distance to target FIXED:", distance_to_target_hacked)
        print("     current=      ",[round(self.pose.pose.position.x, 1), round(self.pose.pose.position.y, 1), round(self.pose.pose.position.z, 1)])
        print("     fixed_current=",[round(self.pose.pose.position.x, 1), round(self.pose.pose.position.y, 1), round(z_position_hacked, 1)])
        print("     target=       ",target)
        if distance_to_target < threshold or distance_to_target_hacked < threshold:
            print("   Chegou no waypoint:", target)
            return True  # O drone chegou ao destino
        

    def get_status(self):
        # Obter a posição e orientação atual do drone
        return self.pose

    def _update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.pose.position.x = round(self.pose.pose.position.x, 4)
        self.pose.pose.position.y = round(self.pose.pose.position.y, 4)
        self.pose.pose.position.z = round(self.pose.pose.position.z, 4)
        return self.pose

    def stop_movement(self):
        self.stopped = True

    def resume_movement(self):
        self.start_movement()

# TODO: Tratamento adequado de erros
# TODO: Adicionar compatibilidade adicional com ROS e MAVROS conforme necessário.