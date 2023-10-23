# This class handles everything related to the movement of the drone
from fall_uav.algorithms.pid import PIDController
from fall_uav.controllers.waypoint_manager import WaypointManager
from fall_uav.robotics_api import RoboticsAPI

class MotionController:
    def __init__(self, robotics_api, pid_controller, waypoint_manager, rate = 10, dst_to_target_threshold=0.1, log=[]):
        self.robotics_api = robotics_api  # instance of RoboticsAPI
        self.pid_controller = pid_controller  # instance of PIDController
        self.waypoint_manager = waypoint_manager  # instance of WaypointManager

        self.rate = rate  # could use rospy.Rate(rate) if needed
        self.dst_to_target_threshold = dst_to_target_threshold
        self.log = log

        self.pose = PoseStamped()

        self.system = system
        self.stopped = False
        self.dst_to_target_threshold = dst_to_target_threshold

        self.fw = FallWaypoints()
        self.curr_target = None

        # Soft start and deceleration
        self.soft_start_duration = 4.0
        self.soft_start_end_time = None
        self.deceleration_radius = 3.5
        self.deceleration_radius_end = 0.5


        # PID paramaters and Controllers
        #kp = np.ones(3)
        #ki = np.zeros(3)
        #kd = np.zeros(3)
        kp = np.ones(3) * 0.3
        ki = np.ones(3) * 0.01
        kd = np.ones(3) * 0.01
        ki_sat = np.ones(3) * 0.1
        self.pid_controller = PIDController(kp, ki, kd, ki_sat, 1/rate)
        
        # Publishers and Subscribers
        self.pose_pub = rospy.Publisher(f"{self.namespace}/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/uav1/mavros/local_position/pose',
                                                PoseStamped, self._update_pose)
        self.velocity_pub = rospy.Publisher(f"{self.namespace}/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        self.velocity_pub_sim = rospy.Publisher('/uav1/control_manager/velocity_reference', VelocityReferenceStamped, queue_size=10)
        
        
    def move_to_waypoints(self, waypoints):
        self.fw.set_waypoints(waypoints)
        self.start_movement()

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
        self._send_velocity_command([0, 0, 0])

    def resume_movement(self):
        self.start_movement()

    def start_movement(self):
        pid_on = True

        self.curr_target = self.fw.next_waypoint()

        print('   Indo para', self.curr_target)
        if self.system == 'uav_manager' and not pid_on:
            self._go_to(self.curr_target)

        self.soft_start_end_time = rospy.Time.now() + rospy.Duration(self.soft_start_duration)

        while not rospy.is_shutdown() and not self.stopped:
            if self.system == 'mavros' or pid_on:
                self._move_towards(self.curr_target)

            if self._reached_target(self.curr_target):
                if not self.fw.has_more_waypoints():
                    break
                self.curr_target = self.fw.next_waypoint()
                print('   Indo para o próximo:', self.curr_target)
                if self.system == 'uav_manager' and not pid_on:
                    self._go_to(self.curr_target)
                continue
            
            self.rate.sleep()
        print('Fim do movimento!')

    def slow_descent(self, min_height):
        current_height = self.get_status().pose.position.z  # Fetching current altitude
        descent_rate = 0.05  # Altitude reduction per iteration. Adjust as needed.

        # Store the original PID values
        original_kp = np.copy(self.pid_controller.kp)
        original_ki = np.copy(self.pid_controller.ki)
        original_kd = np.copy(self.pid_controller.kd)

        # Set new PID values for a slower descent
        self.pid_controller.kp = np.array([0.0, 0.0, 0.4])
        self.pid_controller.ki = np.array([0.00, 0.00, 0.01])
        self.pid_controller.kd = np.array([0, 0, 0])

        print("Initiating slow descent...")
        while True:
            current_height = self.get_status().pose.position.z
            altitude_error = min_height - current_height
            print("Current height:", current_height, "m")
            print("Altitude error:", altitude_error, "m")
            
            # If the drone is already below the min_height, stop
            if altitude_error > 0:
                print("Descent complete.")
                break

            # Use PID controller to compute the velocity command
            control_input = self.pid_controller.compute(np.array([0, 0, altitude_error]), np.array([0, 0, 0]))

            # Ensure the control input is negative (descending)
            descent_velocity = min(control_input[2], 0)
            descent_velocity = -0.15
            print("Descent velocity:", descent_velocity, "m/s")
            
            # Send the velocity command to the drone
            soft_start_factor = self._soft_start_scaling()
            deceleration_factor = self._deceleration_scaling(np.array([0, 0, altitude_error]))
            descent_velocity = soft_start_factor * deceleration_factor * descent_velocity
            print("Descent velocity (scaled):", descent_velocity, "m/s")

            self._send_velocity_command([0, 0, descent_velocity])

            self.rate.sleep()

        # Reset PID controller after descent
        self.pid_controller.reset()

        # Restore the original PID values
        self.pid_controller.kp = original_kp
        self.pid_controller.ki = original_ki
        self.pid_controller.kd = original_kd

        print("Descent complete.")

    def _go_to(self, target):
        if self.system == 'mavros':
            rospy.wait_for_service('SERVICE GO_TO DO DRONE')
            goto_service = rospy.ServiceProxy('SERVICE GO_TO DO DRONE', Vec4)
            response = goto_service.call(target)  # Altitude de decolagem em metros
            print('/> GOTO Response:', response)
        elif self.system == 'uav_manager':
            rospy.wait_for_service('/uav1/control_manager/goto')
            goto_service = rospy.ServiceProxy('/uav1/control_manager/goto', Vec4)
            response = goto_service.call(target)  # Altitude de decolagem em metros
            print('/> GOTO Response:', response)
        

    def _move_towards(self, target):
        curr_pos = np.array([self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z])
        pos_error = self._calc_pos_error(target, curr_pos)
        vel_error = self._calc_vel_error([0, 0, 0], [0, 0, 0])
        velocity_command = self.pid_controller.compute(pos_error, vel_error)

        # Soft start scaling
        soft_start_factor = self._soft_start_scaling()
        deceleration_factor = self._deceleration_scaling(target)
        velocity_command = soft_start_factor * deceleration_factor * velocity_command
        
        # Send the velocity command to the drone
        self._send_velocity_command(velocity_command)

    def _soft_start_scaling(self):
        # If we're still in the soft start period
        if rospy.Time.now() < self.soft_start_end_time:
            elapsed_time = (self.soft_start_end_time - rospy.Time.now()).to_sec()
            return 1 - (elapsed_time / self.soft_start_duration)
        return 1.0  # Otherwise, return 1 (no scaling)
    
    def _deceleration_scaling(self, target):
        current_x, current_y, current_z = self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z
        distance_to_target = ((target[0] - current_x) ** 2 + (target[1] - current_y) ** 2 + (target[2] - current_z) ** 2) ** 0.5
        
        # If inside the deceleration radius
        if distance_to_target < self.deceleration_radius:
            scaling_factor = distance_to_target / self.deceleration_radius
            return max(0.4, scaling_factor)  # Ensure a minimum scaling factor (e.g., 0.1) to avoid a complete stop
        return 1.0  # Otherwise, return 1 (no scaling)

    def _send_velocity_command(self, v):
        if self.system == 'mavros':
            velocity_msg = TwistStamped()
            vx, vy, vz = v[0], v[1], v[2]
            velocity_msg.twist.linear.x = vx
            velocity_msg.twist.linear.y = vy
            velocity_msg.twist.linear.z = vz
            self.velocity_pub.publish(velocity_msg)
        elif self.system == 'uav_manager':
            velocity_msg = VelocityReferenceStamped()
            vx, vy, vz = v[0], v[1], v[2]
            velocity_msg.reference.velocity.x = vx
            velocity_msg.reference.velocity.y = vy
            velocity_msg.reference.velocity.z = vz
            self.velocity_pub_sim.publish(velocity_msg)

    def _reached_target(self, target):
        current_x, current_y, current_z = self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.position.z
        distance_to_target = ((target[0] - current_x) ** 2 + (target[1] - current_y) ** 2 + (target[2] - current_z) ** 2) ** 0.5
        z_position_hacked = self.pose.pose.position.z + 0.8
        distance_to_target_hacked = ((target[0] - current_x) ** 2 + (target[1] - current_y) ** 2 + (target[2] - z_position_hacked) ** 2) ** 0.5
        threshold = self.dst_to_target_threshold
        if 'position' in self.log:
            print(">> Distance to target:      ", distance_to_target)
            print(">> Distance to target FIXED:", distance_to_target_hacked)
            print("     current=      ",[round(self.pose.pose.position.x, 1), round(self.pose.pose.position.y, 1), round(self.pose.pose.position.z, 1)])
            print("     fixed_current=",[round(self.pose.pose.position.x, 1), round(self.pose.pose.position.y, 1), round(z_position_hacked, 1)])
            print("     target=       ",target)
        if distance_to_target < threshold or distance_to_target_hacked < threshold:
            print("   Chegou no waypoint:", target,"\n")
            # Reset PID controllers upon reaching a waypoint
            self.pid_controller.reset()
            return True  # O drone chegou ao destino
        return False
    
    def _calc_pos_error(self, target, current):
        ''' Returns the error between actual position and reference position'''
        pos_error = np.array(target[:3]) - np.array(current)
        return pos_error
    
    def _calc_vel_error(self, target, current):
        ''' Returns the error between actual velocity and reference velocity'''
        vel_error = np.array(target[:3]) - np.array(current)
        return vel_error
        

