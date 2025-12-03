import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SmartSplitter(Node):
    def __init__(self):
        super().__init__('smart_splitter')
        
        # Parameters
        self.declare_parameter('target_distance', 0.5)
        self.declare_parameter('auto_set_distance', False)
        # Control gains
        self.declare_parameter('k_dist', 1.0) # Gain for distance correction

        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.auto_set_distance = self.get_parameter('auto_set_distance').get_parameter_value().bool_value
        self.k_dist = self.get_parameter('k_dist').get_parameter_value().double_value

        self.robot_0_pose = None
        self.robot_1_pose = None
        self.roles_assigned = False
        self.current_distance = 0.0

        # Logic mapping
        self.left_robot_pub = None
        self.right_robot_pub = None

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.process_cmd, 10)
        self.create_subscription(Odometry, '/tb3_0/odom', self.cb_odom_0, 10)
        self.create_subscription(Odometry, '/tb3_1/odom', self.cb_odom_1, 10)

        # Publishers
        self.pub_0 = self.create_publisher(Twist, '/tb3_0/cmd_vel', 10)
        self.pub_1 = self.create_publisher(Twist, '/tb3_1/cmd_vel', 10)

        self.get_logger().info("Waiting for Odom to assign Left/Right roles...")

    def get_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def cb_odom_0(self, msg):
        self.robot_0_pose = self._extract_pose(msg)
        if not self.roles_assigned:
            self.attempt_initialization()

    def cb_odom_1(self, msg):
        self.robot_1_pose = self._extract_pose(msg)
        if not self.roles_assigned:
            self.attempt_initialization()

    def _extract_pose(self, msg):
        return {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': self.get_yaw(msg.pose.pose.orientation)
        }

    def attempt_initialization(self):
        if self.robot_0_pose is None or self.robot_1_pose is None:
            return
        
        self.update_distance()
        self.detect_roles()

    def update_distance(self):
        # Helper to calculate Euclidean distance
        if self.robot_0_pose and self.robot_1_pose:
            dx = self.robot_0_pose['x'] - self.robot_1_pose['x']
            dy = self.robot_0_pose['y'] - self.robot_1_pose['y']
            self.current_distance = math.sqrt(dx*dx + dy*dy)

    def detect_roles(self):
        # 1. Determine Heading Vector of Robot 0
        heading_x = math.cos(self.robot_0_pose['yaw'])
        heading_y = math.sin(self.robot_0_pose['yaw'])

        # 2. Vector from Robot 0 -> Robot 1
        to_other_x = self.robot_1_pose['x'] - self.robot_0_pose['x']
        to_other_y = self.robot_1_pose['y'] - self.robot_0_pose['y']

        # 3. 2D Cross Product (Determinant)
        cross_product = heading_x * to_other_y - heading_y * to_other_x

        if cross_product > 0:
            self.get_logger().info("ROLES: TB3_1 is LEFT, TB3_0 is RIGHT")
            self.left_robot_pub = self.pub_1
            self.right_robot_pub = self.pub_0
        else:
            self.get_logger().info("ROLES: TB3_0 is LEFT, TB3_1 is RIGHT")
            self.left_robot_pub = self.pub_0
            self.right_robot_pub = self.pub_1

        if self.auto_set_distance:
            self.target_distance = self.current_distance
            self.get_logger().info(f"Auto-set target distance: {self.target_distance:.3f} m")

        self.roles_assigned = True

    def process_cmd(self, msg):
        if not self.roles_assigned:
            return

        # Ensure we have the latest distance calculation
        self.update_distance()

        v = msg.linear.x
        w = msg.angular.z

        # --- CONTROL LOGIC ---
        
        # 1. Kinematics for the "Virtual Formation" turning
        # Standard Differential Drive equations to split velocities based on turn radius
        # v_L = v - (w * d / 2)
        # v_R = v + (w * d / 2)
        v_left_nominal = v - (w * self.target_distance / 2.0)
        v_right_nominal = v + (w * self.target_distance / 2.0)

        # 2. Distance Maintenance (Steering Correction)
        # Error: Positive if we are too wide (current > target)
        # We need to steer INWARD if error is positive.
        error = self.current_distance - self.target_distance
        
        # Correction Angle (Angular velocity injection)
        # If error > 0 (too far), Left Robot turns Right (-w), Right Robot turns Left (+w)
        yaw_correction = self.k_dist * error

        # Apply corrections
        # Left Robot: Needs to steer Right (negative z) to close gap
        w_left = w - yaw_correction
        
        # Right Robot: Needs to steer Left (positive z) to close gap
        w_right = w + yaw_correction

        # --- PUBLISHING ---
        t_left = Twist()
        t_left.linear.x = v_left_nominal
        t_left.angular.z = w_left

        t_right = Twist()
        t_right.linear.x = v_right_nominal
        t_right.angular.z = w_right

        self.left_robot_pub.publish(t_left)
        self.right_robot_pub.publish(t_right)

def main():
    rclpy.init()
    node = SmartSplitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()