import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class SmartSplitter(Node):
    def __init__(self):
        super().__init__('smart_splitter')
        
        self.declare_parameter('target_distance', 0.5)
        self.declare_parameter('auto_set_distance', False)

        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        self.auto_set_distance = self.get_parameter('auto_set_distance').get_parameter_value().bool_value

        self.robot_0_pose = None
        self.robot_1_pose = None
        self.roles_assigned = False
        self.current_distance = 0.0

        # Subscribers
        self.create_subscription(Twist,'/cmd_vel', self.process_cmd,10)
        self.create_subscription(Odometry, '/tb3_0/odom', self.cb_odom_0,10)
        self.create_subscription(Odometry, '/tb3_1/odom', self.cb_odom_1,10)

        # Publishers
        self.pub_left_wheel_logic = None
        self.pub_right_wheel_logic = None

        self.pub_0 = self.create_publisher(Twist,'/tb3_0/cmd_vel',10)
        self.pub_1 = self.create_publisher(Twist,'/tb3_1/cmd_vel',10)

        self.get_logger().info("Waiting for Odom to assign Left/Right roles.")

    def get_yaw(self, q):
        # Convert quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def cb_odom_0(self, msg):
        q = msg.pose.pose.orientation
        self.robot_0_pose = {'x': msg.pose.pose.position.x,
                       'y': msg.pose.pose.position.y,
                       'yaw': self.get_yaw(q)}
        self.check_initialization()

    def cb_odom_1(self, msg):
        q = msg.pose.pose.orientation
        self.robot_1_pose = {'x': msg.pose.pose.position.x,
                       'y': msg.pose.pose.position.y,
                       'yaw': self.get_yaw(q)}
        self.check_initialization()

    def check_initialization(self):
        if self.robot_0_pose is None or self.robot_1_pose is None:
            return
        
        dx = self.robot_0_pose['x'] - self.robot_1_pose['x']
        dy = self.robot_0_pose['y'] - self.robot_1_pose['y']
        self.current_distance = math.sqrt(dx*dx + dy*dy)

        if not self.roles_assigned:
            self.detect_roles()

    def detect_roles(self):
        # Calculate heading vector for robot 0
        heading_x = math.cos(self.robot_0_pose['yaw'])
        heading_y = math.sin(self.robot_0_pose['yaw'])

        # Vector from robot 0 to robot 1
        to_other_x = self.robot_1_pose['x'] - self.robot_0_pose['x']
        to_other_y = self.robot_1_pose['y'] - self.robot_0_pose['y']

        cross_product = heading_x * to_other_y - heading_y * to_other_x

        if cross_product > 0:
            self.get_logger().info("ROLES ASSIGNED: TB3_1 is LEFT, TB3_0 is RIGHT")
            self.pub_left_wheel_logic = self.pub_1
            self.pub_right_wheel_logic = self.pub_0
        else:
            self.get_logger().info("ROLES ASSIGNED: TB3_0 is LEFT, TB3_1 is RIGHT")
            self.pub_left_wheel_logic = self.pub_0
            self.pub_right_wheel_logic = self.pub_1

        if self.auto_set_distance:
            self.target_distance = self.current_distance
            self.get_logger().info(f"AUTO-SET TARGET DISTANCE: {self.target_distance:.3f} meters")

        self.roles_assigned = True

    def process_cmd(self, msg):
        if not self.roles_assigned:
            self.get_logger().warn("Roles not assigned yet. Ignoring cmd_vel.")
            return

        v = msg.linear.x
        w = msg.angular.z

        error = self.target_distance - self.current_distance
        k_p = 0.5
        correction = k_p * error

        eff_L = self.target_distance + correction
        v_left_logic = v - (w * eff_L / 2)
        v_right_logic = v + (w * eff_L / 2)

        # publish to left and right wheel logic robots
        twist_left = Twist()
        twist_left.linear.x = v_left_logic
        twist_left.angular.z = w
        twist_right = Twist()
        twist_right.linear.x = v_right_logic
        twist_right.angular.z = w

        self.pub_left_wheel_logic.publish(twist_left)
        self.pub_right_wheel_logic.publish(twist_right)

def main():
    rclpy.init()
    node = SmartSplitter()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()