import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DualBurgerSplitter(Node):
    def __init__(self):
        super().__init__('dual_burger_splitter')

        # --- TUNING PARAMETERS ---
        self.separation_len = 0.5  # Distance between the two robots (meters)
        self.wheel_radius_error = 1.0 # Use 1.0 initially. Adjust if one robot is faster.

        # Subscribe to the "Virtual" command (from Joystick or Nav2)
        self.sub = self.create_subscription(
            Twist, 
            '/cmd_vel', # Topic Nav2 publishes to
            self.process_command, 
            10)

        # Publishers for the individual robots
        self.pub_left = self.create_publisher(Twist, '/tb3_0/cmd_vel', 10)
        self.pub_right = self.create_publisher(Twist, '/tb3_1/cmd_vel', 10)

    def process_command(self, msg):
        v = msg.linear.x      # Forward speed of the center point
        omega = msg.angular.z # Rotation speed of the formation

        # --- KINEMATIC SPLIT ---
        # v_left = v - (omega * L / 2)
        # v_right = v + (omega * L / 2)
        
        v_left = v - (omega * self.separation_len / 2.0)
        v_right = v + (omega * self.separation_len / 2.0)

        # Create messages
        # Note: angular.z is 0 because the robots themselves don't spin relative 
        # to the formation; they drive arcs. However, to drive an arc, 
        # differential drive robots usually need an angular component locally.
        # For a rigidly connected feel:
        # Local Angular Velocity = Global Angular Velocity
        
        twist_left = Twist()
        twist_left.linear.x = v_left
        twist_left.angular.z = omega 

        twist_right = Twist()
        twist_right.linear.x = v_right
        twist_right.angular.z = omega

        self.pub_left.publish(twist_left)
        self.pub_right.publish(twist_right)

def main():
    rclpy.init()
    node = DualBurgerSplitter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
