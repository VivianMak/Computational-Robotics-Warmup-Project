"""A node that drives the neato in a 1m by 1m square path."""
import rclpy  
from rclpy.node import Node 
from neato2_interfaces.msg import Bump 
from geometry_msgs.msg import Twist

class DriveSquareNode(Node):
    def __init__(self):
        """Initializes the class."""
        super().__init__("drive+square_node") # node names should be unique
        # Create a timer that runs the robot motors
        self.create_timer(0.1, self.run_loop)

        # Initialize the publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Block for odom

def main(args=None):
    """Initialize our node, run it, cleanup on shut down"""
    rclpy.init(args=args)  # Initialize ROS2 network
    node = DriveSquareNode()  # Create our node
    rclpy.spin(node)  # Run our node
    rclpy.shutdown()  # If interrupted, gracefully shutdown the ROS2 network


if __name__ == '__main__':
    main()
