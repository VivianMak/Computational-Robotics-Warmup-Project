"""WALL FOLLOWING - This script is for publishing and subscribing to ROS msgs in Python."""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header

# from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist

from neato2_interfaces.msg import Scan 


class wallFollowing(Node):
    """Thisis a msg sending node, which inherits from Node. Sends that the robot detects a wall"""

    def __init__(self):
        """Initializes the class"""

        # Create a node
        super().__init__("send_message_node")  # calls the node "send_message_node"

        # Initialize variables
        self.detect_wall_state = False
        self.turn_angle = 0

        # Create a timer
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.run_loop)

        # Create a publisher for the motors
        self.publisher = self.create_publisher(, "", 10)


        # Create subscriber to the bumper sensor
        self.sub = self.create_subscription(, "", self., 10)


    def process_bump(self, msg):
        """Callback for handling a wall detector sensor input."""
        # Process msg
        self.detect_wall_state = (
            # if the lidar bounces
        )
        # Determine if a bump has happened
        # Set the bumper state variable

        # Logic to determine how much to turn
        self.turn_angle = msg.angle_increment # angle_min or angle_max

    def run_loop(self):
        """Move robot and turn wheel towards wall"""
        # Set the subscriber to be Twist type
        cmd_vel = Twist

        # Command the robot to move
        # If there is a wall, turn towards wall and go towards it
        cmd_vel.angular.z = self.turn_angle
        cmd_vel.linear.x = 0.1

        self.publisher.publish(cmd_vel)

def main(args=None):
    """Initializes a node, runs it, and cleans up after termination.
    Input: args(list) -- list of arguments to pass into rclpy. Default None.
    """
    rclpy.init(args=args)  # Initialize communication with ROS
    node = wallFollowing()  # Create our Node
    rclpy.spin(node)  # Run the Node until ready to shutdown
    rclpy.shutdown()  # cleanup


if __name__ == "__main__":
    main()