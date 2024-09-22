"""WALL FOLLOWING - This script is for publishing and subscribing to ROS msgs in Python."""

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class wallFollowing(Node):
    """
    This node drives the robot along the wall in a parallel line

    Publishers:
        - Twist cmd_vel message: commands velocity and rotation
    SubscribersL
        - Laserscan scan message: gives the distance from a lidar scan
    """

    def __init__(self):
        """Initializes the class"""

        # Create a node
        super().__init__("wall_follower")

        # Initialize variables
        self.align_state = False

        self.angle_error = 0
        self.slope = 0

        self.heading = ""

        # Create a timer -- runs until the laser data sense that it is close to a wall
        self.timer = self.create_timer(0.1, self.run_loop)

        # Create a publisher for the motors
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Create subscriber to the bumper sensor
        self.sub = self.create_subscription(LaserScan, "scan", self.process_scan, 10)

    def process_scan(self, msg):
        """
        Callback for handling a wall detector sensor input.
        Args:
            - msg (Laserscan): a Laserscan type message from the subscriber
        """
        # Placing Neato to the right of the wall
        theta = math.radians(45)
        index1 = 45
        index2 = 50

        # Convert point to coordinates
        x1 = msg.ranges[index1] * math.cos(theta)
        y1 = msg.ranges[index1] * math.sin(theta)
        x2 = msg.ranges[index2] * math.cos(theta + 5 * msg.angle_increment)
        y2 = msg.ranges[index2] * math.sin(theta + 5 * msg.angle_increment)

        # Find the slope of the wall
        self.slope = (y2 - y1) / (x2 - x1)

        # Angular Error
        self.angle_error = 90 - math.atan(self.slope)

        # Start aligning once the angle error is greater than tolerance
        if self.angle_error > 0.1:
            self.align_state = True

        # Heading of Neato front -- wall always to the left
        if self.slope > 0:
            self.heading = "towards"
        else:
            self.heading = "away"

    def run_loop(self):
        """Move robot towards detected wall"""

        # Set the subscriber to be Twist type
        cmd_vel = Twist()

        if self.align_state is False:
            cmd_vel.linear.x = 0.1
        else:
            # Check if trajectory is parallel
            if self.angle_error <= 90.2 and self.angle_error >= 89.8:
                self.align_state = False

            # Align the Neato
            if self.heading == "towards":
                cmd_vel.angular.z = 0.1  # Turn clockwise
            else:
                cmd_vel.angular.z = -0.1  # Turn COUNTERclockwise
            cmd_vel.linear.x = 0.1

        self.publisher.publish(cmd_vel)


def main(args=None):
    """Initialize our node, run it, cleanup on shut down"""
    rclpy.init(args=args)  # Initialize communication with ROS
    node = wallFollowing()  # Create our Node
    rclpy.spin(node)  # Run the Node until ready to shutdown
    rclpy.shutdown()  # cleanup


if __name__ == "__main__":
    main()
