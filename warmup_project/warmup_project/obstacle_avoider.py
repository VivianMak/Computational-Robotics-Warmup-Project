"""Obstacle Avoidance - This script is for publishing and subscribing to ROS msgs in Python."""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class obstacleAvoidance(Node):
    """
    Sends message that robot
    """

    def __init__(self):
        """Initializes the class"""

        # Create a node
        super().__init__("obstacle_avoider")

        # Initialize variables
        self.detect_obstacle_state = False
        self.turn_direction = ""

        self.away_distance = 1.2  # Any object 1.2m away is an obstacle
        self.clearance_distance = 0.5  # Robot will turn until points are 0.5m away

        self.angle = 45
        self.left_scan = []
        self.right_scan = []

        # Create timer
        self.timer = self.create_timer(0.1, self.run_loop)

        # Create publisher
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Create subscriber
        self.sub = self.create_subscription(LaserScan, "scan", self.detect_scan, 10)

    def detect_scan(self, msg):
        """Check if there is an obstacle near and turn until clearance"""

        # Store scanned list from lidar
        self.right_scan = msg.ranges[-self.angle :]
        self.left_scan = msg.ranges[0 : self.angle]

        # Checking for obstacles in a 90 degree range
        if any(
            val <= self.away_distance for val in self.left_scan
        ):  # Obstacle on the left
            self.detect_obstacle_state = True
            self.turn_direction = "right"

        if any(
            val <= self.away_distance for val in self.right_scan
        ):  # Obstacle on the right
            self.detect_obstacle_state = True
            self.turn_direction = "left"

    def run_loop(self):
        """Neato continues going forward unless an obstacle is detected"""

        cmd_vel = Twist()

        if self.detect_obstacle_state is False:
            # Move forward
            cmd_vel.linear.x = 0.1
        else:
            # Turn the robot when obstacle is detected
            if self.turn_direction == "left":
                cmd_vel.angular.z = 0.1  # Turns counterclockwise
            if self.turn_direction == "right":
                cmd_vel.angular.z = -0.1  # Turns clockwise

            # Check for clearance
            if all(val >= self.clearance_distance for val in self.left_scan) and all(
                val >= self.clearance_distance for val in self.right_scan
            ):
                self.detect_obstacle_state = False

        self.publisher.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)  # Initialize communication with ROS
    node = obstacleAvoidance()  # Create our Node
    rclpy.spin(node)  # Run the Node until ready to shutdown
    rclpy.shutdown()  # cleanup


if __name__ == "__main__":
    main()
