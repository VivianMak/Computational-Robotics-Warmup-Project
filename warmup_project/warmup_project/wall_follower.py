"""WALL FOLLOWING - This script is for publishing and subscribing to ROS msgs in Python."""

import rclpy
from rclpy.node import Node

import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data


class wallFollowing(Node):
    """Sends message that the robot detects a wall"""

    def __init__(self):
        """Initializes the class"""

        # Create a node
        super().__init__("wall_follower")

        # Initialize variables
        self.align_state = False
        self.turn_angle = 0
        self.slope = 0

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

        """
        # Initialize variables
        theta = math.radians(45)
        index1 = 270
        index2 = 271

        # Find two points from list of ranges and convert them to x and y coordinates
        x1 = msg.ranges[index1] * math.cos(theta)
        y1 = msg.ranges[index1] * math.sin(theta)
        x2 = msg.ranges[index2] * math.cos(theta + msg.angle_increment)
        y2 = msg.ranges[index2] * math.sin(theta + msg.angle_increment)

        # debug
        print("The ranges on left side is:")
        print(msg.ranges[90])

        # print(f"the angle min is: {msg.angle_min}")
        # print(f"the angle max is: {msg.angle_max}")
        # print(f"the angle increment is: {msg.angle_increment}")

        # Find the slope of the wall
        self.slope = (y2 - y1) / (x2 - x1)

        # Find the angular error for the Neato to adjust
        self.turn_angle = math.radians(180 - math.atan(self.slope))

        # print(f"the slope is {self.slope}")

        # Process msg
        if self.turn_angle != 0:
            self.align_state = True

    def run_loop(self):
        """Move robot towards detected wall"""
        # Set the subscriber to be Twist type
        cmd_vel = Twist()

        # Initially set the forware velocity and angular state to 0
        if self.align_state is False:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        else:
            # cmd_vel.angular.z = -0.1
            if self.slope < 0:
                # Turn counterclockwise
                cmd_vel.angular.z = 0.1
            else:
                # Turn clockwise
                cmd_vel.angular.z = -0.1  # self.turn_angle / self.timer
                cmd_vel.linear.x = 0.1

        # cmd_vel.linear.x = 0.1

        self.publisher.publish(cmd_vel)


def main(args=None):
    """ """
    rclpy.init(args=args)  # Initialize communication with ROS
    node = wallFollowing()  # Create our Node
    rclpy.spin(node)  # Run the Node until ready to shutdown
    rclpy.shutdown()  # cleanup


if __name__ == "__main__":
    main()
