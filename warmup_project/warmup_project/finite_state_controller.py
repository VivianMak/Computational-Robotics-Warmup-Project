"""FINITE STATE CONTROL - This script is for publishing and subscribing to ROS msgs in Python."""

import rclpy
from rclpy.node import Node

from tf_transformations import euler_from_quaternion

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
from nav_msgs.msg import Odometry

import math
from math import pi

# Importing functions
from warmup_project.drive_square import convert_pose_to_xy_and_theta
from warmup_project.person_follower import PI_control


class finiteStateControl(Node):

    def __init__(self):
        """Initializes the class"""
        # Create a node
        super().__init__("finite_state_controller")

        # Create a timer
        self.create_timer(0.1, self.run_loop)

        # Control state
        self.node_state = "drive_sqare_node"  # Start with driving in a square

        # Initialize the publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Initialize subscriber for odom
        self.odom_sub = self.create_subscription(Odometry, "odom", self.get_odom, 10)

        # Initialize subscriber for LiDAR scan
        self.sub = self.create_subscription(LaserScan, "scan", self.process_scan, 10)

        # DRIVE IN A SQAURE ########################################
        # Initialize variables
        self.body_pos = None
        self.num_turns = 0

        self.pos = [0, 0, 0]
        self.travel_distance = 1.0  # 1 meter travel distance
        self.rotation = pi / 2

        # PERSON FOLLOWING ###########################################
        self.degree_of_view = 15
        self.max_range = 1.5  # The max distance we limit the Neato's IR to see
        self.r_list = None
        self.distance_from_person = 0.6

        self.dt = 0.3
        self.kP = 1.0
        self.e_integral = 0

    # DRIVE IN A SQAURE ################
    def get_odom(self, msg):
        """Callback for handling odometry position
        Input:
            msg (Odometry): an Odometry type message from the subscriber.
        """
        self.pos = convert_pose_to_xy_and_theta(msg.pose.pose)

    def process_scan(self, msg):
        """
        Callback for handling a wall detector sensor input.

        Args:

        """
        r_list = []
        # Initialize variables
        for idx in range(-self.degree_of_view, self.degree_of_view):
            dist = msg.ranges[idx]
            r_list.append(dist)
        self.r_list = r_list

    def run_loop_square(self):
        """Drives the robot in a square."""
        # Set the starting position
        if self.body_pos is None:
            self.body_pos = self.pos
        # Create a Twist message to describe the robot motion
        vel = Twist()
        # Define relative position as the delta of pos and body frame
        rel_pos = [a - b for a, b in zip(self.pos, self.body_pos)]
        print("Relative Position: ", rel_pos)

        # Repeat the process of going straight and turning to the right
        if self.num_turns < 4:
            if (
                abs(rel_pos[0]) - self.travel_distance < 0.001
                and abs(rel_pos[1]) - self.travel_distance < 0.001
            ):
                vel.linear.x = 0.15

            # Turn to the right 90 degrees
            elif abs(rel_pos[2]) - self.rotation < 0.001:
                vel.linear.x = 0.0
                vel.angular.z = -0.3

            else:
                self.num_turns += 1
                print("Num turns = ", self.num_turns)
                self.body_pos = self.pos

        else:
            # Finished driving in a square
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            quit()

        # Publish the Twist message to cmd_vel target
        self.publisher.publish(vel)

    def run_loop_person(self):
        """Tracks the relative position of the person and tracks them."""

        # Create a Twist message to describe the robot motion
        vel = Twist()
        # Initialize sum variables and person_points dict
        sum_person_r = 0.0
        sum_person_angle_idx = 0.0
        person_points = {}
        # Prevent index out of bounds error if the loop runs before the subscriber updates self.r_list
        if self.r_list is not None:
            for idx, r in enumerate(self.r_list):
                # Remove all LiDAR points that don't exist (inf) or are too far away. Reduces noise.
                if r != float("inf") and r <= self.max_range:
                    person_points[idx] = (
                        r  # adds index of person_points in case there is a gap the LiDAR scan (i.e 11, 12, 14, 15)
                    )
                    sum_person_r += r
                    sum_person_angle_idx += idx

            if len(person_points) == 0:  # prevent divide by 0 error
                vel.linear.x = 0.0
                vel.angular.z = 0.0
            else:
                # Estimate that the person is at the average of the distances and angles taken from the LiDAR scan
                average_r = sum_person_r / len(person_points)
                average_person_angle_idx = (
                    sum_person_angle_idx / len(person_points) - self.degree_of_view
                )  # to correct for idx starting at 0 instead of -15
                polar_endpoint = (average_r, average_person_angle_idx)
                print(f"Polar endpoint: {polar_endpoint}")

                # print(f"delta angle {rel_pos[2] - polar_endpoint[1]}") # come back to this, why is delta angle 13.8 while polar_endpoint[1] is -13.5?

                if polar_endpoint[1] > 0.05:

                    # vel.angular.z = PI_control("angular", error, self.e_integral)
                    # vel.linear.x = PI_control("linear", error, self.e_integral)

                    vel.angular.z = 0.1

                elif polar_endpoint[1] < -0.05:
                    vel.angular.z = -0.1

                else:
                    vel.angular.z = 0.0

                if polar_endpoint[0] > self.distance_from_person:
                    # speed = self.distance_from_person / self.kP
                    # if speed > 0.25: speed = 0.25
                    # vel.linear.x = speed
                    vel.linear.x = 0.15
                else:
                    vel.linear.x = 0.0

        # Publish the Twist message to cmd_vel target
        self.publisher.publish(vel)

    def run_loop(self):
        # self.run_loop_wall()
        if self.node_state == "drive_square_node":
            self.run_loop_square()
        else:
            self.run_loop_person()

        if self.num_turns >= 4:
            print("STOPPPPPPPPPPPPPPPP GO PERSON")
            self.run_loop_person()


def main(args=None):
    """Initialize our node, run it, cleanup on shut down"""
    rclpy.init(args=args)  # Initialize communication with ROS
    node = finiteStateControl()  # Create our Node
    rclpy.spin(node)  # Run the Node until ready to shutdown
    rclpy.shutdown()  # cleanup


if __name__ == "__main__":
    main()