"""FINITE STATE CONTROL - This script is for publishing and subscribing to ROS msgs in Python."""

from math import pi

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Importing functions
from warmup_project.drive_square import convert_pose_to_xy_and_theta


class FiniteStateControl(Node):
    """
    This node combines the two behaviors: driving in a square and person following
    The Neato will continously drive in a circle unless a person is detect.
    As it is following hte person, if the person goes out of range, then the Neato will

    """

    def __init__(self):
        """Initializes the class"""
        # Create a node
        super().__init__("finite_state_controller")

        # Create a timer
        self.create_timer(0.1, self.run_loop)

        # Intiialize state control variables
        self.person_detect_state = False
        self.node_state = "drive_square_node"  # Start with driving in a square

        # Initialize the publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Initialize subscriber for odom
        self.odom_sub = self.create_subscription(Odometry, "odom", self.get_odom, 10)

        # Initialize subscriber for LiDAR scan
        self.sub = self.create_subscription(LaserScan, "scan", self.process_scan, 10)

        # Initialize drive_square variables
        self.body_pos = None
        self.num_turns = 0
        self.pos = [0, 0, 0]
        self.travel_distance = 1.0  # 1 meter travel distance
        self.rotation = pi / 2

        # Initialize person_follower variables
        self.degree_of_view = 15
        self.max_range = 1.0  # The max distance we limit the Neato's IR to see
        self.r_list = None
        self.distance_from_person = 0.6
        

    def get_odom(self, msg):
        """Callback for handling odometry position
        Input:
            msg (Odometry): an Odometry type message from the subscriber.
        """
        self.pos = convert_pose_to_xy_and_theta(msg.pose.pose)

    def process_scan(self, msg):
        """
        Callback for handling the LiDAR scan points in the degree of view.

        Args: msg (LaserScan): a LaserScan type message from the subscriber. Neato /scan topic.
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
                vel.linear.x = 0.1

            # Turn to the right 90 degrees
            elif abs(rel_pos[2]) - self.rotation < 0.001:
                vel.linear.x = 0.0
                vel.angular.z = -0.2

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

    def check_for_person(self):
        """
        Uses LaserScan from /scan topic to check if there is a person in the 30 degree of view
        of the Neato.

        Returns:
        None if there is no person detected by the LiDAR scan, or the polar coordinates of the calculated
        center of mass of the person in the form of a tuple of floats.
        """
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
                self.person_detect_state = False
            else:
                self.person_detect_state = True
                average_r = sum_person_r / len(person_points)
                average_person_angle_idx = (
                    sum_person_angle_idx / len(person_points) - self.degree_of_view
                )  # to correct for idx starting at 0 instead of -15
                return average_r, average_person_angle_idx
        return None

    def run_loop_person(self):
        """Tracks the relative position of the person and tracks them."""

        # Create a Twist message to describe the robot motion
        vel = Twist()
        # Get the output from check_for_person() 
        polar_endpoint = self.check_for_person()
        if polar_endpoint is None: # No person
            vel.linear.x = 0.0
            vel.angular.z = 0.0

        # There is a person detected by the LiDAR
        else:
            # Rotation to align heading with person
            if polar_endpoint[1] > 0.05:
                vel.angular.z = 0.1 # Person is on the left of the Neato

            elif polar_endpoint[1] < -0.05:
                vel.angular.z = -0.1 # Person is on the right of the Neato

            else:
                vel.angular.z = 0.0 # Set the angular velocity to 0 if the person is approx. straight ahead

            # Linear movement to follow person 
            if polar_endpoint[0] > self.distance_from_person: # Ensure the vehicle doesn't crash into the person. Maintain distance.
                vel.linear.x = 0.15
            else:
                vel.linear.x = 0.0

        # Publish the Twist message to cmd_vel target
        self.publisher.publish(vel)

    def run_loop(self):  
        self.check_for_person() # updates self.person_detect_state
        if self.node_state == "drive_square_node":
            print("I am in drive_square")
            self.run_loop_square()
            if self.person_detect_state is True:
                self.node_state = "person_follower_node"
        
        elif self.node_state == "person_follower_node":
            print("I am in person_follower")
            self.run_loop_person()
            if self.person_detect_state is False:
                self.node_state = "drive_square_node"

def main(args=None):
    """Initialize our node, run it, cleanup on shut down"""
    rclpy.init(args=args)  # Initialize communication with ROS
    node = FiniteStateControl()  # Create our Node
    rclpy.spin(node)  # Run the Node until ready to shutdown
    rclpy.shutdown()  # cleanup

if __name__ == "__main__":
    main()