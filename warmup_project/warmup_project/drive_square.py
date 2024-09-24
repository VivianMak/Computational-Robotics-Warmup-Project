"""A node that drives the neato in a 1m by 1m square path."""

import rclpy
from tf_transformations import euler_from_quaternion
from rclpy.node import Node
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi


def convert_pose_to_xy_and_theta(pose):
    """Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple"""
    orientation_tuple = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])


class DriveSquareNode(Node):
    """
    This is a node which drives the robot in a 1m x 1m square path.

    The class should allow the vehicle to move forward and turn clockwise
    until it has completed its square path.

    Publishers Needed:
        - Twist cmd_vel message, which commands vehicle velocity.
    Subscribers Needed:
        - Odometry pose message, which reads the position of the robot.
    """

    def __init__(self):
        """Initializes the class."""
        super().__init__("drive_square_node")  # node names should be unique
        # Create a timer that runs the robot motors
        self.create_timer(0.1, self.run_loop)
        self.body_pos = None
        self.num_turns = 0

        self.pos = [0, 0, 0]
        self.travel_distance = 1.0  # 1 meter travel distance
        self.rotation = pi / 2

        # Initialize the publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)

        # Initialize subscriber for odom
        self.subscription = self.create_subscription(
            Odometry, "odom", self.get_odom, 10
        )

    def get_odom(self, msg):
        """Callback for handling odometry position
        Input:
            msg (Odometry): an Odometry type message from the subscriber.
        """
        self.pos = convert_pose_to_xy_and_theta(msg.pose.pose)

    def run_loop(self):
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
                self.num_turns += 1 # update num_turns
                print("Num turns = ", self.num_turns)
                self.body_pos = self.pos # update body_pos to be the starting point for the next segment

        else:
            # Finished driving in a square
            vel.linear.x = 0.0
            vel.angular.z = 0.0
            quit()

        # Publish the Twist message to cmd_vel target
        self.publisher.publish(vel)


def main(args=None):
    """Initialize our node, run it, cleanup on shut down"""
    rclpy.init(args=args)  # Initialize ROS2 network
    node = DriveSquareNode()  # Create our node
    rclpy.spin(node)  # Run our node
    rclpy.shutdown()  # If interrupted, gracefully shutdown the ROS2 network


if __name__ == "__main__":
    main()
