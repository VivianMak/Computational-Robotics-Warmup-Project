"""A node that controls the vehicle to identify and autonomously follow a person around."""

import rclpy  
from rclpy.node import Node 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PersonFollowerNode(Node):
    """
    This is a node which controls the robot to follow a person around.
    
    The class should allow the vehicle to track the location of the person
    using its LiDAR sensor and re-orient itself to move towards the person
    and maintain a distance between it using a Proportional Controller.
    
    Publishers Needed:
        - Twist cmd_vel message, which commands vehicle velocity.
    Subscribers Needed:
        - LaserScan scan message, which reads the distance and angle of objects
          around the vehicle.
    """
    def __init__(self):
        """Initializes the class."""
        super().__init__("person_follower_node") # node names should be unique
        # Create a timer that runs the robot motors
        self.create_timer(0.1, self.run_loop)
        # Initialize how wide the person-following zone is for the Neato
        self.degree_of_view = 15
        # Initialize the max distance we limit for the person-follower range. Reduces noise.
        self.max_range = 1.5 
        # Initialize the distance to stop before reaching the person. Includes buffer for
        # imprecise person location. 
        self.distance_from_person = 0.6
        # Initialize variable for the list of distances in the cone of view
        self.r_list = None
        # Initialize the publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # Initialize subscriber for LiDAR scan
        self.sub = self.create_subscription(LaserScan, "scan", self.process_scan, 10)

    def process_scan(self, msg):
        """
        Callback for handling the LiDAR scan points in the degree of view.

        Args: msg (Laserscan): a Laserscan type message from the subscriber
        """
        r_list = [] 
        # Loop through each index scans, each index representing a degree from 0-360
        for idx in range(-self.degree_of_view, self.degree_of_view):
            dist = msg.ranges[idx]
            r_list.append(dist)
        self.r_list = r_list 

    def run_loop(self):
        """Tracks the relative position of the person and follows them."""

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
                if r != float('inf') and r <= self.max_range:
                    person_points[idx] = r # includes index of person_points in case there is a gap the LiDAR scan (i.e 11, 12, 14, 15)
                    sum_person_r += r
                    sum_person_angle_idx += idx

            if len(person_points) == 0: # prevent divide by 0 error
                vel.linear.x = 0.0
                vel.angular.z = 0.0
            else:
                # Estimate that the person is at the average of the distances and angles taken from the LiDAR scan
                average_r = sum_person_r / len(person_points)
                # Calculate angle that the person is located from the vehicle's heading. Subtract to correct 
                # for idx starting at 0 instead of -self.degree_of_view
                average_person_angle_idx = sum_person_angle_idx / len(person_points) - self.degree_of_view 
                # The person is the endpoint, represented in polar coordinates (r,theta)
                polar_endpoint = (average_r, average_person_angle_idx)
                print(f"Polar endpoint: {polar_endpoint}")

                # Rotation to align heading with person
                if polar_endpoint[1] > 0.05: # Person is on the left of the Neato
                    vel.angular.z = 0.15
                elif polar_endpoint[1] < -0.05: # Person is on the right of the Neato
                    vel.angular.z = -0.15
                else:
                    vel.angular.z = 0.0 # Set the angular velocity to 0 if the person is approx. straight ahead
                # Linear movement to follow person 
                if polar_endpoint[0] > self.distance_from_person: # Ensure the vehicle doesn't crash into the person. Maintain a distance.
                    vel.linear.x = 0.1
                else:
                    vel.linear.x = 0.0
        # Publish the Twist message to cmd_vel target
        self.publisher.publish(vel)


def main(args=None):
    """Initialize our node, run it, cleanup on shut down"""
    rclpy.init(args=args)  # Initialize ROS2 network
    node = PersonFollowerNode()  # Create our node
    rclpy.spin(node)  # Run our node
    rclpy.shutdown()  # If interrupted, gracefully shutdown the ROS2 network


if __name__ == '__main__':
    main()
