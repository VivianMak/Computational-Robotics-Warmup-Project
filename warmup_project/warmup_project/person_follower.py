import rclpy  
from tf_transformations import euler_from_quaternion
from rclpy.node import Node 
from neato2_interfaces.msg import Bump 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math, time

def PI_control(dir, error, e_integral):

    if dir == "linear":
        dt = 0.6    
        Kp = 1.0
        Ki = 0.001
        e_integral = e_integral * dt

        speed = dt * (Kp * error) + Ki * e_integral
        if speed > 0.25:
            speed = 0.25
    elif dir == "angular":
        dt = 0.4    
        Kp = 1.0
        Ki = 0.001
        e_integral = e_integral * dt
        speed = dt * (Kp * error) + Ki * e_integral
        if speed > 0.35:
            speed = 0.35
    
    return speed


class PersonFollowerNode(Node):
    """
    This is a node which controls the robot to follow a person around.
    
    The class should allow the vehicle to track the location of the person
    using its LiDAR sensor and re-orient itself to move towards the person
    and maintain a distance between it using a Proportional Controller.
    
    Publishers Needed:
        - Twist cmd_vel message, which commands vehicle velocity.
    Subscribers Needed:
        - Odometry pose message, which reads the position of the robot.
    """
    def __init__(self):
        """Initializes the class."""
        super().__init__("person_follower_node") # node names should be unique
        # Create a timer that runs the robot motors
        self.create_timer(0.1, self.run_loop)
        self.degree_of_view = 15
        self.max_range = 1.5 # The max distance we limit the Neato's IR to see
        self.r_list = None
        self.distance_from_person = 0.6

        self.dt = 0.3
        self.kP = 1.0
        self.e_integral = 0



        # Initialize the publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # Initialize subscriber for LiDAR scan
        self.sub = self.create_subscription(LaserScan, "scan", self.process_scan, 10)

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

    def run_loop(self):
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
                if r != float('inf') and r <= self.max_range:
                    person_points[idx] = r # adds index of person_points in case there is a gap the LiDAR scan (i.e 11, 12, 14, 15)
                    sum_person_r += r
                    sum_person_angle_idx += idx

            if len(person_points) == 0: # prevent divide by 0 error
                vel.linear.x = 0.0
                vel.angular.z = 0.0
            else:
                # Estimate that the person is at the average of the distances and angles taken from the LiDAR scan
                average_r = sum_person_r / len(person_points)
                average_person_angle_idx = sum_person_angle_idx / len(person_points) - self.degree_of_view # to correct for idx starting at 0 instead of -15
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


def main(args=None):
    """Initialize our node, run it, cleanup on shut down"""
    rclpy.init(args=args)  # Initialize ROS2 network
    node = PersonFollowerNode()  # Create our node
    rclpy.spin(node)  # Run our node
    rclpy.shutdown()  # If interrupted, gracefully shutdown the ROS2 network


if __name__ == '__main__':
    main()
