import rclpy  
from tf_transformations import euler_from_quaternion
from rclpy.node import Node 
from neato2_interfaces.msg import Bump 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math, time

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

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
        self.body_pos = None
        self.pos = [0, 0, 0]
        self.degree_of_view = 15
        self.max_range = 2.0 # The max distance we limit the Neato's IR to see
        self.r_list = None

        self.travel_distance = 1.0
        self.rotation = math.pi/2
        self.e_integral = 0

        # Initialize the publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize subscriber for odom
        self.subscription = self.create_subscription(Odometry, 'odom', self.get_odom, 10)
        # Initialize subscriber for LiDAR scan
        self.sub = self.create_subscription(LaserScan, "scan", self.process_scan, 10)

    def process_scan(self, msg):
        """
        Callback for handling a wall detector sensor input.

        Args:

        """
        # r_list = {}
        r_list = []
        # Initialize variables
        for idx in range(-self.degree_of_view, self.degree_of_view):
            dist = msg.ranges[idx]
            # r_list[idx] = point
            r_list.append(dist)
        # print(r_list)
        

        self.r_list = r_list

    def get_odom(self, msg):
        """Callback for handling odometry position
        Input:
            msg (Odometry): an Odometry type message from the subscriber.
        """
        self.pos = convert_pose_to_xy_and_theta(msg.pose.pose)

    def run_loop(self):
        """Drives the robot in a square."""
        if self.body_pos is None:
            self.body_pos = self.pos
        # Create a Twist message to describe the robot motion
        vel = Twist()
        # Define relative position as the delta of pos and body frame
        rel_pos = [a - b for a, b in zip(self.pos, self.body_pos)]
        # print("Relative Position: ", rel_pos)
        error = 1 - abs(rel_pos[0])
        # print("Error: ", error)
 
        sum_person_r = 0.0
        sum_person_angle_idx = 0
        person_points = {}
        if self.r_list is not None:
            for idx, r in enumerate(self.r_list):
                if r != float('inf') and r <= self.max_range:
                    person_points[idx] = r
                    sum_person_r += r
                    sum_person_angle_idx += idx

            average_r = sum_person_r / len(person_points)
            average_person_angle_idx = math.ceil(sum_person_angle_idx / len(person_points)) - self.degree_of_view # to correct for idx starting at 0 instead of -15
            # print(person_points)
            polar_endpoint = (average_r, average_person_angle_idx)
            print(f"Polar endpoint: {polar_endpoint}")
            
            print(f"delta angle {rel_pos[2] - polar_endpoint[1]}")

            

            # if polar_endpoint[1] > 0.05:
                
            #     # vel.angular.z = PI_control("angular", error, self.e_integral)
            #     # vel.linear.x = PI_control("linear", error, self.e_integral)

            #     vel.angular.z = -0.1

            # elif polar_endpoint[1] < -0.05:
            #     vel.angular.z = 0.1


            # else:
            #     vel.linear.x = 0.0
            #     vel.angular.z = 0.0

        # Publish the Twist message to cmd_vel target
        self.publisher.publish(vel)




        """
        # Set the starting position
        if self.body_pos is None:
            self.body_pos = self.pos
        # Create a Twist message to describe the robot motion
        vel = Twist()
        # Define relative position as the delta of pos and body frame
        rel_pos = [a - b for a, b in zip(self.pos, self.body_pos)]
        print("Relative Position: ", rel_pos)
        error = 1 - abs(rel_pos[0])
        self.e_integral = self.e_integral + error
        print("Error: ", error)

        # Repeat the process of going straight and turning to the right
        if self.num_turns < 4:
            if abs(rel_pos[0]) - self.travel_distance < 0.001 and abs(rel_pos[1]) - self.travel_distance < 0.001:
                vel.linear.x = PI_control(error, self.e_integral)

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
        """



def main(args=None):
    """Initialize our node, run it, cleanup on shut down"""
    rclpy.init(args=args)  # Initialize ROS2 network
    node = PersonFollowerNode()  # Create our node
    rclpy.spin(node)  # Run our node
    rclpy.shutdown()  # If interrupted, gracefully shutdown the ROS2 network


if __name__ == '__main__':
    main()
