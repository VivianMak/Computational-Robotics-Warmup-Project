"""A node that drives the neato in a 1m by 1m square path."""
import rclpy  
from rclpy.node import Node 
from neato2_interfaces.msg import Bump 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class DriveSquareNode(Node):
    def __init__(self):
        """Initializes the class."""
        super().__init__("drive_square_node") # node names should be unique
        # Create a timer that runs the robot motors
        self.create_timer(0.1, self.run_loop)
        self.position = [0, 0, 0]
        self.lin_vel = [0, 0, 0]
        self.ang_vel = [0, 0, 0]


        # Initialize the publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize subscriber for odom
        self.subscription = self.create_subscription(Odometry, 'odom', self.get_odom, 10)


    def get_odom(self, msg):
        """Callback for handling odometry position and velocity
        Input:
            msg (Odometry): an Odometry type message from the subscriber.
        """
        print(msg)
        self.position = [msg.pose.pose.position.x, msg.position.y, msg.position.z]
        self.lin_vel = [msg.twist.twist.linear.x, msg.linear.y, msg.linear.z]
        self.ang_vel = [msg.twist.twist.angular.x, msg.angular.y, msg.angular.z]



    def run_loop(self):
        """Keeps the robot moving unless a bump is registered."""
        # Create a Twist message to describe the robot motion
        vel = Twist()

        # If the bump sensor is triggered, stop the vehicle
        if self.position[0] == 0.0:
            vel.linear.x = 0.0
        else:
            vel.linear.x = 0.1

        # Publish the Twist message to cmd_vel target
        self.publisher.publish(vel)




def main(args=None):
    """Initialize our node, run it, cleanup on shut down"""
    rclpy.init(args=args)  # Initialize ROS2 network
    node = DriveSquareNode()  # Create our node
    rclpy.spin(node)  # Run our node
    rclpy.shutdown()  # If interrupted, gracefully shutdown the ROS2 network


if __name__ == '__main__':
    main()
