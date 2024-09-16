"""
A teleoperation node that allows user input to control the Neato's 
direction.
"""
import tty
import select
import sys
import termios
import rclpy  
from rclpy.node import Node 
from neato2_interfaces.msg import Bump 
from geometry_msgs.msg import Twist


class TeleopNode(Node):


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    """
    
    This is a node which stops the motors when the bump sensor is triggered.
    
    The class should allow the vehicle to move forward at a slow rate, until the
    bump sensor is triggered, in which case the robot should stop.
    
    Publishers Needed:
        - Twist cmd_vel message; which commands vehicle velocity
    Subscribers Needed:
        - Bump bump message handling; which listens for the bump sensor data



    
    """
    def __init__(self):
        """Initializes the class."""
        super().__init__("teleop_node") # node names should be unique
        # Create a timer that runs the robot motors
        self.create_timer(0.1, self.run_loop)
        self.settings = termios.tcgetattr(sys.stdin)

        '''
        Create a publisher for the motors.
        Method create_publisher takes a message type, the message topic name, and a filter queue.
        A publisher will not publish anything once initialized, it must be called.
        '''
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def run_loop(self):
        """Keeps the robot moving unless a bump is registered."""

        key = self.getKey()
        print(key)
        
    
        # Create a Twist message to describe the robot motion
        vel = Twist()
        
        if key == "w":
            vel.linear.x = 0.2
            vel.angular.z = 0.0
        elif key == "s":
            vel.linear.x = -0.2
            vel.angular.z = 0.0
        elif key == "a":
            vel.linear.x = 0.0
            vel.angular.z = 1.0
        elif key == "d":
            vel.linear.x = 0.0
            vel.angular.z = -1.0
        elif key == "x":
            vel.linear.x = 0.0
            vel.angular.z = 0.0
        elif key == "\x03":
            quit()
    
        # Publish the Twist message to cmd_vel target
        self.publisher.publish(vel)



    
def main(args=None):
    """Initialize our node, run it, cleanup on shut down"""
    rclpy.init(args=args)  # Initialize ROS2 network
    node = TeleopNode()  # Create our node
    rclpy.spin(node)  # Run our node
    rclpy.shutdown()  # If interrupted, gracefully shutdown the ROS2 network

if __name__ == '__main__':
    main()