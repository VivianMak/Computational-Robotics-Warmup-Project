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
    """
    
    This is a node which controls the robot's movement using keyboard input.
    
    The class should allow the vehicle to move forward, backward, and rotate
    both clockwise and counterclockwise.
    
    Publishers Needed:
        - Twist cmd_vel message, which commands vehicle velocity.
    """

    def __init__(self):
        """Initializes the class."""
        super().__init__("teleop_node") # node names should be unique
        # Create a timer that runs the robot motors
        self.create_timer(0.1, self.run_loop)
        # Define settings for getKey()
        self.settings = termios.tcgetattr(sys.stdin)
        # Initialize the publisher for cmd_vel
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
    
    def getKey(self):
        """
        Tracks keyboard input.

        Returns
            String key that represents the user input.
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_loop(self):
        """Controls the movement of the robot through teleoperation."""

        key = self.getKey()
        print(key)
        
    
        # Create a Twist message to describe the robot motion.
        vel = Twist()
        # If w key is pressed, go forward.
        if key == "w":
            vel.linear.x = 0.2
            vel.angular.z = 0.0
        # If s key is pressed, go backward.
        elif key == "s":
            vel.linear.x = -0.2
            vel.angular.z = 0.0
        # If a key is pressed, turn to counterclockwise.
        elif key == "a":
            vel.linear.x = 0.0
            vel.angular.z = 1.0
        # If d key is pressed, turn clockwise.
        elif key == "d":
            vel.linear.x = 0.0
            vel.angular.z = -1.0
        # If Ctrl+C is pressed, terminate the node.
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