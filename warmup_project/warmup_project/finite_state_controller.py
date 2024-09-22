"""FINITE STATE CONTROL - This script is for publishing and subscribing to ROS msgs in Python."""

import rclpy
from rclpy.node import Node

from warmup_project.drive_square import DriveSquareNode
from warmup_project.wall_follower import wallFollowing


class finiteSateControl(Node):

    # def __init__(self):
    #     """Initializes the class"""
    #     # Create a node
    #     super().__init__("finite_state_controller")

    #     # Create a timer -- runs until the laser data sense that it is close to a wall
    #     self.timer = self.create_timer(0.1, self.run_loop)

    # def run_loop(self):
    #     """Run the combined behavior"""
    #     # print("Running driving in a sqaure node...")
    #     # DriveSquareNode()
    #     print("Running wall follower node...")
    #     wall_follower_node = wallFollowing()  # Create an instance of WallFollowing
    #     rclpy.spin(wall_follower_node)

    def __init__(self):
        """Initializes the class"""
        super().__init__("finite_state_controller")

        # States for FSM
        self.state = "drive_square"
        self.drive_square_done = False  # Track if square driving is done

        # Create a timer -- runs the loop
        self.timer = self.create_timer(0.1, self.run_loop)

        # Instantiate nodes for drive square and wall following, but don't spin them yet
        self.drive_square_node = DriveSquareNode()
        self.wall_follower_node = wallFollowing()

    def run_loop(self):
        """Finite State Control loop to manage transitions between states."""
        if self.state == "drive_square":
            # Run DriveSquareNode behavior
            print("Running DriveSquareNode...")
            self.run_drive_square()
        elif self.state == "wall_following":
            # Run WallFollowing behavior
            print("Running WallFollowing...")
            self.run_wall_follower()

    def run_drive_square(self):
        """Run the driving in a square node logic."""
        if not self.drive_square_done:
            rclpy.spin_once(self.drive_square_node)
            # You'd implement logic in the DriveSquareNode to stop after it completes the square.
            # For example, if DriveSquareNode has a flag or a callback indicating completion.
            self.drive_square_done = self.check_if_square_complete()

        if self.drive_square_done:
            # Transition to wall following
            self.state = "wall_following"

    def run_wall_follower(self):
        """Run the wall follower node logic."""
        rclpy.spin_once(self.wall_follower_node)
        # You can add additional logic if needed, e.g., stopping after some condition.

    def check_if_square_complete(self):
        """Check if driving square is complete. Placeholder for actual logic."""
        # Add your completion logic here, such as checking turn counts or time.
        return True  # For now, assume it's complete after first iteration.


def main(args=None):
    """Initialize our node, run it, cleanup on shut down"""
    rclpy.init(args=args)  # Initialize communication with ROS
    node = finiteSateControl()  # Create our Node
    rclpy.spin(node)  # Run the Node until ready to shutdown
    rclpy.shutdown()  # cleanup


if __name__ == "__main__":
    main()
