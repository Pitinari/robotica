import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquareWalkerNode(Node):
    def __init__(self):
        super().__init__('square_walker_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Square Walker Node has been started.')
        self.run_square()

    def run_square(self):
        # Define the Twist messages for moving, turning, and stopping
        move_cmd = Twist()
        move_cmd.linear.x = 0.26

        turn_cmd = Twist()
        turn_cmd.angular.z = 0.785 # 90 degrees in 2 seconds (pi/2 radians)

        stop_cmd = Twist()
        
        self.publisher_.publish(stop_cmd)
        time.sleep(1.0) # Give it a moment to stop

        # Loop 4 times to make a square
        for i in range(4):
            # Move forward
            self.get_logger().info(f'Moving forward for side {i+1}...')
            self.publisher_.publish(move_cmd)
            time.sleep(7.69)

            # Turn
            self.get_logger().info(f'Turning for corner {i+1}...')
            self.publisher_.publish(turn_cmd)
            time.sleep(2.0)

        # Stop the robot after the square is complete
        self.get_logger().info('Square complete. Stopping robot.')
        self.publisher_.publish(stop_cmd)
        time.sleep(1.0) # Give it a moment to stop

def main(args=None):
    rclpy.init(args=args)
    node = SquareWalkerNode()
    # The run_square logic is called in the constructor, so we just need to keep the node alive briefly
    # In a more complex node, you'd use rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()