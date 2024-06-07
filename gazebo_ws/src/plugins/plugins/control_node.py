import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_ = self.create_timer(1.0, self.publish_velocity)

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 10.0  # Set linear velocity to 10 m/s
        msg.linear.z = 0.0  # Set linear velocity to 10 m/s
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
