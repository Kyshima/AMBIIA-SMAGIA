import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher_ = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/robot1/odom', self.odom_callback, 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/laser/out',
            self.pointcloud_callback,
            10
        )
        self.target_x = 2.5  # Target x position
        self.target_y = -4.0  # Target y position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.state = 'rotate'
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def timer_callback(self):
        msg = Twist()
        if self.state == 'rotate':
            target_angle = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
            angle_diff = target_angle - self.current_yaw
            if abs(angle_diff) > 0.1:
                msg.angular.z = 0.3 if angle_diff > 0 else -0.3
            else:
                msg.angular.z = 0.0
                self.state = 'move'

        elif self.state == 'move':
            # Calculate distance projection onto forward direction
            distance_x = (self.target_x - self.current_x) * math.cos(self.current_yaw)
            distance_y = (self.target_y - self.current_y) * math.sin(self.current_yaw)
            distance = math.sqrt(distance_x ** 2 + distance_y ** 2)
            
            if distance > 0.3:
                msg.linear.x = 0.4
            else:
                msg.linear.x = 0.0
                self.state = 'stop'

        self.publisher_.publish(msg)
        self.get_logger().info(f'Current state: {self.state}, Current position: ({self.current_x}, {self.current_y}), Velocity: ({msg.linear.x})')

    def pointcloud_callback(self, msg):
        msg1 = Twist()
        points = []
        for point in pc2.read_points(msg, skip_nans=True):
            angle = (math.atan2(point[1], point[0]))*180/math.pi
            dist = math.sqrt( (point[0]*point[0]) + (point[1]*point[1]) )

            if dist < 1.0:
                msg1.linear.x = 0.0
                self.state = 'stop'
                self.publisher_.publish(msg1)
                print(f'Received a point with an angle of {angle} and distance {dist}')

def main(args=None):
    rclpy.init(args=args)
    robot_mover = RobotMover()
    rclpy.spin(robot_mover)
    robot_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
