import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import math
import sys
import paho.mqtt.client as mqtt
import json

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover4')
        self.publisher_ = self.create_publisher(Twist, '/robot4/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/robot4/odom', self.odom_callback, 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            '/laser4/out',
            self.pointcloud_callback,
            10
        )
        self.target_x = -8.75  # Target x position
        self.target_y = -4.56 # Target y position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.state = 'rotate'
        self.obstacle_found = False
        self.timer_period = 0.5  # seconds
        self.energy_waste = 0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.former_state = 'rotate'

        # MQTT configuration
        mqtt_broker = "127.0.0.1"
        mqtt_port = 1883
        self.mqtt_topic_command = "Coordinates4"
        self.client = mqtt.Client()
        self.client.connect(mqtt_broker, mqtt_port, 60)
        
        # MQTT configuration
        mqtt_broker = "127.0.0.1"
        mqtt_port = 1883
        self.mqtt_topic_command2 = "target_coordinates4"
        self.client2 = mqtt.Client()
        self.client2.on_connect = self.on_connect
        self.client2.on_message = self.on_message
        self.client2.connect(mqtt_broker, mqtt_port, 60)

        # Start MQTT client loop in a separate thread
        self.client2.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"Connected to MQTT broker with result code {rc}")
        # Subscribe to MQTT topic for target coordinates
        self.client2.subscribe(self.mqtt_topic_command2)

    def on_message(self, client, userdata, msg):
        self.get_logger().info("Received")
        try:
            payload = json.loads(msg.payload.decode())
            self.target_x = payload["x"]
            self.target_y = payload["y"]
            self.get_logger().info(f"Received new target coordinates: ({self.target_x}, {self.target_y})")
            self.state = 'move'
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding MQTT message payload: {e}")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def timer_callback(self):
        msg = Twist()
        if not self.obstacle_found:
            target_angle = math.atan2(self.target_y - self.current_y, self.target_x - self.current_x)
            angle_diff = target_angle - self.current_yaw
            self.get_logger().info(f'State: {self.state}')
            if angle_diff > math.pi:
                angle_diff = angle_diff - math.pi*2
            elif angle_diff < -math.pi:
                angle_diff = angle_diff + math.pi*2

            if abs(angle_diff) > 0.4:
                self.former_state = 'rotate'
                msg.angular.z = 0.3 if angle_diff > 0 else -0.3
                self.energy_waste = self.energy_waste + 0.5
            else:
                msg.angular.z = 0.0
                self.state = 'move'

            if self.state == 'move':
                self.former_state = 'move'
                if not self.obstacle_found:
                    if abs(angle_diff) > 0.6:
                        self.state = 'rotate'
                # Calculate distance projection onto forward direction
                distance_x = (self.target_x - self.current_x) * math.cos(self.current_yaw)
                distance_y = (self.target_y - self.current_y) * math.sin(self.current_yaw)
                distance = math.sqrt(distance_x ** 2 + distance_y ** 2)
                self.get_logger().info(f'Distance: {distance}, Distance X: {distance_x}, Distance Y: {distance_y}')
                #if distance > 0.6:
                if abs(self.target_x - self.current_x) > 0.4 or abs(self.target_y - self.current_y) > 0.4:
                    if distance_x > 0.3 or distance_y > 0.3:
                        msg.linear.x = 0.4
                        self.energy_waste = self.energy_waste + 1.5
                else:
                    msg.linear.x = 0.0
                    self.state = 'stop'
            elif self.state == 'stop' and abs(self.current_x - 9.14) > 0.3 and abs(self.current_y - 3.94) > 0.3 :
                if abs(angle_diff) > 0.1:
                    msg.angular.z = 0.3 if angle_diff > 0 else -0.3
                else:
                    if abs(self.target_x - self.current_x) > 0.3 or abs(self.target_y - self.current_y) > 0.3:
                        msg.linear.x = 0.4
                    else:
                        msg.linear.x = 0.0
                self.get_logger().info(f'diff angle: {angle_diff}')

        data = {
            "availability": self.state,
            "x": self.current_x,
            "y": self.current_y,
            "energy_waste": self.energy_waste
        }
        msgMqtt = json.dumps(data)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Current state: {self.state}, Current position: ({self.current_x}, {self.current_y}), Velocity: ({msg.linear.x}), goal: ({self.target_x}, {self.target_y})')
        self.client.publish(self.mqtt_topic_command, msgMqtt,0)
        self.energy_waste = 0

    def pointcloud_callback(self, msg):
        self.obstacle_found = False
        msg1 = Twist()
        if self.state != 'stop':
            for point in pc2.read_points(msg, skip_nans=True):
                angle = (math.atan2(point[1], point[0]))*180/math.pi
                dist = math.sqrt( (point[0]*point[0]) + (point[1]*point[1]) )

                if angle > 180:
                    angle = angle - 360
                elif angle < -180:
                    angle = angle + 360

                if dist < 0.8:
                    if angle <= 0 and angle > -90:
                        self.obstacle_found = True
                        msg1.angular.z = 0.3
                        msg1.linear.x = 0.1
                    elif angle > 0 and angle < 90:
                        self.obstacle_found = True
                        msg1.angular.z = -0.3
                        msg1.linear.x = 0.1
                    print(f'Received a point with an angle of {angle} and distance {dist} and state {self.state}')
                    break

            if self.obstacle_found == True:
                self.publisher_.publish(msg1)
        
def main(args=None):
    rclpy.init(args=args)
    robot_mover = RobotMover()
    rclpy.spin(robot_mover)
    robot_mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
