import rclpy
from rclpy.node import Node
import random
import paho.mqtt.client as mqtt
import json

TIMER = 1.0
RANDS = 0.5
SENSOR = "1"


class HumidityUpdater(Node):
    def __init__(self):
        super().__init__('humidity_updater')
        self.humidity = 72
        self.timer_period = TIMER
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        mqtt_broker = "127.0.0.1"
        mqtt_port = 1883
        self.client2 = mqtt.Client()
        self.client2.on_connect = self.on_connect
        self.client2.on_message = self.on_message
        self.client2.connect(mqtt_broker, mqtt_port, 60)

        # Start MQTT client loop in a separate thread
        self.client2.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info(f"Connected to MQTT broker with result code {rc}")
        # Subscribe to MQTT topic for target coordinates
        self.client2.subscribe("Water" + SENSOR)

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            water = payload["water"]

            self.humidity += water

            self.get_logger().info(f"Received water: {water}, Current water: {self.humidity}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding MQTT message payload: {e}")
    def timer_callback(self):
        humidity_loss = random.uniform(0.1, RANDS)

        # Updating humidity values
        self.humidity -= humidity_loss

        mqtt_broker = "127.0.0.1"
        mqtt_port = 1883
        mqtt_topic_command = "UpdateSensor"  + SENSOR
        client = mqtt.Client()
        client.connect(mqtt_broker, mqtt_port, 60)

        data = {
            "humidity": self.humidity,
        }
        msg_mqtt = json.dumps(data)

        client.publish(mqtt_topic_command, msg_mqtt, 0)
        self.get_logger().info(f'Humidity: {self.humidity,}')


def main(args=None):
    rclpy.init(args=args)
    humidity_updater = HumidityUpdater()
    rclpy.spin(humidity_updater)
    humidity_updater.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
