import rclpy
from rclpy.node import Node
from my_msgs.msg import Humidity
import random

TIMER = 3.0  # segundos
RANDS = 3.0
ADJUSTEMENT = 0.5


class HumidityUpdater(Node):
    def __init__(self, namespace):
        super().__init__(f'humidity_updater_{namespace}')
        self.namespace = namespace
        self.rand = random.uniform(0.01, RANDS)
        self.humidity = random.uniform(80, 100)
        self.timer_period = TIMER
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.iteration_count = 0

        # Publisher
        self.publisher_ = self.create_publisher(Humidity, f'{self.namespace}/humidity_data', 10)

        # Subscriber
        self.subscription = self.create_subscription(
            Humidity,
            f'{self.namespace}/humidity_adjust',
            self.humidity_adjust_callback,
            10
        )

        print(f'Initial value for namespace {self.namespace}:')
        print(f'    Humidity: {self.humidity:.2f}, Random: {self.rand:.2f}')

    def humidity_adjust_callback(self, msg):
        self.humidity = msg.humidity
        self.rand = msg.rand
        self.get_logger().info(
            f'Received adjustment for namespace {self.namespace} - Humidity: {self.humidity:.2f}, Random: {self.rand:.2f}')

    def timer_callback(self):
        self.iteration_count += 1

        # Adjusting self.rand every 10 iterations
        if self.iteration_count % 10 == 0:
            print(f'Adjusting random value for namespace {self.iteration_count}:')
            adjustment = random.uniform(-ADJUSTEMENT, ADJUSTEMENT)
            self.rand += adjustment
            self.rand = max(0.01, min(RANDS, self.rand))
            print(f'    Updated rand for namespace {self.namespace}: {self.rand:.2f} (Adjusted by {adjustment:.2f})')

        # Updating humidity value
        self.humidity = max(0.0, self.humidity - self.rand)
        self.get_logger().info(f'Humidity for namespace {self.namespace}: {self.humidity:.2f}')

        # Publishing the updated values
        msg = Humidity()
        msg.humidity = self.humidity
        msg.rand = self.rand
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    namespaces = [f'pot{i}' for i in range(1, 17)]  # com nomes pot1,2,3,... em namespaces
    nodes = [HumidityUpdater(namespace) for namespace in namespaces]

    try:
        rclpy.spin(nodes[0])
    except KeyboardInterrupt:
        pass

    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
