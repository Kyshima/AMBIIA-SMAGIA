import rclpy
from rclpy.node import Node
import random

TIMER = 3.0  # segundos
RANDS = 3.0
ADJUSTEMENT = 0.5

class HumidityUpdater(Node):
    def __init__(self):
        super().__init__('humidity_updater')
        self.rand = random.uniform(0.01, RANDS)
        self.humidity = random.uniform(80, 100)
        self.timer_period = TIMER
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.iteration_count = 0

        print('Initial value:')
        print(f'    Humidity: {self.humidity:.2f}, Random: {self.rand:.2f}')

    def timer_callback(self):
        self.iteration_count += 1

        # Adjusting self.rand every 10 iterations
        if self.iteration_count % 10 == 0:
            print(f'Adjusting random value {self.iteration_count}:')
            adjustment = random.uniform(-ADJUSTEMENT, ADJUSTEMENT)
            self.rand += adjustment
            self.rand = max(0.01, min(RANDS, self.rand))
            print(f'    Updated rand: {self.rand:.2f} (Adjusted by {adjustment:.2f})')

        # Updating humidity value
        self.humidity = max(0.0, self.humidity - self.rand)
        self.get_logger().info(f'Humidity: {self.humidity:.2f}')

def main(args=None):
    rclpy.init(args=args)
    humidity_updater = HumidityUpdater()
    rclpy.spin(humidity_updater)
    humidity_updater.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()