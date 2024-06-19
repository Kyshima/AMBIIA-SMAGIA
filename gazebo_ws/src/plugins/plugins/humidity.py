import rclpy
from rclpy.node import Node
import random

TIMER = 3.0 #segundos
RANDS = 3.0
ADJUSTEMENT = 0.5

class HumidityUpdater(Node):
    def __init__(self):
        super().__init__('humidity_updater')
        self.rands = [random.uniform(0.01, RANDS) for _ in range(16)]
        self.humidity = [random.uniform(80, 100) for _ in range(16)] #[100.0 for _ in range(16)]
        self.timer_period = TIMER
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.iteration_count = 0

        print('Initial values:')
        for i in range(len(self.rands)):
            print(f'    Humidity[{i}]: {self.humidity[i]:.2f}, Random[{i}]: {self.rands[i]:.2f}')
            #print(f'    Random[{i}]: {self.rands[i]:.2f}')

    def timer_callback(self):
        self.iteration_count += 1

        # Adjusting self.rands every 10 iterations
        if self.iteration_count % 10 == 0:
            print(f'Adjusting random values {self.iteration_count}:')
            for i in range(len(self.rands)):
                adjustment = random.uniform(-ADJUSTEMENT, ADJUSTEMENT)
                self.rands[i] += adjustment
                self.rands[i] = max(0.01, min(RANDS, self.rands[i]))
                print(f'    Updated rands[{i}]: {self.rands[i]:.2f} (Adjusted by {adjustment:.2f})')

        # Updating humidity values
        self.humidity = [max(0.0, h - rand) for h, rand in zip(self.humidity, self.rands)]
        humidity_str = ', '.join([f'{h:.2f}' for h in self.humidity])
        self.get_logger().info(f'Humidity: [{humidity_str}]')


def main(args=None):
    rclpy.init(args=args)
    humidity_updater = HumidityUpdater()
    rclpy.spin(humidity_updater)
    humidity_updater.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
