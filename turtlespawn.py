import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random
import time

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        self.client = self.create_client(Spawn, 'spawn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.timer = self.create_timer(1.0, self.spawn_turtle)

    def spawn_turtle(self):
        request = Spawn.Request()
        request.x = random.uniform(0.0, 11.0)
        request.y = random.uniform(0.0, 11.0)
        request.theta = random.uniform(0.0, 2 * 3.14159)
        request.name = f'turtle{random.randint(0, 1000)}'
        self.client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    turtle_spawner = TurtleSpawner()
    rclpy.spin(turtle_spawner)
    turtle_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
