import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random

class SpawnTarget(Node):
    def __init__(self):
        super().__init__('spawn_target')
        
        self.client = self.create_client(Spawn, '/spawn')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('En attente du service /spawn...')
        
        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)
        theta = random.uniform(0.0, 6.28)
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = 'turtle_target'
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(
                f'turtle_target spawnée en x={x:.2f}, y={y:.2f}, theta={theta:.2f}'
            )
        else:
            self.get_logger().error('Échec du spawn !')

def main(args=None):
    rclpy.init(args=args)
    node = SpawnTarget()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()