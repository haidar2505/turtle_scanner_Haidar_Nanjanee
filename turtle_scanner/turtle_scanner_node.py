import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class TurtleScannerNode(Node):
    def __init__(self):
        super().__init__('turtle_scanner_node')

        self.pose_scanner = None
        self.pose_target = None

        self.sub_scanner = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.callback_scanner,
            10
        )

        self.sub_target = self.create_subscription(
            Pose,
            '/turtle_target/pose',
            self.callback_target,
            10
        )

        self.get_logger().info('TurtleScannerNode démarré !')

    def callback_scanner(self, msg):
        self.pose_scanner = msg
        self.get_logger().info(
            f'Scanner -> x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}'
        )

    def callback_target(self, msg):
        self.pose_target = msg
        self.get_logger().info(
            f'Target  -> x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = TurtleScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()