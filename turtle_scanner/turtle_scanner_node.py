import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

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

        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.nb_lignes = 5
        self.y_start = 1.0
        self.y_step = 2.0
        self.x_min = 1.0
        self.x_max = 10.0

        self.Kp_ang = 4.0
        self.Kp_lin = 1.0
        self.linear_speed_max = 2.0
        self.waypoint_tolerance = 0.3

        self.waypoints = self.generate_serpentine_waypoints()
        self.current_index = 0

        self.timer = self.create_timer(0.05, self.scan_step)

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

    def generate_serpentine_waypoints(self):
        waypoints = []

        for i in range(self.nb_lignes):
            y = self.y_start + i * self.y_step

            if i % 2 == 0:
                waypoints.append((self.x_min, y))
                waypoints.append((self.x_max, y))
            else:
                waypoints.append((self.x_max, y))
                waypoints.append((self.x_min, y))

        return waypoints

    def compute_angle(self, A, B):
        return math.atan2(B[1] - A[1], B[0] - A[0])

    def compute_distance(self, A, B):
        return math.sqrt((B[0] - A[0])**2 + (B[1] - A[1])**2)

    def scan_step(self):

        # wait until pose is received
        if self.pose_scanner is None:
            return

        # end condition
        if self.current_index >= len(self.waypoints):
            self.stop_turtle()
            self.get_logger().info("Balayage terminé")
            return

        current_pos = (self.pose_scanner.x, self.pose_scanner.y)

        target = self.waypoints[self.current_index]

        # distance
        distance = self.compute_distance(current_pos, target)

        # waypoint reached
        if distance < self.waypoint_tolerance:
            self.current_index += 1
            return

        # desired angle
        theta_desired = self.compute_angle(current_pos, target)
        theta = self.pose_scanner.theta

        # angular error (normalized)
        error = math.atan(math.tan((theta_desired - theta) / 2))

        # proportional control
        angular_z = self.Kp_ang * error
        linear_x = self.Kp_lin * distance

        # speed limit
        linear_x = min(linear_x, self.linear_speed_max)

        # publish command
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z

        self.publisher.publish(cmd)

    def stop_turtle(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()