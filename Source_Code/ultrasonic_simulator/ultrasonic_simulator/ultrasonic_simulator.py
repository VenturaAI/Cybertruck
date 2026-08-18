import rclpy
from rclpy.node import Node

from cybertruck_msgs.msg import Ultrasonic


class UltrasonicSimulator(Node):

    def __init__(self):
        super().__init__('ultrasonic_simulator')

        self.publisher = self.create_publisher(
            Ultrasonic,
            '/ultrasonic',
            10)

        self.timer = self.create_timer(
            5.0,
            self.publish_distances)

        self.state = 0

        self.get_logger().info(
            'Ultrasonic Simulator Started')

    def publish_distances(self):

        msg = Ultrasonic()

        if self.state == 0:
            msg.front = 100.0
            msg.left = 100.0
            msg.right = 100.0

        elif self.state == 1:
            msg.front = 20.0
            msg.left = 120.0
            msg.right = 25.0

        elif self.state == 2:
            msg.front = 20.0
            msg.left = 25.0
            msg.right = 120.0

        else:
            msg.front = 15.0
            msg.left = 15.0
            msg.right = 15.0

        self.publisher.publish(msg)

        self.get_logger().info(
            f'Front={msg.front}  '
            f'Left={msg.left}  '
            f'Right={msg.right}'
        )

        self.state = (self.state + 1) % 4


def main(args=None):
    rclpy.init(args=args)

    node = UltrasonicSimulator()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
