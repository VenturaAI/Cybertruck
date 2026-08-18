import rclpy
from rclpy.node import Node

from cybertruck_msgs.msg import Ultrasonic

from ultrasonic_node.ultrasonic_driver import UltrasonicDriver


class UltrasonicNode(Node):

    def __init__(self):

        super().__init__('ultrasonic_node')

        self.driver = UltrasonicDriver()

        self.publisher = self.create_publisher(
            Ultrasonic,
            '/ultrasonic',
            10
        )

        self.timer = self.create_timer(
            0.2,
            self.publish_distances
        )

        self.get_logger().info(
            'Ultrasonic Node Started'
        )

    def publish_distances(self):

        try:
            front, left, right = self.driver.read_all()
        
        except Exception as e:
            self.get_logger().warn(
                f'Ultrasonic read failed, skipping cycle: {e}'
            )
            return


        msg = Ultrasonic()

        msg.front = front
        msg.left = left
        msg.right = right

        # msg.front = self.driver.read_front()
        # msg.left = self.driver.read_left()
        # msg.right = self.driver.read_right()

        self.publisher.publish(msg)

        self.get_logger().info(
            f'Front={msg.front:.1f} '
            f'Left={msg.left:.1f} '
            f'Right={msg.right:.1f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = UltrasonicNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
