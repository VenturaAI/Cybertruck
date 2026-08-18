import rclpy
from rclpy.node import Node

from cybertruck_msgs.msg import VehicleCommand

from motor_controller_node.motor_driver import MotorDriver


class MotorControllerNode(Node):

    def __init__(self):
        super().__init__('motor_controller_node')

        self.driver = MotorDriver()

        self.subscription = self.create_subscription(
            VehicleCommand,
            '/vehicle_command',
            self.command_callback,
            10
        )

        self.get_logger().info(
            'Motor Controller Node Started'
        )

    def command_callback(self, msg):

        if msg.brake:

            self.get_logger().info(
                'EMERGENCY STOP'
            )

            self.driver.emergency_stop()

            return

        pwm = int(abs(msg.speed) * 255)

        if msg.speed > 0:

            self.driver.drive_forward(pwm)

            self.get_logger().info(
                f'DRIVE FORWARD PWM={pwm}'
            )

        elif msg.speed < 0:

            self.driver.drive_reverse(pwm)

            self.get_logger().info(
                f'DRIVE REVERSE PWM={pwm}'
            )

        else:

            self.driver.stop_drive()

            self.get_logger().info(
                'DRIVE STOP'
            )

        if abs(msg.steering) < 0.1:

            self.driver.stop_steering()

        else:

            if abs(msg.steering) < 0.3:
                pulse = 100

            elif abs(msg.steering) < 0.7:
                pulse = 200

            else:
                pulse = 300

            if msg.steering < 0:

                self.driver.steer_left(pulse)

                self.get_logger().info(
                    f'STEER LEFT {pulse} ms'
                )

            else:

                self.driver.steer_right(pulse)

                self.get_logger().info(
                    f'STEER RIGHT {pulse} ms'
                )


def main(args=None):

    rclpy.init(args=args)

    node = MotorControllerNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()