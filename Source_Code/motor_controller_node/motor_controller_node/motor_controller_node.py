
import rclpy
from rclpy.node import Node

from cybertruck_msgs.msg import VehicleCommand

from .rpc_motor_driver import RPCMotorDriver


class MotorControllerNode(Node):

    def __init__(self):
        super().__init__("motor_controller_node")

        self.driver = RPCMotorDriver()

        self.subscription = self.create_subscription(
            VehicleCommand,
            "/vehicle_command",
            self.command_callback,
            1
        )

        self.get_logger().info(
            "Motor Controller Node Started (RPC Mode)"
        )

    def command_callback(self, msg):

        try:

            self.driver.drive(
                msg.speed,
                msg.steering,
                msg.brake
            )

            self.get_logger().info(
                f"speed={msg.speed:.2f}  "
                f"steering={msg.steering:.2f}  "
                f"brake={msg.brake}"
            )

        except Exception as e:

            self.get_logger().error(
                f"RPC Error : {e}"
            )

    def destroy_node(self):

        try:

            self.driver.emergency_stop()

        except Exception:

            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = MotorControllerNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
