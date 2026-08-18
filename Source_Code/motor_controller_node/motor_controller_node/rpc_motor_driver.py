from .bridge import Bridge


class RPCMotorDriver:

    def __init__(self):
        # Verify communication with STM32
        Bridge.call("cybertruck.heartbeat")

    def drive(self, speed: float, steering: float, brake: bool):

        return Bridge.call(
            "cybertruck.drive",
            float(speed),
            float(steering),
            bool(brake),
            timeout=0.15
        )

    def emergency_stop(self):

        return Bridge.call(
            "cybertruck.estop"
        )
