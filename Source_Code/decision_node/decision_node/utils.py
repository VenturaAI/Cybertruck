"""
============================================================

utils.py

============================================================

This file contains reusable helper functions used by the
decision node.

Nothing here should perform navigation.

Nothing here should contain ROS subscriptions.

Nothing here should know about the state machine logic.

============================================================
"""

import time


class StateLogger:
    """
    Prints compact robot status.

    We intentionally avoid printing every loop because
    the decision node runs at 20 Hz.

    Instead we print once every second.
    """

    def __init__(self, interval):

        self.interval = interval

        self.last_print = 0.0

    def should_print(self):

        now = time.monotonic()

        if (now - self.last_print) >= self.interval:

            self.last_print = now

            return True

        return False

    def print_status(
            self,
            logger,
            state_name,
            environment,
            action):

        if not self.should_print():
            return

        logger.info("------------------------------------------------")

        logger.info(f"STATE : {state_name}")

        logger.info(
            f"US    : "
            f"F={environment.ultrasonic.front:.1f} "
            f"L={environment.ultrasonic.left:.1f} "
            f"R={environment.ultrasonic.right:.1f}"
        )

        logger.info(
            f"LIDAR : "
            f"F={environment.lidar.front:.2f} "
            f"L={environment.lidar.left:.2f} "
            f"R={environment.lidar.right:.2f}"
        )

        logger.info(
            f"CAM   : "
            f"{environment.camera.detected} "
            f"{environment.camera.class_name} "
            f"{environment.camera.confidence:.2f}"
        )

        logger.info(f"ACTION: {action}")

        logger.info("------------------------------------------------")


def change_state(node, new_state):
    """
    Centralized state transition.

    WHY?

    Previously every transition looked like:

        self.state = X
        self.state_start_time = now
        logger.info(...)

    That resulted in duplicated code
    throughout the state machine.

    Now every transition goes through
    one function.
    """

    if node.state == new_state:
        return

    previous = node.state

    node.state = new_state

    node.state_start_time = node.get_clock().now()

    node.get_logger().info(
        f"STATE CHANGE : {previous} -> {new_state}"
    )


def elapsed_seconds(node):
    """
    Returns the time spent
    in the current state.
    """

    return (
        node.get_clock().now()
        - node.state_start_time
    ).nanoseconds / 1e9
