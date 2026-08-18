#!/usr/bin/env python3

"""
decision_node.py

Main ROS2 interface for autonomous navigation.

Responsibilities:
    - Receive sensor data
    - Update Environment
    - Execute NavigationStateMachine
    - Publish vehicle commands
"""

import math
#import statistics

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from cybertruck_msgs.msg import (
    Obstacle,
    Ultrasonic,
    VehicleCommand,
)

from .environment import Environment
from .navigation_parameters import NavigationParameters
from .state_machine import NavigationStateMachine


class DecisionNode(Node):

    def __init__(self):

        super().__init__("decision_node")

        #
        # Parameters
        #

        self.params = NavigationParameters()

        #
        # Shared environment
        #

        self.environment = Environment()

        #
        # Navigation state machine
        #

        self.state_machine = NavigationStateMachine(
            self,
            self.environment,
        )

        #
        # Ultrasonic moving-average buffers
        #

        self.front_history = []

        self.left_history = []

        self.right_history = []

        #
        # Publisher
        #

        self.publisher = self.create_publisher(
            VehicleCommand,
            "/vehicle_command",
            10,
        )

        #
        # Subscribers
        #

        self.ultrasonic_subscription = self.create_subscription(
            Ultrasonic,
            "/ultrasonic",
            self.ultrasonic_callback,
            10,
        )

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_callback,
            10,
        )

        self.obstacle_subscription = self.create_subscription(
            Obstacle,
            "/obstacle",
            self.obstacle_callback,
            10,
        )

        #
        # Navigation timer
        #

        self.timer = self.create_timer(
            0.05,
            self.timer_callback,
        )

        self.get_logger().info(
            "Decision Node Started"
        )

    # =======================================================
    # Ultrasonic Callback
    # =======================================================


    def ultrasonic_callback(self, msg):

        self.front_history.append(msg.front)
        self.left_history.append(msg.left)
        self.right_history.append(msg.right)

        if len(self.front_history) > self.params.ULTRASONIC_FILTER_SIZE:
            self.front_history.pop(0)

        if len(self.left_history) > self.params.ULTRASONIC_FILTER_SIZE:
            self.left_history.pop(0)

        if len(self.right_history) > self.params.ULTRASONIC_FILTER_SIZE:
            self.right_history.pop(0)

        self.environment.update_ultrasonic(
            min(self.front_history),
            min(self.left_history),
            min(self.right_history)
            # statistics.mean(self.front_history),
            # statistics.mean(self.left_history),
            # statistics.mean(self.right_history)
        )    

    # =======================================================
    # LiDAR Callback
    # =======================================================


    def lidar_callback(self, msg):

        ranges = list(msg.ranges)

        def safe_min(start_deg, end_deg):

            values = []

            for angle in range(start_deg, end_deg + 1):

                index = int(
                    (math.radians(angle) - msg.angle_min)
                    / msg.angle_increment
                )

                if 0 <= index < len(ranges):

                    value = ranges[index]

                    if (
                        math.isfinite(value)
                        and msg.range_min < value < msg.range_max
                    ):
                        values.append(value)

            if values:
                return min(values)

            return 999.0

        front = safe_min(-15, 15)
        left = safe_min(-90, -45)
        right = safe_min(45, 90)

        self.environment.update_lidar(
            front,
            left,
            right
        )


    # =======================================================
    # Camera / YOLO Callback
    # =======================================================


    def obstacle_callback(self, msg):

        self.environment.update_camera(
            msg.detected,
            msg.class_name,
            msg.confidence,
            999.0,
            msg.center_x
        )




    # =======================================================
    # Main Navigation Timer
    # =======================================================

    def timer_callback(self):

        self.state_machine.run()


def main(args=None):

    rclpy.init(args=args)

    node = DecisionNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
