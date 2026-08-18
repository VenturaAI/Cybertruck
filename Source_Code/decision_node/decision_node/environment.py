"""
============================================================
environment.py
============================================================

PURPOSE

The Environment class represents the robot's understanding
of the outside world.

NO decision making happens here.

This class ONLY stores the latest sensor information and
provides helper functions to determine whether that
information is still valid.



============================================================
"""

from dataclasses import dataclass, field
import time

from .navigation_parameters import NavigationParameters


# ==========================================================
# Ultrasonic Sensor
# ==========================================================

@dataclass
class UltrasonicData:
    front: float = 999.0
    left: float = 999.0
    right: float = 999.0

    timestamp: float = field(default_factory=time.monotonic)


# ==========================================================
# LiDAR
# ==========================================================

@dataclass
class LidarData:
    front: float = 999.0
    left: float = 999.0
    right: float = 999.0

    timestamp: float = field(default_factory=time.monotonic)


# ==========================================================
# Camera / YOLO
# ==========================================================

@dataclass
class CameraData:
    detected: bool = False

    class_name: str = ""

    confidence: float = 0.0

    distance: float = 999.0

    center_x: float = 320.0

    timestamp: float = field(default_factory=time.monotonic)


# ==========================================================
# Environment
# ==========================================================

class Environment:
    """
    Represents the robot's current understanding of the world.

    This class NEVER decides what to do.

    It only stores sensor information.

    Decision making happens inside StateMachine.
    """

    def __init__(self):

        self.params = NavigationParameters()

        self.ultrasonic = UltrasonicData()

        self.lidar = LidarData()

        self.camera = CameraData()

    # ======================================================
    # Update Functions
    # ======================================================

    def update_ultrasonic(self,
                          front: float,
                          left: float,
                          right: float):

        self.ultrasonic.front = front
        self.ultrasonic.left = left
        self.ultrasonic.right = right

        self.ultrasonic.timestamp = time.monotonic()

    def update_lidar(self,
                     front: float,
                     left: float,
                     right: float):

        self.lidar.front = front
        self.lidar.left = left
        self.lidar.right = right

        self.lidar.timestamp = time.monotonic()

    def update_camera(self,
                      detected: bool,
                      class_name: str,
                      confidence: float,
                      distance: float,
                      center_x: float = 320.0):

        self.camera.detected = detected
        self.camera.class_name = class_name
        self.camera.confidence = confidence
        self.camera.distance = distance
        self.camera.center_x = center_x

        self.camera.timestamp = time.monotonic()

    # ======================================================
    # Freshness Checks
    # ======================================================

    def ultrasonic_fresh(self):

        return (
            time.monotonic() -
            self.ultrasonic.timestamp
        ) < self.params.ULTRASONIC_TIMEOUT_SEC

    def lidar_fresh(self):

        return (
            time.monotonic() -
            self.lidar.timestamp
        ) < self.params.LIDAR_TIMEOUT_SEC

    def camera_fresh(self):

        return (
            time.monotonic() -
            self.camera.timestamp
        ) < self.params.CAMERA_TIMEOUT_SEC

    # ======================================================
    # Convenience Helpers
    # ======================================================

    def front_obstacle_ultrasonic(self):

        if not self.ultrasonic_fresh():
            return True

        return (
            self.ultrasonic.front <
            self.params.FRONT_AVOID_DISTANCE_CM
        )

    def emergency_stop_required(self):

        if not self.ultrasonic_fresh():
            return True

        return (
            self.ultrasonic.front <
            self.params.FRONT_STOP_DISTANCE_CM
        )

    def front_obstacle_lidar(self):

        if not self.lidar_fresh():
            #return False
            return True
        front_from_bumper = (
            self.lidar.front - self.params.LIDAR_FRONT_OFFSET_M
        )    

        return (
            front_from_bumper <
            self.params.SAFE_GAP_DISTANCE_M
        )
        #return (
        #    self.lidar.front <
        #    self.params.SAFE_GAP_DISTANCE_M
        #)

    def camera_obstacle(self):

        if not self.camera_fresh():
            return False

        return self.camera.detected
