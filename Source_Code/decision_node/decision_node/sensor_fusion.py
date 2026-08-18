"""
============================================================

sensor_fusion.py

============================================================

PURPOSE

This module combines information coming from

    Ultrasonic
    LiDAR
    Camera (YOLO)

into one navigation decision.

The StateMachine never needs to know which sensor
generated the information.

============================================================
"""

from dataclasses import dataclass

from .environment import Environment


# ==========================================================
# Fusion Result
# ==========================================================

@dataclass
class FusionResult:

    #
    # Highest priority
    #

    emergency_stop: bool = False

    #
    # Main navigation result
    #

    obstacle_detected: bool = False

    front_obstacle: bool = False

    obstacle_source: str = "NONE"

    #
    # Gap selection
    #

    best_gap: str = "NONE"

    best_left_gap: float = 0.0

    best_right_gap: float = 0.0

    boxed_in: bool = False

    # Camera fields

    camera_class_name: str = ""
    
    camera_confidence: float = 0.0
    
    camera_detected: bool = False
    
    camera_center_x: float = 320.0

# ==========================================================
# Sensor Fusion
# ==========================================================

class SensorFusion:

    """
    Combines all sensors into one navigation result.
    """

    def __init__(self, environment: Environment):

        self.environment = environment

    def evaluate(self):

        result = FusionResult()

        # --------------------------------------------------
        # Highest priority
        #
        # Emergency stop
        # --------------------------------------------------

        if self.environment.emergency_stop_required():

            result.emergency_stop = True

            result.obstacle_detected = True
            result.front_obstacle = True

            result.obstacle_source = "ULTRASONIC"

            return result

        # --------------------------------------------------
        # Ultrasonic
        # --------------------------------------------------

        if self.environment.front_obstacle_ultrasonic():

            result.obstacle_detected = True
            result.front_obstacle = True

            result.obstacle_source = "ULTRASONIC"

        # --------------------------------------------------
        # LiDAR
        # --------------------------------------------------

        elif self.environment.front_obstacle_lidar():

            result.obstacle_detected = True
            result.front_obstacle = True

            result.obstacle_source = "LIDAR"

        # --------------------------------------------------
        # Camera
        # Camera — classification-only. It never triggers
        # navigation on its own (no reliable distance), but
        # we record what it saw for logging/telemetry and as
        # a tie-breaker elsewhere.
        # --------------------------------------------------

        if self.environment.camera_obstacle():

            #result.obstacle_detected = True
            #result.front_obstacle = True

            #result.obstacle_source = "CAMERA"

            result.camera_detected = True
            result.camera_class_name = self.environment.camera.class_name
            result.camera_confidence = self.environment.camera.confidence
            result.camera_center_x = self.environment.camera.center_x

        # --------------------------------------------------
        # Gap Selection
        #
        # LiDAR is always trusted for navigation.
        # Even if the obstacle came from another sensor.
        # --------------------------------------------------

        if self.environment.lidar_fresh():

            result.best_left_gap = self.environment.lidar.left

            result.best_right_gap = self.environment.lidar.right

            if result.best_left_gap > result.best_right_gap:

                result.best_gap = "LEFT"

            else:

                result.best_gap = "RIGHT"

            if (result.best_left_gap < self.environment.params.SAFE_GAP_DISTANCE_M
                    and result.best_right_gap < self.environment.params.SAFE_GAP_DISTANCE_M):
                result.boxed_in = True

                

        return result
