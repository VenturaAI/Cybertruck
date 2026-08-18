"""
============================================================
navigation_parameters.py
============================================================

This file contains ALL configurable parameters used by the
autonomous navigation system.


============================================================
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class NavigationParameters:
    """
    Immutable navigation configuration.

    Using a frozen dataclass prevents accidental modification
    while the robot is running.
    """

    # ==========================================================
    # Vehicle Speeds
    # ==========================================================

    # Forward cruising speed
    #
    # Range:
    #   0.0 = stop
    #   1.0 = maximum
    #
    # Conservative for indoor testing.
    FORWARD_SPEED: float = 0.15

    # Slow speed while analysing the surroundings.
    GAP_APPROACH_SPEED: float = 0.15

    CAMERA_CAUTION_SPEED: float = 0.08

    # Early-exit safety margin: stop creeping and commit to a
    # turn the moment front distance drops to this, even if the
    # GAP_APPROACH_TIME_SEC timer hasn't expired yet.

    GAP_APPROACH_MIN_DISTANCE_CM: float = 40.00 #Prev it was 35 cm
     
    # Minimum improvement (meters) the opposite side needs over
    # the committed side before switching gap direction — avoids
    # flip-flopping on marginal/noisy LiDAR differences.

    GAP_SWITCH_MARGIN_M: float = 0.35

    # Turning speed.
    TURN_SPEED: float = 0.20

    # Reverse speed during recovery.
    RECOVERY_REVERSE_SPEED: float = -0.18

    # ==========================================================
    # Steering
    # ==========================================================

    # Steering command.
    #
    # -1.0 = full left
    # +1.0 = full right
    #
    # Start conservatively.
    TURN_STEERING: float = 0.60

    # ==========================================================
    # Ultrasonic Thresholds
    # ==========================================================

    # Immediate stop distance.
    #
    # If anything is closer than this,
    # collision is likely.
    FRONT_STOP_DISTANCE_CM: float = 25.0

    # Begin obstacle avoidance.
    #
    # Gives enough space to manoeuvre.
    FRONT_AVOID_DISTANCE_CM: float = 90.0

    # Desired distance after recovery.
    FRONT_CLEAR_DISTANCE_CM: float = 70.0

    # ==========================================================
    # LiDAR
    # ==========================================================

    # Minimum opening considered safe.
    SAFE_GAP_DISTANCE_M: float = 0.70

    IMAGE_CENTER_X: float = 320.0
    
    # How long to hold a full stop before attempting to back
    # away, if the obstacle is still present. Prevents
    # EMERGENCY_STOP from waiting forever for a static obstacle
    # to move on its own.
    EMERGENCY_STOP_HOLD_SEC: float = 0.5
    
    # LiDAR is mounted ~24cm behind the front-facing ultrasonic
    # sensor. Subtract this to convert LiDAR distances into true
    # distance-from-front-bumper for consistent comparisons.
    LIDAR_FRONT_OFFSET_M: float = 0.24


    # ==========================================================
    # Sensor Filtering
    # ==========================================================

    # Number of ultrasonic readings averaged.
    #
    # HC-SR04 sensors are noisy.
    #
    # Five samples remove spikes while
    # maintaining fast response.
    ULTRASONIC_FILTER_SIZE: int = 2

    # ==========================================================
    # Sensor Timeouts
    # ==========================================================

    # Ignore ultrasonic data older than this.
    ULTRASONIC_TIMEOUT_SEC: float = 0.50

    # Ignore LiDAR data older than this.
    LIDAR_TIMEOUT_SEC: float = 0.50

    # Ignore camera detections older than this.
    CAMERA_TIMEOUT_SEC: float = 1.00

    # ==========================================================
    # State Durations
    # ==========================================================

    # Time spent looking for the best gap.
    GAP_APPROACH_TIME_SEC: float = 0.6

    # Time spent executing a turn.
    TURN_TIME_SEC: float = 1.00

    # Reverse duration.
    RECOVERY_REVERSE_TIME_SEC: float = 1.00

    # Recovery turn duration.
    RECOVERY_TURN_TIME_SEC: float = 1.00

    # ==========================================================
    # Recovery
    # ==========================================================

    # Maximum recovery attempts before stopping.
    MAX_RECOVERY_ATTEMPTS: int = 5

    # After this many failed recovery attempts, try one longer,
    # more committed reverse before giving up entirely.
    EXTENDED_REVERSE_TRIGGER: int = 3
    
    # Duration of that extended reverse — longer than a normal
    # recovery reverse, to actually create real clearance.
    EXTENDED_REVERSE_TIME_SEC: float = 2.5


    

    # ==========================================================
    # Decision Loop
    # ==========================================================

    # Decision frequency.
    #
    # 20 Hz gives good responsiveness
    # while leaving CPU time for
    # perception nodes.
    DECISION_RATE_HZ: float = 20.0

    # ==========================================================
    # Logging
    # ==========================================================

    # Print status every second instead of
    # flooding the terminal.
    STATUS_PRINT_RATE_SEC: float = 1.0
