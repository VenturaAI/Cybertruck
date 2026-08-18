"""
=============================================================

state_machine.py

Main navigation state machine for the CyberTruck.

This module is responsible ONLY for navigation decisions.

It never reads ROS topics directly.

Sensor data comes from:

    Environment
        ↓
    SensorFusion
        ↓
    NavigationStateMachine

=============================================================
"""

from enum import Enum, auto

from cybertruck_msgs.msg import VehicleCommand

from .navigation_parameters import NavigationParameters
from .sensor_fusion import SensorFusion
from .utils import (
    StateLogger,
    change_state,
    elapsed_seconds,
)


# ============================================================
# Vehicle States
# ============================================================

class VehicleState(Enum):

    STARTUP = auto()

    FORWARD = auto()

    GAP_APPROACH = auto()

    TURN_LEFT = auto()

    TURN_RIGHT = auto()

    CHECK_PROGRESS = auto()

    RECOVERY_REVERSE = auto()

    RECOVERY_TURN = auto()

    EMERGENCY_STOP = auto()


# ============================================================
# Navigation State Machine
# ============================================================

class NavigationStateMachine:

    def __init__(self, node, environment):

        self.node = node

        self.environment = environment

        self.params = NavigationParameters()

        self.fusion = SensorFusion(environment)

        self.logger = StateLogger(
            self.params.STATUS_PRINT_RATE_SEC
            
        )

        self.state = VehicleState.FORWARD

        self.node.state = self.state

        self.node.state_start_time = (
            self.node.get_clock().now()
        )

        self.recovery_attempts = 0

        self.last_turn = "LEFT"

        self.best_gap = "NONE"

        self.front_distance_before_turn = 999.0

        self.turn_completed = False

        self.progress_reference = 999.0

        self.force_next_gap = False

        self.extended_reverse_used = False

    # =======================================================
    # Generic Helpers
    # =======================================================

    def set_state(self, new_state):

        if self.state == new_state:
            return

        self.state = new_state

        self.node.state = new_state

        change_state(
            self.node,
            new_state.name
        )

    def state_time(self):

        return elapsed_seconds(self.node)

    # =======================================================
    # Command Publisher
    # =======================================================

    def publish_command(
        self,
        speed,
        steering,
        brake=False
    ):

        command = VehicleCommand()

        command.speed = float(speed)

        command.steering = float(steering)

        command.brake = bool(brake)

        self.node.publisher.publish(command)

    # =======================================================
    # Logging
    # =======================================================

    def log(self, action):

        self.logger.print_status(
            self.node.get_logger(),
            self.state.name,
            self.environment,
            action
        )

    # =======================================================
    # Entry Point
    # =======================================================

    def run(self):

        fusion = self.fusion.evaluate()

        #
        # Highest priority
        #

        # Enter emergency stop immediately if required.
        # If we are already in emergency stop,
        # let the emergency handler decide when to exit.
        #
        
        if (
            fusion.emergency_stop and
            self.state != VehicleState.EMERGENCY_STOP
            and self.state != VehicleState.RECOVERY_REVERSE
        ):
        
            self.set_state(
                VehicleState.EMERGENCY_STOP
            )

        #
        # Dispatch
        #

        if self.state == VehicleState.FORWARD:

            self.handle_forward(fusion)

            return

        if self.state == VehicleState.GAP_APPROACH:

            self.handle_gap_approach(fusion)

            return

        if self.state == VehicleState.TURN_LEFT:

            self.handle_turn_left()

            return

        if self.state == VehicleState.TURN_RIGHT:

            self.handle_turn_right()

            return

        if self.state == VehicleState.CHECK_PROGRESS:

            self.handle_check_progress()

            return

        if self.state == VehicleState.RECOVERY_REVERSE:

            self.handle_recovery_reverse()

            return

        if self.state == VehicleState.RECOVERY_TURN:

            self.handle_recovery_turn()

            return

        if self.state == VehicleState.EMERGENCY_STOP:

            self.handle_emergency_stop()

            return

    # =======================================================
    # FORWARD
    # =======================================================

    def handle_forward(self, fusion):

        self.log("FORWARD")

        if fusion.obstacle_detected:

            self.front_distance_before_turn = (
                self.environment.ultrasonic.front
            )

            self.best_gap = fusion.best_gap

            self.set_state(
                VehicleState.GAP_APPROACH
            )

            return

        if fusion.camera_detected:
        
            self.publish_command(
                speed=self.params.CAMERA_CAUTION_SPEED,
                steering=0.0,
                brake=False
            )
        
            return


        self.publish_command(
            speed=self.params.FORWARD_SPEED,
            steering=0.0,
            brake=False
        )


    #def choose_gap(self, fusion):
        """
        Hybrid gap selection: keeps the direction committed at
        detection time unless the opposite side has since become
        significantly more open (by GAP_SWITCH_MARGIN_M). Guards
        against switching on marginal/noisy differences while
        still correcting a stale early commitment.
        """
         
        #committed = self.best_gap
         
        #if committed == "NONE":
            ## No committed direction (LiDAR wasn't fresh when the
            ## obstacle was first detected) - fall back to camera
            ## position, or alternate if camera has nothing either.
            
            #return "RIGHT" if self.last_turn == "LEFT" else "LEFT"
         
        #live = fusion.best_gap
         
        #if live == "NONE" or live == committed:
            #return committed
         
        #if committed == "LEFT":
            #committed_distance = fusion.best_left_gap
            #live_distance = fusion.best_right_gap
        #else:
            #committed_distance = fusion.best_right_gap
            #live_distance = fusion.best_left_gap
         
        #if live_distance > committed_distance + self.params.GAP_SWITCH_MARGIN_M:
            #self.node.get_logger().info(
                #f"Gap changed {committed} -> {live}"
            #)
            #return live
         
        #return committed     

    def choose_gap(self, fusion):
        
        if not self.environment.lidar_fresh():
            if fusion.camera_detected:
                if fusion.camera_center_x < self.params.IMAGE_CENTER_X:
                    return "RIGHT"
                return "LEFT"
            return "RIGHT" if self.last_turn == "LEFT" else "LEFT"
        
        left_distance = fusion.best_left_gap
        right_distance = fusion.best_right_gap
        
            # A clear, strong difference - trust the live sensor
            # data regardless of what was tried before.
        if abs(left_distance - right_distance) > self.params.GAP_SWITCH_MARGIN_M:
            result = "LEFT" if left_distance > right_distance else "RIGHT"
        
            # Genuinely ambiguous / roughly equal sides - alternate
            # from the last attempt.
        else: 
             result = "RIGHT" if self.last_turn == "LEFT" else "LEFT"

        self.node.get_logger().info(
            f"choose_gap: L={left_distance:.2f} R={right_distance:.2f} -> {result}"
        )
        
        return result
    
    # =======================================================
    # GAP APPROACH
    # =======================================================

    def handle_gap_approach(self, fusion):

        self.log("GAP_APPROACH")

        #
        # Continue approaching while obstacle
        # remains in front.
        #
        
        if fusion.front_obstacle and fusion.boxed_in:
            self.set_state(VehicleState.RECOVERY_REVERSE)
            return

        
        #if fusion.front_obstacle:
        # Keep approaching for a fixed time
        if (
            self.state_time() < self.params.GAP_APPROACH_TIME_SEC
            and self.environment.ultrasonic.front > self.params.GAP_APPROACH_MIN_DISTANCE_CM
        ):

            self.publish_command(
                speed=self.params.GAP_APPROACH_SPEED,
                steering=0.0,
                brake=False
            )

            return

        #
        # Obstacle disappeared.
        # Decide which direction to turn.using the gap
            # direction committed when the obstacle was first
            # detected (self.best_gap) rather than re-evaluating
            # fusion.best_gap now, right as the vehicle sits
            # closest to the obstacle where side readings are
            # least reliable.
            #
        #
        #gap = self.best_gap
        
        #if gap == "NONE":
            # LiDAR wasn't fresh when we committed a direction.
            # Don't silently default to RIGHT — alternate from
            # the last known turn direction instead.
            #gap = "RIGHT" if self.last_turn == "LEFT" else "LEFT"


        gap = self.choose_gap(fusion)

        #if self.force_next_gap:
            #gap = self.best_gap
            #self.force_next_gap = False
        #else:
            #gap = self.choose_gap(fusion)
        
        if gap == "LEFT":

        #if fusion.best_gap == "LEFT":

            self.last_turn = "LEFT"

            self.set_state(
                VehicleState.TURN_LEFT
            )

            return

        self.last_turn = "RIGHT"

        self.set_state(
            VehicleState.TURN_RIGHT
        )

    # =======================================================
    # TURN LEFT
    # =======================================================

    def handle_turn_left(self):

        self.log("TURN_LEFT")

        self.publish_command(
            speed=self.params.TURN_SPEED,
            steering=-self.params.TURN_STEERING,
            brake=False
        )
        

        if (
            self.state_time()
            >= self.params.TURN_TIME_SEC
        ):

            self.progress_reference = (
                self.environment.ultrasonic.front
            )

            self.set_state(
                VehicleState.CHECK_PROGRESS
            )

    # =======================================================
    # TURN RIGHT
    # =======================================================

    def handle_turn_right(self):

        self.log("TURN_RIGHT")

        self.publish_command(
            speed=self.params.TURN_SPEED,
            steering=self.params.TURN_STEERING,
            brake=False
        )

        if (
            self.state_time()
            >= self.params.TURN_TIME_SEC
        ):
       
            self.progress_reference = (
                self.environment.ultrasonic.front
            )

            self.set_state(
                VehicleState.CHECK_PROGRESS
            )            


    # =======================================================
    # CHECK PROGRESS
    # =======================================================

    def handle_check_progress(self):

        self.log("CHECK_PROGRESS")

        #
        # If we have gained enough clearance,
        # resume forward navigation.
        #

        if (
            self.environment.ultrasonic.front
            >
            self.params.FRONT_CLEAR_DISTANCE_CM
        ):

            self.recovery_attempts = 0

            self.set_state(
                VehicleState.FORWARD
            )

            return

        #
        # Otherwise start recovery.
        #

        self.set_state(
            VehicleState.RECOVERY_REVERSE
        )


    # =======================================================
    # RECOVERY REVERSE
    # =======================================================

    def handle_recovery_reverse(self):

        self.log("RECOVERY_REVERSE")


        # Steer opposite to the direction that just failed, so
        # the reverse motion swings the vehicle toward the new
        # chosen direction — instead of reversing straight and
        # only turning once moving forward again.
        if self.last_turn == "LEFT":
            recovery_steering = self.params.TURN_STEERING
        else:
            recovery_steering = -self.params.TURN_STEERING

            
        self.publish_command(
            speed=self.params.RECOVERY_REVERSE_SPEED,
            steering=recovery_steering,
            brake=False
        )
        

        #if (
        #    self.state_time()
        #    >=
        #    self.params.RECOVERY_REVERSE_TIME_SEC
        #):


        #    self.recovery_attempts += 1

        # Use a longer reverse duration once we've already
        # failed several times, as a last real attempt before
        # giving up.
        
        if (
            self.recovery_attempts >= self.params.EXTENDED_REVERSE_TRIGGER
            and not self.extended_reverse_used
        ):
            required_time = self.params.EXTENDED_REVERSE_TIME_SEC
        else:
            required_time = self.params.RECOVERY_REVERSE_TIME_SEC

        if self.state_time() >= required_time:

            if (
                self.recovery_attempts >= self.params.EXTENDED_REVERSE_TRIGGER
                and not self.extended_reverse_used
            ):
                self.extended_reverse_used = True
            else:
                self.recovery_attempts += 1
                
        
            if (
                self.recovery_attempts
                >=
                self.params.MAX_RECOVERY_ATTEMPTS
            ):

                self.set_state(
                    VehicleState.EMERGENCY_STOP
                )
                
                return

            # The previous attempt (self.last_turn) didn't get
            # through — that's why we're back here. Force the next
            # attempt toward the opposite side, rather than letting
            # choose_gap() naturally re-pick the same direction
            # again (which it will, if LiDAR keeps reporting that
            # side as open — LiDAR's side reading doesn't guarantee
            # the front stays clear while pivoting that way).
            self.best_gap = "RIGHT" if self.last_turn == "LEFT" else "LEFT"

            

            # Hand back to GAP_APPROACH rather than blindly
            # turning: this re-runs fusion against a fresh
            # LiDAR read (now that we've backed off), so the
            # turn direction is chosen from a real "check
            # progress" moment instead of the fixed
            # last_turn heuristic in RECOVERY_TURN.

            self.set_state(
                VehicleState.GAP_APPROACH
            )


    # =======================================================
    # RECOVERY TURN
    # =======================================================

    def handle_recovery_turn(self):

        self.log("RECOVERY_TURN")

        if self.last_turn == "LEFT":

            steering = self.params.TURN_STEERING

        else:

            steering = -self.params.TURN_STEERING

        self.publish_command(
            speed=self.params.RECOVERY_REVERSE_SPEED,
            steering=steering,
            brake=False
        )

        if (
            self.state_time()
            >=
            self.params.RECOVERY_TURN_TIME_SEC
        ):

            self.recovery_attempts += 1

            if (
                self.recovery_attempts
                >=
                self.params.MAX_RECOVERY_ATTEMPTS
            ):

                self.set_state(
                    VehicleState.EMERGENCY_STOP
                )

                return

            self.set_state(
                VehicleState.CHECK_PROGRESS
            )


    # =======================================================
    # EMERGENCY STOP
    # =======================================================

    def handle_emergency_stop(self):
    
        self.log("EMERGENCY_STOP")
    
        self.publish_command(
            speed=0.0,
            steering=0.0,
            brake=True
        )
    
        #
        # Remain stopped while an emergency
        # condition still exists.
        #
    
        if self.fusion.evaluate().emergency_stop:
            #return
            if self.state_time() >= self.params.EMERGENCY_STOP_HOLD_SEC:
            
                self.set_state(
                    VehicleState.RECOVERY_REVERSE
                )
            
            return

        
        #
        # Path is clear again.
        # Release brake and resume navigation.
        #
    
        self.node.get_logger().info(
            "Emergency cleared. Resuming navigation."
        )
    
        self.clear_emergency()

    # =======================================================
    # Reset Navigation
    # =======================================================

    def reset_navigation(self):

        self.recovery_attempts = 0

        self.last_turn = "LEFT"

        self.best_gap = "NONE"

        self.front_distance_before_turn = 999.0

        self.progress_reference = 999.0

        self.set_state(
            VehicleState.FORWARD
        )

    # =======================================================
    # Emergency Reset
    # =======================================================

    def clear_emergency(self):

        self.recovery_attempts = 0

        self.publish_command(
            speed=0.0,
            steering=0.0,
            brake=False
        )

        self.set_state(
            VehicleState.FORWARD
        )

    # =======================================================
    # Current State
    # =======================================================

    def current_state(self):

        return self.state

    # =======================================================
    # Recovery Status
    # =======================================================

    def recovery_count(self):

        return self.recovery_attempts            
