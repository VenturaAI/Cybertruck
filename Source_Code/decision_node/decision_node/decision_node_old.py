import rclpy
from rclpy.node import Node

from cybertruck_msgs.msg import Obstacle
from cybertruck_msgs.msg import Ultrasonic
from cybertruck_msgs.msg import VehicleCommand
from sensor_msgs.msg import LaserScan
import math
import statistics
from .environment import Environment


# ==========================================================
# Navigation Parameters
# ==========================================================

SAFE_GAP_DISTANCE = 0.7          # metres

FORWARD_SPEED = 0.5
TURN_SPEED = 0.3
REVERSE_SPEED = -0.3

TURN_STEERING = 0.5

FRONT_CLEAR_DISTANCE = 50.0      # centimetres

TURN_DURATION = 1.0              # seconds
RECOVERY_DURATION = 1.0          # seconds

ULTRASONIC_FILTER_SIZE = 5



class VehicleStateMachine:
    STARTUP = 0
    FORWARD_NAVIGATION = 1
    GAP_APPROACH = 2
    RECOVERY = 3
    EMERGENCY_STOP = 4
    CHECK_PROGRESS = 5
    TURN_LEFT = 6 
    TURN_RIGHT = 7 
    RECOVERY_REVERSE = 8 
    RECOVERY_TURN = 9


class DecisionNode(Node):


    def __init__(self):
        super().__init__('decision_node')
        self.previous_state = None
        self.environment = Environment()
        self.front_distance = 999.0
        self.left_distance = 999.0
        self.right_distance = 999.0

        self.lidar_front = 999.0
        self.lidar_left = 999.0
        self.lidar_right = 999.0
        self.best_lidar_left_gap = 0.0
        self.best_lidar_right_gap = 0.0
       

        self.state = VehicleStateMachine.FORWARD_NAVIGATION
        self.state_start_time = self.get_clock().now()
        self.recovery_count = 0
        
        self.front_distance_before_maneuver = 999.0
        self.last_turn_direction = ""
        self.best_left_gap = 0.0
        self.best_right_gap = 0.0
        self.front_history = []
        self.left_history = []
        self.right_history = []
        

        self.obstacle_subscription = self.create_subscription(
            Obstacle,
            '/obstacle',
            self.obstacle_callback,
            10
         )

        self.ultrasonic_subscription = self.create_subscription(
            Ultrasonic,
            '/ultrasonic',
            self.ultrasonic_callback,
            10
        )

        self.publisher = self.create_publisher(
            VehicleCommand,
            '/vehicle_command',
            10
        )
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Decision loop (20 Hz)
        self.decision_timer = self.create_timer(
            0.05,
            self.decision_loop
        )



        self.get_logger().info('Decision Node Started')

    def ultrasonic_callback(self, msg):



        # Update each ultrasonic sensor independently.
        # Ignore invalid readings while keeping the last valid value.
        
        if msg.front > 0:
        
            self.front_history.append(msg.front)
        
            if len(self.front_history) > ULTRASONIC_FILTER_SIZE:
                self.front_history.pop(0)
        
            self.front_distance = (
                sum(self.front_history) / len(self.front_history)
            )
        
        
        if msg.left > 0:
        
            self.left_history.append(msg.left)
        
            if len(self.left_history) > ULTRASONIC_FILTER_SIZE:
                self.left_history.pop(0)
        
            self.left_distance = (
                sum(self.left_history) / len(self.left_history)
            )
        
        
        if msg.right > 0:
        
            self.right_history.append(msg.right)
        
            if len(self.right_history) > ULTRASONIC_FILTER_SIZE:
                self.right_history.pop(0)
        
            self.right_distance = (
                sum(self.right_history) / len(self.right_history)
            )
        
        
        self.environment.front_distance = self.front_distance
        self.environment.left_distance = self.left_distance
        self.environment.right_distance = self.right_distance

    
    def lidar_callback(self, msg):

        ranges = list(msg.ranges)

        valid = []

        for r in ranges:

            if math.isfinite(r):
                valid.append(r)

        if len(valid) == 0:
           return

        total = len(ranges)

        front_index = total // 2

        left_start = int(total * 0.65)
        left_end = int(total * 0.85)

        right_start = int(total * 0.15)
        right_end = int(total * 0.35)

        front_ranges = [
            r for r in ranges[front_index-20:front_index+20]
            if math.isfinite(r)
        ]

        left_ranges = [
            r for r in ranges[left_start:left_end]
            if math.isfinite(r)
        ]

        right_ranges = [
            r for r in ranges[right_start:right_end]
            if math.isfinite(r)
        ]

        if front_ranges:
            self.lidar_front = statistics.median(front_ranges)
            self.environment.lidar_front = self.lidar_front
        if left_ranges:
            self.lidar_left = statistics.median(left_ranges)
            self.environment.lidar_left = self.lidar_left
        if right_ranges:
            self.lidar_right = statistics.median(right_ranges)
            self.environment.lidar_right = self.lidar_right

    def state_elapsed_seconds(self):

        now = self.get_clock().now()

        return (
            now - self.state_start_time
        ).nanoseconds / 1e9

    def decision_loop(self):
        """
        Runs the decision logic at a fixed 20 Hz.
        """
        
        self.run_state_machine()

    def obstacle_callback(self, msg):
        """
        Camera callback.
        Only updates the latest obstacle information.
        Decision making is handled separately by the timer.
        """
    
        self.environment.obstacle_detected = msg.detected
        self.environment.obstacle_class = msg.class_name
        self.environment.obstacle_confidence = msg.confidence
        self.environment.obstacle_distance = msg.distance



    def run_state_machine(self):
        """
        Main decision state machine.
        Runs at a fixed 20 Hz from decision_loop().

        All sensor callbacks only update the Environment object.
        This function reads the latest Environment and decides the
        next vehicle command.
        """

        
        if self.state != self.previous_state:
            self.get_logger().info(f"State changed -> {self.state}")
            self.previous_state = self.state

        command = VehicleCommand()
        # --------------------------------------------------------
        # GAP APPROACH
        # --------------------------------------------------------


        if self.state == VehicleStateMachine.GAP_APPROACH:

            self.best_lidar_left_gap = max(
                self.best_lidar_left_gap,
                self.lidar_left
            )

            self.best_lidar_right_gap = max(
                self.best_lidar_right_gap,
                self.lidar_right
            )

            command.speed = 0.2
            command.steering = 0.0
            command.brake = False

            if self.state_elapsed_seconds() > 1.0:

                if (
                    self.best_lidar_left_gap > self.best_lidar_right_gap
                    and self.best_lidar_left_gap > SAFE_GAP_DISTANCE
                ):

                    self.last_turn_direction = "LEFT"
                    self.state = VehicleStateMachine.TURN_LEFT
                    self.get_logger().info(
                        f'Best Gap LEFT = {self.best_lidar_left_gap}'
                    )

                elif self.best_lidar_right_gap > SAFE_GAP_DISTANCE:

                    self.last_turn_direction = "RIGHT"
                    self.state = VehicleStateMachine.TURN_RIGHT
                    self.get_logger().info(
                        f'Best Gap RIGHT = {self.best_lidar_right_gap}'
                    )

                else:

                    self.state = VehicleStateMachine.RECOVERY_REVERSE
                    self.get_logger().info(
                        'No Suitable Gap -> RECOVERY_REVERSE'
                    )
                
                self.get_logger().info(
                    f'Best Left={self.best_lidar_left_gap:.2f} '
                    f'Best Right={self.best_lidar_right_gap:.2f}'
                )
                self.state_start_time = self.get_clock().now()

            self.publisher.publish(command)
            return
        
        if self.state == VehicleStateMachine.EMERGENCY_STOP:

           command.speed = 0.0
           command.steering = 0.0
           command.brake = True

           self.publisher.publish(command)

           return

        if self.state == VehicleStateMachine.TURN_LEFT:

            command.speed = 0.3
            command.steering = -TURN_STEERING
            command.brake = False

            if self.state_elapsed_seconds() > 1.0:

                self.state = VehicleStateMachine.CHECK_PROGRESS
                self.state_start_time = self.get_clock().now()

                self.get_logger().info(
                    'TURN_LEFT complete -> CHECK_PROGRESS'
                )

            self.publisher.publish(command)
            return

        if self.state == VehicleStateMachine.TURN_RIGHT:

            command.speed = 0.3
            command.steering = TURN_STEERING
            command.brake = False

            if self.state_elapsed_seconds() > 1.0:

                self.state = VehicleStateMachine.CHECK_PROGRESS
                self.state_start_time = self.get_clock().now()

                self.get_logger().info(
                    'TURN_RIGHT complete -> CHECK_PROGRESS'
                )

            self.publisher.publish(command)
            return

        if self.state == VehicleStateMachine.RECOVERY_REVERSE:

            command.speed = REVERSE_SPEED
            command.steering = 0.0
            command.brake = False

            if self.state_elapsed_seconds() > 1.0:

                self.state = VehicleStateMachine.RECOVERY_TURN
                self.state_start_time = self.get_clock().now()

                self.get_logger().info(
                    'RECOVERY_REVERSE complete -> RECOVERY_TURN'
                )

            self.publisher.publish(command)
            return

        if self.state == VehicleStateMachine.RECOVERY_TURN:

            command.speed = REVERSE_SPEED

            if self.last_turn_direction == "LEFT":

                command.steering = TURN_STEERING

            else:

                command.steering = -TURN_STEERING

            command.brake = False

            if self.state_elapsed_seconds() > 1.0:

                self.recovery_count += 1

                self.get_logger().info(
                    f'Recovery Attempt = {self.recovery_count}'
                )

                if self.recovery_count >= 3:

                   self.state = VehicleStateMachine.EMERGENCY_STOP

                   self.get_logger().info(
                       'RECOVERY FAILED -> EMERGENCY_STOP'
                   )

                else:

                    self.state = VehicleStateMachine.CHECK_PROGRESS
                    self.state_start_time = self.get_clock().now()

                    self.get_logger().info(
                        'RECOVERY_TURN complete -> CHECK_PROGRESS'
                    )

            self.publisher.publish(command)
            return


        if self.state == VehicleStateMachine.CHECK_PROGRESS:
            self.get_logger().info(
                f'Front Distance={self.front_distance}'
                
            )  
          
            if self.front_distance > FRONT_CLEAR_DISTANCE:

                self.state = VehicleStateMachine.FORWARD_NAVIGATION
                self.state_start_time = self.get_clock().now()
                self.recovery_count = 0
                self.get_logger().info(
                    'Progress Improved -> FORWARD_NAVIGATION'
                )

            else:

                self.state = VehicleStateMachine.RECOVERY_REVERSE
                self.state_start_time = self.get_clock().now()
                self.get_logger().info(
                    'No Improvement -> RECOVERY_REVERSE'
                )
            self.publisher.publish(command)
            return

            
        if not self.environment.obstacle_detected:

           if self.state == VehicleStateMachine.FORWARD_NAVIGATION:
           
               command.speed = FORWARD_SPEED
               command.steering = 0.0
               command.brake = False
               self.publisher.publish(command)
           return


        if self.environment.obstacle_detected:

            if self.state == VehicleStateMachine.FORWARD_NAVIGATION:

               self.front_distance_before_maneuver = self.front_distance
               self.best_lidar_left_gap = 0.0
               self.best_lidar_right_gap = 0.0
               self.get_logger().info(
                   f'Saved Front Distance = {self.front_distance_before_maneuver}'
               )
               self.state = VehicleStateMachine.GAP_APPROACH
               self.state_start_time = self.get_clock().now()

               self.get_logger().info(
                   'Transition -> GAP_APPROACH'
               )
            self.publisher.publish(command)
            return



def main(args=None):

   rclpy.init(args=args)

   node = DecisionNode()

   rclpy.spin(node)

   node.destroy_node()

   rclpy.shutdown()

if __name__ == '__main__':
   main()
