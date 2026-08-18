class MotorDriver:

    def __init__(self):
        print("Motor Driver Initialized")

    def drive_forward(self, pwm):
        print(f"FORWARD PWM={pwm}")

    def drive_reverse(self, pwm):
        print(f"REVERSE PWM={pwm}")

    def stop_drive(self):
        print("DRIVE STOP")

    def steer_left(self, duration_ms):
        print(f"STEER LEFT {duration_ms} ms")

    def steer_right(self, duration_ms):
        print(f"STEER RIGHT {duration_ms} ms")

    def stop_steering(self):
        print("STEERING STOP")

    def emergency_stop(self):
        print("EMERGENCY STOP")