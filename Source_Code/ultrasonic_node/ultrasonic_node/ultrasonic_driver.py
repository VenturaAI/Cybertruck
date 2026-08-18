from .bridge import Bridge


class UltrasonicDriver:

    def __init__(self):
        print("Ultrasonic Driver Initialized")
    
    def read_all(self):
        front, left, right = Bridge.call("cybertruck.ultrasonic", timeout=0.15)
        return float(front), float(left), float(right)
    

   # def _read(self):
   #     """
   #     Returns:
   #         [front, left, right]
   #     """
   #     return Bridge.call("cybertruck.ultrasonic")

   # def read_front(self):
   #     return self._read()[0]

   # def read_left(self):
   #     return self._read()[1]

   # def read_right(self):
   #     return self._read()[2]
