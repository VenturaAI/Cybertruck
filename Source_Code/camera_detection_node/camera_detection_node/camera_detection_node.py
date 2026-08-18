import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from cybertruck_msgs.msg import Obstacle

from camera_detection_node.yolo_driver import YoloDriver
import cv2

class CameraDetectionNode(Node):

    def __init__(self):

        super().__init__('camera_detection_node')

        self.bridge = CvBridge()

        self.driver = YoloDriver()

        self.publisher = self.create_publisher(
            Obstacle,
            '/obstacle',
            10
        )

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info(
            'Camera Detection Node Started'
        )

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )
        cv2.imwrite("/tmp/ros_frame.jpg", frame)

        result = self.driver.detect(frame)

        obstacle = Obstacle()

        obstacle.detected = result["detected"]
        obstacle.class_name = result["class_name"]
        obstacle.confidence = result["confidence"]
        obstacle.distance = result["distance"]
        obstacle.center_x = result["center_x"]
        obstacle.center_y = result["center_y"]

        self.publisher.publish(obstacle)

        self.get_logger().info(
            f'Detected={obstacle.detected} '
            f'Class={obstacle.class_name} '
            f'Conf={obstacle.confidence:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = CameraDetectionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
