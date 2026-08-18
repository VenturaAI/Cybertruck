from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        LogInfo(msg="========================================"),
        LogInfo(msg="CyberTruck Autonomous System Starting"),
        LogInfo(msg="Launching all ROS2 nodes..."),
        LogInfo(msg="========================================"),

        #########################################################
        # RPLidar C1
        #########################################################

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='rplidar',
            output='both',
            parameters=[
                {
                    'serial_port': '/dev/ttyUSB0',
                    'serial_baudrate': 460800,
                    'frame_id': 'laser',
                    'inverted': False,
                    'angle_compensate': True
                }
            ]
        ),

        ##################################################
        # Camera
        # USB Camera (v4l2)
        ##################################################

        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="v4l2_camera",
            output="both",
            parameters=[
                {
                    "video_device": "/dev/video0",
                    "image_size": [320, 240],
                    "pixel_format": "YUYV",
                    "output_encoding": "rgb8",
                }
            ],
        ),



        ##################################################
        # Perception
        # Camera Detection (YOLO ONNX)
        ##################################################

        Node(
            package="camera_detection_node",
            executable="camera_detection_node",
            name="camera_detection_node",
            output="both",
            respawn=True,
            respawn_delay=2.0,
        ),


        ##################################################
        # Perception
        # Ultrasonic
        ##################################################

        Node(
            package="ultrasonic_node",
            executable="ultrasonic_node",
            name="ultrasonic_node",
            output="both",
            respawn=True,
            respawn_delay=2.0,
        ),

        ##################################################
        # Decision
        ##################################################

        Node(
            package="decision_node",
            executable="decision_node",
            name="decision_node",
            output="both",
            respawn=True,
            respawn_delay=2.0,
        ),


        ##################################################
        # Control
        # Motor Controller
        ##################################################

        Node(
            package="motor_controller_node",
            executable="motor_controller_node",
            name="motor_controller_node",
            output="both",
            respawn=True,
            respawn_delay=2.0,
        ),


    ])
