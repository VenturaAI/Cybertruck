# Autonomous 1:8 Cybertruck
NeuralDrive: Autonomous 1:8 Scale (4x4) CyberTruck

<img width="782" height="532" alt="image" src="https://github.com/user-attachments/assets/6b3c28cc-a969-473d-9763-2c9ad451b484" />


Developing a reliable autonomous vehicle requires the integration of multiple engineering disciplines such as embedded systems, robotics, computer vision, artificial intelligence, control systems, electrical systems and real-time software engineering. While commercial autonomous vehicles employ expensive sensors and high-performance computing platforms, many of the underlying concepts can be demonstrated using affordable hardware like the UNO Q and open-source software such as local LLMs etc.

This project presents the design and development of an autonomous CyberTruck-inspired robotic platform built using a 1:8 scale 4x4 vehicle having Ackermann type steering. The objective is to create a modular, low-cost, and scalable autonomous vehicle capable of detecting obstacles, making navigation decisions, and operating without human intervention in structured (indoor) and semi-structured environments.

The project utilizes the Robot Operating System 2 (ROS 2) as the middleware for communication between different software modules. A LiDAR sensor provides real-time environmental perception, ultrasonic sensor enhances close-range obstacle detection providing an emergency manoeuvre, and a camera integrated with a YOLO-26n based object detection model provides semantic understanding of the surroundings. A custom navigation state machine performs sensor fusion and determines the appropriate driving behaviour, while an embedded edge-computing SBC platform (Arduino UNO Q) executes all perception and control algorithms in real time.

The software architecture emphasizes modularity, maintainability, and scalability, allowing future integration of additional sensors, simultaneous localization and mapping (SLAM), path planning, and advanced autonomous driving algorithms.
The project also serves as a practical platform for understanding the complete autonomous driving pipeline that is from sensor data acquisition and perception to decision-making and actuation. These technologies are widely adopted in the robotics and autonomous vehicle industries.

