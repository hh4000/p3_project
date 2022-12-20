# p3_project
P3 Project of group ROB3_gr01:

ROS SOURCE PACKAGE CONTENTS:

Scripts:
- anomaly_detection_and_tdoa.py
    Script that detects sound anomalies and calculates the DOA of them.
    Uses a 3-microphone setup positioned in an equilateral triangle
- driver.py
    Movement controller.
    Moves the robot to a relative angle based on the /localization_topics/goal_angle topic
- motion_detection.py
    detects motion based on RGB camera input.
    Outputs true if motion has been detected.
