#!/usr/bin/env python3
"""반복 가능한 짧은 주행 자극: 직진가속 -> 완만한 회전 -> 정지."""
import time
import rclpy
from geometry_msgs.msg import Twist

PHASES = [
    (2.0, 0.0, 0.0),
    (3.0, 0.8, 0.0),
    (3.0, 0.8, 0.8),
    (0.5, 0.0, 0.0),
    (6.0, 0.0, 0.0),
]

def main():
    rclpy.init()
    node = rclpy.create_node("drive_test")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    time.sleep(1.0)
    for dur, vx, wz in PHASES:
        t0 = time.time()
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        while time.time() - t0 < dur:
            pub.publish(msg)
            time.sleep(0.05)
    pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
