#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def quat_to_roll_pitch(x, y, z, w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return roll, pitch


def normalize(v):
    n = math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    if n < 1e-9:
        return (0.0, 0.0, 1.0)
    return (v[0] / n, v[1] / n, v[2] / n)


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vector_align_quat(v_from, v_to):
    v_from = normalize(v_from)
    v_to = normalize(v_to)
    d = dot(v_from, v_to)

    if d > 0.999999:
        return (0.0, 0.0, 0.0, 1.0)
    if d < -0.999999:
        axis = cross((1.0, 0.0, 0.0), v_from)
        if math.sqrt(sum(c * c for c in axis)) < 1e-6:
            axis = cross((0.0, 1.0, 0.0), v_from)
        axis = normalize(axis)
        return (axis[0], axis[1], axis[2], 0.0)

    axis = cross(v_from, v_to)
    s = math.sqrt((1.0 + d) * 2.0)
    invs = 1.0 / s
    q = (axis[0] * invs, axis[1] * invs, axis[2] * invs, s * 0.5)
    n = math.sqrt(sum(c * c for c in q))
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


# ---- Convolved ZV 입력 성형기 (100Hz 기준, 4-tap) ----
CONV_ZV_BUFFER_SIZE = 40
CONV_ZV_TAPS = [(0, 0.25), (11, 0.25), (21, 0.25), (32, 0.25)]  # (지연 샘플수, 가중치)


class ConvolvedZVShaper:
    def __init__(self, buffer_size=CONV_ZV_BUFFER_SIZE, taps=CONV_ZV_TAPS):
        self.buffer_size = buffer_size
        self.taps = taps
        self.history = [0.0] * buffer_size
        self.idx = 0

    def update(self, x_new):
        self.history[self.idx] = x_new
        y = 0.0
        for delay, weight in self.taps:
            i = (self.idx - delay) % self.buffer_size
            y += weight * self.history[i]
        self.idx = (self.idx + 1) % self.buffer_size
        return y


class GimbalLevelingController(Node):
    def __init__(self):
        super().__init__('gimbal_leveling_controller')
        self.declare_parameter('kp', 1.0)
        self.kp = self.get_parameter('kp').value
        self.accel0 = None

        self.roll_shaper = ConvolvedZVShaper()
        self.pitch_shaper = ConvolvedZVShaper()

        self.roll_pub = self.create_publisher(Float64, '/gimbal_roll_cmd', 10)
        self.pitch_pub = self.create_publisher(Float64, '/gimbal_pitch_cmd', 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.get_logger().info(f'accel-based + Convolved ZV controller started, kp={self.kp}')

    def imu_callback(self, msg: Imu):
        a = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)

        if self.accel0 is None:
            self.accel0 = a
            self.get_logger().info(f'baseline resultant-force vector captured: {a}')
            return

        qx, qy, qz, qw = vector_align_quat(self.accel0, a)
        roll_raw, pitch_raw = quat_to_roll_pitch(qx, qy, qz, qw)

        roll_shaped = self.roll_shaper.update(roll_raw)
        pitch_shaped = self.pitch_shaper.update(pitch_raw)

        roll_cmd = max(-0.4363, min(0.4363, self.kp * roll_shaped))
        pitch_cmd = max(-0.4363, min(0.4363, self.kp * pitch_shaped))

        self.get_logger().info(
            f'roll_raw={roll_raw:.4f} roll_shaped={roll_shaped:.4f} '
            f'pitch_raw={pitch_raw:.4f} pitch_shaped={pitch_shaped:.4f}')

        self.roll_pub.publish(Float64(data=roll_cmd))
        self.pitch_pub.publish(Float64(data=pitch_cmd))


def main():
    rclpy.init()
    node = GimbalLevelingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()