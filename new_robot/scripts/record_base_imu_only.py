#!/usr/bin/env python3
"""
record_base_imu_only.py

record_gimbal_accel_for_dsph.py의 단일 채널(base만) 버전 — 오염 검증
(1단계 승인 답변 추가사항 2)에서만 쓴다.

record_gimbal_accel_for_dsph.py는 /gimbal_pitch_cmd(짐벌 ACTIVE 상태에서만
발행)의 첫 수신을 t=0 기준으로 삼는데, 오염 검증의 "짐벌 OFF" 실행은
컨트롤러 노드 자체를 아예 기동하지 않으므로 그 신호가 영원히 오지 않는다.
이 스크립트는 게이팅 없이 /imu 첫 수신 시각을 그대로 t=0으로 삼는다 —
OFF/ON 두 실행 모두 "노드 기동 후 즉시 기록 시작"으로 통일해야 두 base IMU
트레이스를 같은 기준으로 비교할 수 있다.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

GRAVITY = 9.81
GRAVITY_VEC = (0.0, 0.0, -GRAVITY)
ACCEL_CLAMP_MPS2 = 40.0  # record_gimbal_accel_for_dsph.py와 동일 근거

BASE_IMU_TOPIC = "/imu"
BODY2_CURRENT_BASE = ((0, 1.0), (2, 1.0), (1, -1.0))  # gimbal_leveling_controller.py AXIS_PRESETS와 동일


def remap3(v, spec):
    return (spec[0][1] * v[spec[0][0]],
            spec[1][1] * v[spec[1][0]],
            spec[2][1] * v[spec[2][0]])


def clamp_vec_norm(v, max_norm):
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n <= max_norm or n < 1e-9:
        return v, False
    scale = max_norm / n
    return (v[0] * scale, v[1] * scale, v[2] * scale), True


class BaseImuRecorder(Node):
    def __init__(self, output_path):
        super().__init__("record_base_imu_only")
        self.sub = self.create_subscription(Imu, BASE_IMU_TOPIC, self._on_imu, 100)
        self.f = open(output_path, "w", newline="")
        self._t0 = None
        self._n = 0
        self.get_logger().info(f"'{BASE_IMU_TOPIC}' 수신 즉시 기록 시작 -> {output_path}")

    def _on_imu(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t
        rel_t = t - self._t0

        a_body = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        a_remap = remap3(a_body, BODY2_CURRENT_BASE)
        a_world = (a_remap[0] + GRAVITY_VEC[0], a_remap[1] + GRAVITY_VEC[1], a_remap[2] + GRAVITY_VEC[2])
        a_world, _ = clamp_vec_norm(a_world, ACCEL_CLAMP_MPS2)

        w_body = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        w = remap3(w_body, BODY2_CURRENT_BASE)

        self.f.write(f"{rel_t:.6f},{a_world[0]:.6f},{a_world[1]:.6f},{a_world[2]:.6f},"
                      f"{w[0]:.6f},{w[1]:.6f},{w[2]:.6f}\n")
        self._n += 1
        if self._n % 500 == 0:
            self.f.flush()

    def finish(self):
        self.f.flush()
        self.f.close()
        self.get_logger().info(f"총 {self._n}개 샘플 저장 완료")


def main(args=None):
    output_path = sys.argv[1] if len(sys.argv) > 1 else "base_imu_only.csv"
    rclpy.init(args=args)
    node = BaseImuRecorder(output_path)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
