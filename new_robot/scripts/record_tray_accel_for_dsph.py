#!/usr/bin/env python3
"""
record_tray_accel_for_dsph.py

트레이 IMU(/imu_tray)를 직접 기록해서 DualSPHysics accinput용 CSV
(new_robot_accel.csv와 동일한 형식: time,linx,liny,linz,angx,angy,angz,
헤더 없음)를 바로 생성한다.

기존 process_odometry.py + convert_to_accinput.py 파이프라인을 대체한다.
그 두 스크립트는 root(차체) pose에 "water_tank로 가는 고정 오프셋"을
그대로 곱해서 트레이 pose를 계산했는데, 그 오프셋이 상수라 짐벌
(revolute_1/revolute_2)이 실제로 얼마나 돌았는지가 전혀 반영되지 않는
문제가 있었다. 레벨링 컨트롤러를 켜든 끄든 계산 결과가 똑같이 나오는
구조라 ON/OFF 비교가 불가능했다.

이 스크립트는 대신 트레이에 실제로 붙어있는 IMU(/imu_tray)를 직접
읽는다. 짐벌이 얼마나 움직였든 상관없이 "트레이가 실제로 느끼는" 값을
센서가 그대로 알려주기 때문에, 별도로 조인트 각도를 합성할 필요가
없다.

가속도 변환 (중요):
  가속도계는 "고유힘"(specific force) = 진짜가속도 - 중력벡터 를
  측정한다. 가만히 있어도 중력 반작용으로 (0,0,+G) 근처 값이 찍힌다.
  DualSPHysics accinput에는(globalgravity=1로 실제 중력은 이미 별도
  유지되므로) 순수 운동학적 가속도가 들어가야 하므로 아래 식으로
  복원한다:

      a_world = R(orientation) @ linear_acceleration + gravity_vector

  각속도(angx,angy,angz)는 자이로 실측값(angular_velocity, body frame)을
  그대로 사용한다 — 기존 스크립트의 오일러각 수치미분 근사보다 더
  정확하다.

⚠️ DualSPHysics의 accinput이 정확히 어느 좌표계(월드 vs 트레이 로컬)를
   기대하는지, globalgravity=1과 조합했을 때 부호가 맞는지는 반드시
   간단한 사인 체크로 검증할 것: 로봇을 완전 정지시키고 짐벌도 0으로
   고정한 상태로 몇 초 기록해서 나온 값이 대략 0 근처인지 확인.
   (정지 + 수평 상태에서는 물에 추가로 걸리는 힘이 없어야 정상)

사용법:
    python3 record_tray_accel_for_dsph.py [출력파일경로]
    (기본 출력: new_robot_accel.csv)
"""

import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

GRAVITY = 9.81  # GenCase XML의 <gravity z="-9.81">와 동일하게 맞춤
GRAVITY_VEC = (0.0, 0.0, -GRAVITY)

TRAY_IMU_TOPIC = "/imu_tray"
DEFAULT_OUTPUT_PATH = "new_robot_accel.csv"


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)),
        (2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)),
    )


def mat_vec_mul(R, v):
    return (
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    )


class TrayAccelRecorder(Node):
    def __init__(self, output_path: str):
        super().__init__("record_tray_accel_for_dsph")
        self.sub = self.create_subscription(Imu, TRAY_IMU_TOPIC, self._on_imu, 100)
        self.f = open(output_path, "w", newline="")
        self._t0 = None
        self._n = 0
        self.get_logger().info(f"'{TRAY_IMU_TOPIC}' 기록 시작 -> {output_path}")

    def _on_imu(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t
        rel_t = t - self._t0

        q = msg.orientation
        R = quat_to_matrix(q.x, q.y, q.z, q.w)
        a_body = (msg.linear_acceleration.x,
                  msg.linear_acceleration.y,
                  msg.linear_acceleration.z)
        a_world_specific = mat_vec_mul(R, a_body)
        a_world = (
            a_world_specific[0] + GRAVITY_VEC[0],
            a_world_specific[1] + GRAVITY_VEC[1],
            a_world_specific[2] + GRAVITY_VEC[2],
        )

        w = msg.angular_velocity  # body frame 그대로 사용

        self.f.write(
            f"{rel_t:.6f},"
            f"{a_world[0]:.6f},{a_world[1]:.6f},{a_world[2]:.6f},"
            f"{w.x:.6f},{w.y:.6f},{w.z:.6f}\n"
        )
        self._n += 1
        if self._n % 500 == 0:
            self.f.flush()
            self.get_logger().info(f"{self._n}개 샘플 기록됨 (t={rel_t:.2f}s)")

    def finish(self):
        self.f.flush()
        self.f.close()
        self.get_logger().info(f"총 {self._n}개 샘플 저장 완료")


def main(args=None):
    output_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_OUTPUT_PATH
    rclpy.init(args=args)
    node = TrayAccelRecorder(output_path)
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