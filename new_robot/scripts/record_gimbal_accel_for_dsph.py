#!/usr/bin/env python3
"""
record_gimbal_accel_for_dsph.py

record_tray_accel_for_dsph.py의 확장판. 한 번 주행하면서 /imu(차체)와
/imu_tray(트레이)를 동시에 기록해 DualSPHysics accinput CSV 두 개를
한꺼번에 만든다.

    <output>_off.csv  <- /imu (차체 그대로, "짐벌 없음"과 동일:
                          탱크가 차체에 고정됐다면 느꼈을 가속도)
    <output>_on.csv   <- /imu_tray ("짐벌 있음": 짐벌이 보정한 뒤
                          트레이가 실제로 느끼는 가속도)

같은 주행 입력에서 동시에 뽑으므로 두 케이스가 완벽하게 짝지어진다 —
원본 주행 로그를 다시 재생할 필요가 없다.

자동 처리 2가지 (2026-09-05, 실제 Gazebo 실행으로 발견한 문제 대응)
  1. /gimbal_state가 ACTIVE가 되기 전에는 기록하지 않는다. 짐벌 워밍업
     구간(약 7~8초, 상태기계 SENSOR_CHECK->MOTOR_CHECK->READY)에는 위치
     PID가 무게 때문에 완전히 0으로 못 잡고 살짝 처져서(~0.5 m/s^2) 트레이가
     아직 수평이 아닌데, 이 구간이 섞이면 "짐벌 있음" 데이터가 오염된다.
     두 CSV의 t=0은 ACTIVE가 처음 된 순간으로 맞춘다(off/on 공통 기준).
  2. 가속도 크기를 ACCEL_CLAMP_MPS2로 자른다. 접촉솔버가 한 프레임짜리
     비정상 스파이크(실측 최대 420 m/s^2!)를 만드는데, 그대로 넣으면
     DualSPHysics에서 물이 한 프레임 만에 폭발하듯 튄다. 40.0은 임의값이
     아니라 sim_control/gimbal_control_core.py의
     CoreConfig.accel_norm_max_mps2(실제 주행에서 나올 수 있는 비력 상한으로
     이미 검증/사용 중인 값)를 그대로 재사용한 것이다. 방향은 유지하고
     크기만 자른다(스파이크 방향까지 믿을 이유는 없지만, 자르는 것과
     버리는 것 중 시계열 연속성을 위해 자르는 쪽을 택함).

사용법 (터미널 3개)
    터미널1: ros2 launch new_robot gazebo.launch.py
    터미널2: python3 gimbal_leveling_controller.py --ros-args -p use_sim_time:=true \\
                 -p enable_hw_style:=false -p tank_radius_m:=0.06 -p fill_height_m:=0.088
    터미널3: python3 record_gimbal_accel_for_dsph.py my_run
             (터미널2 로그에 "[상태] ACTIVE"가 뜬 뒤 -- 이 스크립트도 그
             시점부터 기록을 시작하지만, auto_activate=True가 기본이라
             보통 몇 초 안에 자동으로 ACTIVE 된다 -- 키보드 텔레옵이나
             조이스틱으로 원하는 만큼 수동 주행)
             Ctrl+C로 종료 -> my_run_off.csv, my_run_on.csv 생성

가속도 변환은 record_tray_accel_for_dsph.py와 동일 (specific force ->
운동학적 가속도 복원, DualSPHysics globalgravity=1과 짝을 맞춤).
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

GRAVITY = 9.81  # GenCase XML의 <gravity z="-9.81">와 동일하게 맞춤
GRAVITY_VEC = (0.0, 0.0, -GRAVITY)

# sim_control/gimbal_control_core.py CoreConfig.accel_norm_max_mps2와 동일한 값.
# 그 이상은 접촉솔버 스파이크로 간주해 크기만 이 값으로 자른다(방향 유지).
ACCEL_CLAMP_MPS2 = 40.0

BASE_IMU_TOPIC = "/imu"
TRAY_IMU_TOPIC = "/imu_tray"
# /gimbal_state는 상태가 "바뀔 때"만 한 번 발행되는 volatile 토픽이라, 이
# 레코더가 그 전환 순간을 놓치면(대개 놓친다 — 컨트롤러가 이미 켜져 있고
# auto_activate로 몇 초 만에 ACTIVE 되는 경우) 다시는 못 받는다. 대신
# 컨트롤러가 ACTIVE일 때만(enable=True) 매 틱 발행하는 /gimbal_pitch_cmd의
# "첫 수신"을 활성화 신호로 쓴다 — 늦게 구독해도 다음 틱에 바로 온다.
GIMBAL_ACTIVE_PROBE_TOPIC = "/gimbal_pitch_cmd"
DEFAULT_OUTPUT_PREFIX = "gimbal_run"

# gimbal_leveling_controller.py의 AXIS_PRESETS와 동일한 값
# (2026-09-05 body2_current_base로 기본값 변경된 것과 짝을 맞춘다).
BODY2_CURRENT_BASE = ((0, 1.0), (2, 1.0), (1, -1.0))
CAD_ROTATED_TRAY = ((1, 1.0), (2, -1.0), (0, -1.0))


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


class ActivationGate(Node):
    """/gimbal_pitch_cmd 첫 수신 시각을 ACTIVE 시작으로 보고 두 레코더에 공유한다."""

    def __init__(self):
        super().__init__("record_activation_gate")
        self.active = False
        self.create_subscription(Float64, GIMBAL_ACTIVE_PROBE_TOPIC, self._on_cmd, 10)

    def _on_cmd(self, msg: Float64):
        if not self.active:
            self.active = True
            self.get_logger().info(">>> ACTIVE 감지(/gimbal_pitch_cmd 수신) — 이 시점부터 두 CSV 기록 시작")


class AccelRecorder(Node):
    def __init__(self, topic, axis_spec, output_path, label, gate: ActivationGate):
        super().__init__(f"record_{label}_accel_for_dsph")
        self.axis_spec = axis_spec
        self.label = label
        self.gate = gate
        self.sub = self.create_subscription(Imu, topic, self._on_imu, 100)
        self.f = open(output_path, "w", newline="")
        self._t0 = None  # 활성화된 뒤 받은 첫 IMU 메시지의 시각(이 레코더 기준 t=0)
        self._n = 0
        self._n_clamped = 0
        self.get_logger().info(f"[{label}] '{topic}' 대기 중 (ACTIVE 되면 기록) -> {output_path}")

    def _on_imu(self, msg: Imu):
        if not self.gate.active:
            return  # 아직 워밍업 구간 — 기록하지 않는다
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t  # gate.active_since 대신 각자 첫 활성 샘플로 t0를 잡는다
                          # (Float64엔 header가 없어 시계 도메인을 맞춰 비교할 수
                          # 없다 — off/on 두 레코더의 t0는 IMU 발행 주기(~수 ms)
                          # 이내로만 어긋나므로 무시할 수 있는 수준이다)
        rel_t = t - self._t0

        a_body = (msg.linear_acceleration.x,
                  msg.linear_acceleration.y,
                  msg.linear_acceleration.z)
        a_remap = remap3(a_body, self.axis_spec)
        a_world = (
            a_remap[0] + GRAVITY_VEC[0],
            a_remap[1] + GRAVITY_VEC[1],
            a_remap[2] + GRAVITY_VEC[2],
        )
        a_world, clamped = clamp_vec_norm(a_world, ACCEL_CLAMP_MPS2)
        if clamped:
            self._n_clamped += 1

        w_body = (msg.angular_velocity.x,
                  msg.angular_velocity.y,
                  msg.angular_velocity.z)
        w = remap3(w_body, self.axis_spec)

        self.f.write(
            f"{rel_t:.6f},"
            f"{a_world[0]:.6f},{a_world[1]:.6f},{a_world[2]:.6f},"
            f"{w[0]:.6f},{w[1]:.6f},{w[2]:.6f}\n"
        )
        self._n += 1
        if self._n % 500 == 0:
            self.f.flush()
            self.get_logger().info(f"[{self.label}] {self._n}개 샘플 (t={rel_t:.2f}s)")

    def finish(self):
        self.f.flush()
        self.f.close()
        self.get_logger().info(
            f"[{self.label}] 총 {self._n}개 샘플 저장 완료 "
            f"(그중 {self._n_clamped}개는 {ACCEL_CLAMP_MPS2}m/s^2로 스파이크 클램프됨)")


def main(args=None):
    prefix = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_OUTPUT_PREFIX
    off_path = f"{prefix}_off.csv"
    on_path = f"{prefix}_on.csv"

    rclpy.init(args=args)
    gate = ActivationGate()
    off_node = AccelRecorder(BASE_IMU_TOPIC, BODY2_CURRENT_BASE, off_path, "off_base", gate)
    on_node = AccelRecorder(TRAY_IMU_TOPIC, CAD_ROTATED_TRAY, on_path, "on_tray", gate)

    executor = SingleThreadedExecutor()
    executor.add_node(gate)
    executor.add_node(off_node)
    executor.add_node(on_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        off_node.finish()
        on_node.finish()
        gate.destroy_node()
        off_node.destroy_node()
        on_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
