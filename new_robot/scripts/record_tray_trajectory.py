#!/usr/bin/env python3
"""
record_tray_trajectory.py

Gazebo 시뮬레이션에서 물탱크(water_tank 링크)의 월드 기준 위치+자세를
시간에 따라 기록해서, DualSPHysics의 외부 파일 기반 강제 모션(motion
from file) 입력으로 쓸 수 있는 형식으로 저장한다.

동작 방식:
  gz-sim-pose-publisher-system 플러그인(new_robot.urdf에 추가됨)이
  /model/new_robot/pose 로 모든 링크의 월드 pose를 gz.msgs.Pose_V로
  퍼블리시하고, launch 파일에서 이걸 tf2_msgs/msg/TFMessage로 브릿지한다.
  이 스크립트는 그중 TARGET_FRAME(기본값 water_tank)에 해당하는 transform만
  골라서 기록한다.

출력 형식 (공백 구분, 헤더 없음):
    time  dx  dy  dz  roll_deg  pitch_deg  yaw_deg

  dx/dy/dz는 기록 시작 시점(첫 프레임)의 위치를 원점으로 뺀 상대 변위다.
  GenCase에서 물탱크를 이미 원하는 위치에 배치해뒀을 테니, 모션 파일은
  "거기서부터 얼마나 움직였는지"만 표현하면 되기 때문이다.

⚠️ DualSPHysics 버전마다 외부 모션 파일을 참조하는 GenCase XML의 정확한
   태그/속성명(예: <mvfile>, fieldx, fieldang1 등)이 다를 수 있다. 설치된
   DualSPHysics의 examples/ 폴더에서 motion 관련 예제 케이스를 찾아 실제
   문법을 확인하고 GenCase XML에 반영할 것 — 여기서는 데이터 포맷까지만
   책임진다.

사용법:
    python3 record_tray_trajectory.py [출력파일경로]

Ctrl+C로 멈추면 지금까지 기록한 내용을 남기고 종료한다.
"""

import csv
import math
import sys

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

TARGET_FRAME = "new_robot/default_4"  # water_tank는 default_4에 fixed조인트로
# 합쳐져서(URDF->SDF 변환 시 fixed조인트 링크가 부모로 lumping됨) 별도 프레임으로
# 안 나오므로, 물리적으로 완전히 같이 움직이는 default_4(롤 짐벌 링크)를 대신 추적한다.
DEFAULT_OUTPUT_PATH = "tray_motion.dat"
POSE_TOPIC = "/model/new_robot/pose"


def quat_to_euler_deg(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class TrayTrajectoryRecorder(Node):
    def __init__(self, output_path: str):
        super().__init__("record_tray_trajectory")
        self.sub = self.create_subscription(TFMessage, POSE_TOPIC, self._on_pose, 50)
        self.csv_file = open(output_path, "w", newline="")
        self.writer = csv.writer(self.csv_file, delimiter=" ")
        self._t0 = None
        self._origin = None
        self._n_written = 0
        self._seen_frames = set()
        self.get_logger().info(
            f"'{TARGET_FRAME}' 프레임 기록 시작 (토픽: {POSE_TOPIC}) -> {output_path}"
        )

    def _on_pose(self, msg: TFMessage):
        for tf in msg.transforms:
            self._seen_frames.add(tf.child_frame_id)
            if tf.child_frame_id != TARGET_FRAME:
                continue
            t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            pos = tf.transform.translation
            if self._t0 is None:
                self._t0 = t
                self._origin = (pos.x, pos.y, pos.z)
                self.get_logger().info(f"기준 위치 고정: {self._origin}")

            rel_t = t - self._t0
            dx = pos.x - self._origin[0]
            dy = pos.y - self._origin[1]
            dz = pos.z - self._origin[2]
            q = tf.transform.rotation
            roll, pitch, yaw = quat_to_euler_deg(q.x, q.y, q.z, q.w)

            self.writer.writerow([
                f"{rel_t:.4f}", f"{dx:.6f}", f"{dy:.6f}", f"{dz:.6f}",
                f"{roll:.4f}", f"{pitch:.4f}", f"{yaw:.4f}",
            ])
            self._n_written += 1
            if self._n_written % 200 == 0:
                self.csv_file.flush()
                self.get_logger().info(f"{self._n_written}개 샘플 기록됨 (t={rel_t:.2f}s)")

    def finish(self):
        self.csv_file.flush()
        self.csv_file.close()
        self.get_logger().info(f"총 {self._n_written}개 샘플 저장 완료")
        if self._n_written == 0:
            self.get_logger().warn(
                f"'{TARGET_FRAME}' 프레임을 한 번도 못 찾았습니다. "
                f"실제로 들어온 프레임 이름들: {sorted(self._seen_frames)}"
            )


def main(args=None):
    output_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_OUTPUT_PATH
    rclpy.init(args=args)
    node = TrayTrajectoryRecorder(output_path)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        # Ctrl+C 도중 rclpy가 처리 중이던 메시지 변환을 RuntimeError로 던지는
        # 경우가 있는데, 정상적인 인터럽트와 동일하게 취급하고 넘어간다.
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()