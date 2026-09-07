#!/usr/bin/env python3
"""
gimbal_leveling_controller.py — ROS2 / Gazebo 어댑터

gimbal_control_core.ControlCore를 가제보에 연결하는 얇은 계층이다. 제어 수식은
이 파일에 없다. 전부 코어에 있고, 여기서는 토픽 <-> 데이터 계약 변환만 한다.
실물에서는 이 파일 대신 SPI/CAN 어댑터를 쓰고 코어는 그대로 재사용한다.

실행
    # 같은 폴더에 gimbal_control_core.py 가 있어야 한다. 기본 프리셋
    # (base=body2_current_base, tray=cad_rotated_tray)이 현재 URDF에
    # 맞춰져 있으므로 축 정합 파라미터는 따로 안 줘도 된다.
    python3 gimbal_leveling_controller.py --ros-args -p use_sim_time:=true

    # URDF가 바뀌어 identity(센서가 이미 REP-103대로 정렬됨)로 돌려야 할 때
    python3 gimbal_leveling_controller.py --ros-args -p use_sim_time:=true \
        -p base_axis_preset:=identity -p tray_axis_preset:=identity

토픽
    구독  /imu           sensor_msgs/Imu     하단 차체 (root 링크)
          /imu_tray      sensor_msgs/Imu     상단 트레이 (default_4 링크)
          /joint_states  sensor_msgs/JointState  선택. 모터 피드백 + 진자각 로깅
          /gimbal_enable std_msgs/Bool       활성/대기 전환
    발행  /gimbal_roll_cmd   std_msgs/Float64  revolute_1 목표각 [rad]
          /gimbal_pitch_cmd  std_msgs/Float64  revolute_2 목표각 [rad]
          /gimbal_state      std_msgs/String   상태기계 상태 + 사유

축 정합 (중요)
    현재 new_robot.urdf는 스폰 시 -R -1.5708 -P 1.5708 로 회전시켜 세우고,
    base IMU(chassis_imu)는 root가 아니라 body2 링크에 <sensor> 정렬 없이
    붙어 있어 그 회전이 센서 프레임에 그대로 남는다. 코어는 REP-103(Z 상방)을
    가정하므로 그대로 두면 목표각이 엉킨다.

    근본 해결은 URDF의 <sensor>에 <pose>를 넣어 센서를 정렬하는 것이다.
    그 전까지는 이 노드의 *_axis_preset 파라미터로 소프트웨어 리맵을 걸며,
    기본값을 현재 URDF에 맞춘 body2_current_base/cad_rotated_tray로
    두었다(2026-09-05, 가제보 headless 실행으로 실측 확인). 노드 시작 시
    자동 진단을 출력하므로 URDF가 바뀌면 그 로그로 다시 확인할 것.
"""

import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64, String

from gimbal_control_core import (
    GRAVITY, ControlCore, ControlState, CoreConfig, ImuPair, ImuSample,
    MotorFeedback, compute_slosh_modes, compute_slosh_modes_rect, housner_pendulum,
)

# ---------------------------------------------------------------------------
# 축 리맵 — (출력축) <- (입력축 인덱스, 부호)
#
# URDF의 <sensor><pose>로 정합을 맞추면 identity를 쓰면 된다. 아래 cad_* 는
# 현재(2026-08) new_robot.urdf 상태에서 측정/계산한 값이다.
#   base: 센서 +X -> world -Z,  +Y -> -X,  +Z -> +Y
#   tray: 센서 +X -> world -Z,  +Y -> +X,  +Z -> -Y
# 두 IMU의 순열이 서로 다르므로 프리셋도 따로 둔다.
# ---------------------------------------------------------------------------
AXIS_PRESETS = {
    "identity": ((0, 1.0), (1, 1.0), (2, 1.0)),
    "cad_rotated_base": ((1, -1.0), (2, 1.0), (0, -1.0)),
    "cad_rotated_tray": ((1, 1.0), (2, -1.0), (0, -1.0)),
    # main 브랜치의 new_robot.urdf는 chassis_imu가 root가 아니라 body2 링크에
    # 붙어 있어(<gazebo reference="body2">) cad_rotated_base가 맞지 않는다.
    # root->body2 순정기구학(스폰 R=-90 P=90 포함)으로 재계산한 값:
    #   body2 +X -> world +X,  +Y -> world -Z,  +Z -> world +Y
    # 정지 실측값(0.006, -9.80, 0.006)을 대입하면 (0.006, 0.006, 9.80)으로
    # 정상 매핑된다.
    "body2_current_base": ((0, 1.0), (2, 1.0), (1, -1.0)),
}


def remap3(v, spec):
    return (spec[0][1] * v[spec[0][0]],
            spec[1][1] * v[spec[1][0]],
            spec[2][1] * v[spec[2][0]])


def diagnose_axes(ax, ay, az):
    """정지 상태 가속도계 읽음값으로 축 정합 상태를 진단한다."""
    v = (ax, ay, az)
    mag = math.sqrt(ax * ax + ay * ay + az * az)
    if mag < 1e-6:
        return "가속도 0 — 센서 미수신?"
    idx = max(range(3), key=lambda i: abs(v[i]))
    axis = "XYZ"[idx]
    sign = "+" if v[idx] > 0 else "-"
    if idx == 2:
        return (f"정상 (중력이 Z축, {sign}{abs(v[2]):.2f} m/s^2, |a|={mag:.2f})")
    return (f"어긋남: 중력이 {axis}축에 실림 ({sign}{abs(v[idx]):.2f} m/s^2, "
            f"|a|={mag:.2f}). REP-103이면 Z에 있어야 한다. "
            f"URDF <sensor><pose>로 정합하거나 *_axis_preset을 설정할 것")


class GimbalLevelingController(Node):

    def __init__(self):
        super().__init__("gimbal_leveling_controller")

        # --- 파라미터 ---------------------------------------------------
        p = self.declare_parameter
        p("control_hz", 100.0)
        p("base_imu_topic", "/imu")
        p("tray_imu_topic", "/imu_tray")
        p("joint_states_topic", "/joint_states")
        p("roll_cmd_topic", "/gimbal_roll_cmd")
        p("pitch_cmd_topic", "/gimbal_pitch_cmd")
        # 2026-09-05: 기본값을 identity에서 실측 확정 프리셋으로 변경.
        # 현재 new_robot.urdf는 chassis_imu가 root가 아니라 body2 링크에,
        # tray_imu가 default_4 링크에 <sensor><pose> 정렬 없이 붙어 있어
        # identity로는 중력이 Z가 아니라 다른 축에 실린다(축 진단 경고 참조).
        # Gazebo 실행 중 실측(가제보 headless 실행, /imu, /imu_tray 원시값
        # 확인)으로 base=body2_current_base가 중력을 정확히 +Z로 옮기는
        # 것을 확인했다. tray는 정지 상태 값만으로는 cad_rotated_base와
        # cad_rotated_tray가 둘 다 우연히 Z축에 실려 구분이 안 되므로,
        # 파일 상단 docstring에 이미 기록된 기하학적 유도값(cad_rotated_tray)을
        # 그대로 채택했다. 근본 해결은 URDF <sensor><pose> 정렬이며, 이
        # 프리셋들은 그 전까지의 소프트웨어 우회다.
        p("base_axis_preset", "body2_current_base")
        p("tray_axis_preset", "cad_rotated_tray")
        p("roll_joint_name", "revolute_1")
        p("pitch_joint_name", "revolute_2")
        # Housner 등가 진자 조인트 이름. URDF에 추가되면 로깅에 사용된다.
        p("slosh_joint_names", ["slosh_pendulum_x", "slosh_pendulum_y"])
        # 시뮬레이션 편의. 실물 어댑터는 반드시 False여야 한다(팀 안전 원칙).
        p("auto_activate", True)
        p("log_csv_path", "")
        p("log_hz", 50.0)
        # 코어 설정 중 실험에서 자주 바꾸는 것만 노출
        p("tank_radius_m", 0.055)
        p("fill_height_m", 0.090)
        # "cylinder"(기본) | "rect". rect일 때 tank_side_m 사용, tank_radius_m은
        # 무시된다. 실물 탱크(11x11cm, 350mL) 재현: tank_shape:=rect
        # tank_side_m:=0.11 fill_height_m:=0.02893
        p("tank_shape", "cylinder")
        p("tank_side_m", 0.11)
        p("enable_feedforward", True)
        p("enable_zv", True)
        p("enable_gain_scheduling", True)
        p("enable_trim", True)
        p("accel_lpf_cutoff_hz", 1.2)
        p("enable_integral", True)
        p("ki_trim", 0.01)
        p("kp_min", 0.10)
        p("kp_max", 0.50)
        # 목표각 지수평활 계수 — 값이 작을수록 느리고 부드럽다(슬루율 완화에
        # 해당). 기본값은 원본 pid_control_parkver 계승값(roll 0.10/pitch 0.30).
        p("smooth_alpha_roll", 0.10)
        p("smooth_alpha_pitch", 0.30)
        p("joint_limit_rad", 0.4363)
        p("require_motor_feedback", False)
        # 조인트각 0 == 트레이 완전 수평이 아닌 만큼(조립/CAD 기하 오프셋).
        # 2026-09-05 실측: 정지+ACTIVE 상태에서 tray_pitch_deg가 약 -3.39도,
        # tray_roll_deg가 약 -0.76도에 계속 고정되는 걸 발견. p_gain을
        # 3배(15->45) 올려도 값이 그대로라(-3.39도) 중력 처짐(정상상태
        # 오차)이 아니라 고정 기하 오프셋으로 판단, p_gain은 원복하고 대신
        # 여기서 보정한다.
        #
        # 2026-09-06 재보정: 위 1차 보정값을 적용한 뒤에도 tray_roll_deg가
        # -0.6833도로 계속 남는 걸 발견. 서로 다른 시각에 독립적으로 두 번
        # 실행해도 소수점 4자리까지 완전히 똑같은 값이 나와(물리엔진 노이즈
        # 아님, 100% 결정론적) 1차 측정값 자체가 부정확했던 것으로 확인.
        # raw_bias = 잔차 + 기존 보정값 역산으로 정확한 값을 다시 구함
        # (pitch도 같은 방식으로 잔차 0.1824도를 마저 흡수):
        #   roll:  raw = -0.6833° + (-0.7550°) = -1.4383° -> -0.025103 rad
        #   pitch: raw =  0.1824° + (-3.3897°) = -3.2076° -> -0.055983 rad
        # 다른 URDF/씬으로 바꾸면 diag 절차(README 참조)로 다시 재야 한다.
        p("tray_zero_offset_roll_rad", -0.025103)
        p("tray_zero_offset_pitch_rad", -0.055983)
        # 2026-09-06: 실측으로 발견한 폐루프 버그 진단/수정용 플래그.
        # gimbal_control_core.CoreConfig.apply_tray_zero_offset_in_error 참조.
        p("apply_tray_zero_offset_in_error", True)

        # --- HW 스타일 경로 (실물 Liquid_Control_Robot 시리얼 튜닝 명령 대응) ---
        p("enable_hw_style", False)
        p("gain_horiz", 1.00)        # [g]  GAIN
        p("gain_accel", 1.00)        # [ag] ACC_GAIN
        p("force_lpf_alpha", 0.20)   # [ad] ACC_LPF
        p("dir_accel", 1.0)          # DIR_ACC — TODO: 실측 필요
        p("cmd_limit_deg", 45.0)     # LIMIT_DEG
        p("acc_ref_limit_deg", 45.0)  # ACC_REF_LIMIT
        p("act_limit_deg", 55.0)     # ACT_LIMIT_DEG
        p("cmd_slew_deg_s", 120.0)   # [r] MAX_RATE 실측 확정 120°/s
        p("cmd_lpf_alpha", 0.30)     # [f] CMD_LPF
        p("enable_zv_hw", True)      # [zv]
        p("zv_hw_freq_hz", 2.00)     # [zf]
        p("zv_hw_mode", 2)           # [zm] 2=ZV, 3=ZVD
        p("enable_soft_return", False)  # [sr] — True면 enable_zv_hw는 자동 OFF
        p("soft_return_active_deg", 1.50)   # [sra]
        p("soft_return_end_deg", 0.75)      # [sre]
        p("soft_return_dwell_ms", 30.0)     # [srd]
        p("soft_return_hold_ms", 100.0)     # [srh]
        p("soft_return_return_ms", 120.0)   # [srr]
        p("motor_kp_pitch", 4.0)     # [kpp]
        p("motor_kd_pitch", 0.20)    # [kdp]
        p("motor_kp_roll", 2.0)      # [kpr]
        p("motor_kd_roll", 0.13)     # [kdr]
        p("motor_ff_j_pitch", 0.0038)  # [fj]
        p("motor_ff_j_roll", 0.0)      # [fjr]

        g = lambda n: self.get_parameter(n).value  # noqa: E731

        hz = float(g("control_hz"))
        self.dt_nominal = 1.0 / hz

        self.base_preset_name = g("base_axis_preset")
        self.tray_preset_name = g("tray_axis_preset")
        self.base_spec = AXIS_PRESETS.get(self.base_preset_name,
                                          AXIS_PRESETS["identity"])
        self.tray_spec = AXIS_PRESETS.get(self.tray_preset_name,
                                          AXIS_PRESETS["identity"])
        self.roll_joint = g("roll_joint_name")
        self.pitch_joint = g("pitch_joint_name")
        self.slosh_joints = list(g("slosh_joint_names"))
        self.auto_activate = bool(g("auto_activate"))

        # --- 코어 설정 --------------------------------------------------
        cfg = CoreConfig()
        cfg.nominal_dt_s = self.dt_nominal
        cfg.tank_radius_m = float(g("tank_radius_m"))
        cfg.fill_height_m = float(g("fill_height_m"))
        cfg.tank_shape = str(g("tank_shape"))
        cfg.tank_side_m = float(g("tank_side_m"))
        cfg.enable_feedforward = bool(g("enable_feedforward"))
        cfg.enable_zv = bool(g("enable_zv"))
        cfg.enable_gain_scheduling = bool(g("enable_gain_scheduling"))
        cfg.enable_trim = bool(g("enable_trim"))
        cfg.accel_lpf_cutoff_hz = float(g("accel_lpf_cutoff_hz"))
        cfg.enable_integral = bool(g("enable_integral"))
        cfg.ki_trim = float(g("ki_trim"))
        cfg.kp_min = float(g("kp_min"))
        cfg.kp_max = float(g("kp_max"))
        cfg.smooth_alpha_roll = float(g("smooth_alpha_roll"))
        cfg.smooth_alpha_pitch = float(g("smooth_alpha_pitch"))
        cfg.joint_limit_rad = float(g("joint_limit_rad"))
        cfg.require_motor_feedback = bool(g("require_motor_feedback"))
        cfg.tray_zero_offset_roll_rad = float(g("tray_zero_offset_roll_rad"))
        cfg.tray_zero_offset_pitch_rad = float(g("tray_zero_offset_pitch_rad"))
        cfg.apply_tray_zero_offset_in_error = bool(g("apply_tray_zero_offset_in_error"))

        cfg.enable_hw_style = bool(g("enable_hw_style"))
        cfg.gain_horiz = float(g("gain_horiz"))
        cfg.gain_accel = float(g("gain_accel"))
        cfg.force_lpf_alpha = float(g("force_lpf_alpha"))
        cfg.dir_accel = float(g("dir_accel"))
        cfg.cmd_limit_rad = math.radians(float(g("cmd_limit_deg")))
        cfg.acc_ref_limit_rad = math.radians(float(g("acc_ref_limit_deg")))
        cfg.act_limit_rad = math.radians(float(g("act_limit_deg")))
        cfg.cmd_slew_rads = math.radians(float(g("cmd_slew_deg_s")))
        cfg.cmd_lpf_alpha = float(g("cmd_lpf_alpha"))
        cfg.enable_zv_hw = bool(g("enable_zv_hw"))
        cfg.zv_hw_freq_hz = float(g("zv_hw_freq_hz"))
        cfg.zv_hw_mode = int(g("zv_hw_mode"))
        cfg.enable_soft_return = bool(g("enable_soft_return"))
        cfg.soft_return_active_deg = float(g("soft_return_active_deg"))
        cfg.soft_return_end_deg = float(g("soft_return_end_deg"))
        cfg.soft_return_dwell_ms = float(g("soft_return_dwell_ms"))
        cfg.soft_return_hold_ms = float(g("soft_return_hold_ms"))
        cfg.soft_return_return_ms = float(g("soft_return_return_ms"))
        cfg.motor_kp_pitch = float(g("motor_kp_pitch"))
        cfg.motor_kd_pitch = float(g("motor_kd_pitch"))
        cfg.motor_kp_roll = float(g("motor_kp_roll"))
        cfg.motor_kd_roll = float(g("motor_kd_roll"))
        cfg.motor_ff_j_pitch = float(g("motor_ff_j_pitch"))
        cfg.motor_ff_j_roll = float(g("motor_ff_j_roll"))

        self.core = ControlCore(cfg)
        self.core.init_control()

        # --- 상태 -------------------------------------------------------
        self._base = ImuSample()
        self._tray = ImuSample()
        self._motor_roll = MotorFeedback()
        self._motor_pitch = MotorFeedback()
        self._slosh_angles = {}
        self._last_step_ns = None
        self._axis_report_done = False
        self._last_state = None
        self._log = None
        self._log_decim = max(1, int(round(hz / max(1.0, float(g("log_hz"))))))
        self._tick = 0

        # --- 통신 -------------------------------------------------------
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(Imu, g("base_imu_topic"), self._on_base, sensor_qos)
        self.create_subscription(Imu, g("tray_imu_topic"), self._on_tray, sensor_qos)
        self.create_subscription(JointState, g("joint_states_topic"),
                                 self._on_joints, 10)
        self.create_subscription(Bool, "/gimbal_enable", self._on_enable, 10)

        self.pub_roll = self.create_publisher(Float64, g("roll_cmd_topic"), 10)
        self.pub_pitch = self.create_publisher(Float64, g("pitch_cmd_topic"), 10)
        self.pub_state = self.create_publisher(String, "/gimbal_state", 10)

        self._open_log(str(g("log_csv_path")))
        self._announce(cfg)
        self.create_timer(self.dt_nominal, self._on_timer)

    # -- 시작 안내 -------------------------------------------------------

    def _announce(self, cfg: CoreConfig):
        log = self.get_logger()
        if cfg.tank_shape == "rect":
            f1, f2 = compute_slosh_modes_rect(cfg.tank_side_m, cfg.fill_height_m)
        else:
            f1, f2 = compute_slosh_modes(cfg.tank_radius_m, cfg.fill_height_m)
        log.info("=" * 64)
        log.info("짐벌 레벨링 제어 시작")
        log.info(f"  제어주기      {1.0/self.dt_nominal:.1f} Hz")
        if cfg.tank_shape == "rect":
            log.info(f"  탱크          rect L={cfg.tank_side_m*1000:.1f}mm "
                     f"h={cfg.fill_height_m*1000:.1f}mm")
            log.info(f"  슬로싱 모드   f1={f1:.3f}Hz  f2={f2:.3f}Hz")
            log.info("  등가 진자     rect 탱크는 물리 진자 모델(housner_pendulum)"
                     " 미구현 — URDF slosh_pendulum은 원통 근사값을 그대로 씀")
        else:
            h = housner_pendulum(cfg.tank_radius_m, cfg.fill_height_m)
            log.info(f"  탱크          cylinder R={cfg.tank_radius_m*1000:.1f}mm "
                     f"h={cfg.fill_height_m*1000:.1f}mm")
            log.info(f"  슬로싱 모드   f1={f1:.3f}Hz  f2={f2:.3f}Hz")
            log.info(f"  등가 진자     m1={h['m1_kg']:.4f}kg  L={h['length_m']*1000:.2f}mm  "
                     f"피벗={h['pivot_height_m']*1000:.2f}mm")
        log.info(f"  플래그        FF={cfg.enable_feedforward} ZV={cfg.enable_zv} "
                 f"GS={cfg.enable_gain_scheduling} TRIM={cfg.enable_trim} "
                 f"I={cfg.enable_integral}")
        log.info(f"  가속도 LPF   {cfg.accel_lpf_cutoff_hz:.2f}Hz "
                 f"(목표 법선 계산 전 노이즈 저역통과)")
        log.info(f"  적분 게인    ki={cfg.ki_trim:.3f} "
                 f"(지속되는 정상상태 기울기 제거용)")
        log.info(f"  게인          kp {cfg.kp_min:.3f}~{cfg.kp_max:.3f} "
                 f"(실물에서 재튜닝 대상)")
        log.info(f"  조인트 한계   +-{math.degrees(cfg.joint_limit_rad):.2f}deg")
        log.info(f"  모터 피드백   요구={cfg.require_motor_feedback}")
        log.info(f"  HW 스타일     {cfg.enable_hw_style} "
                 f"(g={cfg.gain_horiz:.2f} ag={cfg.gain_accel:.2f} "
                 f"zf={cfg.zv_hw_freq_hz:.2f}Hz sr={cfg.enable_soft_return} "
                 f"limit=+-{math.degrees(cfg.cmd_limit_rad):.0f}deg "
                 f"slew={math.degrees(cfg.cmd_slew_rads):.0f}deg/s)")
        if not self.get_parameter("use_sim_time").value:
            log.warn("  use_sim_time=false — 가제보와 함께 쓸 때는 true로 두세요")
        if self.auto_activate:
            log.warn("  auto_activate=true (시뮬레이션 편의). "
                     "실물 어댑터에서는 반드시 false로 두고 /gimbal_enable을 쓸 것")
        log.info("=" * 64)

    # -- 콜백 ------------------------------------------------------------

    def _imu_to_sample(self, msg: Imu, spec):
        ax, ay, az = remap3((msg.linear_acceleration.x,
                             msg.linear_acceleration.y,
                             msg.linear_acceleration.z), spec)
        gx, gy, gz = remap3((msg.angular_velocity.x,
                             msg.angular_velocity.y,
                             msg.angular_velocity.z), spec)
        t_us = msg.header.stamp.sec * 1_000_000 + msg.header.stamp.nanosec // 1000
        if t_us == 0:
            t_us = self._now_us()
        return ImuSample(ax_mps2=ax, ay_mps2=ay, az_mps2=az,
                         gx_rads=gx, gy_rads=gy, gz_rads=gz,
                         sampled_at_us=t_us, valid=True)

    def _on_base(self, msg: Imu):
        self._base = self._imu_to_sample(msg, self.base_spec)
        if not self._axis_report_done:
            self._axis_report_done = True
            raw = (msg.linear_acceleration.x, msg.linear_acceleration.y,
                   msg.linear_acceleration.z)
            self.get_logger().info(f"[축 진단] base 원시값: {diagnose_axes(*raw)}")
            self.get_logger().info(
                f"[축 진단] base 리맵후({self.base_preset_name}): "
                f"{diagnose_axes(self._base.ax_mps2, self._base.ay_mps2, self._base.az_mps2)}")

    def _on_tray(self, msg: Imu):
        self._tray = self._imu_to_sample(msg, self.tray_spec)

    def _on_joints(self, msg: JointState):
        now_ms = self._now_us() // 1000
        for i, name in enumerate(msg.name):
            pos = msg.position[i] if i < len(msg.position) else 0.0
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            eff = msg.effort[i] if i < len(msg.effort) else 0.0
            if name == self.roll_joint:
                self._motor_roll = MotorFeedback(position_rad=pos, velocity_rads=vel,
                                                 torque_nm=eff, received_at_ms=now_ms,
                                                 valid=True)
            elif name == self.pitch_joint:
                self._motor_pitch = MotorFeedback(position_rad=pos, velocity_rads=vel,
                                                  torque_nm=eff, received_at_ms=now_ms,
                                                  valid=True)
            elif name in self.slosh_joints:
                self._slosh_angles[name] = pos

    def _on_enable(self, msg: Bool):
        if msg.data:
            self.core.request_activate()
            self.get_logger().info("[활성화 요청] ACTIVE 전환 요청됨")
        else:
            self.core.request_standby()
            self.get_logger().info("[대기 요청] READY로 복귀")

    # -- 주기 실행 --------------------------------------------------------

    def _now_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def _on_timer(self):
        now_ns = self.get_clock().now().nanoseconds
        if self._last_step_ns is None:
            self._last_step_ns = now_ns
            return
        dt = (now_ns - self._last_step_ns) * 1e-9
        self._last_step_ns = now_ns

        if self.auto_activate:
            self.core.request_activate()

        now_us = now_ns // 1000
        imu = ImuPair(base=self._base, tray=self._tray)
        out = self.core.step(imu, self._motor_roll, self._motor_pitch,
                             dt, now_us, now_us // 1000)

        if out.enable:
            self.pub_roll.publish(Float64(data=out.x_position_rad))
            self.pub_pitch.publish(Float64(data=out.y_position_rad))

        d = self.core.diag
        if d.state != self._last_state:
            self._last_state = d.state
            txt = ControlState(d.state).name
            if d.fault_reason:
                txt += " (" + d.fault_reason + ")"
            self.pub_state.publish(String(data=txt))
            self.get_logger().info(f"[상태] {txt}")

        self._tick += 1
        if self._log is not None and self._tick % self._log_decim == 0:
            self._write_log(now_ns * 1e-9, out)

    # -- 로깅 -------------------------------------------------------------

    LOG_COLUMNS = [
        "time_s", "state", "dt_s",
        "base_roll_deg", "base_pitch_deg",
        "tray_roll_deg", "tray_pitch_deg",
        "target_roll_deg", "target_pitch_deg",
        "shaped_roll_deg", "shaped_pitch_deg",
        "abs_target_roll_deg", "abs_target_pitch_deg",
        "error_roll_deg", "error_pitch_deg",
        "trim_roll_deg", "trim_pitch_deg",
        "kp_roll", "kp_pitch",
        "integral_roll_deg", "integral_pitch_deg",
        "cmd_roll_deg", "cmd_pitch_deg",
        "joint_roll_deg", "joint_pitch_deg",
        "slosh_x_deg", "slosh_y_deg",
        "zv_f1_hz", "zv_f2_hz", "freq_source", "enabled",
        # HW 스타일 경로 진단(analyze_log.py 비교용). enable_hw_style=False면
        # hw_style=0이고 나머지는 0/DIRECT로 고정된다.
        "hw_style", "force_roll_deg", "force_pitch_deg",
        "hw_shaped_roll_deg", "hw_shaped_pitch_deg",
        "soft_return_phase_roll", "soft_return_phase_pitch",
        "ff_velocity_roll_deg_s", "ff_velocity_pitch_deg_s",
        "ff_torque_roll_nm", "ff_torque_pitch_nm",
    ]

    def _open_log(self, path):
        if not path:
            return
        try:
            d = os.path.dirname(os.path.abspath(path))
            if d and not os.path.isdir(d):
                os.makedirs(d, exist_ok=True)
            self._log = open(path, "w", buffering=1)
            self._log.write(",".join(self.LOG_COLUMNS) + "\n")
            self.get_logger().info(f"[로깅] {path}")
        except OSError as exc:
            self._log = None
            self.get_logger().error(f"[로깅] 파일 열기 실패: {exc}")

    def _write_log(self, t_s, out):
        d = self.core.diag
        deg = math.degrees
        sx = self._slosh_angles.get(self.slosh_joints[0], float("nan")) \
            if len(self.slosh_joints) > 0 else float("nan")
        sy = self._slosh_angles.get(self.slosh_joints[1], float("nan")) \
            if len(self.slosh_joints) > 1 else float("nan")
        row = [
            f"{t_s:.4f}", ControlState(d.state).name, f"{d.dt_s:.5f}",
            f"{deg(d.base_roll_rad):.4f}", f"{deg(d.base_pitch_rad):.4f}",
            f"{deg(d.tray_roll_rad):.4f}", f"{deg(d.tray_pitch_rad):.4f}",
            f"{deg(d.target_roll_rad):.4f}", f"{deg(d.target_pitch_rad):.4f}",
            f"{deg(d.shaped_roll_rad):.4f}", f"{deg(d.shaped_pitch_rad):.4f}",
            f"{deg(d.abs_target_roll_rad):.4f}", f"{deg(d.abs_target_pitch_rad):.4f}",
            f"{deg(d.error_roll_rad):.4f}", f"{deg(d.error_pitch_rad):.4f}",
            f"{deg(d.trim_roll_rad):.4f}", f"{deg(d.trim_pitch_rad):.4f}",
            f"{d.kp_roll:.4f}", f"{d.kp_pitch:.4f}",
            f"{deg(d.integral_roll_rad):.4f}", f"{deg(d.integral_pitch_rad):.4f}",
            f"{deg(out.x_position_rad):.4f}", f"{deg(out.y_position_rad):.4f}",
            f"{deg(self._motor_roll.position_rad):.4f}",
            f"{deg(self._motor_pitch.position_rad):.4f}",
            f"{deg(sx):.4f}" if sx == sx else "nan",
            f"{deg(sy):.4f}" if sy == sy else "nan",
            f"{d.zv_f1_hz:.4f}", f"{d.zv_f2_hz:.4f}", d.slosh_freq_source,
            "1" if out.enable else "0",
            "1" if d.hw_style_active else "0",
            f"{deg(d.force_roll_rad):.4f}", f"{deg(d.force_pitch_rad):.4f}",
            f"{deg(d.hw_shaped_roll_rad):.4f}", f"{deg(d.hw_shaped_pitch_rad):.4f}",
            d.soft_return_phase_roll, d.soft_return_phase_pitch,
            f"{deg(out.ff_velocity_roll_rads):.4f}", f"{deg(out.ff_velocity_pitch_rads):.4f}",
            f"{out.ff_torque_roll_nm:.4f}", f"{out.ff_torque_pitch_nm:.4f}",
        ]
        self._log.write(",".join(row) + "\n")

    def destroy_node(self):
        if self._log is not None:
            self._log.close()
            self._log = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GimbalLevelingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
