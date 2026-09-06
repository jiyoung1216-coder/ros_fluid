#!/usr/bin/env python3
"""
gimbal_leveling_controller.py

2026-09-05: 이 노드의 로직(ActivityGate/SloshEstimator1D 등)은
sim_control/gimbal_control_core.py + sim_control/gimbal_leveling_controller.py로
이전되었습니다(플랫폼 독립 코어 + 얇은 ROS2 어댑터 구조, HW 스타일 경로
포함). 신규 실험은 그쪽을 사용하세요. 이 파일 삭제 여부는 보류 중입니다.

pid_control_parkver/Control.cpp (ESP32 펌웨어)의 제어 알고리즘을
ROS2(rclpy) + Gazebo Harmonic(gz sim) 시뮬레이션용으로 이식한 노드.

원본과의 차이점 (가제보_연동_확인사항_답변.md 기준):
  - 액추에이터가 이미 위치 컨트롤러(gz-sim-joint-position-controller-system,
    내부 PID p=15.0 i=0 d=0.75)이므로, 여기서는 "목표 각도(rad)"만 계산해서
    /gimbal_roll_cmd, /gimbal_pitch_cmd로 보낸다. 원래 실제 짐벌 모터
    motor_test_MIT.ino 값(MOTOR_KP=1.0/MOTOR_KD=0.05)을 그대로 넣었더니
    sim의 CAD 트레이+짐벌 조립체 무게를 못 버텨서 정지 상태에서도 조인트가
    한계(±25도)까지 처졌다(2026-08-31 실측). 실제 모터값은 실물 부하 기준
    튜닝이라 sim 부하가 다르면 그대로 못 씀 — p_gain만 올려 비율(d/p=0.05)은
    유지한 채 중력을 버틸 강성을 확보했다.
  - ▶ 추가: motor_test_MIT.ino 상단 경고("MIT 모드는 속도 제한이 없다 — 계단
    명령 금지")를 반영한 출력단 하드 슬루레이트 리미터(MAX_CMD_RATE_DEG_S).
    실제 MIT 구동은 τ=Kp*오차 라서 목표각이 한 번에 크게 튀면 그 순간
    포화토크가 그대로 걸린다. OUTPUT_SMOOTH(지수 저역통과)만으로는 오차가
    클 때 첫 스텝의 순간 변화율이 이 한도를 넘을 수 있어, 최종 발행 직전에
    한 번 더 하드 캡을 건다. 상한 150deg/s는 motor_test_MIT.ino의
    SWEEP_RATE_MAX(시리얼 다이얼인 상한, 2026-08-31 갱신 — 이전 60에서 상향)
    값을 그대로 가져왔다.
  - 원본 Phase 4(모터 피드백 기반 게인 스케줄링 PID)는 그대로 옮기지 않고,
    tray IMU가 있을 때만 활성화되는 "보정 trim"으로 역할을 바꿨다.
  - orientation 쿼터니언의 "노드 시작 시점 값을 0으로 잡고 상대 회전만 사용"
    보정은 팀 쪽 기존 프로토타입 방식을 그대로 따라했다.
  - ▶ 추가: 실시간 물 출렁임(Housner) 추정 보정 레이어. tray IMU가 겪는
    초과가속도(baseline 대비 차이)를 스프링-질량-댐퍼 방정식(1차 슬로싱
    모드)에 실시간으로 흘려서 예상 출렁임을 추정하고, 그만큼을 목표각에
    더해준다. tray IMU가 없으면 항상 0이라 원본과 동일하게 동작한다.
  - ▶ 수정: 원본 Control.cpp는 roll만 관성보상/데드존을 건너뛰고
    -roll_filtered를 그대로 ZV에 넣는 비대칭 구조였는데, 정지 상태에서도
    roll이 과민반응(촐랑거림)하는 원인이라 pitch와 동일하게 스무딩+데드존을
    거치도록 대칭으로 고쳤다.
  - ▶ 추가: 최종 출력(roll_cmd, pitch_cmd)에 저역통과 필터(OUTPUT_SMOOTH)를
    한 번 더 씌워서, 어떤 레이어에서 온 노이즈든 급격한 반응 없이 부드럽게
    나가도록 했다.
  - ▶ 추가: 활성/대기 게이트(ActivityGate). 정지 상태에서도 슬로싱
    추정기(SloshEstimator1D)가 감쇠비 1%짜리 거의 무감쇠 공진기라서,
    트레이 IMU의 순수 센서 노이즈만으로도 계속 링잉하며 짐벌을 흔드는
    문제가 있었다. cmd_vel(주행 명령) + 실측 자이로/가속도 크기를
    히스테리시스로 판정해서, 정지 상태(gate≈0)에서는 슬로싱 추정 레이어를
    완전히 끄고(추정기 내부 상태도 0으로 리셋) 실제 주행/외란이 감지될
    때만 부드러운 램프로 게인을 올려 활성화한다.
  - ▶ 수정: Phase 2 데드존을 하드 컷(임계값 이하는 즉시 0으로 점프)에서
    연속적인 데드밴드(임계값만큼 빼는 방식)로 바꿨다. 하드 컷은 값이
    임계값 근처에서 흔들릴 때 0으로 순간 점프하는 불연속을 만들고, 이게
    ZV 셰이퍼에 계단 입력을 넣는 것과 같은 효과를 내서 정지 상태에서도
    미세한 촐랑거림의 원인이 됐었다.
  - ▶ 전면 수정(2026-08-31): 실제 하드웨어 영상으로 확인된 요구 거동(①
    병진 없이 회전만 하면 반대방향 롤 뱅킹 ② 등속 병진은 수평 ③ 가속/감속
    병진은 충격 완화 방향으로 기울임 ④ 초기상태 수평)에 맞춰 Phase 2를
    base IMU 원시가속도 대신 오도메트리(root 프레임) 기반 운동학
    추정치(v, omega, dv/dt)로 재작성했다. base IMU가 붙은 body2 링크는
    CAD 임포트 과정에서 생긴 고정 장착 회전 때문에 로컬 축이 REP-103과
    안 맞아(정지 상태에서도 중력 전체가 로컬 Y축에서 읽힘) 원시가속도
    기반 보상이 근본적으로 왜곡됐었다 — 오도메트리는 이미 표준 축이라
    이 문제가 없다.
  - ▶ 추가(2026-08-31): 차체 자세 레벨링. 위 Phase 2(스핀뱅킹/충격완화)는
    "움직임"에만 반응해서, 주행 없이 차체 자체가 (지형이든 Gazebo에서
    수동으로 기울이든) 기울어져 있으면 트레이가 그걸 상쇄하는 기능이
    없었다. Phase1에서 이미 계산하던 roll_filtered/pitch_filtered(base
    IMU 상보필터)를 재사용해서 "차체가 기운 만큼 반대로 돌려 항상 월드
    수평 유지"를 구현했다. gate와 무관하게 항상 켜짐(정지 상태에서도
    차체가 기울어 있으면 계속 보정해야 하므로).

⚠️ 반드시 시뮬레이션에서 직접 확인/조정해야 하는 부분
  - "축 부호 설정" 블록의 부호(+-1).
  - BASE_LEVEL_ROLL_SIGN / BASE_LEVEL_PITCH_SIGN: 차체를 기울였을 때
    트레이가 반대로 도는 게 맞는지 눈으로 보고 확인. 반대로 돌면(더
    기울어지면) 해당 부호를 뒤집을 것.
  - KP_MIN_DEG / KP_MAX_DEG: tray IMU 추가 후 재튜닝 필요.
  - K_SLOSH: 물 출렁임 보정 게인. 부호(+-)와 크기 모두 튜닝 필요.
  - OUTPUT_SMOOTH: 낮출수록 부드럽지만 반응이 느려짐. 0.15부터 시작해서 조정.
  - ACTIVE_ENTER_*/ACTIVE_EXIT_*: 활성/대기 게이트 진입·이탈 임계값.
    실제 로봇 정지 시 센서 노이즈 크기, 주행 시 최소 속도값에 맞춰 조정.
  - SLOSH_DAMPING_RATIO(0.2로 상향)/SLOSH_FORCING_DEADBAND: 슬로싱
    추정기가 노이즈에 링잉하지 않도록 하는 값. 튜닝 필요.
  - K_SPIN_BANK_DEG_PER_RADS / SPIN_BANK_SIGN: 제자리 회전 시 뱅킹 각도와
    방향(반대방향이 맞는지 실제로 눈으로 보고 부호 확인). V_FADE_MS는
    "병진 없음"으로 볼 속도 상한.
  - ACCEL_EST_ALPHA: 오도메트리 미분 저역통과 계수. 낮추면 부드럽지만
    가감속 반응이 느려짐.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

# ---------------------------------------------------------------------------
# 상수 (Config.h / Control.cpp 원본값 이식)
# ---------------------------------------------------------------------------
G = 9.80665  # m/s^2

ALPHA_BASE = 0.98
ALPHA_TRAY = 0.995
DT = 0.01  # 100Hz 제어 주기

ZV_BUFFER_SIZE = 40
DELTA_IDX = 21
DELTA_IDX_SMALL = 11

DEADZONE_PITCH_DEG = 2.0
DEADZONE_ROLL_DEG = 3.0

PITCH_SMOOTH_NEW = 0.3
ROLL_SMOOTH_NEW = 0.1

KP_MIN_DEG = 0.05
KP_MAX_DEG = 0.30
ERROR_MAX_DEG = 15.0

# trim/슬로싱 보정은 베이스 레벨링 목표각(shaped_roll/pitch) 위에 "얹히는"
# 값이라, 업스트림(트레이 IMU 드리프트 등)이 어떤 이상한 값을 내놓든
# 이 폭 이상으로는 목표각을 밀어붙이지 못하도록 상한을 둔다. 이게 없으면
# trim/슬로싱 계산값이 폭주할 때 목표각이 조인트 리밋(JOINT_LIMIT_RAD)까지
# 끝까지 밀려서 짐벌이 한계각에 짓눌린 채 계속 진동하는 문제가 있었다.
TRIM_MAX_DEG = 5.0
SLOSH_MAX_DEG = 5.0

JOINT_LIMIT_RAD = 0.4363  # ±25°

OUTPUT_SMOOTH = 0.05  # 최종 출력 저역통과 필터. 0에 가까울수록 부드럽고 느림

# 실제 짐벌 모터(motor_test_MIT.ino)의 SWEEP_RATE_MAX(시리얼 다이얼인 상한,
# 2026-08-31 기준 150deg/s). MIT 모드는 자체 속도 제한이 없어(τ=Kp*오차)
# 목표각을 이보다 빨리 움직이면 실물에서 폭주/기구 튕김 위험이 있다.
# 시뮬레이션에도 동일 한도를 강제해서, 여기서 검증한 제어상수(ZV, 데드존,
# 게인 등)가 실물에서도 그대로 안전하게 통하도록 한다.
MAX_CMD_RATE_DEG_S = 150.0

# ---------------------------------------------------------------------------
# 축 부호 설정 — 시뮬레이션에서 실제로 기울여보고 검증/조정할 것
# ---------------------------------------------------------------------------
GYRO_ROLL_SIGN = 1.0
GYRO_PITCH_SIGN = 1.0
ACC_ROLL_SIGN = 1.0
ACC_PITCH_SIGN = 1.0

# Phase 2(관성 보상)는 base IMU 원시 가속도 대신 오도메트리 기반 운동학
# 추정치를 쓴다 — chassis_imu가 붙은 body2 링크가 CAD(Onshape) 임포트
# 과정에서 생긴 고정 장착 회전 때문에 로컬 축이 REP-103(Z-up)과 안 맞아서
# (정지 상태에서도 중력 9.8 m/s^2 전체가 로컬 Y축에서 읽힘), 원시
# linear_acceleration을 직접 roll/pitch에 매핑하면 그 장착 회전만큼
# 왜곡된 값이 나온다. 반면 /model/new_robot/odometry(root 프레임)는
# 이미 표준 프레임이라 이 문제가 없다.
V_FADE_MS = 0.15  # 이 속도[m/s] 밑에서는 "병진 없음"으로 보고 스핀뱅킹을 최대로 튼다
K_SPIN_BANK_DEG_PER_RADS = 15.0  # 제자리 회전 시 반대방향으로 기울이는 세기(deg per rad/s)
SPIN_BANK_SIGN = 1.0
ACCEL_EST_ALPHA = 0.15  # 오도메트리 미분(전후 가속도 추정) 저역통과 계수

# 차체 자세 레벨링 — 주행/회전 여부와 무관하게, 차체가 (지형이든 수동
# 조작이든) 기울어진 만큼 트레이가 반대로 돌아서 항상 월드 바닥과 수평을
# 유지해야 한다는 요구사항(2026-08-31). Phase1의 roll_filtered/
# pitch_filtered(base IMU 상보필터, base_q_rel 기반)를 그대로 재사용.
# gate로 안 묶고 항상 켜둔다 — 정지 상태에서도(등속 주행 중 언덕 경사로
# 차체가 기운 경우 포함) 차체가 기울어 있으면 계속 보정해야 하니까.
# BASE_LEVEL_GAIN=1.0은 "측정된 기울기만큼 정확히 반대로" — 부호가
# 반대로 나오면 *_SIGN을 뒤집을 것.
#
# 2026-08-31: body2 링크가 CAD 임포트 과정에서 축이 꼬여있어(정지 상태
# 에서도 중력 9.8 m/s^2 전체가 로컬 Y축에서 읽힘 — 로컬 Y가 실제
# 수직축) quat_to_roll_pitch_deg(Z-up 가정)를 그대로 쓰면 90~180도급
# 오작동이 났다. BASE_LEVEL_FIX_Q로 로컬 프레임을 X축 -90도 회전시켜
# (standard_X=body2_X, standard_Y=-body2_Z, standard_Z=body2_Y) "로컬
# Z가 수직"인 표준 프레임으로 바꾼 뒤에 roll/pitch를 뽑는다. 그래도
# 어느 쪽이 진짜 앞/옆인지, 부호가 맞는지는 실측으로 확인 필요 — 확인
# 전까지는 GAIN을 낮게, MAX_DEG로 상한을 걸어 혹시 축이 또 틀려도
# 조인트가 폭주하지 않게 한다.
BASE_LEVEL_FIX_Q = (-0.70710678, 0.0, 0.0, 0.70710678)  # X축 -90도
BASE_LEVEL_GAIN = 0.5
BASE_LEVEL_MAX_DEG = 20.0
BASE_LEVEL_ROLL_SIGN = 1.0
BASE_LEVEL_PITCH_SIGN = 1.0

BASE_IMU_TOPIC = "/imu"
TRAY_IMU_TOPIC = "/imu_tray"
ROLL_CMD_TOPIC = "/gimbal_roll_cmd"
PITCH_CMD_TOPIC = "/gimbal_pitch_cmd"
CMD_VEL_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/model/new_robot/odometry"

# ---------------------------------------------------------------------------
# 물 출렁임(Housner) 실시간 추정 관련 상수 — 원본 Control.cpp에는 없음, 추가 레이어
# ---------------------------------------------------------------------------
TANK_RADIUS = 0.06
FILL_HEIGHT = 0.09
# 정지 상태에서 순수 센서 노이즈로도 거의 감쇠 없이 계속 링잉하던 게
# 정지 시 짐벌 촐랑거림의 주 원인이었다. 물리적으로 정확한 값(원래 0.01)
# 대신, 보정 신호가 빨리 잦아들도록 의도적으로 높인 "제어용" 감쇠비.
SLOSH_DAMPING_RATIO = 0.2
# 이 이하의 초과가속도(센서 노이즈 수준)는 추정기에 아예 입력하지 않는다.
SLOSH_FORCING_DEADBAND = 0.15  # m/s^2
K_SLOSH = 0.15
LAMBDA1 = 1.8412

# ---------------------------------------------------------------------------
# 활성/대기 게이트 — 정지 상태에서는 슬로싱 추정(반응형 보정)을 억제하고,
# 실제 주행 명령(cmd_vel)이나 측정 외란(자이로/가속도)이 감지될 때만
# 활성화한다. 진입/이탈 임계값을 다르게 둬서(히스테리시스) 임계값 부근에서
# 게이트가 자주 껐다 켜졌다 하는 채터링을 막고, 게인 자체도 지수 램프로
# 부드럽게 움직인다.
# ---------------------------------------------------------------------------
ACTIVE_ENTER_CMDVEL_LIN = 0.03  # m/s
ACTIVE_ENTER_CMDVEL_ANG = 0.05  # rad/s
ACTIVE_EXIT_CMDVEL_LIN = 0.01
ACTIVE_EXIT_CMDVEL_ANG = 0.02

# 정지 상태 실측 기준(2026-08-20 세션): gyro_deg 1.5~18deg/s, acc_mag
# 15~90(가끔 200대 스파이크) 수준의 잔여 물리 노이즈가 있음을 확인.
# 그 노이즈 위에 확실히 걸리도록 임계값을 재조정했다 — 물리적으로 완전히
# 0을 만드는 것보다, 이 노이즈 바닥 위에서 게이트가 안정적으로 꺼지게
# 하는 쪽을 택함.
ACTIVE_ENTER_GYRO_DEG = 25.0  # deg/s, base 자이로 rate 크기
ACTIVE_EXIT_GYRO_DEG = 15.0

ACTIVE_ENTER_ACC = 250.0  # m/s^2, base 가속도(x,y) 크기
ACTIVE_EXIT_ACC = 150.0

GATE_ATTACK_TAU = 0.15  # s — 외란 감지 시 빠르게 활성화
GATE_RELEASE_TAU = 0.6  # s — 정지로 판단되면 완만하게 대기 상태로 복귀

# 노드 시작 직후 로봇이 스폰 낙하/착지 충격으로 흔들리는 동안 IMU 값을
# baseline(기준 자세)으로 잘못 고정해버리는 걸 막기 위한 정착 대기 시간.
# 이 시간 동안은 baseline을 잡지 않고 대기만 한다.
SETTLE_TIME_SEC = 2.0


def sloshing_parameters(radius=TANK_RADIUS, fill_height=FILL_HEIGHT):
    R, h = radius, fill_height
    omega1 = math.sqrt((LAMBDA1 * G / R) * math.tanh(LAMBDA1 * h / R))
    x = LAMBDA1 * h / R
    h1_over_h = 1 - (math.cosh(x) - 1) / (x * math.sinh(x))
    h1 = h1_over_h * h
    return omega1, h1


class SloshEstimator1D:
    """단일 축 방향 Housner 등가 진자 실시간 추정기 (RK4, 매 틱 1스텝)."""

    def __init__(self, omega1, h1, damping_ratio=SLOSH_DAMPING_RATIO):
        self.omega1 = omega1
        self.h1 = h1
        self.c1 = 2 * damping_ratio * omega1
        self.xi = 0.0
        self.xi_dot = 0.0

    def _deriv(self, xi, xi_dot, a_forcing):
        return xi_dot, -self.omega1 ** 2 * xi - self.c1 * xi_dot - a_forcing

    def update(self, a_forcing, dt=DT):
        x0, v0 = self.xi, self.xi_dot
        k1x, k1v = self._deriv(x0, v0, a_forcing)
        k2x, k2v = self._deriv(x0 + 0.5 * dt * k1x, v0 + 0.5 * dt * k1v, a_forcing)
        k3x, k3v = self._deriv(x0 + 0.5 * dt * k2x, v0 + 0.5 * dt * k2v, a_forcing)
        k4x, k4v = self._deriv(x0 + dt * k3x, v0 + dt * k3v, a_forcing)
        self.xi = x0 + (dt / 6.0) * (k1x + 2 * k2x + 2 * k3x + k4x)
        self.xi_dot = v0 + (dt / 6.0) * (k1v + 2 * k2v + 2 * k3v + k4v)
        return self.xi / self.h1  # 등가 보정각 (rad)


def slew_limit(target: float, prev: float, max_delta: float) -> float:
    """prev에서 target 방향으로 한 스텝에 max_delta 이상 못 움직이게 자른다.

    motor_test_MIT.ino의 `target += dir * step` 램핑(계단 명령 금지)을
    그대로 이식한 하드 리미터. 지수 저역통과(OUTPUT_SMOOTH)는 오차가 클 때
    첫 스텝의 순간 변화율을 못 막지만, 이건 물리적으로 절대 못 넘는다.
    """
    delta = target - prev
    if delta > max_delta:
        delta = max_delta
    elif delta < -max_delta:
        delta = -max_delta
    return prev + delta


def apply_deadband(value: float, threshold: float) -> float:
    """threshold 이하는 0, 그 이상은 끊김 없이 통과시키는 연속 데드밴드.

    (기존의 하드 컷 `abs(x) < threshold -> 0`은 x가 threshold 근처에서
    흔들릴 때 0으로 순간 점프하는 불연속을 만든다. 그 불연속이 ZV
    셰이퍼에 계단 입력을 넣는 것과 같은 효과를 내서 정지 상태에서도
    미세한 촐랑거림을 유발했다.)
    """
    if value > threshold:
        return value - threshold
    if value < -threshold:
        return value + threshold
    return 0.0


def quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_to_roll_pitch_deg(q):
    """REP-103 기준 roll(X축 회전)/pitch(Y축 회전)을 degree로 반환."""
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    return math.degrees(roll), math.degrees(pitch)


class ConvolvedZV:
    """Control.cpp의 applyConvolvedZV() 그대로 이식 (4-tap, 가중치 0.25씩)."""

    def __init__(self):
        self.buffer = [0.0] * ZV_BUFFER_SIZE
        self.head = 0

    def apply(self, target: float) -> float:
        buf = self.buffer
        idx0 = self.head
        idx1 = (self.head - DELTA_IDX) % ZV_BUFFER_SIZE
        idx2 = (self.head - DELTA_IDX_SMALL) % ZV_BUFFER_SIZE
        idx3 = (self.head - (DELTA_IDX + DELTA_IDX_SMALL)) % ZV_BUFFER_SIZE

        buf[self.head] = target
        shaped = 0.25 * buf[idx0] + 0.25 * buf[idx1] + 0.25 * buf[idx2] + 0.25 * buf[idx3]
        self.head = (self.head + 1) % ZV_BUFFER_SIZE
        return shaped


class ActivityGate:
    """cmd_vel + 측정 IMU 기반 히스테리시스 활성/대기 게이트.

    정지 상태(cmd_vel과 실측 자이로/가속도가 모두 이탈 임계값 이하)에서는
    게인이 0으로 수렴해 슬로싱 추정 보정을 끄고, 주행 명령이나 실제
    외란이 진입 임계값을 넘으면 게인이 빠르게(GATE_ATTACK_TAU) 1로 올라가
    보정을 켠다. 다시 조용해지면 GATE_RELEASE_TAU로 천천히 대기 상태로
    복귀한다.
    """

    def __init__(self):
        self.active = False
        self.gain = 0.0

    def update(self, cmd_lin: float, cmd_ang: float, gyro_deg: float,
               acc_mag: float, dt: float) -> float:
        enter = (abs(cmd_lin) > ACTIVE_ENTER_CMDVEL_LIN
                 or abs(cmd_ang) > ACTIVE_ENTER_CMDVEL_ANG
                 or gyro_deg > ACTIVE_ENTER_GYRO_DEG
                 or acc_mag > ACTIVE_ENTER_ACC)
        stay_idle = (abs(cmd_lin) < ACTIVE_EXIT_CMDVEL_LIN
                     and abs(cmd_ang) < ACTIVE_EXIT_CMDVEL_ANG
                     and gyro_deg < ACTIVE_EXIT_GYRO_DEG
                     and acc_mag < ACTIVE_EXIT_ACC)

        if not self.active and enter:
            self.active = True
        elif self.active and stay_idle:
            self.active = False

        target = 1.0 if self.active else 0.0
        tau = GATE_ATTACK_TAU if target > self.gain else GATE_RELEASE_TAU
        alpha = dt / (tau + dt)
        self.gain += alpha * (target - self.gain)
        return self.gain


class GimbalLevelingController(Node):
    def __init__(self):
        super().__init__("gimbal_leveling_controller")

        self.base_sub = self.create_subscription(Imu, BASE_IMU_TOPIC, self._on_base_imu, 50)
        self.tray_sub = self.create_subscription(Imu, TRAY_IMU_TOPIC, self._on_tray_imu, 50)
        self.cmd_sub = self.create_subscription(Twist, CMD_VEL_TOPIC, self._on_cmd_vel, 10)
        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)

        self.roll_pub = self.create_publisher(Float64, ROLL_CMD_TOPIC, 10)
        self.pitch_pub = self.create_publisher(Float64, PITCH_CMD_TOPIC, 10)

        self._latest_base = None
        self._latest_tray = None
        self._base_baseline_q = None
        self._tray_baseline_q = None
        self._tray_seen = False
        self._latest_cmd_lin = 0.0
        self._latest_cmd_ang = 0.0
        self.activity_gate = ActivityGate()
        self._start_time = self.get_clock().now()

        # 오도메트리(root 프레임) 기반 운동 상태 — Phase 2 관성보상의 입력
        self._odom_v = 0.0
        self._odom_omega = 0.0
        self._odom_seen = False
        self._prev_odom_v = 0.0
        self._accel_fwd_filt = 0.0

        # Phase 1 상태
        self.pitch_filtered = 0.0
        self.roll_filtered = 0.0
        self.tray_pitch_filtered = 0.0
        self.tray_roll_filtered = 0.0

        # Phase 2 상태
        self.internal_pitch = 0.0
        self.internal_roll = 0.0

        self.zv_pitch = ConvolvedZV()
        self.zv_roll = ConvolvedZV()

        # 출력단 저역통과 필터 상태
        self.roll_cmd_filtered = 0.0
        self.pitch_cmd_filtered = 0.0

        # 출력단 하드 슬루레이트 리미터 상태 (실제로 발행되는 최종값)
        self.roll_cmd_out = 0.0
        self.pitch_cmd_out = 0.0

        # 물 출렁임 실시간 추정
        self._tray_accel0 = None
        self._tray_excess = (0.0, 0.0, 0.0)
        omega1, h1 = sloshing_parameters()
        self.get_logger().info(f"Housner: omega1={omega1:.3f} rad/s, h1={h1:.4f} m")
        self.slosh_roll = SloshEstimator1D(omega1, h1)
        self.slosh_pitch = SloshEstimator1D(omega1, h1)

        self.timer = self.create_timer(DT, self.run_control_step)
        self.get_logger().info(
            "gimbal_leveling_controller 시작 (base IMU만 사용, tray IMU 대기 중)"
        )

    # -- 콜백 ----------------------------------------------------------------
    def _elapsed_sec(self) -> float:
        return (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

    def _on_cmd_vel(self, msg: Twist):
        self._latest_cmd_lin = msg.linear.x
        self._latest_cmd_ang = msg.angular.z

    def _on_odom(self, msg: Odometry):
        self._odom_v = msg.twist.twist.linear.x
        self._odom_omega = msg.twist.twist.angular.z
        self._odom_seen = True

    def _on_base_imu(self, msg: Imu):
        self._latest_base = msg
        if self._base_baseline_q is None and self._elapsed_sec() >= SETTLE_TIME_SEC:
            q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            self._base_baseline_q = quat_conjugate(q)

    def _on_tray_imu(self, msg: Imu):
        self._latest_tray = msg
        if self._tray_baseline_q is None:
            if self._elapsed_sec() < SETTLE_TIME_SEC:
                return
            q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            self._tray_baseline_q = quat_conjugate(q)
            self._tray_accel0 = (msg.linear_acceleration.x, msg.linear_acceleration.y,
                                  msg.linear_acceleration.z)
            self._tray_seen = True
            self.get_logger().info("tray IMU 정착 완료 — 이후부터 보정 trim + 슬로싱 추정 활성화")
            return

        a = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        self._tray_excess = tuple(a[i] - self._tray_accel0[i] for i in range(3))

    # -- 메인 루프 -------------------------------------------------------------
    def run_control_step(self):
        if self._latest_base is None or self._base_baseline_q is None:
            return

        base = self._latest_base
        base_q_rel = quat_multiply(self._base_baseline_q,
                                    (base.orientation.x, base.orientation.y,
                                     base.orientation.z, base.orientation.w))
        # body2 로컬 Y가 실제 수직축이라 표준(Z-up) roll/pitch 공식을 바로
        # 쓰면 안 된다 — BASE_LEVEL_FIX_Q로 로컬 Z가 수직인 프레임으로
        # 바꾼 뒤에 뽑는다. base_q_rel에 그냥 우측곱만 하면(이전 버그)
        # baseline=identity일 때도 결과가 BASE_LEVEL_FIX_Q 자체가 돼버려서
        # "안 움직였는데도 고정 편향(-90도)"이 생겼다. 컨쥬게이션
        # (conj(FIX)*rel*FIX)으로 해야 "그대로면 0"이 보존된다.
        base_q_rel_fixed = quat_multiply(
            quat_multiply(quat_conjugate(BASE_LEVEL_FIX_Q), base_q_rel),
            BASE_LEVEL_FIX_Q)
        # quat_to_roll_pitch_deg가 돌려주는 (표준_X 회전, 표준_Y 회전) 중
        # 표준_X(=body2_X, 그대로)가 실제로는 언덕/다리를 오를 때(차체
        # pitch) 반응하는 축임을 실측으로 확인(2026-08-31, ramp_bridge
        # 30도 경사 등판 중 roll_filtered만 ~30deg로 움직이고 pitch는
        # 안 움직임 — 즉 라벨이 뒤바뀌어 있었다). 그래서 여기서 미리
        # swap한다: "표준_X 회전"을 pitch로, "표준_Y 회전"을 roll로 쓴다.
        std_x_rot, std_y_rot = quat_to_roll_pitch_deg(base_q_rel_fixed)
        roll_acc = std_y_rot * ACC_ROLL_SIGN
        pitch_acc = std_x_rot * ACC_PITCH_SIGN

        # 자이로도 같은 축 대응으로 맞춘다: 표준_Y 회전속도 = -body2_Z,
        # 표준_X 회전속도 = body2_X(그대로).
        rate_roll = -math.degrees(base.angular_velocity.z) * GYRO_ROLL_SIGN
        rate_pitch = math.degrees(base.angular_velocity.x) * GYRO_PITCH_SIGN

        # Phase 1: 하단 상보필터
        self.pitch_filtered = ALPHA_BASE * (self.pitch_filtered + rate_pitch * DT) \
            + (1.0 - ALPHA_BASE) * pitch_acc
        self.roll_filtered = ALPHA_BASE * (self.roll_filtered + rate_roll * DT) \
            + (1.0 - ALPHA_BASE) * roll_acc

        # Phase 2: 관성 보상 목표각 + ZV
        #
        # 실제 하드웨어 영상 기준 요구 거동(2026-08-31):
        #   1) 병진 없이 회전만 할 때 -> 회전 반대방향으로 롤 뱅킹
        #   2) 등속 병진 -> 수평 유지
        #   3) 가속/감속 병진 -> 충격 완화(출렁임 최소화) 방향으로 기울임
        #   4) 초기 상태 -> 수평
        #
        # base IMU 원시 가속도(linear_acceleration)로 직접 구현하려 했으나,
        # chassis_imu가 붙은 body2 링크가 CAD(Onshape) 임포트 과정에서 생긴
        # 고정 장착 회전 때문에 로컬 축이 REP-103(Z-up, X-forward)과 맞지
        # 않는다 — 완전 정지 상태에서도 중력 9.8 m/s^2 전부가 로컬 Y축에서
        # 읽히고(Z가 아님), 지면 접촉 진동 노이즈까지 겹쳐서 base_q_rel을
        # 통한 중력 보정으로는 신뢰할 만한 "초과가속도"를 못 얻었다(실측
        # 확인됨). 대신 /model/new_robot/odometry(root 프레임, 이미 표준
        # 축)의 선속도/각속도라는 깨끗한 운동학 신호로 세 요구사항을 직접
        # 구현한다 — IMU 노이즈에 흔들리지 않고, 항상 실제 REP-103 forward/
        # lateral 축과 일치한다.
        v = self._odom_v
        omega = self._odom_omega

        accel_fwd_raw = (v - self._prev_odom_v) / DT
        self._prev_odom_v = v
        self._accel_fwd_filt = ACCEL_EST_ALPHA * accel_fwd_raw \
            + (1.0 - ACCEL_EST_ALPHA) * self._accel_fwd_filt

        # 좌우(원심) 가속도 추정 — 등속 코너링 시의 실제 물리(v*omega),
        # v=0일 때는 자동으로 0이 되므로 아래 스핀뱅킹 항과 안 겹친다.
        a_lat_est = v * omega

        # 요구사항 1: 병진 속도가 거의 0인데 회전만 하면, 회전 반대방향으로
        # 롤을 최대치로 기울인다. |v|가 커질수록(=실제로 달리기 시작하면)
        # 부드럽게 꺼져서 위 a_lat_est(코너링) 항에 자리를 넘겨준다.
        fade = max(0.0, 1.0 - abs(v) / V_FADE_MS)
        spin_bank_deg = -SPIN_BANK_SIGN * K_SPIN_BANK_DEG_PER_RADS * omega * fade

        raw_target_roll = math.degrees(math.atan2(a_lat_est, G)) * ACC_ROLL_SIGN + spin_bank_deg
        raw_target_pitch = math.degrees(math.atan2(self._accel_fwd_filt, G)) * ACC_PITCH_SIGN

        self.internal_pitch = PITCH_SMOOTH_NEW * raw_target_pitch \
            + (1.0 - PITCH_SMOOTH_NEW) * self.internal_pitch
        self.internal_roll = ROLL_SMOOTH_NEW * raw_target_roll \
            + (1.0 - ROLL_SMOOTH_NEW) * self.internal_roll

        final_target_pitch = apply_deadband(self.internal_pitch, DEADZONE_PITCH_DEG)

        # roll도 pitch와 동일하게 스무딩+데드밴드를 거치도록 대칭으로 수정
        # (원본 Control.cpp는 -roll_filtered를 그대로 ZV에 넣는 비대칭
        # 구조였는데, 정지 상태에서도 roll이 과민반응하는 원인이었다)
        final_target_roll = apply_deadband(self.internal_roll, DEADZONE_ROLL_DEG)

        shaped_pitch = self.zv_pitch.apply(final_target_pitch)
        shaped_roll = self.zv_roll.apply(final_target_roll)

        target_pitch_deg = shaped_pitch
        target_roll_deg = shaped_roll

        # Phase 3+4: tray IMU 기반 보정 trim
        if self._tray_seen and self._latest_tray is not None:
            tray = self._latest_tray
            tray_q_rel = quat_multiply(self._tray_baseline_q,
                                        (tray.orientation.x, tray.orientation.y,
                                         tray.orientation.z, tray.orientation.w))
            tray_roll_acc, tray_pitch_acc = quat_to_roll_pitch_deg(tray_q_rel)

            tray_rate_roll = math.degrees(tray.angular_velocity.x) * GYRO_ROLL_SIGN
            tray_rate_pitch = math.degrees(tray.angular_velocity.y) * GYRO_PITCH_SIGN

            self.tray_pitch_filtered = ALPHA_TRAY * (self.tray_pitch_filtered + tray_rate_pitch * DT) \
                + (1.0 - ALPHA_TRAY) * tray_pitch_acc
            self.tray_roll_filtered = ALPHA_TRAY * (self.tray_roll_filtered + tray_rate_roll * DT) \
                + (1.0 - ALPHA_TRAY) * tray_roll_acc
            # 자이로를 거의 그대로 적분하는 필터(alpha=0.995)라, 물리적으로
            # 큰 각속도가 한동안 들어오면 실제 각도와 무관하게 계속 표류할
            # 수 있다. 짐벌 자체가 물리적으로 낼 수 있는 각도보다 훨씬 큰
            # 값으로는 못 가게 안전 범위로 묶어둔다.
            self.tray_pitch_filtered = max(-90.0, min(90.0, self.tray_pitch_filtered))
            self.tray_roll_filtered = max(-90.0, min(90.0, self.tray_roll_filtered))

            error_roll = shaped_roll - self.tray_roll_filtered
            ratio = max(0.0, min(1.0, abs(error_roll) / ERROR_MAX_DEG))
            kp_dyn = KP_MIN_DEG + (KP_MAX_DEG - KP_MIN_DEG) * ratio * ratio
            trim_roll = max(-TRIM_MAX_DEG, min(TRIM_MAX_DEG, kp_dyn * error_roll))
            target_roll_deg = shaped_roll + trim_roll

            error_pitch = shaped_pitch - self.tray_pitch_filtered
            trim_pitch = max(-TRIM_MAX_DEG, min(TRIM_MAX_DEG, kp_dyn * error_pitch))
            target_pitch_deg = shaped_pitch + trim_pitch

        # 활성/대기 게이트: 주행 명령(cmd_vel)이나 실측 자이로/가속도가
        # 임계값을 넘을 때만 게인이 1로 올라간다. 정지 상태에서는 0으로
        # 수렴해서 아래 슬로싱 추정 레이어를 사실상 꺼버린다.
        gyro_deg = math.hypot(rate_roll, rate_pitch)
        acc_mag = math.hypot(a_lat_est, self._accel_fwd_filt)
        gate = self.activity_gate.update(
            self._latest_cmd_lin, self._latest_cmd_ang, gyro_deg, acc_mag, DT)

        # 추가 레이어: 실시간 물 출렁임(Housner) 추정 보정
        # gate가 0에 가까우면(=정지) 추정기를 아예 돌리지 않고 내부 상태를
        # 0으로 유지한다 — 감쇠비를 올려도 완전한 무입력 상태를 보장하는
        # 편이 안전하고, 다시 활성화될 때 잔류 에너지 없이 깨끗하게 시작한다.
        if gate > 0.01:
            ex, ey, _ez = self._tray_excess
            ex = apply_deadband(ex, SLOSH_FORCING_DEADBAND)
            ey = apply_deadband(ey, SLOSH_FORCING_DEADBAND)
            roll_slosh_corr_rad = self.slosh_roll.update(ey)
            pitch_slosh_corr_rad = self.slosh_pitch.update(ex)
        else:
            self.slosh_roll.xi = self.slosh_roll.xi_dot = 0.0
            self.slosh_pitch.xi = self.slosh_pitch.xi_dot = 0.0
            roll_slosh_corr_rad = 0.0
            pitch_slosh_corr_rad = 0.0

        slosh_roll_deg = max(-SLOSH_MAX_DEG, min(SLOSH_MAX_DEG,
                             K_SLOSH * math.degrees(roll_slosh_corr_rad)))
        slosh_pitch_deg = max(-SLOSH_MAX_DEG, min(SLOSH_MAX_DEG,
                              K_SLOSH * math.degrees(pitch_slosh_corr_rad)))
        target_roll_deg += slosh_roll_deg
        target_pitch_deg += slosh_pitch_deg

        # 게이트를 동적 보정(스핀뱅킹/가감속 충격완화 + trim + 슬로싱)에
        # 적용한다. 정지/잔잔한 상태(gate≈0)에서는 이 성분들에 남아있는
        # 잔여 오차/드리프트가 있어도 0으로 수렴하고, 실제 주행/외란이
        # 감지되면(gate≈1) 원래대로 작동한다.
        target_roll_deg *= gate
        target_pitch_deg *= gate

        # 차체 자세 레벨링 — gate와 무관하게 항상 켜져 있다. 주행 여부와
        # 상관없이 차체가 (지형이든 수동 조작이든) 기울어져 있으면 트레이가
        # 그만큼 반대로 돌아서 항상 월드 바닥과 수평을 유지해야 하기 때문.
        # 축 보정이 혹시 틀려도 조인트가 폭주하지 않도록 상한을 건다.
        base_level_roll = max(-BASE_LEVEL_MAX_DEG, min(BASE_LEVEL_MAX_DEG,
            BASE_LEVEL_ROLL_SIGN * BASE_LEVEL_GAIN * (-self.roll_filtered)))
        base_level_pitch = max(-BASE_LEVEL_MAX_DEG, min(BASE_LEVEL_MAX_DEG,
            BASE_LEVEL_PITCH_SIGN * BASE_LEVEL_GAIN * (-self.pitch_filtered)))
        target_roll_deg += base_level_roll
        target_pitch_deg += base_level_pitch

        # 최종 출력: degree -> radian, 조인트 리밋 클램프 후 출력단 저역통과 필터
        roll_cmd_raw = max(-JOINT_LIMIT_RAD, min(JOINT_LIMIT_RAD, math.radians(target_roll_deg)))
        pitch_cmd_raw = max(-JOINT_LIMIT_RAD, min(JOINT_LIMIT_RAD, math.radians(target_pitch_deg)))

        self.roll_cmd_filtered = OUTPUT_SMOOTH * roll_cmd_raw \
            + (1.0 - OUTPUT_SMOOTH) * self.roll_cmd_filtered
        self.pitch_cmd_filtered = OUTPUT_SMOOTH * pitch_cmd_raw \
            + (1.0 - OUTPUT_SMOOTH) * self.pitch_cmd_filtered

        max_step_rad = math.radians(MAX_CMD_RATE_DEG_S) * DT
        self.roll_cmd_out = slew_limit(self.roll_cmd_filtered, self.roll_cmd_out, max_step_rad)
        self.pitch_cmd_out = slew_limit(self.pitch_cmd_filtered, self.pitch_cmd_out, max_step_rad)

        self.roll_pub.publish(Float64(data=self.roll_cmd_out))
        self.pitch_pub.publish(Float64(data=self.pitch_cmd_out))

        # 디버그: roll 쪽 각 단계 값을 분리해서 확인 (원인 파악용, 끝나면 지울 것)
        self.get_logger().info(
            f"gate={gate:.2f} cmd_lin={self._latest_cmd_lin:.2f} cmd_ang={self._latest_cmd_ang:.2f} "
            f"gyro_deg={gyro_deg:.2f} acc_mag={acc_mag:.2f} "
            f"roll_filtered={self.roll_filtered:.2f}deg "
            f"internal_roll={self.internal_roll:.2f}deg "
            f"shaped_roll={shaped_roll:.2f}deg "
            f"trim_roll={trim_roll if self._tray_seen else 0.0:.2f}deg "
            f"slosh_roll={slosh_roll_deg:.2f}deg "
            f"target_roll={target_roll_deg:.2f}deg",
            throttle_duration_sec=0.3)


def main(args=None):
    rclpy.init(args=args)
    node = GimbalLevelingController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()