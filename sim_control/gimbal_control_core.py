#!/usr/bin/env python3
"""
gimbal_control_core.py — 액체 화물 이송 로봇 짐벌 제어 코어 (플랫폼 독립)

이 파일은 ROS도 Arduino도 모른다. 센서 표본(ImuPair)과 모터 피드백
(MotorFeedback)을 받아 모터 목표(ControlOutput)를 돌려주는 순수 계산 계층이다.
따라서 아래 두 어댑터가 같은 코어를 공유할 수 있다.

    시뮬레이션 :  ROS2 토픽        <-> 이 코어   (gimbal_leveling_controller.py)
    실물       :  SPI IMU / CAN    <-> 이 코어   (firmware/gimbal_control/Control.cpp)

C++ 이식을 전제로 작성했다. 순수 float 연산, 고정 크기 배열, 동적 할당 없음,
예외 없음(성공/실패는 반환값), 모든 상수에 단위 주석. 클래스 -> struct/class,
tuple 반환 -> 출력 인자로 바꾸면 기계적으로 번역된다. 대응표는 README.md 참조.

좌표계 규약 (REP-103)
    X 전방, Y 좌측, Z 상방.
    자세는 yaw를 뺀 2자유도로만 표현하며 회전 순서는 R = Ry(pitch) * Rx(roll) 이다.
    이 순서를 고른 이유는 짐벌이 pitch(revolute_2) 바깥, roll(revolute_1) 안쪽인
    직렬 구조여서 기구와 표현이 일치하기 때문이다.
    2자유도 짐벌은 자기 법선축 회전(yaw)을 제어할 수 없으므로 yaw는 다루지 않는다.

단위 (팀 계약 COMMON_CONTEXT.md)
    각도 rad, 각속도 rad/s, 가속도 m/s^2, 토크 N*m, 시간 s.
    사람이 읽는 출력에서만 degree를 쓰고 필드명에 단위를 명시한다.

논문 수식 대응
    (1) 상보필터                -> ComplementaryFilter
    (2)(3) 합력 방향 목표각      -> resultant_normal_body / GimbalKinematics.inverse
    (4) Convolved ZV            -> ConvolvedZV
    (5) 비선형 게인 스케줄링 P   -> GainSchedule
"""

import math
from dataclasses import dataclass, field
from enum import IntEnum

GRAVITY = 9.80665  # m/s^2, 표준 중력

# ZV 버퍼는 고정 크기다(C++ 이식). 최저 허용 주파수에서 필요한 최대 지연을
# 담을 수 있어야 한다: N = pi / (w * dt). f=1.0Hz, dt=0.01s 이면 50샘플,
# 두 모드를 합치면 100샘플. 여유를 둬 128로 잡는다.
ZV_BUFFER_SIZE = 128

# set_estimated_slosh_frequency()가 받아들이는 범위. 이 밖의 값은 거부하고
# 고정 주파수로 폴백한다. 원통 탱크 1차 슬로싱은 통상 1~10Hz 대역이다.
SLOSH_FREQ_MIN_HZ = 1.0
SLOSH_FREQ_MAX_HZ = 20.0


# ---------------------------------------------------------------------------
# 공용 데이터 계약 (COMMON_CONTEXT.md의 struct와 필드/단위 일치)
# ---------------------------------------------------------------------------

@dataclass
class ImuSample:
    """IMU 1개의 한 표본. 가속도는 비력(specific force)이다 — 정지 상태에서도
    중력 반작용으로 크기 약 9.8이 잡힌다."""
    ax_mps2: float = 0.0
    ay_mps2: float = 0.0
    az_mps2: float = 0.0
    gx_rads: float = 0.0
    gy_rads: float = 0.0
    gz_rads: float = 0.0
    sampled_at_us: int = 0
    valid: bool = False


@dataclass
class ImuPair:
    base: ImuSample = field(default_factory=ImuSample)  # 하단 차체
    tray: ImuSample = field(default_factory=ImuSample)  # 상단 트레이


@dataclass
class MotorFeedback:
    position_rad: float = 0.0
    velocity_rads: float = 0.0
    torque_nm: float = 0.0
    driver_temperature_c: int = 0
    motor_temperature_c: int = 0
    error_code: int = 0
    received_at_ms: int = 0
    valid: bool = False


@dataclass
class ControlOutput:
    """모터에 보낼 명령. enable=False면 어댑터는 신규 명령을 보내지 않는다."""
    x_position_rad: float = 0.0   # roll  축 (revolute_1)
    y_position_rad: float = 0.0   # pitch 축 (revolute_2)
    max_velocity_rads: float = 0.0
    enable: bool = False

    # HW 스타일 경로 진단 전용(실물 MIT 피드포워드 v_des/t_ff에 대응).
    # Gazebo는 위치 PID로 구동되므로 이 값들은 어댑터가 발행하지 않는다 —
    # analyze_log.py로 실물 로그와 비교할 때만 쓴다. enable_hw_style=False면
    # 항상 0.0이다.
    ff_velocity_roll_rads: float = 0.0
    ff_velocity_pitch_rads: float = 0.0
    ff_torque_roll_nm: float = 0.0
    ff_torque_pitch_nm: float = 0.0


class ControlState(IntEnum):
    """CONTROL_INTEGRATION_TASK.md의 상태기계.

        BOOT -> SENSOR_CHECK -> MOTOR_CHECK -> READY -> ACTIVE
                             \\_______________________/
                                      -> FAULT

    READY에서 ACTIVE로 자동 전환하지 않는다. 반드시 외부 활성화 요청이 필요하다.
    """
    BOOT = 0
    SENSOR_CHECK = 1
    MOTOR_CHECK = 2
    READY = 3
    ACTIVE = 4
    FAULT = 5


# ---------------------------------------------------------------------------
# 설정 — 튜닝 상수를 한곳에 모은다 (실물 완성 후 조정할 항목)
# ---------------------------------------------------------------------------

@dataclass
class CoreConfig:
    # --- 제어 주기 ---
    nominal_dt_s: float = 0.010
    # 이 배수를 넘는 주기 이탈은 주기 누락으로 본다. 연속 누락이 한계를 넘으면
    # FAULT로 간다(팀 요구: 주기 누락 시 정상 명령을 계속 보내지 않는다).
    dt_tolerance_ratio: float = 3.0
    max_consecutive_cycle_faults: int = 5

    # --- 상보필터 (수식 1) ---
    # base는 차체 물리 변화를 기민하게 따라가야 하므로 낮게(가속도 반영 2%).
    # tray는 모터 구동 진동이 직접 전달되므로 높게(가속도 반영 0.5%).
    # 논문 3.1절 값 그대로.
    alpha_base: float = 0.98
    alpha_tray: float = 0.995

    # --- 목표각 평활 ---
    # 원본 pid_control_parkver의 지수평활 계수(pitch 0.3 / roll 0.1)를 계승.
    # 값이 작을수록 부드럽고 느리다. 튜닝 범위 0.05~0.5.
    # 주의: 이 계수의 차단주파수는 5~6Hz대라 슬로싱 공진(2.6~2.9Hz)을 거의
    # 못 거른다. 공진대 억제는 아래 accel_lpf_cutoff_hz가 담당한다.
    smooth_alpha_roll: float = 0.10
    smooth_alpha_pitch: float = 0.30

    # --- 목표 법선 저역통과 (신규) ---
    # base 가속도계 원시값(비력)으로 목표 법선을 뽑기 전에 거치는 저역통과
    # 필터의 차단주파수. 슬로싱 공진(f1, compute_slosh_modes 참조. 이 탱크
    # 치수에서 약 2.88Hz, 실측은 약 2.63Hz)보다 충분히 낮게 잡아 노면
    # 요철/진동 노이즈가 목표각에 직접 실리는 것을 막는다.
    # 상보필터(alpha_base)로 대체하지 않는 이유: 상보필터는 자이로 적분
    # 위주라 회전 중 원심가속도나 오르막 수직가속도까지 일부러 무시하도록
    # 설계돼 있다. 여기서는 그런 실제 주행 가속도는 그대로 살리고 순수
    # 고주파 노이즈만 잘라내야 하므로 원시 비력에 저역통과만 건다.
    # 너무 낮추면(<0.5Hz) 정상적인 조향/오르막 반응까지 느려지므로 사람이
    # 조향하는 대역(대략 0.2~1Hz)보다는 위, 슬로싱 공진보다는 아래로 둔다.
    # 0 이하면 필터 비활성(기존 동작과 동일).
    accel_lpf_cutoff_hz: float = 1.2

    # --- 데드밴드 (rad) ---
    # 수평 근처 미세 노이즈로 모터가 떠는 것을 막는다. 하드 컷이 아니라 연속
    # 데드밴드라서 임계값 부근에서 출력이 튀지 않는다.
    # 원본은 roll 3deg / pitch 2deg. 실물의 엔코더 분해능·유격을 보고 재조정.
    deadband_roll_rad: float = math.radians(3.0)
    deadband_pitch_rad: float = math.radians(2.0)

    # --- Convolved ZV (수식 4) ---
    # 탱크 형상에서 산출한 1·2차 슬로싱 모드. tank_radius_m/fill_height_m을
    # 실물 치수로 맞추면 주파수가 따라 계산된다(compute_slosh_modes 참조).
    # 주의: 원본의 지연 21/11샘플은 f1~2.38Hz(반경 약 80mm)에 대응한다.
    #       현재 시뮬레이션 탱크는 내반경 55mm(f1~2.88Hz)로 21% 어긋난다.
    #       실물 탱크 치수를 확인해 아래 두 값을 맞출 것.
    tank_radius_m: float = 0.055      # water_tank.stl 내반경 (외 60mm - 벽 5mm)
    fill_height_m: float = 0.090      # 정지 수위 (tank_shape="rect"면 side_m 기준 높이로 별도 설정할 것)
    slosh_damping_ratio: float = 0.01  # 저점성 액체 통상값. 실측으로 대체 권장

    # 탱크 형상 선택 (2026-09-05 추가). "cylinder"(기본, 위 tank_radius_m 사용)
    # | "rect"(정사각 단면, tank_side_m 사용, compute_slosh_modes_rect 참조).
    # 실물 HW 스타일의 zv_hw_freq_hz=2.00Hz는 실물의 사각탱크(11x11cm,
    # 350mL)에서 잰 값이라, 이 시뮬레이션의 원통 탱크(f1=2.877Hz, 44% 오차)
    # 보다 "rect" 모드(11cm, fill_height_m=0.02893 로 같이 설정 시 f1=2.194Hz,
    # 9.7% 오차)가 실물과 훨씬 가깝다. 9.7% 오차는 실물 짐벌 자체의
    # 기계적 유격/탄성 때문(ino 주석 실측 기록)이라 탱크 형상으로는 더
    # 못 줄인다. 기본값은 "cylinder"라 안 건드리면 기존 동작과 동일하다.
    tank_shape: str = "cylinder"
    tank_side_m: float = 0.11         # 실물 탱크 한 변 실측값. tank_shape="rect"일 때만 사용
    # 주파수 갱신 시 구/신 셰이퍼 출력을 이 시간 동안 교차 혼합해 불연속을 막는다.
    zv_crossfade_s: float = 0.20

    # --- 게인 스케줄링 P (수식 5) ---
    # 절대각 오차에 대한 보정 이득. 무차원(rad 오차 -> rad 조인트 보정).
    # 피드포워드(역기구학)가 주 경로이고 이건 잔차 보정이므로 1.0 미만이 정상.
    # 오차가 클 때 크게, 목표 근처에서 작게 해서 오버슈트와 미세 진동을 막는다.
    # 실물에서 유격·마찰을 보고 조정할 1순위 항목.
    kp_min: float = 0.10
    kp_max: float = 0.50
    error_max_rad: float = math.radians(15.0)
    # 보정항 상한. 피드포워드가 정상인데 트레이 IMU 드리프트 등으로 보정이
    # 폭주해 조인트 한계까지 밀리는 것을 막는다.
    trim_limit_rad: float = math.radians(5.0)

    # --- 트림 적분항 (신규) ---
    # 언덕처럼 오래 지속되는 기울기는 P(게인 스케줄링)만으로는 절대 0까지
    # 못 없앤다 — 오차가 작아질수록 kp도 같이 작아지도록 설계돼 있어서,
    # "보정력 = 남은 오차를 정확히 상쇄할 만큼"이 아니라 어중간한 지점에서
    # 균형을 이루며 멈춰버린다(정상상태 오차, P제어의 고전적 한계).
    # 오차를 시간에 대해 누적해서, 작아도 오래 남아있으면 계속 보정력을
    # 키우는 적분항으로 이 잔여 기울기를 마저 없앤다.
    # ki_trim 단위: rad 출력 / (rad*s) 누적오차. 와인드업 방지로 누적값
    # 자체를 (ki_trim * 누적값) <= trim_limit_rad 가 되도록 클램프한다.
    enable_integral: bool = True
    ki_trim: float = 0.01

    # --- 기구 한계 ---
    # URDF revolute_1/revolute_2의 limit과 일치시켜야 한다. 실물 확인 필요
    # (현재 값이 GL60II 실제 한계인지 임의값인지 미확인).
    joint_limit_rad: float = 0.4363    # +-25deg
    max_velocity_rads: float = 3.0     # URDF velocity limit과 동일

    # --- 기구 영점 보정 (rad) ---
    # 조인트각 0이 트레이 수평과 일치하지 않는 만큼. 시뮬레이션에서는 트레이
    # IMU가 탱크 기하 프레임과 정렬되면 0이지만, 실물에서는 조립 공차만큼
    # 반드시 존재한다. 무부하 수평 상태에서 측정해 채운다.
    tray_zero_offset_roll_rad: float = 0.0
    tray_zero_offset_pitch_rad: float = 0.0

    # --- 유효성 판정 ---
    sensor_timeout_us: int = 50_000     # 50ms. 100Hz 기준 5주기
    feedback_timeout_ms: int = 100
    # 비력 크기가 이 범위를 벗어나면 포화/이상으로 본다 (m/s^2)
    accel_norm_min_mps2: float = 2.0
    accel_norm_max_mps2: float = 40.0
    # 가속도 이상이 이 프레임 수만큼 연속으로 나야 FAULT로 간다. 접촉 솔버의
    # 1프레임짜리 순간 튐(실측: 100Hz에서 단일 샘플 30~50 m/s^2, 다음 프레임엔
    # 정상 복귀)까지 FAULT로 보지 않기 위함 — 실물의 진동성 노이즈에도 동일하게
    # 적용된다. IMU 무효/타임아웃은 디바운스 없이 즉시 FAULT로 간다(별개 문제).
    accel_fault_debounce_frames: int = 3
    # 실물은 True여야 한다(모터 피드백 없이 명령을 보내지 않는다).
    # 가제보의 gz-sim-joint-position-controller-system은 피드백을 주지 않으므로
    # 시뮬레이션 어댑터는 이 값을 False로 두고 MOTOR_CHECK를 건너뛴다.
    require_motor_feedback: bool = True

    # --- 기능 플래그 (보고서용 ON/OFF 비교) ---
    enable_feedforward: bool = True      # 역기구학 피드포워드
    enable_zv: bool = True               # Convolved ZV 입력성형
    enable_gain_scheduling: bool = True  # False면 kp_max 고정
    enable_trim: bool = True             # 트레이 IMU 절대각 보정

    # -------------------------------------------------------------------
    # HW 스타일 경로 (실물 Liquid_Control_Robot, zv_shaping_rtos.ino 실측값)
    # enable_hw_style=False(기본)면 위 경로가 그대로 쓰인다. True면 아래
    # 값들로 _compute_hw_style()이 실행된다. 2026-09-04/05 실측 확정.
    # -------------------------------------------------------------------
    enable_hw_style: bool = False

    # 합력·수평 게인 (zv_shaping_rtos.ino 183, 279행 [g]/[ag])
    gain_horiz: float = 1.00     # GAIN — 차체 기울기를 몇 % 상쇄할지
    gain_accel: float = 1.00     # ACC_GAIN — 합력 목표각을 얼마나 적용할지
    # 3축 합력벡터 저역통과 (281행 [ad] ACC_LPF). 각도가 아니라 벡터를 거른다.
    force_lpf_alpha: float = 0.20
    # 합력 목표각 부호. 실측 확정이나(DIR_ACC, 280행) IMU 장착 방향에 종속.
    # TODO: 실측 필요 — 시뮬레이션 IMU 장착 방향 기준으로 재확인 전까지 +1 가정.
    dir_accel: float = 1.0

    # 각도 상한 (184, 286, 397행 — 명령 없음, 코드에만 있는 값)
    cmd_limit_rad: float = math.radians(45.0)       # LIMIT_DEG, 기구 한계
    acc_ref_limit_rad: float = math.radians(45.0)   # ACC_REF_LIMIT, θ_ref 상한 = 1.0g
    act_limit_rad: float = math.radians(55.0)       # ACT_LIMIT_DEG, 폭주 FAULT 문턱

    # 지령 응답 (185, 187행 [r]/[f]). 150은 motor_test_MIT.ino의 별개
    # 오픈루프 스윕 테스트 상한(SWEEP_RATE_MAX)이며 여기 슬루값이 아니다.
    cmd_slew_rads: float = math.radians(120.0)  # MAX_RATE 실측 확정 120°/s
    cmd_lpf_alpha: float = 0.30                 # CMD_LPF 실측 확정. 0.35부터 3.85Hz 진동

    # ZV 입력성형 — 고정 주파수 단일 모드 (348~351행 [zv]/[zf]/[zm]).
    # 시뮬레이션 기존 ConvolvedZV(2모드, 감쇠비 slosh_damping_ratio)와는 별개 경로.
    enable_zv_hw: bool = True
    zv_hw_freq_hz: float = 2.00   # 실측 확정. 11x11cm 350mL, 카트 위 고정 트레이 실측
    # 실물 기본 ZV_ZETA=0.02이지만, 고정주파수 단일모드 포팅은 지시에 따라
    # zeta≈0으로 가정한다(slosh_damping_ratio는 이 계산에 쓰지 않는다).
    zv_hw_zeta: float = 0.0
    zv_hw_mode: int = 2           # 2=ZV(2임펄스), 3=ZVD(3임펄스)

    # 가속 종료 소프트복귀 (362~376행 [sr]/[sra]/[sre]/[srd]/[srh]/[srr]).
    # 켜지면 ZV(HW 경로 한정)는 자동 OFF — 둘 다 "잔류 흔들림 억제"가 목적이라 중복 금지.
    enable_soft_return: bool = False
    soft_return_active_deg: float = 1.50
    soft_return_end_deg: float = 0.75
    soft_return_dwell_ms: float = 30
    soft_return_hold_ms: float = 100
    soft_return_return_ms: float = 120

    # 모터 임피던스·피드포워드 (231~234, 261~263행). Gazebo는 계속 위치
    # PID로 구동되므로 이 값들은 명령을 만들지 않는다 — ControlOutput의
    # 진단 필드(ff_velocity_rads/ff_torque_nm)로만 노출해 실물 로그와
    # 비교하는 데 쓴다.
    motor_kp_pitch: float = 4.0    # 안쪽축(0x02) 실측 확정
    motor_kd_pitch: float = 0.20
    motor_kp_roll: float = 2.0     # 바깥축(0x01) — 실물에서도 "올려야 할 쪽"으로 기록됨
    motor_kd_roll: float = 0.13
    motor_ff_j_pitch: float = 0.0038   # kg·m², 안쪽 관성 실측 (Kp4/Kd0.2 로그 3개 회귀)
    motor_ff_j_roll: float = 0.0       # 바깥 관성 미측정 → 0 (v_des만 적용됨)
    motor_ff_tmax_nm: float = 1.5

    # 참고: 실물의 REJ_RUN_FAULT(피드백/IMU 무보정 연속 카운트, 442행)와
    # 피드백 타임아웃은 기존 CoreConfig의 accel_fault_debounce_frames(=3)와
    # feedback_timeout_ms(=100)가 이미 두 경로 공통으로 처리한다(사용자
    # 지시 "실물 안전값: ... 이미 있음" 참조) — HW 경로 전용 필드를 새로
    # 두지 않았다. HW 경로에서 유일하게 새로 필요한 것은 act_limit_rad
    # (실측 위치 한계) FAULT뿐이다.


# ---------------------------------------------------------------------------
# 회전 유틸 — 3x3 행렬을 tuple로 다룬다 (C++에서는 float[3][3])
# ---------------------------------------------------------------------------

def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def mat_mul(a, b):
    return tuple(
        tuple(a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] for j in range(3))
        for i in range(3)
    )


def mat_col_z(m):
    """행렬의 3번째 열 = 그 자세의 body z축을 기준 프레임에서 본 방향."""
    return (m[0][2], m[1][2], m[2][2])


def attitude_matrix(roll_rad, pitch_rad):
    """규약 R = Ry(pitch) * Rx(roll)."""
    return mat_mul(rot_y(pitch_rad), rot_x(roll_rad))


def normalize3(x, y, z):
    """단위벡터와 원래 크기를 함께 돌려준다. 크기가 0이면 (0,0,1)로 폴백."""
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-9:
        return 0.0, 0.0, 1.0, 0.0
    return x / n, y / n, z / n, n


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def normal_to_roll_pitch(nx, ny, nz):
    """단위 법선 -> (roll, pitch, ok). attitude_matrix()의 역변환.

    R = Ry(p)Rx(r) 일 때 body z축은
        n = ( sin(p)cos(r), -sin(r), cos(p)cos(r) )
    이므로 r = asin(-ny), p = atan2(nx, nz) 이다.

    |ny| ~ 1 이면 cos(r) ~ 0 이라 pitch가 정의되지 않는다(짐벌 락).
    그 경우 pitch는 0을 돌려주고 ok=False로 알린다.
    """
    r = math.asin(clamp(-ny, -1.0, 1.0))
    if abs(nx) < 1e-9 and abs(nz) < 1e-9:
        return r, 0.0, False
    return r, math.atan2(nx, nz), True


def apply_deadband(value, threshold):
    """threshold 이하는 0, 그 이상은 끊김 없이 통과하는 연속 데드밴드.

    하드 컷(|x|<th -> 0)은 x가 임계값 부근에서 흔들릴 때 0으로 순간 점프하는
    불연속을 만든다. 그 계단 입력이 ZV 셰이퍼를 자극해 오히려 떨림을 만든다.
    """
    if value > threshold:
        return value - threshold
    if value < -threshold:
        return value + threshold
    return 0.0


# ---------------------------------------------------------------------------
# 슬로싱 모드 — Housner/Abramson 등가 모델 (직립 원통, 선형 소진폭 이론)
# ---------------------------------------------------------------------------

# J1'(x) = 0 의 근. 1차/2차 반대칭 슬로싱 모드.
BESSEL_ROOTS = (1.8412, 5.3314)


def compute_slosh_modes(radius_m, fill_height_m):
    """(f1_hz, f2_hz) 반환. w_n^2 = (lam_n * g / R) * tanh(lam_n * h / R)"""
    f = []
    for lam in BESSEL_ROOTS:
        w = math.sqrt((lam * GRAVITY / radius_m) * math.tanh(lam * fill_height_m / radius_m))
        f.append(w / (2.0 * math.pi))
    return f[0], f[1]


def compute_slosh_modes_rect(side_m, fill_height_m):
    """직육면체(정사각 단면) 탱크의 (f1_hz, f2_hz).

    w_n^2 = (n*pi*g / L) * tanh(n*pi*h / L)   (선형 소진폭 이론 1차/2차 모드)

    실물 zv_shaping_rtos.ino 주석(342~346행)에 이미 이 식이 명시돼 있다.
    실물 탱크(11x11cm, 350mL -> h=350e-6/0.11^2=28.93mm)를 대입하면
    f1=2.194Hz가 나오는데, 같은 주석의 "책상 계산값 2.19"와 일치해
    이 식과 실물 치수 둘 다 검증된다(실측 2.00Hz와의 9% 차는 짐벌
    자체의 기계적 유격/탄성 때문이라고 ino 주석에 이미 설명돼 있다 —
    탱크 형상 오차가 아니다).

    베셀 근(BESSEL_ROOTS) 대신 n*pi(n=1,2)를 쓰는 것 말고는
    compute_slosh_modes()와 구조가 같다.
    """
    f = []
    for n in (1, 2):
        w = math.sqrt((n * math.pi * GRAVITY / side_m) *
                      math.tanh(n * math.pi * fill_height_m / side_m))
        f.append(w / (2.0 * math.pi))
    return f[0], f[1]


def housner_pendulum(radius_m, fill_height_m, rho=1000.0):
    """가제보 URDF에 넣을 등가 진자 파라미터.

    슬로싱 액체를 '고정 질량 m0 + 진자 질량 m1'으로 치환하는 표준 등가
    기계 모델이다(NASA SP-106 계열 폐형식). 가제보에는 유체가 없으므로
    이 진자를 탱크 안에 넣어 (1) 출렁임이 짐벌에 반력을 주게 하고
    (2) 진자각을 슬로싱 지표로 계측한다.

    반환 dict 단위: 질량 kg, 길이 m, 주파수 rad/s, 감쇠 N*m*s/rad
    """
    R, h = radius_m, fill_height_m
    m_liquid = rho * math.pi * R * R * h
    x = BESSEL_ROOTS[0] * h / R
    w1 = math.sqrt((BESSEL_ROOTS[0] * GRAVITY / R) * math.tanh(x))
    m1 = m_liquid * (0.318 * (R / h) * math.tanh(1.84 * h / R))
    m0 = m_liquid - m1
    h1 = (1.0 - (math.cosh(x) - 1.0) / (x * math.sinh(x))) * h
    length = GRAVITY / (w1 * w1)
    zeta = 0.01
    return {
        "m_liquid_kg": m_liquid,
        "omega1_rads": w1,
        "f1_hz": w1 / (2.0 * math.pi),
        "m1_kg": m1,
        "m0_kg": m0,
        "h1_m": h1,
        "length_m": length,
        "pivot_height_m": h1 + length,
        "damping_nms_per_rad": 2.0 * zeta * m1 * length * length * w1,
    }


# ---------------------------------------------------------------------------
# 저역통과 (accel_lpf_cutoff_hz) — 목표 법선 계산 직전에만 쓴다
# ---------------------------------------------------------------------------

class LowPass1Pole:
    """단일 극 저역통과, 성분별(x,y,z) 독립 적용. cutoff_hz<=0이면 통과."""

    def __init__(self, cutoff_hz):
        self.cutoff_hz = cutoff_hz
        self.x = self.y = self.z = 0.0
        self.initialized = False

    def reset(self):
        self.initialized = False

    def update(self, x, y, z, dt_s):
        if self.cutoff_hz <= 0.0 or dt_s <= 0.0:
            return x, y, z
        if not self.initialized:
            self.x, self.y, self.z = x, y, z
            self.initialized = True
            return x, y, z
        rc = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        a = dt_s / (rc + dt_s)
        self.x += a * (x - self.x)
        self.y += a * (y - self.y)
        self.z += a * (z - self.z)
        return self.x, self.y, self.z


# ---------------------------------------------------------------------------
# 상보필터 (수식 1)
# ---------------------------------------------------------------------------

class ComplementaryFilter:
    """자이로 적분(고주파 신뢰)과 가속도계 절대각(저주파 신뢰)을 융합한다.

    theta_k = alpha * (theta_{k-1} + w*dt) + (1-alpha) * theta_acc

    오일러 각속도는 근사하지 않고 정확히 쓴다. R = Ry(p)Rx(r) 규약에서
        w_body = ( r_dot, p_dot*cos(r), -p_dot*sin(r) )
    이므로 역으로
        r_dot = w_x
        p_dot = w_y*cos(r) - w_z*sin(r)
    가 되며 특이점이 없다. (통상 ZYX 규약의 tan(pitch) 항이 나타나지 않는다.)
    """

    def __init__(self, alpha):
        self.alpha = alpha
        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.initialized = False

    def reset(self):
        self.roll_rad = 0.0
        self.pitch_rad = 0.0
        self.initialized = False

    def update(self, s: ImuSample, dt_s: float):
        """가속도계 절대각을 기준으로 자이로 적분을 보정한다. 실패 시 False."""
        ux, uy, uz, norm = normalize3(s.ax_mps2, s.ay_mps2, s.az_mps2)
        if norm < 1e-6:
            return False

        # 비력 방향 = body 프레임에서 본 '위' 방향. R^T * z_hat 이므로
        #   f/|f| = ( -sin(p), sin(r)cos(p), cos(r)cos(p) )
        # 따라서 r = atan2(fy, fz), p = atan2(-fx, hypot(fy, fz)).
        roll_acc = math.atan2(uy, uz)
        pitch_acc = math.atan2(-ux, math.hypot(uy, uz))

        if not self.initialized:
            # 첫 표본은 가속도계 값으로 바로 잡는다. 0에서 수렴시키면
            # 초기 과도구간에 잘못된 목표각이 나간다.
            self.roll_rad = roll_acc
            self.pitch_rad = pitch_acc
            self.initialized = True
            return True

        sr, cr = math.sin(self.roll_rad), math.cos(self.roll_rad)
        roll_rate = s.gx_rads
        pitch_rate = s.gy_rads * cr - s.gz_rads * sr

        a = self.alpha
        self.roll_rad = a * (self.roll_rad + roll_rate * dt_s) + (1.0 - a) * roll_acc
        self.pitch_rad = a * (self.pitch_rad + pitch_rate * dt_s) + (1.0 - a) * pitch_acc
        return True


# ---------------------------------------------------------------------------
# 짐벌 기구학 — 직렬 2축 (pitch 바깥, roll 안쪽)
# ---------------------------------------------------------------------------

class GimbalKinematics:
    """R_tray_in_base = Ry(q_pitch) * Rx(q_roll)

    두 축이 직렬이므로 roll 축은 pitch가 이미 회전시킨 프레임 위에서 돈다.
    원본 서보 코드는 두 축을 독립 스칼라로 다뤘는데 이는 소각도 근사이며
    +-25deg 영역에서는 교차 결합 오차가 무시할 수 없다. 여기서는 닫힌 형태로 푼다.
    """

    @staticmethod
    def forward_normal(q_roll_rad, q_pitch_rad):
        """조인트각 -> base 프레임에서 본 트레이 법선."""
        return mat_col_z(attitude_matrix(q_roll_rad, q_pitch_rad))

    @staticmethod
    def inverse(nx, ny, nz):
        """목표 법선(base 프레임) -> (q_roll, q_pitch, ok)."""
        return normal_to_roll_pitch(nx, ny, nz)

    @staticmethod
    def compose_absolute(base_roll, base_pitch, q_roll, q_pitch):
        """base 자세 + 조인트각 -> 트레이의 절대 자세 (roll, pitch, ok).

        R_tray_world = R_base_world * R_tray_in_base
        """
        r_world = mat_mul(attitude_matrix(base_roll, base_pitch),
                          attitude_matrix(q_roll, q_pitch))
        nx, ny, nz = mat_col_z(r_world)
        return normal_to_roll_pitch(nx, ny, nz)


def resultant_normal_body(s: ImuSample):
    """액체 표면이 수직이 되어야 할 방향(= 목표 트레이 법선)을 센서 프레임에서.

    논문 수식 (2)(3)의 정확형이다.

    가속도 a로 움직이는 프레임 안의 유체는 유효 체적력 (g_vec - a)를 받는다.
    유효 '위' 방향은 그 반대인 (a - g_vec)이고, 이것이 바로 가속도계가 재는
    비력이다. 즉 목표 법선 = normalize(가속도계 읽은 값) 이다.

    수식 (2)(3)은 중력 성분을 G*sin(theta)로 빼서 선형가속도를 뽑고 arctan을
    취하는데, 그것은 소각도에서 이 식과 일치하는 근사다. 여기서는 근사 없이
    비력 방향을 바로 쓴다 — 큰 경사와 수직 가속도(험지)까지 자동으로 담긴다.

    반환 (ux, uy, uz, norm). norm이 0에 가까우면(자유낙하) 방향이 정의되지
    않으므로 호출자가 이전 목표를 유지해야 한다.
    """
    return normalize3(s.ax_mps2, s.ay_mps2, s.az_mps2)


# ---------------------------------------------------------------------------
# Convolved ZV 입력성형 (수식 4)
# ---------------------------------------------------------------------------

class ZvShaper:
    """2모드 Convolved ZV. 임펄스 4개의 지연/진폭을 주파수와 감쇠비로 산출한다.

    단일 모드 2-임펄스 ZV:
        K  = exp(-zeta*pi / sqrt(1-zeta^2))
        A  = [ 1/(1+K),  K/(1+K) ]
        지연 = pi / (w_n * sqrt(1-zeta^2))        (감쇠 반주기)

    두 모드를 컨볼루션하면 임펄스가 4개가 된다:
        지연 {0, N2, N1, N1+N2}
        진폭 {A1a*A1b, A1a*A2b, A2a*A1b, A2a*A2b}      합 = 1

    zeta -> 0 이면 A = [0.5, 0.5]이므로 진폭 4개가 모두 0.25가 되어 논문
    수식 (4)와 정확히 일치한다. 즉 상위 호환이다.
    """

    def __init__(self):
        self.delays = [0, 0, 0, 0]
        self.amps = [0.25, 0.25, 0.25, 0.25]
        self.max_delay = 0

    def configure(self, f1_hz, f2_hz, zeta, dt_s):
        """성공 시 True. 필요한 지연이 버퍼를 넘으면 아무것도 바꾸지 않고 False."""
        if f1_hz <= 0.0 or f2_hz <= 0.0 or dt_s <= 0.0 or not (0.0 <= zeta < 1.0):
            return False

        rt = math.sqrt(1.0 - zeta * zeta)
        k = math.exp(-zeta * math.pi / rt)
        a1, a2 = 1.0 / (1.0 + k), k / (1.0 + k)

        # 감쇠 반주기 = pi / w_d,  w_d = 2*pi*f*sqrt(1-zeta^2)
        n1 = max(1, int(round(0.5 / (f1_hz * rt * dt_s))))
        n2 = max(1, int(round(0.5 / (f2_hz * rt * dt_s))))
        if n1 + n2 >= ZV_BUFFER_SIZE:
            return False

        self.delays = [0, n2, n1, n1 + n2]
        self.amps = [a1 * a1, a1 * a2, a2 * a1, a2 * a2]
        self.max_delay = n1 + n2
        return True

    def shape(self, buf, head, count):
        """순환버퍼에서 성형값을 계산한다. head는 '다음에 쓸' 위치.

        이력이 모자란 초기 구간에서는 가장 오래된 유효 표본으로 대체한다.
        0으로 대체하면 기동 직후 목표각이 인위적으로 눌려 계단이 생긴다.
        """
        out = 0.0
        newest = (head - 1 + ZV_BUFFER_SIZE) % ZV_BUFFER_SIZE
        for i in range(4):
            d = self.delays[i]
            if d >= count:
                d = count - 1 if count > 0 else 0
            out += self.amps[i] * buf[(newest - d + ZV_BUFFER_SIZE) % ZV_BUFFER_SIZE]
        return out


class ConvolvedZV:
    """축 1개용 ZV 성형기. 주파수 갱신 시 교차 혼합으로 불연속을 막는다.

    팀 완료 기준: "ZV 초기화와 주파수 변경 시 불연속 명령을 방지함".
    구/신 셰이퍼가 같은 버퍼를 읽으므로 두 출력을 섞어도 유효한 성형값이며
    (진폭 합이 둘 다 1), 혼합 구간 동안 출력이 매끄럽게 이어진다.
    """

    def __init__(self, dt_s, crossfade_s):
        self.buf = [0.0] * ZV_BUFFER_SIZE
        self.head = 0
        self.count = 0
        self.active = ZvShaper()
        self.pending = ZvShaper()
        self.fade = 1.0          # 1.0이면 active만 사용
        self.fade_step = 1.0
        if crossfade_s > 0.0 and dt_s > 0.0:
            self.fade_step = dt_s / crossfade_s

    def configure(self, f1_hz, f2_hz, zeta, dt_s, immediate=False):
        """주파수 설정. immediate=True면 혼합 없이 즉시 교체(초기화 시)."""
        target = self.active if immediate else self.pending
        if not target.configure(f1_hz, f2_hz, zeta, dt_s):
            return False
        self.fade = 1.0 if immediate else 0.0
        return True

    def reset(self, value=0.0):
        """버퍼를 현재값으로 채운다. 0으로 비우면 재개 시 계단이 생긴다."""
        for i in range(ZV_BUFFER_SIZE):
            self.buf[i] = value
        self.head = 0
        self.count = ZV_BUFFER_SIZE
        self.fade = 1.0

    def apply(self, target):
        self.buf[self.head] = target
        self.head = (self.head + 1) % ZV_BUFFER_SIZE
        if self.count < ZV_BUFFER_SIZE:
            self.count += 1

        shaped_active = self.active.shape(self.buf, self.head, self.count)
        if self.fade >= 1.0:
            return shaped_active

        shaped_pending = self.pending.shape(self.buf, self.head, self.count)
        out = self.fade * shaped_pending + (1.0 - self.fade) * shaped_active
        self.fade += self.fade_step
        if self.fade >= 1.0:
            # 혼합 완료: pending을 active로 승격
            self.active, self.pending = self.pending, self.active
            self.fade = 1.0
        return out


# ---------------------------------------------------------------------------
# 비선형 게인 스케줄링 (수식 5)
# ---------------------------------------------------------------------------

class GainSchedule:
    """K_dyn = K_min + (K_max - K_min) * (|e| / e_max)^2

    오차가 큰 초기 구동에서는 높은 이득으로 응답을 빠르게, 목표에 근접하면
    낮은 이득으로 오버슈트와 미세 진동을 막는다. 제곱이라 목표 근처에서
    이득이 완만하게 떨어진다.
    """

    def __init__(self, kp_min, kp_max, error_max_rad):
        self.kp_min = kp_min
        self.kp_max = kp_max
        self.error_max_rad = error_max_rad

    def gain(self, error_rad):
        if self.error_max_rad <= 0.0:
            return self.kp_max
        ratio = clamp(abs(error_rad) / self.error_max_rad, 0.0, 1.0)
        return self.kp_min + (self.kp_max - self.kp_min) * ratio * ratio


# ---------------------------------------------------------------------------
# HW 스타일 경로 — 실물 zv_shaping_rtos.ino 이식 (enable_hw_style 전용)
# ---------------------------------------------------------------------------

def hw_force_vector_angle(ax, ay, az):
    """비력 벡터(가속도계 읽음값) -> (pitch, roll) 각.

    실물 force_pitch/force_roll 식 그대로(atan2, 481~497행 resultant_normal_body와
    동일 계열이지만 여기서는 3축 성분을 먼저 저역통과한 뒤 넘겨받는다는 점이 다르다).
    """
    pitch = math.atan2(ay, math.hypot(ax, az))
    roll = math.atan2(-ax, math.hypot(ay, az))
    return pitch, roll


def slew_rate_limit(target, prev, max_step):
    """실물 slew() 그대로. 한 스텝에 낼 수 있는 최대 변화량으로 자른다."""
    d = target - prev
    if d > max_step:
        d = max_step
    elif d < -max_step:
        d = -max_step
    return prev + d


class FixedFreqZV:
    """고정 주파수 단일 모드 ZV/ZVD (실물 zvRecalc()/ZV 성형 그대로).

    시뮬레이션 기존 ConvolvedZV(2모드, 감쇠비 기반)와 달리 축 1개·모드 1개다.
    zeta=0 가정이면 ZV 진폭은 정확히 [0.5, 0.5], ZVD는 [0.25, 0.5, 0.25]다.
    """

    def __init__(self):
        self.buf = [0.0] * ZV_BUFFER_SIZE
        self.head = 0
        self.n1 = 1
        self.amps = (0.5, 0.5, 0.0)  # (A1, A2, A3). A3=0이면 ZV(2임펄스)

    def configure(self, freq_hz, zeta, mode, dt_s):
        """실측 확정 상수로 반주기 지연·진폭을 다시 계산한다."""
        z = clamp(zeta, 0.0, 0.9)
        wd = math.sqrt(1.0 - z * z)
        k = math.exp(-z * math.pi / wd) if wd > 1e-9 else 0.0
        td_half = 1.0 / (freq_hz * wd) * 0.5 if freq_hz > 0.0 and wd > 1e-9 else dt_s

        max_n = (ZV_BUFFER_SIZE - 1) // 2 if mode == 3 else (ZV_BUFFER_SIZE - 1)
        self.n1 = int(clamp(round(td_half / dt_s), 1, max_n))

        if mode == 3:
            denom = (1.0 + k) ** 2
            self.amps = (1.0 / denom, 2.0 * k / denom, (k * k) / denom)
        else:
            self.amps = (1.0 / (1.0 + k), k / (1.0 + k), 0.0)

    def clear(self):
        for i in range(ZV_BUFFER_SIZE):
            self.buf[i] = 0.0
        self.head = 0

    def apply(self, target):
        self.head = (self.head + 1) % ZV_BUFFER_SIZE
        self.buf[self.head] = target
        i1 = (self.head - self.n1 + ZV_BUFFER_SIZE) % ZV_BUFFER_SIZE
        i2 = (self.head - 2 * self.n1 + ZV_BUFFER_SIZE) % ZV_BUFFER_SIZE
        a1, a2, a3 = self.amps
        return a1 * self.buf[self.head] + a2 * self.buf[i1] + a3 * self.buf[i2]


class SoftReturnPhase:
    DIRECT = "DIRECT"
    CONFIRM = "CONFIRM"
    HOLD = "HOLD"
    RETURN = "RETURN"
    LEVEL = "LEVEL"


def _ms_to_ticks(ms, dt_s):
    ticks = int(math.ceil((ms * 1e-3) / dt_s - 1e-9))
    return max(1, ticks)


class SoftReturnAxis:
    """실물 SoftReturnAxis/softReturnStep() 그대로 (622~700행).

    가속 종료 시 잔류 목표각을 srh만큼 유지한 뒤 최소저크 곡선으로 srr 동안
    0으로 복귀시켜, 급정지가 새 슬로싱을 만드는 것을 줄인다. ZV와 목적이
    겹치므로 동시에 쓰지 않는다(호출자가 상호배타를 보장해야 한다).
    """

    def __init__(self):
        self.phase = SoftReturnPhase.DIRECT
        self.saw_active = False
        self.quiet_ticks = 0
        self.phase_ticks = 0
        self.hold_angle = 0.0
        self.last_active_angle = 0.0

    def reset(self):
        self.__init__()

    def step(self, ref, raw_ref, active_deg, end_deg, dwell_ms, hold_ms, return_ms, dt_s):
        active_rad = math.radians(active_deg)
        end_rad = math.radians(end_deg)
        dwell_ticks = _ms_to_ticks(dwell_ms, dt_s)
        hold_ticks = _ms_to_ticks(hold_ms, dt_s)
        return_ticks = _ms_to_ticks(return_ms, dt_s)
        a = abs(raw_ref)

        if self.phase != SoftReturnPhase.DIRECT and a >= active_rad:
            self.reset()
            self.saw_active = True
            self.last_active_angle = ref
            return ref

        if self.phase == SoftReturnPhase.DIRECT:
            if a >= active_rad:
                self.saw_active = True
                self.quiet_ticks = 0
                self.last_active_angle = ref
            elif self.saw_active:
                if a > end_rad:
                    self.last_active_angle = ref
                else:
                    self.phase = SoftReturnPhase.CONFIRM
                    self.phase_ticks = 0
                    self.hold_angle = self.last_active_angle
                    if abs(self.hold_angle) < math.radians(0.15):
                        self.phase = SoftReturnPhase.LEVEL
                        self.saw_active = False
                    return self.hold_angle
            return ref

        if self.phase == SoftReturnPhase.CONFIRM:
            if a > end_rad:
                self.phase = SoftReturnPhase.DIRECT
                self.phase_ticks = 0
                self.last_active_angle = ref
                return ref
            self.phase_ticks += 1
            if self.phase_ticks >= dwell_ticks:
                self.phase = SoftReturnPhase.HOLD
                self.phase_ticks = 0
            return self.hold_angle

        if self.phase == SoftReturnPhase.HOLD:
            self.phase_ticks += 1
            if self.phase_ticks >= hold_ticks:
                self.phase = SoftReturnPhase.RETURN
                self.phase_ticks = 0
            return self.hold_angle

        if self.phase == SoftReturnPhase.RETURN:
            if self.phase_ticks < return_ticks:
                self.phase_ticks += 1
            x = clamp(self.phase_ticks / float(return_ticks), 0.0, 1.0)
            smooth = x * x * x * (10.0 + x * (-15.0 + 6.0 * x))
            out = self.hold_angle * (1.0 - smooth)
            if self.phase_ticks >= return_ticks:
                self.phase = SoftReturnPhase.LEVEL
                self.saw_active = False
                self.hold_angle = 0.0
                out = 0.0
            return out

        # LEVEL
        if a >= active_rad:
            self.reset()
            self.saw_active = True
        return ref


# ---------------------------------------------------------------------------
# 진단 — 어댑터가 로깅할 내부 상태
# ---------------------------------------------------------------------------

@dataclass
class Diagnostics:
    state: ControlState = ControlState.BOOT
    fault_reason: str = ""
    base_roll_rad: float = 0.0
    base_pitch_rad: float = 0.0
    tray_roll_rad: float = 0.0
    tray_pitch_rad: float = 0.0
    target_roll_rad: float = 0.0      # 성형 전 목표 조인트각
    target_pitch_rad: float = 0.0
    shaped_roll_rad: float = 0.0      # ZV 통과 후
    shaped_pitch_rad: float = 0.0
    abs_target_roll_rad: float = 0.0  # 성형 목표를 절대 자세로 환산
    abs_target_pitch_rad: float = 0.0
    error_roll_rad: float = 0.0
    error_pitch_rad: float = 0.0
    trim_roll_rad: float = 0.0
    trim_pitch_rad: float = 0.0
    kp_roll: float = 0.0
    kp_pitch: float = 0.0
    integral_roll_rad: float = 0.0    # 누적값(적분기 상태), ki 곱하기 전
    integral_pitch_rad: float = 0.0
    zv_f1_hz: float = 0.0
    zv_f2_hz: float = 0.0
    slosh_freq_source: str = "config"  # "config" | "estimator"
    dt_s: float = 0.0
    cycle_faults: int = 0

    # --- HW 스타일 경로 전용 (enable_hw_style=False면 전부 기본값 유지) ---
    hw_style_active: bool = False
    force_roll_rad: float = 0.0        # 3축 LPF 후 합력벡터 각(θ_force)
    force_pitch_rad: float = 0.0
    raw_ref_roll_rad: float = 0.0      # LPF 전, 소프트복귀 문턱 판정용
    raw_ref_pitch_rad: float = 0.0
    hw_shaped_roll_rad: float = 0.0    # ZV 또는 소프트복귀 통과 후
    hw_shaped_pitch_rad: float = 0.0
    soft_return_phase_roll: str = "DIRECT"
    soft_return_phase_pitch: str = "DIRECT"


# ---------------------------------------------------------------------------
# 제어 코어
# ---------------------------------------------------------------------------

class ControlCore:
    """상태기계 + 제어 파이프라인.

    파이프라인 (수식 순서대로):
        1  base/tray 상보필터로 절대 자세 추정                        (1)
        2  base 비력 방향 = 목표 트레이 법선 -> 역기구학으로 조인트각  (2)(3)
        3  지수평활 + 연속 데드밴드
        4  Convolved ZV 성형                                          (4)
        5  성형 목표를 절대 자세로 환산 -> 트레이 실측과 오차
           -> 게인 스케줄링 P 보정 -> 조인트 명령                     (5)

    2단계 피드포워드(기하학적 해)를 주 경로로 쓰고 5단계 P는 잔차 보정이다.
    원본 서보 코드는 P만 있었는데, 그건 서보 내부 루프가 무르고 출력이 PWM
    카운트여서 성립한 구조다. GL60II는 위치 명령을 받으므로 기하학적 해를
    직접 줄 수 있고 그래야 정상상태 오차가 남지 않는다.
    enable_feedforward=False로 두면 원본과 같은 P 단독 구조가 된다.
    """

    def __init__(self, cfg: CoreConfig = None):
        self.cfg = cfg if cfg is not None else CoreConfig()
        c = self.cfg

        self.base_filter = ComplementaryFilter(c.alpha_base)
        self.tray_filter = ComplementaryFilter(c.alpha_tray)
        self.accel_lpf = LowPass1Pole(c.accel_lpf_cutoff_hz)
        self.zv_roll = ConvolvedZV(c.nominal_dt_s, c.zv_crossfade_s)
        self.zv_pitch = ConvolvedZV(c.nominal_dt_s, c.zv_crossfade_s)
        self.gain = GainSchedule(c.kp_min, c.kp_max, c.error_max_rad)

        self.state = ControlState.BOOT
        self.diag = Diagnostics()

        self._smoothed_roll = 0.0
        self._smoothed_pitch = 0.0
        self._last_q_roll = 0.0
        self._last_q_pitch = 0.0
        self._integral_roll = 0.0
        self._integral_pitch = 0.0
        self._activate_requested = False
        self._cycle_faults = 0
        self._accel_fault_count = 0

        if c.tank_shape == "rect":
            self._fixed_f1_hz, self._fixed_f2_hz = compute_slosh_modes_rect(
                c.tank_side_m, c.fill_height_m)
        else:
            self._fixed_f1_hz, self._fixed_f2_hz = compute_slosh_modes(
                c.tank_radius_m, c.fill_height_m)
        self._f1_hz = self._fixed_f1_hz
        self._f2_hz = self._fixed_f2_hz
        self._freq_source = "config"

        # --- HW 스타일 경로 상태 (enable_hw_style=False면 그냥 미사용) ---
        # 실물 force_ax/ay/az 그대로: 고정 dt를 가정한 매 틱 지수평활
        # (force_ax = ACC_LPF*ax + (1-ACC_LPF)*force_ax, 1490~1492행).
        # LowPass1Pole은 dt에 맞춰 계수를 다시 계산하는 RC필터라 여기서는
        # 쓰지 않는다 — 실물과 다른 필터가 된다.
        self._force_ax = 0.0
        self._force_ay = 0.0
        self._force_az = 1.0
        # 실물 자체 상보필터 상태(θ_base). 시뮬레이션 기존 ComplementaryFilter는
        # R=Ry(pitch)*Rx(roll) 닫힌형 분해를 쓰는데, 실물은 두 축을 독립
        # 스칼라로 다루는 근사식(atan2(ay,·)=pitch, atan2(-ax,·)=roll)이다.
        # 두 식은 소각도에서만 일치하고 축 정의 자체가 다르므로, force-vector
        # 식(θ_base − θ_force)이 의도대로 상쇄되려면 θ_base도 반드시 같은
        # 실물 식으로 구해야 한다 — 기존 ComplementaryFilter를 재사용하지 않는다.
        self._hw_base_pitch = 0.0
        self._hw_base_roll = 0.0
        self._hw_base_initialized = False
        self.zv_hw_roll = FixedFreqZV()
        self.zv_hw_pitch = FixedFreqZV()
        self.zv_hw_roll.configure(c.zv_hw_freq_hz, c.zv_hw_zeta, c.zv_hw_mode, c.nominal_dt_s)
        self.zv_hw_pitch.configure(c.zv_hw_freq_hz, c.zv_hw_zeta, c.zv_hw_mode, c.nominal_dt_s)
        self.soft_return_roll = SoftReturnAxis()
        self.soft_return_pitch = SoftReturnAxis()
        self._hw_cmd_lpf_roll = 0.0
        self._hw_cmd_lpf_pitch = 0.0
        self._hw_cmd_roll = 0.0
        self._hw_cmd_pitch = 0.0
        self._hw_ff_vel_roll = 0.0
        self._hw_ff_vel_pitch = 0.0
        self._hw_ff_acc_roll = 0.0
        self._hw_ff_acc_pitch = 0.0

    # -- 초기화 ------------------------------------------------------------

    def init_control(self):
        """부팅 시 1회. 모터를 활성화하지 않는다(팀 안전 원칙)."""
        c = self.cfg
        self.base_filter.reset()
        self.tray_filter.reset()
        self.accel_lpf.reset()
        self._smoothed_roll = 0.0
        self._smoothed_pitch = 0.0
        self._last_q_roll = 0.0
        self._last_q_pitch = 0.0
        self._integral_roll = 0.0
        self._integral_pitch = 0.0
        self._activate_requested = False
        self._cycle_faults = 0
        self._accel_fault_count = 0
        self._f1_hz = self._fixed_f1_hz
        self._f2_hz = self._fixed_f2_hz
        self._freq_source = "config"

        self._force_ax, self._force_ay, self._force_az = 0.0, 0.0, 1.0
        self._hw_base_pitch = 0.0
        self._hw_base_roll = 0.0
        self._hw_base_initialized = False
        self.zv_hw_roll.configure(c.zv_hw_freq_hz, c.zv_hw_zeta, c.zv_hw_mode, c.nominal_dt_s)
        self.zv_hw_pitch.configure(c.zv_hw_freq_hz, c.zv_hw_zeta, c.zv_hw_mode, c.nominal_dt_s)
        self.soft_return_roll.reset()
        self.soft_return_pitch.reset()
        self._hw_cmd_lpf_roll = 0.0
        self._hw_cmd_lpf_pitch = 0.0
        self._hw_cmd_roll = 0.0
        self._hw_cmd_pitch = 0.0
        self._hw_ff_vel_roll = 0.0
        self._hw_ff_vel_pitch = 0.0
        self._hw_ff_acc_roll = 0.0
        self._hw_ff_acc_pitch = 0.0

        ok_r = self.zv_roll.configure(self._f1_hz, self._f2_hz,
                                      c.slosh_damping_ratio, c.nominal_dt_s,
                                      immediate=True)
        ok_p = self.zv_pitch.configure(self._f1_hz, self._f2_hz,
                                       c.slosh_damping_ratio, c.nominal_dt_s,
                                       immediate=True)
        self.reset_zv()
        self.state = ControlState.BOOT
        return ok_r and ok_p

    def reset_zv(self):
        self.zv_roll.reset(0.0)
        self.zv_pitch.reset(0.0)
        self.zv_hw_roll.clear()
        self.zv_hw_pitch.clear()
        self.soft_return_roll.reset()
        self.soft_return_pitch.reset()

    def request_activate(self):
        """READY -> ACTIVE 전환 요청. 자동 전환은 금지되어 있다."""
        self._activate_requested = True

    def request_standby(self):
        self._activate_requested = False

    def clear_fault(self):
        """FAULT 해제. 상태는 BOOT로 되돌려 센서/모터 점검을 다시 거친다."""
        if self.state == ControlState.FAULT:
            self.init_control()
            return True
        return False

    # -- CNN 주파수 입력 인터페이스 ---------------------------------------

    def set_estimated_slosh_frequency(self, x_hz, y_hz, confidence,
                                      confidence_threshold=0.7):
        """추정된 슬로싱 주파수를 반영한다.

        검증된 1D CNN 모델과 학습 데이터가 없으므로 여기서는 입력 검증과
        고정값 폴백까지만 구현한다. 추정 로직을 흉내내지 않는다
        (팀 지시: 실제 모델이 없으면 CNN을 구현한 것처럼 꾸미지 마라).

        반환: 반영했으면 True, 거부하고 고정값을 유지하면 False.
        """
        c = self.cfg
        ok = (confidence >= confidence_threshold
              and SLOSH_FREQ_MIN_HZ <= x_hz <= SLOSH_FREQ_MAX_HZ
              and SLOSH_FREQ_MIN_HZ <= y_hz <= SLOSH_FREQ_MAX_HZ)
        if not ok:
            self._f1_hz = self._fixed_f1_hz
            self._f2_hz = self._fixed_f2_hz
            self._freq_source = "config"
            self.zv_roll.configure(self._f1_hz, self._f2_hz,
                                   c.slosh_damping_ratio, c.nominal_dt_s)
            self.zv_pitch.configure(self._f1_hz, self._f2_hz,
                                    c.slosh_damping_ratio, c.nominal_dt_s)
            return False

        # 2차 모드는 1차와의 형상비를 유지한다(원통 탱크에서 f2/f1은 형상에만
        # 의존하므로, 추정기가 1차만 알려줘도 2차를 합리적으로 둘 수 있다).
        ratio = self._fixed_f2_hz / self._fixed_f1_hz
        r_ok = self.zv_roll.configure(x_hz, x_hz * ratio,
                                      c.slosh_damping_ratio, c.nominal_dt_s)
        p_ok = self.zv_pitch.configure(y_hz, y_hz * ratio,
                                       c.slosh_damping_ratio, c.nominal_dt_s)
        if r_ok and p_ok:
            self._f1_hz, self._f2_hz = x_hz, x_hz * ratio
            self._freq_source = "estimator"
            return True
        return False

    # -- 유효성 판정 -------------------------------------------------------

    ACCEL_FAULT_MSG = "가속도 크기 이상(포화/탈락)"

    def _accel_plausible(self, s: ImuSample):
        n = math.sqrt(s.ax_mps2 ** 2 + s.ay_mps2 ** 2 + s.az_mps2 ** 2)
        return self.cfg.accel_norm_min_mps2 <= n <= self.cfg.accel_norm_max_mps2

    def _sensors_ok(self, imu: ImuPair, now_us: int):
        for s in (imu.base, imu.tray):
            if not s.valid:
                return False, "IMU 표본 무효"
            if now_us - s.sampled_at_us > self.cfg.sensor_timeout_us:
                return False, "IMU 타임아웃"
            if not self._accel_plausible(s):
                return False, self.ACCEL_FAULT_MSG
        return True, ""

    def _motors_ok(self, mx: MotorFeedback, my: MotorFeedback, now_ms: int):
        if not self.cfg.require_motor_feedback:
            return True, ""
        for m in (mx, my):
            if not m.valid:
                return False, "모터 피드백 무효"
            if now_ms - m.received_at_ms > self.cfg.feedback_timeout_ms:
                return False, "모터 피드백 타임아웃"
            if m.error_code != 0:
                return False, "모터 오류코드 " + str(m.error_code)
        return True, ""

    # -- 메인 스텝 ---------------------------------------------------------

    def step(self, imu: ImuPair, mx: MotorFeedback, my: MotorFeedback,
             dt_s: float, now_us: int, now_ms: int) -> ControlOutput:
        """1주기 실행. dt_s는 실제 경과시간이어야 한다(고정값 가정 금지)."""
        c = self.cfg
        d = self.diag
        d.dt_s = dt_s

        # --- 주기 유효성 ---
        if dt_s <= 0.0 or dt_s > c.nominal_dt_s * c.dt_tolerance_ratio:
            self._cycle_faults += 1
            d.cycle_faults = self._cycle_faults
            if self._cycle_faults >= c.max_consecutive_cycle_faults:
                return self._to_fault("제어주기 누락 반복")
            # 이번 주기는 계산을 건너뛴다. 신규 명령을 만들지 않는다.
            return self._disabled_output()
        self._cycle_faults = 0
        d.cycle_faults = 0

        sensors_ok, sensor_msg = self._sensors_ok(imu, now_us)
        motors_ok, motor_msg = self._motors_ok(mx, my, now_ms)

        # --- 상태 전이 ---
        if self.state == ControlState.FAULT:
            return self._disabled_output()

        if self.state == ControlState.BOOT:
            self.state = ControlState.SENSOR_CHECK

        if self.state == ControlState.SENSOR_CHECK:
            if not sensors_ok:
                d.fault_reason = sensor_msg
                return self._disabled_output()
            self.state = ControlState.MOTOR_CHECK

        if self.state == ControlState.MOTOR_CHECK:
            if not motors_ok:
                d.fault_reason = motor_msg
                return self._disabled_output()
            d.fault_reason = ""
            self.state = ControlState.READY

        # READY 이후에는 센서/모터 이상이 곧 FAULT다. 단, 가속도 이상은 접촉
        # 솔버/진동으로 인한 1프레임짜리 순간 튐일 수 있으므로 N프레임 연속일
        # 때만 FAULT로 보낸다(그 외 IMU 무효/타임아웃, 모터 이상은 즉시 FAULT).
        if not sensors_ok:
            if sensor_msg == self.ACCEL_FAULT_MSG:
                self._accel_fault_count += 1
                d.fault_reason = sensor_msg
                if self._accel_fault_count >= c.accel_fault_debounce_frames:
                    return self._to_fault(sensor_msg)
                return self._disabled_output()
            return self._to_fault(sensor_msg)
        self._accel_fault_count = 0
        if not motors_ok:
            return self._to_fault(motor_msg)

        if self.state == ControlState.READY:
            if not self._activate_requested:
                # 대기 중에도 필터는 돌려서 활성화 순간 자세가 이미 수렴해 있게 한다
                self._update_estimates(imu, dt_s)
                return self._disabled_output()
            self.state = ControlState.ACTIVE
            self.reset_zv()

        if self.state == ControlState.ACTIVE and not self._activate_requested:
            self.state = ControlState.READY
            return self._disabled_output()

        # --- ACTIVE: 제어 계산 ---
        return self._compute(imu, dt_s, mx, my)

    # -- 내부 ---------------------------------------------------------------

    def _update_estimates(self, imu: ImuPair, dt_s: float):
        d = self.diag
        self.base_filter.update(imu.base, dt_s)
        self.tray_filter.update(imu.tray, dt_s)
        d.base_roll_rad = self.base_filter.roll_rad
        d.base_pitch_rad = self.base_filter.pitch_rad
        d.tray_roll_rad = self.tray_filter.roll_rad - self.cfg.tray_zero_offset_roll_rad
        d.tray_pitch_rad = self.tray_filter.pitch_rad - self.cfg.tray_zero_offset_pitch_rad

    def _compute(self, imu: ImuPair, dt_s: float,
                 mx: MotorFeedback = None, my: MotorFeedback = None) -> ControlOutput:
        if self.cfg.enable_hw_style:
            return self._compute_hw_style(imu, dt_s, mx, my)

        c = self.cfg
        d = self.diag

        # 1) 자세 추정 (수식 1)
        self._update_estimates(imu, dt_s)

        # 2) 목표 법선 -> 역기구학 (수식 2,3)
        # 원시 비력에 저역통과를 먼저 걸어 슬로싱 공진대 노이즈를 줄인다
        # (accel_lpf_cutoff_hz 설명 참조). 회전/오르막의 실제 가속도 성분은
        # 차단주파수보다 낮은 대역이라 그대로 통과한다.
        ax_f, ay_f, az_f = self.accel_lpf.update(
            imu.base.ax_mps2, imu.base.ay_mps2, imu.base.az_mps2, dt_s)
        nx, ny, nz, norm = normalize3(ax_f, ay_f, az_f)
        if norm < 1e-6:
            q_roll, q_pitch = self._last_q_roll, self._last_q_pitch
        else:
            q_roll, q_pitch, ik_ok = GimbalKinematics.inverse(nx, ny, nz)
            if not ik_ok:
                # 짐벌 락 근처: 이전 목표를 유지한다
                q_roll, q_pitch = self._last_q_roll, self._last_q_pitch
        self._last_q_roll, self._last_q_pitch = q_roll, q_pitch
        d.target_roll_rad, d.target_pitch_rad = q_roll, q_pitch

        # 3) 평활 + 연속 데드밴드
        self._smoothed_roll += c.smooth_alpha_roll * (q_roll - self._smoothed_roll)
        self._smoothed_pitch += c.smooth_alpha_pitch * (q_pitch - self._smoothed_pitch)
        db_roll = apply_deadband(self._smoothed_roll, c.deadband_roll_rad)
        db_pitch = apply_deadband(self._smoothed_pitch, c.deadband_pitch_rad)

        # 4) Convolved ZV (수식 4) — 목표각에만 적용한다.
        #    모터 피드백이나 아래 P 보정 출력을 ZV 입력으로 되먹이지 않는다.
        if c.enable_zv:
            shaped_roll = self.zv_roll.apply(db_roll)
            shaped_pitch = self.zv_pitch.apply(db_pitch)
        else:
            shaped_roll, shaped_pitch = db_roll, db_pitch
        d.shaped_roll_rad, d.shaped_pitch_rad = shaped_roll, shaped_pitch
        d.zv_f1_hz, d.zv_f2_hz = self._f1_hz, self._f2_hz
        d.slosh_freq_source = self._freq_source

        # 5) 절대각 오차 -> 게인 스케줄링 P 보정 (수식 5)
        abs_roll, abs_pitch, ok = GimbalKinematics.compose_absolute(
            d.base_roll_rad, d.base_pitch_rad, shaped_roll, shaped_pitch)
        if not ok:
            abs_roll, abs_pitch = d.tray_roll_rad, d.tray_pitch_rad
        d.abs_target_roll_rad, d.abs_target_pitch_rad = abs_roll, abs_pitch

        e_roll = abs_roll - d.tray_roll_rad
        e_pitch = abs_pitch - d.tray_pitch_rad
        d.error_roll_rad, d.error_pitch_rad = e_roll, e_pitch

        if c.enable_trim:
            if c.enable_gain_scheduling:
                kp_r = self.gain.gain(e_roll)
                kp_p = self.gain.gain(e_pitch)
            else:
                kp_r = kp_p = c.kp_max

            if c.enable_integral and c.ki_trim > 1e-9:
                i_max = c.trim_limit_rad / c.ki_trim
                self._integral_roll = clamp(self._integral_roll + e_roll * dt_s,
                                            -i_max, i_max)
                self._integral_pitch = clamp(self._integral_pitch + e_pitch * dt_s,
                                             -i_max, i_max)
                i_roll = c.ki_trim * self._integral_roll
                i_pitch = c.ki_trim * self._integral_pitch
            else:
                i_roll = i_pitch = 0.0
            d.integral_roll_rad, d.integral_pitch_rad = self._integral_roll, self._integral_pitch

            trim_roll = clamp(kp_r * e_roll + i_roll, -c.trim_limit_rad, c.trim_limit_rad)
            trim_pitch = clamp(kp_p * e_pitch + i_pitch, -c.trim_limit_rad, c.trim_limit_rad)
        else:
            kp_r = kp_p = 0.0
            trim_roll = trim_pitch = 0.0
        d.kp_roll, d.kp_pitch = kp_r, kp_p
        d.trim_roll_rad, d.trim_pitch_rad = trim_roll, trim_pitch

        ff_roll = shaped_roll if c.enable_feedforward else 0.0
        ff_pitch = shaped_pitch if c.enable_feedforward else 0.0

        lim = c.joint_limit_rad
        d.state = self.state
        return ControlOutput(
            x_position_rad=clamp(ff_roll + trim_roll, -lim, lim),
            y_position_rad=clamp(ff_pitch + trim_pitch, -lim, lim),
            max_velocity_rads=c.max_velocity_rads,
            enable=True,
        )

    def _compute_hw_style(self, imu: ImuPair, dt_s: float,
                           mx: MotorFeedback, my: MotorFeedback) -> ControlOutput:
        """실물 ctrlTask() 그대로 (zv_shaping_rtos.ino 1477~1628행).

        base IMU만 쓴다(실물에는 tray IMU가 없다). tray_filter는 로깅
        일관성을 위해 _update_estimates()에서 계속 갱신되지만 이 경로는
        읽지 않는다.
        """
        c = self.cfg
        d = self.diag
        d.hw_style_active = True

        # 실측 위치 한계 FAULT — 실물 drainCAN() 397/1057~1062행 그대로.
        # 지령은 절대 cmd_limit_rad(45°)를 넘지 않으므로, 측정 위치가
        # act_limit_rad(55°)를 넘으면 우리가 시킨 움직임이 아니다.
        if mx is not None and my is not None and mx.valid and my.valid:
            if abs(mx.position_rad) > c.act_limit_rad or abs(my.position_rad) > c.act_limit_rad:
                return self._to_fault("위치 한계 초과 (ACT_LIMIT)")

        # 1) 상보필터로 차체 절대 자세 θ_base — 실물 자체 식 그대로
        # (1481~1516행). 시뮬레이션 기존 ComplementaryFilter(R=Ry*Rx 닫힌형
        # 분해)는 축 정의가 달라 여기서 쓰면 안 된다(클래스 상단 주석 참조).
        # gx->pitch rate, gy->roll rate인 것도 실물 그대로(레지스터 배정,
        # 1481~1482행) — REP-103 사용자 기대와 다를 수 있어 TODO로 남긴다.
        # TODO: 실측 필요 — 시뮬레이션 IMU 축과 실물 gx/gy 배정이 일치하는지.
        s = imu.base
        pitch_acc = math.atan2(s.ay_mps2, math.hypot(s.ax_mps2, s.az_mps2))
        roll_acc = math.atan2(-s.ax_mps2, math.hypot(s.ay_mps2, s.az_mps2))

        if not self._hw_base_initialized:
            self._hw_base_pitch = pitch_acc
            self._hw_base_roll = roll_acc
            self._hw_base_initialized = True
        else:
            rate_pitch = s.gx_rads
            rate_roll = s.gy_rads
            ab = c.alpha_base
            self._hw_base_pitch = ab * (self._hw_base_pitch + rate_pitch * dt_s) + (1.0 - ab) * pitch_acc
            self._hw_base_roll = ab * (self._hw_base_roll + rate_roll * dt_s) + (1.0 - ab) * roll_acc
        theta_base_pitch = self._hw_base_pitch
        theta_base_roll = self._hw_base_roll
        d.base_roll_rad = theta_base_roll
        d.base_pitch_rad = theta_base_pitch

        # tray_filter는 제어에 쓰지 않지만 CSV 로깅 컬럼(tray_roll_deg 등)이
        # 두 경로에서 같은 의미를 유지하도록 계속 갱신해둔다.
        self.tray_filter.update(imu.tray, dt_s)
        d.tray_roll_rad = self.tray_filter.roll_rad - self.cfg.tray_zero_offset_roll_rad
        d.tray_pitch_rad = self.tray_filter.pitch_rad - self.cfg.tray_zero_offset_pitch_rad

        # 3축 합력벡터를 성분 상태로 저역통과(ACC_LPF) 한 뒤 한 번만 각도로
        # 바꾼다 — 축별로 각도를 따로 거르면 두 축이 동시에 움직일 때 벡터
        # 길이·방향이 어긋난다 (1487~1494행 주석 그대로).
        a = c.force_lpf_alpha
        self._force_ax = a * s.ax_mps2 + (1.0 - a) * self._force_ax
        self._force_ay = a * s.ay_mps2 + (1.0 - a) * self._force_ay
        self._force_az = a * s.az_mps2 + (1.0 - a) * self._force_az
        force_pitch, force_roll = hw_force_vector_angle(
            self._force_ax, self._force_ay, self._force_az)
        d.force_pitch_rad, d.force_roll_rad = force_pitch, force_roll

        # 3) 합력 목표각. want = ACC_GAIN*ref - GAIN*theta_base, ref = theta_base - theta_force
        # 이므로 GAIN=ACC_GAIN일 때 theta_base가 대수적으로 소거된다(1536~1550행 주석).
        dir_acc = c.dir_accel
        raw_ref_pitch = clamp((theta_base_pitch - pitch_acc) * dir_acc,
                              -c.acc_ref_limit_rad, c.acc_ref_limit_rad)
        raw_ref_roll = clamp((theta_base_roll - roll_acc) * dir_acc,
                             -c.acc_ref_limit_rad, c.acc_ref_limit_rad)
        ref_pitch = clamp((theta_base_pitch - force_pitch) * dir_acc,
                          -c.acc_ref_limit_rad, c.acc_ref_limit_rad)
        ref_roll = clamp((theta_base_roll - force_roll) * dir_acc,
                         -c.acc_ref_limit_rad, c.acc_ref_limit_rad)
        d.raw_ref_pitch_rad, d.raw_ref_roll_rad = raw_ref_pitch, raw_ref_roll

        # 연속 데드밴드 — 실물에는 없으나 사용자 지시로 기존 시뮬 데드밴드를
        # 재사용한다(모터 떨림 억제 목적은 동일).
        ref_pitch = apply_deadband(ref_pitch, c.deadband_pitch_rad)
        ref_roll = apply_deadband(ref_roll, c.deadband_roll_rad)

        # 3.5) ZV 입력성형 또는 소프트복귀 — 상호배타(실물 sr1이면 zv 자동 OFF)
        if c.enable_soft_return:
            shaped_pitch = self.soft_return_pitch.step(
                ref_pitch, raw_ref_pitch, c.soft_return_active_deg, c.soft_return_end_deg,
                c.soft_return_dwell_ms, c.soft_return_hold_ms, c.soft_return_return_ms, dt_s)
            shaped_roll = self.soft_return_roll.step(
                ref_roll, raw_ref_roll, c.soft_return_active_deg, c.soft_return_end_deg,
                c.soft_return_dwell_ms, c.soft_return_hold_ms, c.soft_return_return_ms, dt_s)
            d.soft_return_phase_pitch = self.soft_return_pitch.phase
            d.soft_return_phase_roll = self.soft_return_roll.phase
        elif c.enable_zv_hw:
            shaped_pitch = self.zv_hw_pitch.apply(ref_pitch)
            shaped_roll = self.zv_hw_roll.apply(ref_roll)
        else:
            shaped_pitch, shaped_roll = ref_pitch, ref_roll
        d.hw_shaped_pitch_rad, d.hw_shaped_roll_rad = shaped_pitch, shaped_roll

        # 4) 제어  motor_cmd = ACC_GAIN*ref - GAIN*theta_base
        want_pitch = clamp(c.gain_accel * shaped_pitch - c.gain_horiz * theta_base_pitch,
                           -c.cmd_limit_rad, c.cmd_limit_rad)
        want_roll = clamp(c.gain_accel * shaped_roll - c.gain_horiz * theta_base_roll,
                          -c.cmd_limit_rad, c.cmd_limit_rad)

        self._hw_cmd_lpf_pitch = c.cmd_lpf_alpha * want_pitch + (1.0 - c.cmd_lpf_alpha) * self._hw_cmd_lpf_pitch
        self._hw_cmd_lpf_roll = c.cmd_lpf_alpha * want_roll + (1.0 - c.cmd_lpf_alpha) * self._hw_cmd_lpf_roll

        max_step = c.cmd_slew_rads * dt_s
        prev_pitch, prev_roll = self._hw_cmd_pitch, self._hw_cmd_roll
        self._hw_cmd_pitch = slew_rate_limit(self._hw_cmd_lpf_pitch, self._hw_cmd_pitch, max_step)
        self._hw_cmd_roll = slew_rate_limit(self._hw_cmd_lpf_roll, self._hw_cmd_roll, max_step)

        # MIT 피드포워드(v_des, t_ff) — 진단 전용, Gazebo 구동에는 쓰이지 않는다.
        if dt_s > 0.0:
            vp = (self._hw_cmd_pitch - prev_pitch) / dt_s
            vr = (self._hw_cmd_roll - prev_roll) / dt_s
            self._hw_ff_acc_pitch = (vp - self._hw_ff_vel_pitch) / dt_s
            self._hw_ff_acc_roll = (vr - self._hw_ff_vel_roll) / dt_s
            self._hw_ff_vel_pitch, self._hw_ff_vel_roll = vp, vr
        ff_torque_pitch = clamp(c.motor_ff_j_pitch * self._hw_ff_acc_pitch,
                                -c.motor_ff_tmax_nm, c.motor_ff_tmax_nm)
        ff_torque_roll = clamp(c.motor_ff_j_roll * self._hw_ff_acc_roll,
                               -c.motor_ff_tmax_nm, c.motor_ff_tmax_nm)

        d.state = self.state
        return ControlOutput(
            x_position_rad=self._hw_cmd_roll,
            y_position_rad=self._hw_cmd_pitch,
            max_velocity_rads=c.max_velocity_rads,
            enable=True,
            ff_velocity_roll_rads=self._hw_ff_vel_roll,
            ff_velocity_pitch_rads=self._hw_ff_vel_pitch,
            ff_torque_roll_nm=ff_torque_roll,
            ff_torque_pitch_nm=ff_torque_pitch,
        )

    def _disabled_output(self) -> ControlOutput:
        self.diag.state = self.state
        return ControlOutput(max_velocity_rads=self.cfg.max_velocity_rads, enable=False)

    def _to_fault(self, reason) -> ControlOutput:
        self.state = ControlState.FAULT
        self.diag.state = self.state
        self.diag.fault_reason = reason
        return ControlOutput(max_velocity_rads=self.cfg.max_velocity_rads, enable=False)
