# 짐벌 제어 — 시뮬레이션 구현 및 실물 이식 대응표

`pid_control_parkver`(ESP32 + PWM 서보)를 참고해 새 짐벌 구조(GL60II)용 제어를
재구현한 것이다. 검증 무대는 가제보이고, 최종 목적지는 `firmware/gimbal_control/`이다.

게인·임계값은 실물 완성 후 기구 유격과 실제 물리환경을 보며 조정할 항목이므로
근거를 붙인 초기값으로 두었다. 그 외 구조·수식·단위·상태관리·안전 로직은 실물로
바로 옮길 수 있는 수준으로 작성했다.

## 파일 구성

| 파일 | 역할 | 실물 이식 |
|---|---|---|
| `gimbal_control_core.py` | 플랫폼 독립 제어 코어. ROS도 Arduino도 모른다 | **번역 대상** |
| `gimbal_leveling_controller.py` | ROS2/가제보 어댑터 | 폐기 (실물은 SPI/CAN 어댑터) |
| `test_core.py` | 코어 단위 시험 104개 (기존 79 + HW 스타일 25) | 검증 근거로 유지 |
| `analyze_log.py` | 실행 로그 분석·조건 비교 | 실물 실험에도 그대로 사용 가능 |

```
        ControlCore  (플랫폼 독립: 필터 / 목표각 / 역기구학 / ZV / 게인 / 상태기계)
             ^  ImuPair, MotorFeedback          |  ControlOutput
   ----------+----------------------------------+-----------------------------
   시뮬레이션 |  ROS2 토픽                        |  gimbal_leveling_controller.py
   실물       |  SPI IMU + GL60II CAN            |  Control.cpp + Sensor/Gimbal
```

## 실행

```bash
# 두 파일이 같은 폴더에 있어야 한다. 기본 축 프리셋(base=body2_current_base,
# tray=cad_rotated_tray)이 현재 URDF에 맞춰져 있어 따로 안 줘도 된다
# (2026-09-05, 가제보 headless 실행으로 실측 확인 — 이전엔 identity가
# 기본값이라 중력이 Z가 아닌 축에 실리는 문제가 있었다).
python3 gimbal_leveling_controller.py --ros-args -p use_sim_time:=true

# URDF가 바뀌어 센서가 이미 REP-103대로 정렬됐다면
python3 gimbal_leveling_controller.py --ros-args -p use_sim_time:=true \
    -p base_axis_preset:=identity -p tray_axis_preset:=identity

# 조건 비교용 기록
python3 gimbal_leveling_controller.py --ros-args -p use_sim_time:=true \
    -p enable_zv:=false -p log_csv_path:=/tmp/run_nozv.csv
python3 analyze_log.py /tmp/run_nozv.csv /tmp/run_zv.csv --plot compare.png

# 코어 단위 시험 (ROS 불필요)
python3 test_core.py

# HW 스타일 경로(실물 zv_shaping_rtos.ino 포팅) 켜기
python3 gimbal_leveling_controller.py --ros-args -p use_sim_time:=true \
    -p enable_hw_style:=true -p gain_horiz:=1.0 -p gain_accel:=1.0 \
    -p zv_hw_freq_hz:=2.0 -p cmd_slew_deg_s:=120.0

# HW 스타일 ON/OFF 비교 기록 후 분석
python3 gimbal_leveling_controller.py --ros-args -p use_sim_time:=true \
    -p enable_hw_style:=false -p log_csv_path:=/tmp/run_default.csv
python3 gimbal_leveling_controller.py --ros-args -p use_sim_time:=true \
    -p enable_hw_style:=true  -p log_csv_path:=/tmp/run_hwstyle.csv
python3 analyze_log.py /tmp/run_default.csv /tmp/run_hwstyle.csv --plot compare_hw.png
```

## 논문 수식 대응

| 논문 | 코어 구현 | 비고 |
|---|---|---|
| (1) 상보필터 | `ComplementaryFilter` | base α=0.98 / tray α=0.995. 오일러 각속도를 근사 없이 정확히 사용 |
| (2) 중력 성분 제거 | `resultant_normal_body` | 수식 (2)(3)을 합쳐 정확형으로 대체 (아래 참조) |
| (3) 합력 목표각 | `resultant_normal_body` + `GimbalKinematics.inverse` | |
| (4) Convolved ZV | `ConvolvedZV` / `ZvShaper` | 지연·진폭을 주파수와 감쇠비에서 산출 |
| (5) 게인 스케줄링 P | `GainSchedule` | |

### 수식 (2)(3)을 정확형으로 대체한 근거

논문은 중력 성분을 빼서 선형가속도를 뽑고 arctan을 취한다.

```
a_lin = a_raw − G·sin(θ_k)          (2)
φ_raw = arctan(a_lin / G)           (3)
```

가속도 **a**로 움직이는 프레임 안의 유체는 유효 체적력 `g_vec − a`를 받는다.
유효 '위' 방향은 그 반대인 `a − g_vec`이고, **이것이 바로 가속도계가 재는 비력**이다.
따라서

```
목표 트레이 법선(센서 프레임) = normalize(가속도계 읽은 값)
```

이 되어 중력 제거 단계 자체가 필요 없다. 수식 (2)(3)은 이 식의 소각도 근사이며,
정확형은 큰 경사와 수직 가속도(험지 주행)까지 자동으로 담는다.
`test_core.py`의 `test_resultant_target`이 두 결과가 소각도에서 일치하고
경사 정지 시 트레이 절대각이 정확히 0이 됨을 확인한다.

## 원본(`pid_control_parkver`) 대비 변경점

| 항목 | 원본 | 재구현 | 이유 |
|---|---|---|---|
| 단위 | degree, PWM count | **rad** | 팀 계약 `COMMON_CONTEXT.md` |
| 출력 | `ledcWrite` PWM 듀티 | `ControlOutput` 위치+속도 | GL60II는 위치/속도 명령을 받는다 |
| 축 처리 | 2축 독립 스칼라 | **직렬 2축 역기구학** | 짐벌이 직렬이라 두 각이 클 때 독립 가정이 깨진다 |
| roll 경로 | 관성보상 생략 | pitch와 **대칭** | 논문 저자 확인: 원본은 실수 |
| 부호 규약 | base/tray 반대, `setpointY` 마이너스, `pidY` REVERSE | **단일 규약** | 저자가 IMU 장착 방향 변경을 허용 |
| ZV 지연 | 상수 21/11 샘플 | **주파수에서 산출** | CNN 주파수 갱신 인터페이스 요구 |
| ZV 진폭 | 0.25 고정 | **감쇠비에서 산출** | ζ→0에서 0.25로 수렴하므로 상위 호환 |
| 주 경로 | P 단독 | **역기구학 피드포워드 + P 보정** | 위치 명령 액추에이터에서는 기하학적 해를 직접 줄 수 있다 |
| 상태 관리 | 없음 (부팅 즉시 구동) | **상태기계 + 안전 게이트** | 팀 안전 원칙 |

`enable_feedforward=False`로 두면 원본과 같은 P 단독 구조가 되어 비교할 수 있다.

### ZV 지연 21/11 샘플 문제

원본의 지연 21/11 샘플(100Hz)은 f₁≈2.38Hz / f₂≈4.55Hz를 겨냥한다. 이는 반경
**약 80mm** 탱크에 해당한다. 그런데 현재 시뮬레이션 탱크는 내반경 55mm로
f₁≈2.88Hz이므로 **21% 어긋난다.** 어긋난 상태로는 셰이퍼가 상쇄하지 못한다.

코어는 `tank_radius_m` / `fill_height_m`에서 주파수를 산출하므로 실물 탱크 치수를
확인해 두 값만 맞추면 된다.

**미해결**: 실물 탱크 치수 확인 필요.

## HW 스타일 경로 (실물 Liquid_Control_Robot 포팅)

위 "실물 이식 대응표"는 이 코어의 기존 경로(닫힌형 3D 역기구학 + ZV + 게인
스케줄링)를 실물 펌웨어로 옮기는 방향을 정리한 것이다. 반대로 이 절은 **실물
`Liquid_Control_Robot`(ESP32-S3, `zv_shaping_rtos.ino`)의 현재 제어 로직을
이 시뮬레이션 쪽으로 그대로 들여온 것**이다 — `CoreConfig.enable_hw_style=True`로
켜는 별도 경로이며 기본값(`False`)에서는 기존 동작이 완전히 그대로 유지된다.
실물과 시뮬레이션 양쪽에서 같은 조건으로 결과를 비교하기 위한 것으로, 최종
목적지는 위 대응표와 같은 `firmware/gimbal_control/`이다.

### 발견한 불일치 — "합력 목표각" 계산식

실물 저장소의 여러 단계 파일 중 `zv_shaping_rtos.ino`가 "현행 본체"인데, 이
파일 **맨 위 docstring**(예전 2단계 설명을 그대로 복사)은

```
a_lin = a − g·sin(θ_base)
θ_ref = atan(a_lin / g)
```

라고 적혀 있지만, **실제 `ctrlTask()` 코드(1536~1550행)는 이미 이 식을
대체한 다른 식**을 쓴다:

```cpp
force_pitch = atan2(force_ay, sqrt(force_ax²+force_az²));  // 3축 벡터 LPF(ACC_LPF=0.20) 후 각도화
ref_pitch   = (pitch_filtered − force_pitch) * DIR_ACC;     // = θ_base − θ_force
want_pitch  = ACC_GAIN·shaped_pitch − GAIN·pitch_filtered;  // GAIN=ACC_GAIN=1이면 θ_base가 대수적으로 소거되어 결국 −θ_force
```

`atan(a_lin/g)`는 가속 중 실제 비력 크기가 g와 달라지는데도 분모를 고정된
g로 놓아 오차가 생긴다(코드 주석 예시: 30° 기울고 0.3g 가속 시 2.14° 부족).
이 코어의 `_compute_hw_style()`은 **docstring이 아니라 이 대체 식을
그대로 포팅**했다. `hw_force_vector_angle()`이 `force_pitch`/`force_roll`
식이고, θ_base는 `ComplementaryFilter`(기존 3D 닫힌형 분해)를 재사용하지
않고 실물과 똑같은 독립 스칼라 근사(`atan2(ay,·)=pitch`, `atan2(-ax,·)=roll`)로
**따로** 구현했다 — 두 θ 정의가 서로 다른 분해식이면 "θ_base가 소거된다"는
성질 자체가 깨지기 때문이다(첫 시도에서 실제로 이 문제로 값이 어긋났었다).

`cmd_slew_rads`도 마찬가지 함정이 있었다: 지시받은 150°/s는 `motor_test_MIT.ino`의
독립적인 오픈루프 스윕 테스트 상한(`SWEEP_RATE_MAX`)이고, 실제 클로즈드루프
슬루는 `zv_shaping_rtos.ino`의 `MAX_RATE = 120°/s`(실측 확정)다. 후자를 썼다.

### 상수 대응표

| 실물 (`zv_shaping_rtos.ino`) | `CoreConfig` 필드 | ROS2 파라미터 | 값 |
|---|---|---|---|
| `GAIN` [g] | `gain_horiz` | `gain_horiz` | 1.00 |
| `ACC_GAIN` [ag] | `gain_accel` | `gain_accel` | 1.00 |
| `ACC_LPF` [ad] | `force_lpf_alpha` | `force_lpf_alpha` | 0.20 |
| `DIR_ACC` | `dir_accel` | `dir_accel` | +1 (TODO: 실측 필요) |
| `LIMIT_DEG` | `cmd_limit_rad` | `cmd_limit_deg` | 45° |
| `ACC_REF_LIMIT` | `acc_ref_limit_rad` | `acc_ref_limit_deg` | 45° |
| `ACT_LIMIT_DEG` | `act_limit_rad` | `act_limit_deg` | 55° |
| `MAX_RATE` [r] | `cmd_slew_rads` | `cmd_slew_deg_s` | 120°/s |
| `CMD_LPF` [f] | `cmd_lpf_alpha` | `cmd_lpf_alpha` | 0.30 |
| `ZV_ON` [zv] | `enable_zv_hw` | `enable_zv_hw` | True |
| `ZV_FREQ` [zf] | `zv_hw_freq_hz` | `zv_hw_freq_hz` | 2.00 Hz |
| `ZV_MODE` [zm] | `zv_hw_mode` | `zv_hw_mode` | 2 (ZV) |
| — (지시에 따른 단순화) | `zv_hw_zeta` | — | 0.0 (실물 기본 0.02와 다름) |
| `SOFT_RETURN_ON` [sr] | `enable_soft_return` | `enable_soft_return` | False |
| `SR_ACTIVE_DEG` [sra] | `soft_return_active_deg` | `soft_return_active_deg` | 1.50° |
| `SR_END_DEG` [sre] | `soft_return_end_deg` | `soft_return_end_deg` | 0.75° |
| `SR_DWELL_MS` [srd] | `soft_return_dwell_ms` | `soft_return_dwell_ms` | 30 ms |
| `SR_HOLD_MS` [srh] | `soft_return_hold_ms` | `soft_return_hold_ms` | 100 ms |
| `SR_RETURN_MS` [srr] | `soft_return_return_ms` | `soft_return_return_ms` | 120 ms |
| `KP_PITCH`/`KD_PITCH` [kpp/kdp] | `motor_kp_pitch`/`motor_kd_pitch` | 동일 | 4.0 / 0.20 |
| `KP_ROLL`/`KD_ROLL` [kpr/kdr] | `motor_kp_roll`/`motor_kd_roll` | 동일 | 2.0 / 0.13 |
| `FF_J_PITCH`/`FF_J_ROLL` [fj/fjr] | `motor_ff_j_pitch`/`motor_ff_j_roll` | 동일 | 0.0038 / 0.0 |
| `FF_TMAX` | `motor_ff_tmax_nm` | — | 1.5 N·m |
| `ACT_LIMIT_DEG` 초과 | (새 FAULT 조건) | — | "위치 한계 초과 (ACT_LIMIT)" |

`motor_kp_*`/`motor_kd_*`/`motor_ff_*`는 Gazebo 구동에는 쓰이지 않는다(가제보는
계속 위치 PID로 구동). `ControlOutput.ff_velocity_*_rads`/`ff_torque_*_nm`
진단 필드로만 계산·노출되며, `analyze_log.py`로 실물 로그와 비교할 때 쓴다.

### 상호배타 규칙

실물처럼 `enable_soft_return=True`이면 `enable_zv_hw`는 (설정값과 무관하게)
자동으로 무시되고 소프트복귀가 우선한다. 둘 다 "잔류 흔들림 억제"가 목적이라
동시에 걸면 안 된다는 실물 규칙(sr1 → zv 자동 OFF)을 코드로 강제했다.

### 탱크 형상 불일치 — ZV 주파수 44% 오차와 부분 수정 (2026-09-05)

`enable_hw_style` ON/OFF를 5회 반복 실행(`run_trials.py`)해서 비교해보니
트레이 roll/pitch RMS는 확실히 개선(-46%/-54%)됐지만, **슬로싱(진자) RMS는
거의 그대로**였다(x +9%±12, y +0%±4 — 표준편차가 커서 방향도 불확실). 원인을
추적하니 ZV 성형기의 주파수 불일치였다.

- HW 스타일 `zv_hw_freq_hz=2.00`은 **실물의 사각 탱크**(11×11cm, 350mL)에서
  잰 값이다.
- 이 시뮬레이션의 (기존) 탱크는 **원통**(R=55mm, h=90mm)이라 `compute_slosh_modes()`가
  f1=2.877Hz를 낸다 — 실물과 **44% 어긋난다.**
- ZV는 두 임펄스를 "가정한 주기의 절반" 간격으로 쏴서 정확히 반대위상으로
  상쇄시키는 원리라 주파수가 틀리면 상쇄가 거의 안 된다. 실제로 계산해보면
  (ζ=0, 2임펄스 잔류진동 공식) 이 44% 오차에서 **잔류진동이 무보정 대비 63.6%**
  — 성형 효과가 사실상 없는 수준이다. 위 5회 실험에서 슬로싱이 안 줄어든
  것과 정확히 들어맞는다.

**부분 수정**: 실물 탱크는 사각인데 기존 `compute_slosh_modes()`/`housner_pendulum()`은
원통 전용(베셀함수) 공식이라 반지름만 조정해서는 사각 탱크를 흉내낼 수 없다.
`compute_slosh_modes_rect(side_m, fill_height_m)`을 새로 추가했다 —
`w_n² = (nπg/L)tanh(nπh/L)` (직육면체 1차/2차 모드, `zv_shaping_rtos.ino`
342~346행 주석에 이미 있던 식). 실물 치수(L=0.11m, 350mL→h=28.93mm)를
넣으면 **f1=2.194Hz**가 나오는데, 이게 같은 주석의 "책상 계산값 2.19"와
정확히 일치해 식과 치수 둘 다 교차검증됐다. 실측 2.00Hz와의 나머지 9.7%
차이는 (ino 주석에 이미 기록된 대로) **탱크 형상이 아니라 실물 짐벌 자체의
기계적 유격/탄성** 때문이라, 탱크 형상만으로는 더 못 줄인다.

`CoreConfig.tank_shape`("cylinder" 기본 | "rect")로 선택한다. **`housner_pendulum()`은
사각탱크용으로 확장하지 않았다** — NASA SP-106 계열 등가진자 질량/피벗
공식을 웹 검색으로도 정확한 계수를 확인 못 해, 틀린 물리 상수를 넣느니
미구현으로 남기는 쪽을 택했다(TODO). 그래서 `tank_shape=rect`는 **ZV
목표주파수만 정확해지고, Gazebo에 실제로 물리 시뮬레이션되는
`slosh_pendulum_x/y`(URDF에 원통 공식으로 이미 박혀 있는 질량·관성·피벗)는
그대로**다 — 로그 지표(진자각)가 여전히 원통 근사 물리로 나온다는 뜻이라,
`tank_shape=rect`를 켜도 이번 5회 비교의 슬로싱 RMS 자체가 당장 좋아지지는
않는다(ZV가 노리는 목표가 바뀔 뿐, 흔들리는 진자의 실제 물리는 안 바뀜).
URDF까지 맞추려면 사각탱크 등가진자 공식을 검증한 뒤 별도 작업이 필요하다.

```bash
# 실물 사각탱크 재현 (ZV 목표주파수만)
python3 gimbal_leveling_controller.py --ros-args -p use_sim_time:=true \
    -p tank_shape:=rect -p tank_side_m:=0.11 -p fill_height_m:=0.02893
```

### 포함하지 않은 것

- **`AccelCorrectionGate`** (실물 570~589행): 자이로 예측과 가속도계 각의
  괴리를 보고 상보필터 보정을 일시 차단하는 게이트. 사용자가 지정한 9단계
  제어 체인에 없어 포팅하지 않았다. 이 게이트가 없으므로, **일정한 가속을
  아주 오래 유지하면 θ_ref가 서서히 0으로 수렴**한다(상보필터가 결국
  "이것도 새 수평"으로 학습하기 때문 — 실물 자체의 알려진 모호성이며
  게이트가 원래 이걸 완화하는 용도다). 반응형 텔레옵 가속(현재 148초
  주행 데이터)에서는 지속시간이 짧아 실제 영향은 제한적일 것으로 본다.
- **`REJ_RUN_FAULT`/피드백 0회 카운터**: 실물은 이걸 위 게이트의 `trust`
  값(가속도 1g 이탈 기반)에 연동해서 센다. 이 코어는 이미 있는
  `accel_fault_debounce_frames`(3프레임)와 `feedback_timeout_ms`가 사용자
  지시의 "가속도 이상 연속 3프레임 FAULT" / "피드백 타임아웃"을 이미
  두 경로 공통으로 처리하므로 별도 필드를 추가하지 않았다.

## 실물 이식 대응표

코어는 순수 float 연산, 고정 크기 배열, 동적 할당 없음, 예외 없음(성공/실패는
반환값)으로 작성했다. 클래스 → struct/class, tuple 반환 → 출력 인자로 바꾸면
기계적으로 번역된다.

| 코어 요소 | 이식 위치 | 담당 |
|---|---|---|
| `ImuSample` `ImuPair` `MotorFeedback` `ControlOutput` | 공용 헤더 | 3자 합의 |
| `CoreConfig` 전체 | `Config.h` / `Config.cpp` | 제어·통합 |
| `ComplementaryFilter` | `Control.cpp` | 제어·통합 |
| `resultant_normal_body` `GimbalKinematics` | `Control.cpp` | 제어·통합 |
| `ZvShaper` `ConvolvedZV` | `Control.cpp` | 제어·통합 |
| `GainSchedule` | `Control.cpp` | 제어·통합 |
| `ControlState` `ControlCore.step` | `Control.cpp` + `gimbal_control.ino` | 제어·통합 |
| `set_estimated_slosh_frequency` | `Control.cpp` | 제어·통합 |
| `ImuSample` 채우기 | `Sensor.cpp` | 센서·데이터 |
| `ControlOutput` → CAN 프레임 | `Gimbal.cpp` | 모터·CAN |
| `MotorFeedback` 채우기 | `Gimbal.cpp` | 모터·CAN |

### 번역 시 주의

- `dataclass`는 기본값 있는 struct로. Python은 참조, C++는 값 전달이므로
  `const&`를 쓸 것
- `ZV_BUFFER_SIZE = 128`은 컴파일 타임 상수. `float buf[128]`
- `math.atan2` `asin` `hypot` `tanh` → `<cmath>` 동일 함수
- `ZvShaper.shape()`의 순환버퍼 인덱스는 음수 모듈로를 피하려고
  `(x + SIZE) % SIZE` 형태로 썼다. C++에서도 그대로 유효하다
- `Diagnostics`는 시리얼 로깅용. 팀 요구인 "센서·목표각·성형각·모터
  명령·피드백·상태 기록"에 대응한다

### 실물에서 반드시 바꿀 것

| 항목 | 시뮬레이션 | 실물 |
|---|---|---|
| `require_motor_feedback` | `False` (가제보 위치 컨트롤러는 피드백을 안 준다) | **`True`** |
| `auto_activate` (어댑터) | `True` (편의) | **`False`** — `/gimbal_enable` 상당의 명시적 트리거 |
| `tray_zero_offset_*_rad` | 0 (IMU를 탱크 프레임에 정렬) | 무부하 수평에서 **측정해 채울 것** |
| `joint_limit_rad` | URDF 값 ±25° | GL60II 실제 기구 한계 |
| `max_velocity_rads` | 3.0 | GL60II 정격 |
| `slosh_damping_ratio` | 0.01 (문헌값) | 실측 권장 |
| 축 리맵 (어댑터) | 프리셋 | 실물 장착 방향에 맞춰 `Sensor.cpp`에서 처리 |

## 튜닝 항목 (실물 완성 후)

우선순위 순.

1. **`kp_min` / `kp_max`** — 절대각 오차 보정 이득. 피드포워드가 주 경로이므로
   1.0 미만이 정상. 유격·마찰을 보며 조정
2. **`deadband_roll_rad` / `deadband_pitch_rad`** — 엔코더 분해능과 유격이
   시뮬레이션과 다르다. 떨림이 없어지는 최소값으로
3. **`tank_radius_m` / `fill_height_m`** — ZV 주파수를 결정. 실물 탱크 치수
4. **`slosh_damping_ratio`** — 자유진동 계측으로 실측
5. **`smooth_alpha_*`** — 반응성과 부드러움의 절충
6. **`trim_limit_rad`** — 보정 폭주 상한

## 검증 상태

| 항목 | 상태 |
|---|---|
| 코어 단위 시험 77개 | **통과** (`python3 test_core.py`) |
| 회전/법선 왕복 변환 | 통과 (1e-9) |
| 경사 정지 → 트레이 절대 수평 | 통과 (base 10° → 조인트 −10.000000°) |
| 가속 → 트레이가 합력 방향 수직 | 통과 (atan(a/g) 일치) |
| ZV 임펄스 응답 4탭 / 진폭합 1 / DC이득 1 | 통과 |
| ZV ζ=0 → 진폭 정확히 0.25 (논문 수식 4 일치) | 통과 |
| 주파수 갱신 시 불연속 없음 | 통과 |
| 상태기계 / 자동활성화 금지 / FAULT 전이 | 통과 |
| CNN 인터페이스 검증·폴백 | 통과 |
| `analyze_log.py` 파이프라인 | 통과 (합성 로그) |
| **가제보 실행** | **미검증** — 로컬에 ROS2/gz 없음 |

가제보 검증은 담당자 환경에서 필요하다. 절차는 `가제보_환경_수정요청.md` 참조.

## 선행 조건 (담당자 작업)

이것들이 안 되면 검증이 무의미하다.

1. ~~**IMU 축 정합**~~ — 2026-09-05 조치: 어댑터 기본값을 `base_axis_preset=body2_current_base`,
   `tray_axis_preset=cad_rotated_tray`로 바꿔 소프트웨어 우회로 해결(가제보
   headless 실행으로 실측 확인, "축 진단" 로그가 정상 표시). **근본 해결
   (URDF `<sensor><pose>` 정렬)은 여전히 미완료** — URDF가 바뀌면 프리셋을
   다시 맞춰야 한다
2. **Housner 등가 진자 추가** — 가제보에는 유체가 없어 출렁임이 짐벌에 반력을
   주지 않는다. 진자를 넣어야 제어 난이도가 현실화되고 슬로싱 지표를 계측할 수 있다
3. **`/joint_states` 발행** — `MotorFeedback`과 진자각 로깅에 필요
4. **`d_gain` 15 → 1.5, `effort` 현실값** — 현재 내부 루프가 100Hz 명령을
   따라오지 못한다

상세는 `가제보_환경_수정요청.md`.
