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
| `test_core.py` | 코어 단위 시험 77개 | 검증 근거로 유지 |
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
# 두 파일이 같은 폴더에 있어야 한다
python3 gimbal_leveling_controller.py --ros-args -p use_sim_time:=true

# 축 정합이 아직 안 된 URDF에서
python3 gimbal_leveling_controller.py --ros-args -p use_sim_time:=true \
    -p base_axis_preset:=cad_rotated_base -p tray_axis_preset:=cad_rotated_tray

# 조건 비교용 기록
python3 gimbal_leveling_controller.py --ros-args -p use_sim_time:=true \
    -p enable_zv:=false -p log_csv_path:=/tmp/run_nozv.csv
python3 analyze_log.py /tmp/run_nozv.csv /tmp/run_zv.csv --plot compare.png

# 코어 단위 시험 (ROS 불필요)
python3 test_core.py
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

1. **IMU 축 정합** — 현재 중력이 Z가 아니라 X에 실린다. 노드가 시작 시 자동
   진단을 출력한다
2. **Housner 등가 진자 추가** — 가제보에는 유체가 없어 출렁임이 짐벌에 반력을
   주지 않는다. 진자를 넣어야 제어 난이도가 현실화되고 슬로싱 지표를 계측할 수 있다
3. **`/joint_states` 발행** — `MotorFeedback`과 진자각 로깅에 필요
4. **`d_gain` 15 → 1.5, `effort` 현실값** — 현재 내부 루프가 100Hz 명령을
   따라오지 못한다

상세는 `가제보_환경_수정요청.md`.
