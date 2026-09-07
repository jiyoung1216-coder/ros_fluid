#!/usr/bin/env python3
"""
bench_drive_profile.py

1단계 벤치마크의 "결정론적 입력 생성기". new_robot/scripts/ 밖(로컬,
미커밋)에 있던 drive_test.py의 고정 시퀀스를 그대로 옮겨 레포에 커밋된
정본으로 승격한 것 — 값 자체(구간 길이/선속도/각속도)는 바꾸지 않았다.

이 파일은 /cmd_vel 위로 바퀴-지면 접촉 기반 주행 명령을 계속 내보내는
역할을 한다. 접촉솔버 자체의 비결정성(Gazebo)은 이 스크립트가 제거하지
않는다 — 그건 dual-IMU 단일실행 기록 방식(record_gimbal_accel_for_dsph.py)
과 N=3 반복 + 중앙값/범위 보고로 흡수한다. 이 파일이 보장하는 것은
"보낸 명령 시퀀스 자체가 매 회차 동일함"뿐이다.

두 가지 검증 수단을 제공한다:
  - profile_hash(phases)      : 프로파일 정의 자체의 해시(정의가 실수로
                                 바뀌지 않았음을 증명. 상수라 항상 동일).
  - validate_cmdvel_log(...)  : 실제 발행 로그(cmdvel csv)가 계획한 구간
                                 길이대로 나갔는지 검증(런치 지연/중간 크래시
                                 등으로 트레이스가 잘렸으면 폐기 대상임을
                                 판정하는 실질적 체크).
"""

import hashlib
import sys
import time

# 2026-09-06: drive_test.py(로컬, 미커밋)와 완전히 동일한 값.
# (직진가속 -> 완만한 회전 -> 정지)
PHASES = [
    (2.0, 0.0, 0.0),
    (3.0, 0.8, 0.0),
    (3.0, 0.8, 0.8),
    (0.5, 0.0, 0.0),
    (6.0, 0.0, 0.0),
]

# 2026-09-06: 사용자 요청("잔잔하게, 가속도 문제 안 일어나게") — PHASES의
# 절반 속도(0.4/0.4)로 같은 구조(직진가속 -> 완만한 회전 -> 정지)를 재사용.
# 언덕 유무가 아니라 "주행 자체가 격렬한지"를 분리해서 보기 위한 대조군.
GENTLE_PHASES = [
    (2.0, 0.0, 0.0),
    (3.0, 0.4, 0.0),
    (3.0, 0.4, 0.4),
    (0.5, 0.0, 0.0),
    (6.0, 0.0, 0.0),
]

# 2026-09-06: 시간이 없다는 요청으로 GENTLE_PHASES를 그대로 축소(같은 순서,
# 절반 길이)한 버전 — 평지 확인 후 빠른 1회성 확인용.
SHORT_GENTLE_PHASES = [
    (1.0, 0.0, 0.0),
    (2.0, 0.4, 0.0),
    (2.0, 0.4, 0.4),
    (0.5, 0.0, 0.0),
    (2.0, 0.0, 0.0),
]

# 정지 캘리브레이션 케이스 전용 — 자극 없이 대기만 한다(사용자 승인 답변
# 추가사항 1). 길이 8초는 DualSPHysics accinput/extract_free_surface 파이프
# 라인이 초기 과도응답을 지나 정상상태에 도달하기에 충분한 여유로 선택.
# TODO: 실측 필요 — 실제로 h_calm/sigma가 8초 이내에 수렴하는지 첫 실행 시
# extract_free_surface 결과로 확인하고, 부족하면 이 값만 늘릴 것.
CALIBRATION_PHASES = [
    (8.0, 0.0, 0.0),
]

PUBLISH_DT_S = 0.05  # drive_test.py와 동일


def excitation_end_time(phases):
    """마지막으로 (vx,wz) != (0,0)이었던 구간이 끝나는 절대 시각
    (프로파일 시작을 t=0으로)."""
    t = 0.0
    end = 0.0
    for dur, vx, wz in phases:
        if vx != 0.0 or wz != 0.0:
            end = t + dur
        t += dur
    return end


def total_duration(phases):
    return sum(dur for dur, _, _ in phases)


def profile_hash(phases, dt=PUBLISH_DT_S):
    """phases를 dt 간격으로 전개한 (vx,wz) 시퀀스의 SHA256.
    phases가 모듈 상수라 이 값 자체는 항상 동일하다 — "정의가 실수로
    바뀌지 않았음"을 리포트에 남기기 위한 것이지, 실행 시점의 비결정성을
    잡아내는 용도가 아니다(그건 validate_cmdvel_log가 한다).
    """
    h = hashlib.sha256()
    for dur, vx, wz in phases:
        n = max(1, round(dur / dt))
        for _ in range(n):
            h.update(f"{vx:.6f},{wz:.6f}\n".encode())
    return h.hexdigest()


def validate_cmdvel_log(rows, phases, tol_s=0.5):
    """실제 발행 로그(rows: (t, vx, wz) 시퀀스)가 phases 계획대로 나갔는지
    검증한다. 총 길이가 계획 대비 tol_s 이상 짧으면(런치 지연/중도 종료로
    잘린 경우) invalid로 판정 — 1단계 "해시 불일치 시 폐기·재시도" 규칙의
    실질 구현.

    반환: (valid: bool, reason: str)
    """
    if not rows:
        return False, "빈 로그"
    expected_total = total_duration(phases)
    actual_total = rows[-1][0] - rows[0][0]
    if actual_total < expected_total - tol_s:
        return False, (f"로그 길이 부족: 실제 {actual_total:.2f}s < "
                        f"계획 {expected_total:.2f}s - 허용오차 {tol_s}s")

    # 각 자극 구간(vx 또는 wz != 0)이 최소 한 번은 그 값 근처로 발행됐는지 확인
    t_cursor = 0.0
    for dur, vx, wz in phases:
        if vx != 0.0 or wz != 0.0:
            seen = any(
                t_cursor - tol_s <= t <= t_cursor + dur + tol_s
                and abs(rv - vx) < 1e-3 and abs(rw - wz) < 1e-3
                for t, rv, rw in rows
            )
            if not seen:
                return False, (f"구간 [{t_cursor:.2f}, {t_cursor + dur:.2f}]s "
                                f"(vx={vx}, wz={wz})의 발행 기록이 없음")
        t_cursor += dur
    return True, "ok"


def publish_profile(pub, phases, dt=PUBLISH_DT_S, cmd_log_path=None):
    """(레거시) wall-clock 기준 phase 전환. Gazebo가 항상 정확히 1x
    real-time으로 돈다는 가정 하에서만 유효하다.

    2026-09-06: headless(-s) Gazebo가 <real_time_factor> 설정값과 무관하게
    wall-clock 대비 임의 배속(실측 2~4배, 실행마다 다름)으로 도는 것을
    실측 확인했다 — 이 함수로 만든 excite_end_t는 DualSPHysics
    accinput(sim-time 기준 IMU 타임스탬프)과 어긋난다. 1단계 벤치마크는
    반드시 publish_profile_sim_time()을 써야 한다. 이 함수는 ROS2 없이
    순수 wall-clock 시나리오(오프라인 테스트/문서화용)로만 남겨둔다."""
    from geometry_msgs.msg import Twist  # 테스트 환경에 ROS2가 없어도
                                          # 모듈 임포트는 되게 지연 임포트

    log_f = open(cmd_log_path, "w", newline="") if cmd_log_path else None
    t0 = time.time()
    try:
        for dur, vx, wz in phases:
            phase_t0 = time.time()
            msg = Twist()
            msg.linear.x = vx
            msg.angular.z = wz
            while time.time() - phase_t0 < dur:
                pub.publish(msg)
                if log_f:
                    log_f.write(f"{time.time() - t0:.6f},{vx:.6f},{wz:.6f}\n")
                    log_f.flush()
                time.sleep(dt)
    finally:
        if log_f:
            log_f.close()


def publish_profile_sim_time(node, pub, phases, dt=PUBLISH_DT_S, cmd_log_path=None):
    """phase 전환을 node.get_clock()(use_sim_time=true -> /clock 기반 sim
    time)로 판단한다. headless Gazebo는 wall-clock 대비 임의 배속으로 돌 수
    있으므로(실측), 이게 excite_end_t를 DualSPHysics accinput의 시간축과
    맞출 수 있는 유일한 방법이다. rclpy.spin_once(timeout_sec=dt)가
    "발행 간격 대기"와 "/clock 갱신 수신"을 동시에 한다."""
    import rclpy
    from geometry_msgs.msg import Twist

    while node.get_clock().now().nanoseconds == 0:
        rclpy.spin_once(node, timeout_sec=0.2)

    log_f = open(cmd_log_path, "w", newline="") if cmd_log_path else None
    t0 = node.get_clock().now()
    try:
        for dur, vx, wz in phases:
            phase_t0 = node.get_clock().now()
            msg = Twist()
            msg.linear.x = vx
            msg.angular.z = wz
            while (node.get_clock().now() - phase_t0).nanoseconds * 1e-9 < dur:
                pub.publish(msg)
                if log_f:
                    rel_t = (node.get_clock().now() - t0).nanoseconds * 1e-9
                    log_f.write(f"{rel_t:.6f},{vx:.6f},{wz:.6f}\n")
                    log_f.flush()
                rclpy.spin_once(node, timeout_sec=dt)
    finally:
        if log_f:
            log_f.close()


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.parameter import Parameter

    mode = sys.argv[1] if len(sys.argv) > 1 else "drive"
    prefix = sys.argv[2] if len(sys.argv) > 2 else "bench"
    if mode == "calibration":
        phases = CALIBRATION_PHASES
    elif mode == "gentle":
        phases = GENTLE_PHASES
    elif mode == "short_gentle":
        phases = SHORT_GENTLE_PHASES
    else:
        phases = PHASES

    rclpy.init()
    # use_sim_time을 node 생성 이후 declare_parameter로 켜면 rclpy의
    # TimeSource가 이미 SystemClock으로 굳어 있을 수 있어(경합 조건),
    # 생성 시점에 parameter_overrides로 넘겨 처음부터 ROS 클럭(=/clock
    # 기반 sim time)을 쓰게 한다 — 이 도구는 항상 실행 중인 Gazebo를
    # 대상으로 하므로 True 고정.
    node = rclpy.create_node(
        "bench_drive_profile",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    cmd_log_path = f"{prefix}_cmdvel.csv"
    publish_profile_sim_time(node, pub, phases, cmd_log_path=cmd_log_path)
    pub.publish(Twist())
    node.destroy_node()
    rclpy.shutdown()

    h = profile_hash(phases)
    with open(f"{prefix}_cmdvel.sha256", "w") as f:
        f.write(h + "\n")
    print(f"[bench_drive_profile] mode={mode} phase_hash={h} cmd_log={cmd_log_path}")


if __name__ == "__main__":
    main()
