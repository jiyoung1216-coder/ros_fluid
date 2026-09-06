#!/usr/bin/env python3
"""
test_core.py — gimbal_control_core 단위 시험

코어가 플랫폼 독립이므로 ROS나 가제보 없이 순수 Python으로 검증할 수 있다.
팀 문서 CONTROL_INTEGRATION_TASK.md의 구현 순서 3번
"가짜 센서 입력으로 합력 목표각 검증"에 해당한다.

    python3 test_core.py

의존성 없음(pytest 불필요). 실패하면 AssertionError로 즉시 멈춘다.
"""

import math
import sys

from gimbal_control_core import (
    GRAVITY, ComplementaryFilter, ControlCore, ControlOutput, ControlState,
    ConvolvedZV, CoreConfig, FixedFreqZV, GainSchedule, GimbalKinematics,
    ImuPair, ImuSample, MotorFeedback, SoftReturnAxis, SoftReturnPhase,
    apply_deadband, attitude_matrix, compute_slosh_modes, compute_slosh_modes_rect,
    housner_pendulum, hw_force_vector_angle, mat_col_z, normal_to_roll_pitch,
    resultant_normal_body, slew_rate_limit,
)

DEG = math.pi / 180.0
_passed = 0


def check(name, cond, detail=""):
    global _passed
    if not cond:
        print(f"  [FAIL] {name}  {detail}")
        raise AssertionError(name)
    _passed += 1
    print(f"  [ok] {name}" + (f"  {detail}" if detail else ""))


def close(a, b, tol):
    return abs(a - b) <= tol


# ---------------------------------------------------------------------------
# 가짜 센서 생성기
# ---------------------------------------------------------------------------

def specific_force_body(roll, pitch, ax_w=0.0, ay_w=0.0, az_w=0.0):
    """자세 (roll,pitch)에 있고 월드 가속도 (ax_w,ay_w,az_w)로 움직이는 강체의
    가속도계 읽음값(body 프레임).

    비력 f_world = a_world - g_vec = (ax, ay, az + g)
    f_body = R^T * f_world
    """
    r = attitude_matrix(roll, pitch)
    fw = (ax_w, ay_w, az_w + GRAVITY)
    # R^T * fw
    return tuple(sum(r[k][i] * fw[k] for k in range(3)) for i in range(3))


def make_sample(roll, pitch, t_us, ax_w=0.0, ay_w=0.0, az_w=0.0,
                gx=0.0, gy=0.0, gz=0.0):
    fx, fy, fz = specific_force_body(roll, pitch, ax_w, ay_w, az_w)
    return ImuSample(ax_mps2=fx, ay_mps2=fy, az_mps2=fz,
                     gx_rads=gx, gy_rads=gy, gz_rads=gz,
                     sampled_at_us=t_us, valid=True)


def make_motor(t_ms):
    return MotorFeedback(received_at_ms=t_ms, valid=True, error_code=0)


def geometry_config(**kw):
    """기하 검증용 설정: 데드밴드와 평활을 끄고 순수 기구학만 본다."""
    cfg = CoreConfig()
    cfg.deadband_roll_rad = 0.0
    cfg.deadband_pitch_rad = 0.0
    cfg.smooth_alpha_roll = 1.0
    cfg.smooth_alpha_pitch = 1.0
    cfg.require_motor_feedback = False
    # 적분항은 시간에 걸쳐 서서히 수렴하는 별도 동역학이라 "순수 기구학"
    # 비교(예: ZV ON/OFF가 같은 정상상태로 수렴하는지)를 오염시킨다.
    # 적분항 자체의 동작은 별도 테스트(test_integral_reduces_sustained_bias)에서
    # 명시적으로 켜서 검증한다.
    cfg.enable_integral = False
    for k, v in kw.items():
        setattr(cfg, k, v)
    return cfg


def run_core(core, base_att, tray_att, steps, ax_w=0.0, ay_w=0.0, dt=0.01):
    """core를 steps번 돌린다. tray_att가 None이면 조인트 명령을 실제로 반영한
    닫힌 루프처럼 트레이 자세를 갱신한다(기하 일관성 확인용)."""
    out = ControlOutput()
    t_us = 0
    for i in range(steps):
        t_us += int(dt * 1e6)
        t_ms = t_us // 1000
        br, bp = base_att
        if tray_att is None:
            # 조인트 명령이 즉시 반영된다고 가정한 이상적 짐벌
            tr, tp, _ = GimbalKinematics.compose_absolute(
                br, bp, out.x_position_rad, out.y_position_rad)
        else:
            tr, tp = tray_att
        imu = ImuPair(
            base=make_sample(br, bp, t_us, ax_w=ax_w, ay_w=ay_w),
            tray=make_sample(tr, tp, t_us, ax_w=ax_w, ay_w=ay_w),
        )
        out = core.step(imu, make_motor(t_ms), make_motor(t_ms), dt, t_us, t_ms)
    return out


# ---------------------------------------------------------------------------

def test_rotation_utils():
    print("\n[1] 회전 유틸 / 법선 변환")
    for r_deg, p_deg in ((0, 0), (10, 0), (0, 15), (-12, 20), (24, -18)):
        r, p = r_deg * DEG, p_deg * DEG
        nx, ny, nz = mat_col_z(attitude_matrix(r, p))
        rr, pp, ok = normal_to_roll_pitch(nx, ny, nz)
        check(f"왕복 변환 roll={r_deg} pitch={p_deg}",
              ok and close(rr, r, 1e-9) and close(pp, p, 1e-9),
              f"-> {rr/DEG:.4f}, {pp/DEG:.4f}")

    # 법선 공식 자체 확인: n = (sin p cos r, -sin r, cos p cos r)
    r, p = 12 * DEG, 20 * DEG
    nx, ny, nz = mat_col_z(attitude_matrix(r, p))
    check("법선 해석식 일치",
          close(nx, math.sin(p) * math.cos(r), 1e-12)
          and close(ny, -math.sin(r), 1e-12)
          and close(nz, math.cos(p) * math.cos(r), 1e-12))


def test_deadband():
    print("\n[2] 연속 데드밴드")
    th = 0.05
    check("임계값 이하는 0", apply_deadband(0.04, th) == 0.0)
    check("임계값에서 연속", close(apply_deadband(0.0500001, th), 0.0, 1e-6))
    check("초과분만 통과", close(apply_deadband(0.08, th), 0.03, 1e-12))
    check("음수 대칭", close(apply_deadband(-0.08, th), -0.03, 1e-12))
    # 하드 컷과 달리 임계값 근처에서 점프가 없어야 한다
    jump = abs(apply_deadband(0.0501, th) - apply_deadband(0.0499, th))
    check("임계값 부근 점프 없음", jump < 1e-3, f"jump={jump:.2e}")


def test_complementary_filter():
    print("\n[3] 상보필터")
    f = ComplementaryFilter(0.98)
    # 정지 + 경사 12/-8도. 첫 표본에서 바로 수렴해야 한다(0에서 시작하지 않음)
    r, p = 12 * DEG, -8 * DEG
    s = make_sample(r, p, 0)
    f.update(s, 0.01)
    check("첫 표본에서 즉시 초기화",
          close(f.roll_rad, r, 1e-6) and close(f.pitch_rad, p, 1e-6),
          f"roll={f.roll_rad/DEG:.3f} pitch={f.pitch_rad/DEG:.3f}")

    # 200스텝 정지 유지 -> 드리프트 없어야 함
    for i in range(200):
        f.update(make_sample(r, p, i * 10_000), 0.01)
    check("정지 상태 드리프트 없음",
          close(f.roll_rad, r, 1e-4) and close(f.pitch_rad, p, 1e-4),
          f"roll={f.roll_rad/DEG:.4f} pitch={f.pitch_rad/DEG:.4f}")

    # 자이로만 주고 가속도는 수평으로 고정 -> 가속도항이 서서히 되돌린다
    f2 = ComplementaryFilter(0.98)
    f2.update(make_sample(0, 0, 0), 0.01)
    for i in range(100):
        s = make_sample(0, 0, i * 10_000, gx=1.0)  # 1 rad/s roll
        f2.update(s, 0.01)
    check("자이로 적분이 반영됨", f2.roll_rad > 0.1,
          f"roll={f2.roll_rad/DEG:.2f}deg (자이로 1rad/s x 1s, 가속도항이 억제)")


def test_resultant_target():
    print("\n[4] 합력 목표각 (수식 2,3)")
    # 수평 정지 -> 법선 (0,0,1)
    nx, ny, nz, n = resultant_normal_body(make_sample(0, 0, 0))
    check("수평 정지 -> 법선 수직",
          close(nx, 0, 1e-9) and close(ny, 0, 1e-9) and close(nz, 1, 1e-9))
    check("비력 크기 = g", close(n, GRAVITY, 1e-9), f"{n:.5f}")

    # 수평 + 전방 가속 a -> pitch = atan(a/g), roll = 0
    for a in (0.5, 2.0, 5.0):
        nx, ny, nz, _ = resultant_normal_body(make_sample(0, 0, 0, ax_w=a))
        r, p, ok = normal_to_roll_pitch(nx, ny, nz)
        check(f"전방 가속 {a} m/s^2 -> pitch=atan(a/g)",
              ok and close(p, math.atan2(a, GRAVITY), 1e-9) and close(r, 0, 1e-9),
              f"pitch={p/DEG:.3f}deg (기대 {math.atan2(a,GRAVITY)/DEG:.3f})")

    # 수평 + 좌측 가속 a -> roll = -atan(a/g)
    a = 3.0
    nx, ny, nz, _ = resultant_normal_body(make_sample(0, 0, 0, ay_w=a))
    r, p, ok = normal_to_roll_pitch(nx, ny, nz)
    check("좌측 가속 -> roll 부호 반대",
          ok and close(r, -math.atan2(a, GRAVITY), 1e-9),
          f"roll={r/DEG:.3f}deg")

    # 경사 정지: 목표 법선을 역기구학에 넣으면 base 경사를 정확히 상쇄해야 한다
    for br_deg, bp_deg in ((10, 0), (0, -15), (12, 18), (-20, 8)):
        br, bp = br_deg * DEG, bp_deg * DEG
        nx, ny, nz, _ = resultant_normal_body(make_sample(br, bp, 0))
        qr, qp, ok = GimbalKinematics.inverse(nx, ny, nz)
        ar, ap, ok2 = GimbalKinematics.compose_absolute(br, bp, qr, qp)
        check(f"경사 정지 base=({br_deg},{bp_deg}) -> 트레이 절대각 0",
              ok and ok2 and close(ar, 0, 1e-9) and close(ap, 0, 1e-9),
              f"joint=({qr/DEG:.2f},{qp/DEG:.2f}) abs=({ar/DEG:.2e},{ap/DEG:.2e})")


def test_slosh_modes():
    print("\n[5] 슬로싱 모드 / Housner 파라미터")
    f1, f2 = compute_slosh_modes(0.055, 0.090)
    check("시뮬레이션 탱크 1차 모드 ~2.877Hz", close(f1, 2.877, 0.005), f"f1={f1:.4f}Hz")
    check("2차 모드 ~4.908Hz", close(f2, 4.908, 0.005), f"f2={f2:.4f}Hz")
    check("f2 > f1", f2 > f1)

    h = housner_pendulum(0.055, 0.090)
    check("물 질량 ~0.855kg", close(h["m_liquid_kg"], 0.8553, 1e-3),
          f"{h['m_liquid_kg']:.4f}kg")
    check("m0 + m1 = 전체", close(h["m0_kg"] + h["m1_kg"], h["m_liquid_kg"], 1e-12))
    check("진자 길이 ~30.0mm", close(h["length_m"], 0.03002, 1e-4),
          f"L={h['length_m']*1000:.2f}mm")
    check("진자 주파수 = 1차 모드", close(h["f1_hz"], f1, 1e-9))
    check("피벗 높이 ~92.9mm", close(h["pivot_height_m"], 0.09294, 1e-4),
          f"{h['pivot_height_m']*1000:.2f}mm")

    # 반경이 커지면 주파수가 낮아진다 (원본 21샘플 = 2.38Hz 는 R~80mm)
    f80, _ = compute_slosh_modes(0.080, 0.090)
    check("R=80mm -> f1 ~2.35Hz (원본 ZV 지연이 겨냥한 값)",
          close(f80, 2.354, 0.01), f"f1={f80:.4f}Hz")

    # 사각탱크(compute_slosh_modes_rect) — 실물 탱크 실측값으로 검증.
    # 11x11cm, 350mL -> h=28.93mm. 실물 zv_shaping_rtos.ino 주석의
    # "책상 계산값 2.19"와 일치해야 한다(실측 2.00Hz와의 9%차는 짐벌
    # 기계적 유격 때문이라고 그 주석에 이미 설명돼 있음. 탱크 형상 오차 아님).
    h_real_tank = 350e-6 / (0.11 * 0.11)
    f1r, f2r = compute_slosh_modes_rect(0.11, h_real_tank)
    check("실물 사각탱크(11cm,350mL) 1차 모드 ~2.194Hz (ino 주석 '책상계산 2.19'와 일치)",
          close(f1r, 2.194, 0.005), f"f1={f1r:.4f}Hz, h={h_real_tank*1000:.2f}mm")
    check("사각탱크 2차 모드 > 1차", f2r > f1r, f"f2={f2r:.4f}Hz")

    # ControlCore가 tank_shape="rect"를 실제로 반영하는지 (배선 확인)
    cfg_rect = geometry_config(tank_shape="rect", tank_side_m=0.11,
                               fill_height_m=h_real_tank)
    core_rect = ControlCore(cfg_rect)
    check("ControlCore tank_shape=rect 배선", close(core_rect._fixed_f1_hz, f1r, 1e-9),
          f"core._fixed_f1_hz={core_rect._fixed_f1_hz:.4f}Hz")
    # tank_shape 기본값(cylinder)은 기존 동작 그대로여야 한다 (회귀 확인)
    cfg_default = geometry_config()
    core_default = ControlCore(cfg_default)
    check("ControlCore tank_shape 기본값은 cylinder(기존 동작 유지)",
          close(core_default._fixed_f1_hz, f1, 1e-9))


def test_zv():
    print("\n[6] Convolved ZV (수식 4)")
    dt = 0.01
    zeta = 0.01
    f1, f2 = compute_slosh_modes(0.055, 0.090)

    zv = ConvolvedZV(dt, 0.0)
    ok = zv.configure(f1, f2, zeta, dt, immediate=True)
    check("셰이퍼 설정 성공", ok)

    sh = zv.active
    check("임펄스 4개", len(sh.delays) == 4 and len(sh.amps) == 4)
    check("진폭 합 = 1", close(sum(sh.amps), 1.0, 1e-12), f"sum={sum(sh.amps):.12f}")
    check("지연 = [0, n2, n1, n1+n2] = [0,10,17,27]",
          sh.delays == [0, 10, 17, 27], f"{sh.delays}")
    check("진폭이 모두 0.25 근처 (zeta=0.01)",
          all(abs(a - 0.25) < 0.01 for a in sh.amps),
          f"{[round(a,5) for a in sh.amps]}")

    # zeta=0 이면 논문 수식 (4)와 정확히 일치: 전부 0.25
    zv0 = ConvolvedZV(dt, 0.0)
    zv0.configure(f1, f2, 0.0, dt, immediate=True)
    check("zeta=0 -> 진폭 정확히 0.25",
          all(close(a, 0.25, 1e-12) for a in zv0.active.amps),
          f"{[round(a,6) for a in zv0.active.amps]}")

    # 임펄스 응답: 지연 위치에만 값이 나타나야 한다
    zv.reset(0.0)
    resp = []
    for i in range(40):
        resp.append(zv.apply(1.0 if i == 0 else 0.0))
    nz_idx = [i for i, v in enumerate(resp) if abs(v) > 1e-9]
    check("임펄스 응답이 지연 위치에만", nz_idx == [0, 10, 17, 27], f"{nz_idx}")
    check("임펄스 응답 합 = 1", close(sum(resp), 1.0, 1e-12), f"{sum(resp):.12f}")

    # 계단 입력: DC 이득 1 (정상상태에서 목표값을 그대로 통과)
    zv.reset(0.0)
    last = 0.0
    for i in range(120):
        last = zv.apply(1.0)
    check("계단 입력 DC 이득 = 1", close(last, 1.0, 1e-12), f"{last:.12f}")

    # 주파수 갱신 시 불연속 없음 (팀 완료 기준)
    zv2 = ConvolvedZV(dt, 0.20)
    zv2.configure(f1, f2, zeta, dt, immediate=True)
    zv2.reset(0.0)
    hist = []
    for i in range(60):
        hist.append(zv2.apply(math.sin(2 * math.pi * 0.5 * i * dt)))
    before = hist[-1]
    zv2.configure(2.0, 3.5, zeta, dt)          # 주파수 크게 변경
    after = zv2.apply(math.sin(2 * math.pi * 0.5 * 60 * dt))
    step_jump = abs(after - before)
    # 같은 입력을 계속 넣었을 때의 정상 변화량과 비교한다
    typical = max(abs(hist[i + 1] - hist[i]) for i in range(50, 59))
    check("주파수 갱신 시 출력 불연속 없음",
          step_jump < typical * 3.0 + 1e-6,
          f"jump={step_jump:.6f}, 통상 변화={typical:.6f}")

    # 버퍼를 넘는 지연은 거부되어야 한다 (설정이 바뀌지 않음)
    zv3 = ConvolvedZV(dt, 0.0)
    zv3.configure(f1, f2, zeta, dt, immediate=True)
    saved = list(zv3.active.delays)
    rejected = not zv3.configure(0.05, 0.05, zeta, dt, immediate=True)
    check("버퍼 초과 주파수 거부", rejected and zv3.active.delays == saved)


def test_gain_schedule():
    print("\n[7] 게인 스케줄링 (수식 5)")
    g = GainSchedule(0.10, 0.50, 15 * DEG)
    check("오차 0 -> kp_min", close(g.gain(0.0), 0.10, 1e-12))
    check("오차 e_max -> kp_max", close(g.gain(15 * DEG), 0.50, 1e-12))
    check("오차 e_max 초과 -> kp_max로 포화", close(g.gain(40 * DEG), 0.50, 1e-12))
    mid = g.gain(7.5 * DEG)   # ratio=0.5 -> 0.10 + 0.40*0.25 = 0.20
    check("중간 오차는 제곱 특성", close(mid, 0.20, 1e-12), f"kp={mid:.4f}")
    check("단조 증가", g.gain(2 * DEG) < g.gain(5 * DEG) < g.gain(10 * DEG))


def test_state_machine():
    print("\n[8] 상태기계 / 안전")
    cfg = geometry_config()
    core = ControlCore(cfg)
    core.init_control()
    check("초기 상태 BOOT", core.state == ControlState.BOOT)

    t_us, t_ms = 10_000, 10
    imu = ImuPair(base=make_sample(0, 0, t_us), tray=make_sample(0, 0, t_us))
    out = core.step(imu, make_motor(t_ms), make_motor(t_ms), 0.01, t_us, t_ms)
    check("점검 통과 후 READY", core.state == ControlState.READY)
    check("READY에서는 출력 비활성 (자동 활성화 금지)", out.enable is False)

    # 활성화 요청 전에는 몇 주기를 돌려도 ACTIVE로 가지 않는다
    for i in range(2, 30):
        t_us, t_ms = i * 10_000, i * 10
        imu = ImuPair(base=make_sample(0, 0, t_us), tray=make_sample(0, 0, t_us))
        out = core.step(imu, make_motor(t_ms), make_motor(t_ms), 0.01, t_us, t_ms)
    check("요청 없이 ACTIVE로 자동 전환하지 않음",
          core.state == ControlState.READY and out.enable is False)

    core.request_activate()
    t_us, t_ms = 300_000, 300
    imu = ImuPair(base=make_sample(0, 0, t_us), tray=make_sample(0, 0, t_us))
    out = core.step(imu, make_motor(t_ms), make_motor(t_ms), 0.01, t_us, t_ms)
    check("요청 후 ACTIVE 진입", core.state == ControlState.ACTIVE and out.enable is True)

    # 센서 무효 -> FAULT, 출력 중단
    bad = ImuPair(base=make_sample(0, 0, t_us), tray=ImuSample(valid=False))
    t_us, t_ms = 310_000, 310
    out = core.step(bad, make_motor(t_ms), make_motor(t_ms), 0.01, t_us, t_ms)
    check("센서 무효 -> FAULT", core.state == ControlState.FAULT)
    check("FAULT에서 출력 중단", out.enable is False)
    check("FAULT 사유 기록", "IMU" in core.diag.fault_reason, core.diag.fault_reason)

    # FAULT는 스스로 빠져나오지 않는다
    t_us, t_ms = 320_000, 320
    good = ImuPair(base=make_sample(0, 0, t_us), tray=make_sample(0, 0, t_us))
    out = core.step(good, make_motor(t_ms), make_motor(t_ms), 0.01, t_us, t_ms)
    check("FAULT 자동 복귀 없음",
          core.state == ControlState.FAULT and out.enable is False)
    check("clear_fault로만 복귀", core.clear_fault() and core.state == ControlState.BOOT)

    # 센서 타임아웃
    core2 = ControlCore(geometry_config())
    core2.init_control()
    t_us, t_ms = 1_000_000, 1000
    stale = ImuPair(base=make_sample(0, 0, t_us - 200_000),
                    tray=make_sample(0, 0, t_us - 200_000))
    out = core2.step(stale, make_motor(t_ms), make_motor(t_ms), 0.01, t_us, t_ms)
    check("오래된 표본으로는 READY까지 못 감",
          core2.state == ControlState.SENSOR_CHECK and out.enable is False)

    # 주기 누락 반복 -> FAULT
    core3 = ControlCore(geometry_config())
    core3.init_control()
    for i in range(1, 4):
        t_us, t_ms = i * 10_000, i * 10
        imu = ImuPair(base=make_sample(0, 0, t_us), tray=make_sample(0, 0, t_us))
        core3.step(imu, make_motor(t_ms), make_motor(t_ms), 0.01, t_us, t_ms)
    core3.request_activate()
    t_us, t_ms = 100_000, 100
    imu = ImuPair(base=make_sample(0, 0, t_us), tray=make_sample(0, 0, t_us))
    core3.step(imu, make_motor(t_ms), make_motor(t_ms), 0.01, t_us, t_ms)
    for i in range(6):
        t_us += 500_000
        t_ms = t_us // 1000
        imu = ImuPair(base=make_sample(0, 0, t_us), tray=make_sample(0, 0, t_us))
        out = core3.step(imu, make_motor(t_ms), make_motor(t_ms), 0.5, t_us, t_ms)
    check("주기 누락 반복 -> FAULT",
          core3.state == ControlState.FAULT and out.enable is False,
          core3.diag.fault_reason)

    # 모터 피드백 요구 시 피드백 없으면 READY로 못 간다
    cfg_m = geometry_config()
    cfg_m.require_motor_feedback = True
    core4 = ControlCore(cfg_m)
    core4.init_control()
    t_us, t_ms = 10_000, 10
    imu = ImuPair(base=make_sample(0, 0, t_us), tray=make_sample(0, 0, t_us))
    out = core4.step(imu, MotorFeedback(), MotorFeedback(), 0.01, t_us, t_ms)
    check("모터 피드백 없으면 MOTOR_CHECK에서 대기",
          core4.state == ControlState.MOTOR_CHECK and out.enable is False)


def test_closed_loop_geometry():
    print("\n[9] 닫힌 루프 기하 (이상적 짐벌 가정)")
    # 수평 정지 -> 명령 0
    core = ControlCore(geometry_config())
    core.init_control()
    core.request_activate()
    out = run_core(core, (0.0, 0.0), None, 300)
    check("수평 정지 -> 조인트 명령 ~0",
          close(out.x_position_rad, 0, 1e-6) and close(out.y_position_rad, 0, 1e-6),
          f"roll={out.x_position_rad/DEG:.4f} pitch={out.y_position_rad/DEG:.4f}")

    # 경사 정지 -> 트레이 절대 수평
    for br_deg, bp_deg in ((10, 0), (0, 15), (-12, 18)):
        core = ControlCore(geometry_config())
        core.init_control()
        core.request_activate()
        out = run_core(core, (br_deg * DEG, bp_deg * DEG), None, 400)
        ar, ap, _ = GimbalKinematics.compose_absolute(
            br_deg * DEG, bp_deg * DEG, out.x_position_rad, out.y_position_rad)
        check(f"경사 정지 base=({br_deg},{bp_deg}) -> 트레이 수평 유지",
              abs(ar) < 0.5 * DEG and abs(ap) < 0.5 * DEG,
              f"트레이 절대각=({ar/DEG:.3f},{ap/DEG:.3f})deg "
              f"joint=({out.x_position_rad/DEG:.2f},{out.y_position_rad/DEG:.2f})")

    # 정상 가속 -> 트레이가 합력 방향에 수직
    a = 2.0
    core = ControlCore(geometry_config())
    core.init_control()
    core.request_activate()
    out = run_core(core, (0.0, 0.0), None, 500, ax_w=a)
    expect = math.atan2(a, GRAVITY)
    ar, ap, _ = GimbalKinematics.compose_absolute(0, 0, out.x_position_rad,
                                                  out.y_position_rad)
    check(f"전방 가속 {a} m/s^2 -> 트레이 절대 pitch = atan(a/g)",
          close(ap, expect, 1.0 * DEG),
          f"실제 {ap/DEG:.3f}deg, 기대 {expect/DEG:.3f}deg")

    # 조인트 한계 클램프
    cfg = geometry_config(joint_limit_rad=10 * DEG)
    core = ControlCore(cfg)
    core.init_control()
    core.request_activate()
    out = run_core(core, (40 * DEG, 0.0), None, 400)
    check("조인트 한계 클램프",
          abs(out.x_position_rad) <= 10 * DEG + 1e-9,
          f"roll={out.x_position_rad/DEG:.3f}deg (한계 10)")


def test_flags():
    print("\n[10] 기능 플래그")
    base = (10 * DEG, 0.0)

    cfg = geometry_config(enable_feedforward=False, enable_trim=False)
    core = ControlCore(cfg)
    core.init_control()
    core.request_activate()
    out = run_core(core, base, None, 200)
    check("피드포워드/보정 모두 OFF -> 명령 0",
          close(out.x_position_rad, 0, 1e-12) and close(out.y_position_rad, 0, 1e-12))

    # ZV의 DC 이득이 1이므로 정상상태에서는 ON/OFF가 같은 값에 수렴해야 한다.
    # ZV는 최대 27샘플(0.27s) 지연을 주므로 충분히 오래 돌려야 비교가 성립한다.
    cfg = geometry_config(enable_zv=False)
    core = ControlCore(cfg)
    core.init_control()
    core.request_activate()
    out_nozv = run_core(core, base, None, 3000)
    cfg = geometry_config(enable_zv=True)
    core = ControlCore(cfg)
    core.init_control()
    core.request_activate()
    out_zv = run_core(core, base, None, 3000)
    check("ZV ON/OFF 모두 정상상태에서 같은 값에 수렴 (DC 이득 1)",
          close(out_nozv.x_position_rad, out_zv.x_position_rad, 1e-5),
          f"OFF={out_nozv.x_position_rad/DEG:.6f} ON={out_zv.x_position_rad/DEG:.6f} "
          f"차이={abs(out_nozv.x_position_rad-out_zv.x_position_rad)/DEG:.2e}deg")

    cfg = geometry_config(enable_gain_scheduling=False)
    core = ControlCore(cfg)
    core.init_control()
    core.request_activate()
    run_core(core, base, None, 50)
    check("게인 스케줄링 OFF -> kp 고정",
          close(core.diag.kp_roll, cfg.kp_max, 1e-12), f"kp={core.diag.kp_roll}")


def test_integral_reduces_sustained_bias():
    print("\n[10.5] 적분항 — 지속되는 오차에 대한 동작")
    # 트레이가 목표(수평)까지 못 가고 5deg에 고착됐다고 가정한다(액추에이터
    # 포화/지연 상황을 흉내). tray_att를 고정값으로 넘기면 명령이 트레이
    # 실측에 반영되지 않는 열린 루프라, 매 스텝 똑같은 오차가 지속된다.
    base = (10 * DEG, 0.0)
    stuck_tray = (5 * DEG, 0.0)

    cfg = geometry_config(enable_integral=False)
    core = ControlCore(cfg)
    core.init_control()
    core.request_activate()
    out_5s = run_core(core, base, stuck_tray, 500)
    out_30s = run_core(core, base, stuck_tray, 2500)
    check("P만 있으면 지속 오차에도 출력이 곧바로 고정값에서 안 변함",
          close(out_5s.x_position_rad, out_30s.x_position_rad, 1e-9),
          f"5s={out_5s.x_position_rad/DEG:.4f}deg 30s={out_30s.x_position_rad/DEG:.4f}deg")

    cfg = geometry_config(enable_integral=True)
    core = ControlCore(cfg)
    core.init_control()
    core.request_activate()
    out_5s = run_core(core, base, stuck_tray, 500)
    out_30s = run_core(core, base, stuck_tray, 2500)
    check("P+I는 지속 오차가 남아있는 한 출력이 계속 커짐(고착 상태 안 됨)",
          abs(out_30s.x_position_rad) > abs(out_5s.x_position_rad) + 0.5 * DEG,
          f"5s={out_5s.x_position_rad/DEG:.4f}deg 30s={out_30s.x_position_rad/DEG:.4f}deg")


def test_deadband_in_loop():
    print("\n[11] 데드밴드 동작 (기본 설정)")
    cfg = CoreConfig()
    cfg.require_motor_feedback = False
    core = ControlCore(cfg)
    core.init_control()
    core.request_activate()
    # roll 데드밴드 3도보다 작은 경사 -> 피드포워드는 0, 보정만 남는다
    out = run_core(core, (1.5 * DEG, 0.0), None, 400)
    check("데드밴드 이내 경사에서는 명령이 작게 유지",
          abs(out.x_position_rad) < 2.0 * DEG,
          f"roll={out.x_position_rad/DEG:.4f}deg")
    # 데드밴드보다 큰 경사 -> 확실히 움직인다
    core = ControlCore(cfg)
    core.init_control()
    core.request_activate()
    out = run_core(core, (15 * DEG, 0.0), None, 600)
    check("데드밴드 초과 경사에서는 보정 동작",
          out.x_position_rad < -8 * DEG,
          f"roll={out.x_position_rad/DEG:.3f}deg (base +15 -> 반대 방향)")


def test_cnn_interface():
    print("\n[12] CNN 주파수 입력 인터페이스")
    core = ControlCore(geometry_config())
    core.init_control()
    f1_cfg = core.diag.zv_f1_hz if core.diag.zv_f1_hz else core._fixed_f1_hz

    check("신뢰도 미달 -> 거부, 고정값 유지",
          core.set_estimated_slosh_frequency(3.0, 3.0, confidence=0.2) is False
          and core._freq_source == "config")
    check("범위 밖 주파수 -> 거부",
          core.set_estimated_slosh_frequency(0.1, 3.0, confidence=0.9) is False
          and core._freq_source == "config")
    check("유효 입력 -> 반영",
          core.set_estimated_slosh_frequency(3.2, 3.4, confidence=0.9) is True
          and core._freq_source == "estimator",
          f"f1={core._f1_hz:.3f}Hz")
    check("이후 신뢰도 미달 -> 고정값으로 폴백",
          core.set_estimated_slosh_frequency(3.2, 3.4, confidence=0.1) is False
          and core._freq_source == "config"
          and close(core._f1_hz, core._fixed_f1_hz, 1e-12))


def hw_config(**kw):
    """HW 스타일 경로 검증용 설정. require_motor_feedback=False는 어댑터
    기본값과 동일(가제보 위치 컨트롤러가 피드백을 안 준다)."""
    cfg = CoreConfig()
    cfg.enable_hw_style = True
    cfg.require_motor_feedback = False
    for k, v in kw.items():
        setattr(cfg, k, v)
    return cfg


def run_hw_core(core, base_att, steps, ax_w=0.0, ay_w=0.0, az_w=0.0, dt=0.01,
                 motor_roll=None, motor_pitch=None):
    """core(enable_hw_style=True)를 steps번 돌린다. tray는 실물에 없으므로
    base와 같은 자세로 채워 로깅 일관성만 맞춘다(제어에는 안 쓰인다)."""
    br, bp = base_att
    out = ControlOutput()
    t_us = 0
    for _ in range(steps):
        t_us += int(dt * 1e6)
        t_ms = t_us // 1000
        imu = ImuPair(
            base=make_sample(br, bp, t_us, ax_w=ax_w, ay_w=ay_w, az_w=az_w),
            tray=make_sample(br, bp, t_us, ax_w=ax_w, ay_w=ay_w, az_w=az_w),
        )
        mr = motor_roll if motor_roll is not None else make_motor(t_ms)
        mp = motor_pitch if motor_pitch is not None else make_motor(t_ms)
        out = core.step(imu, mr, mp, dt, t_us, t_ms)
    return out


def test_hw_style():
    print("\n[13] HW 스타일 경로 (실물 zv_shaping_rtos.ino 포팅)")

    # (a) 정지·수평 -> 목표각 0. ax=ay=0이면 force_pitch/roll이 LPF 상태와
    # 무관하게 atan2(0,·)=0으로 즉시 0이 되고, theta_base도 첫 표본에서
    # 가속도계 각으로 바로 초기화되어 역시 0이다.
    core = ControlCore(hw_config())
    core.init_control()
    core.request_activate()
    out = run_hw_core(core, (0.0, 0.0), 5)
    check("정지·수평 -> 목표각 0",
          close(out.x_position_rad, 0.0, 1e-9) and close(out.y_position_rad, 0.0, 1e-9),
          f"roll={out.x_position_rad:.3e} pitch={out.y_position_rad:.3e}")

    # (b) 합력벡터 각 공식 자체의 정확성 (hw_force_vector_angle, 실물
    # force_pitch/force_roll 식 그대로: pitch<-ay, roll<-(-ax)). 축 하나만
    # 가속도를 실어 90도 근방의 명확한 값으로 검증한다.
    p, r = hw_force_vector_angle(0.0, 0.0, GRAVITY)
    check("합력벡터각: 중력만 -> (0,0)", close(p, 0.0, 1e-9) and close(r, 0.0, 1e-9))
    p, r = hw_force_vector_angle(GRAVITY, 0.0, 0.0)
    check("합력벡터각: +ax만 -> roll=-90deg",
          close(p, 0.0, 1e-9) and close(r, -math.pi / 2, 1e-9), f"roll={math.degrees(r):.3f}")
    p, r = hw_force_vector_angle(0.0, GRAVITY, 0.0)
    check("합력벡터각: +ay만 -> pitch=+90deg",
          close(p, math.pi / 2, 1e-9) and close(r, 0.0, 1e-9), f"pitch={math.degrees(p):.3f}")

    # (c) ZV 2.00Hz, zeta=0 -> 반주기 지연 샘플수. Td/2 = 0.5/(f*sqrt(1-z^2))
    # = 0.5/2.00 = 0.25s = 25샘플(dt=0.01s). zeta=0이면 ZV 진폭 정확히
    # [0.5,0.5], ZVD는 [0.25,0.5,0.25] (실물 zvRecalc() 그대로).
    zv = FixedFreqZV()
    zv.configure(2.00, 0.0, 2, 0.01)
    check("ZV 2.00Hz zeta=0 -> 반주기 25샘플", zv.n1 == 25, f"n1={zv.n1}")
    check("ZV(2임펄스) 진폭 [0.5,0.5,0.0]",
          close(zv.amps[0], 0.5, 1e-9) and close(zv.amps[1], 0.5, 1e-9) and zv.amps[2] == 0.0,
          f"amps={zv.amps}")
    zv.configure(2.00, 0.0, 3, 0.01)
    check("ZVD(3임펄스) 진폭 [0.25,0.5,0.25]",
          close(zv.amps[0], 0.25, 1e-9) and close(zv.amps[1], 0.5, 1e-9) and close(zv.amps[2], 0.25, 1e-9),
          f"amps={zv.amps}")

    # (d) 소프트복귀 시퀀스 (실물 SoftReturnAxis 그대로).
    # active=1.5deg, end=0.75deg, dwell=30ms(3틱) hold=100ms(10틱) return=120ms(12틱), dt=10ms.
    sr = SoftReturnAxis()
    active_deg, end_deg, dwell_ms, hold_ms, return_ms, dt = 1.5, 0.75, 30, 100, 120, 0.01
    big = math.radians(5.0)
    small = math.radians(0.5)

    sr.step(big, big, active_deg, end_deg, dwell_ms, hold_ms, return_ms, dt)  # 1: DIRECT, 가속 인정
    check("소프트복귀: 가속 인정 후에도 DIRECT 유지", sr.phase == SoftReturnPhase.DIRECT)
    sr.step(small, small, active_deg, end_deg, dwell_ms, hold_ms, return_ms, dt)  # 2: 종료 감지 -> CONFIRM
    check("소프트복귀: 종료 감지 -> CONFIRM", sr.phase == SoftReturnPhase.CONFIRM)
    for _ in range(2):  # 3, 4: 아직 dwell 미달
        sr.step(small, small, active_deg, end_deg, dwell_ms, hold_ms, return_ms, dt)
    check("소프트복귀: dwell 중 -> 여전히 CONFIRM", sr.phase == SoftReturnPhase.CONFIRM)
    sr.step(small, small, active_deg, end_deg, dwell_ms, hold_ms, return_ms, dt)  # 5: dwell 완료 -> HOLD
    check("소프트복귀: dwell 완료 -> HOLD", sr.phase == SoftReturnPhase.HOLD)
    for _ in range(9):  # 6..14
        sr.step(small, small, active_deg, end_deg, dwell_ms, hold_ms, return_ms, dt)
    check("소프트복귀: hold 중(9/10틱) -> 여전히 HOLD", sr.phase == SoftReturnPhase.HOLD)
    sr.step(small, small, active_deg, end_deg, dwell_ms, hold_ms, return_ms, dt)  # 15: hold 완료 -> RETURN
    check("소프트복귀: hold 완료 -> RETURN", sr.phase == SoftReturnPhase.RETURN)
    for _ in range(11):  # 16..26
        sr.step(small, small, active_deg, end_deg, dwell_ms, hold_ms, return_ms, dt)
    check("소프트복귀: return 중(11/12틱) -> 여전히 RETURN", sr.phase == SoftReturnPhase.RETURN)
    out_final = sr.step(small, small, active_deg, end_deg, dwell_ms, hold_ms, return_ms, dt)  # 27: 복귀 완료 -> LEVEL
    check("소프트복귀: return 완료(27틱) -> LEVEL, 출력 0",
          sr.phase == SoftReturnPhase.LEVEL and close(out_final, 0.0, 1e-9))

    # (e) LIMIT 45도 클램프 & 슬루 120도/s.
    # base는 수평(0,0)으로 두고 ay축에만 강한 가속(38 m/s^2)을 실어 비력
    # 노름을 accel_norm_max(40) 이내로 유지하면서 raw pitch_acc가 75.5deg에
    # 이르게 한다 — theta_base가 여기 직접 초기화되므로 클램프 전 want가
    # 45deg를 크게 넘는다. 60틱(0.6s, 슬루로 최대 72deg까지 이동 가능한
    # 시간)을 돌려 장기적으로도 상한을 벗어나지 않는지 확인한다.
    core = ControlCore(hw_config())
    core.init_control()
    core.request_activate()
    out = run_hw_core(core, (0.0, 0.0), 60, ax_w=0.0, ay_w=38.0)
    limit = core.cfg.cmd_limit_rad
    check("LIMIT 45도 클램프 준수 (0.6s 후에도)",
          abs(out.y_position_rad) <= limit + 1e-6,
          f"pitch={math.degrees(out.y_position_rad):.4f}deg (한계 {math.degrees(limit):.0f})")

    # 슬루만 격리 검증: cmd_lpf_alpha=1.0(무필터)로 두면 LPF 출력이 곧
    # 클램프된 목표와 같아지므로, 첫 틱의 실제 이동량은 순수 슬루 한계
    # (cmd_slew_rads*dt)로만 결정된다.
    core = ControlCore(hw_config(cmd_lpf_alpha=1.0))
    core.init_control()
    core.request_activate()
    out = run_hw_core(core, (0.0, 0.0), 1, ax_w=0.0, ay_w=38.0, dt=0.01)
    max_step = core.cfg.cmd_slew_rads * 0.01
    check("슬루 120도/s -> 첫 틱 이동량이 슬루 한계와 일치",
          close(abs(out.y_position_rad), max_step, 1e-9),
          f"pitch={math.degrees(out.y_position_rad):.4f}deg, 한계={math.degrees(max_step):.4f}deg")

    # slew_rate_limit() 자체도 직접 검증 (실물 slew() 그대로).
    check("slew_rate_limit: 상한 클램프", close(slew_rate_limit(1.0, 0.0, 0.02), 0.02, 1e-12))
    check("slew_rate_limit: 하한 클램프", close(slew_rate_limit(-1.0, 0.0, 0.02), -0.02, 1e-12))
    check("slew_rate_limit: 한계 이내는 그대로", close(slew_rate_limit(0.005, 0.0, 0.02), 0.005, 1e-12))

    # (f) FAULT 조건.
    core = ControlCore(hw_config())
    core.init_control()
    core.request_activate()
    run_hw_core(core, (0.0, 0.0), 1)  # ACTIVE 진입
    check("FAULT 전 ACTIVE 상태 확인", core.state == ControlState.ACTIVE)
    t_us = 100_000
    for i in range(3):  # 가속도 이상(비력범위 밖) 연속 3프레임. 첫 2프레임은
        # 아직 FAULT가 아니어야(디바운스) 진짜 3프레임 조건을 확인한다.
        t_us += 10_000
        # sampled_at_us를 매번 갱신해 IMU 타임아웃이 아니라 가속도 크기
        # 이상만 걸리게 한다(비력 노름 0 << accel_norm_min=2).
        bad = ImuSample(ax_mps2=0.0, ay_mps2=0.0, az_mps2=0.0,
                        sampled_at_us=t_us, valid=True)
        imu = ImuPair(base=bad, tray=bad)
        core.step(imu, make_motor(t_us // 1000), make_motor(t_us // 1000),
                  0.01, t_us, t_us // 1000)
        if i < 2:
            check(f"가속도 이상 {i+1}프레임째는 아직 FAULT 아님",
                  core.state != ControlState.FAULT, f"state={int(core.state)}")
    check("가속도 이상 연속 3프레임 -> FAULT",
          core.state == ControlState.FAULT and core.diag.fault_reason == core.ACCEL_FAULT_MSG,
          core.diag.fault_reason)

    core = ControlCore(hw_config())
    core.init_control()
    core.request_activate()
    run_hw_core(core, (0.0, 0.0), 1)
    over_limit = MotorFeedback(position_rad=math.radians(60.0), valid=True,
                               received_at_ms=1000, error_code=0)
    out = run_hw_core(core, (0.0, 0.0), 1, motor_roll=over_limit, motor_pitch=make_motor(1010))
    check("실측 위치 한계(55도) 초과 -> FAULT",
          core.state == ControlState.FAULT and not out.enable, core.diag.fault_reason)


def main():
    print("=" * 70)
    print("gimbal_control_core 단위 시험")
    print("=" * 70)
    test_rotation_utils()
    test_deadband()
    test_complementary_filter()
    test_resultant_target()
    test_slosh_modes()
    test_zv()
    test_gain_schedule()
    test_state_machine()
    test_closed_loop_geometry()
    test_flags()
    test_integral_reduces_sustained_bias()
    test_deadband_in_loop()
    test_cnn_interface()
    test_hw_style()
    print("\n" + "=" * 70)
    print(f"전체 통과: {_passed}개 검사")
    print("=" * 70)
    return 0


if __name__ == "__main__":
    sys.exit(main())
