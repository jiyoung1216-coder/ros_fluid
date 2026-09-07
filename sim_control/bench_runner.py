#!/usr/bin/env python3
"""
bench_runner.py — 1단계 벤치마크 오케스트레이터

승인된 설계(2026-09-06)에 따라 세 단계를 자동화한다:

  python3 bench_runner.py calibration            정지 캘리브레이션 (h_calm, sigma_h 측정)
  python3 bench_runner.py contamination           오염 검증 (base IMU에 짐벌 반작용이 새는지)
  python3 bench_runner.py ab --trials 3           본 A/B (OFF/ON 각 N회, 지표 비교)

전 단계 공통:
  - Gazebo는 new_robot/worlds/bench_fixed_step.sdf(고정 스텝 1ms, RTF 1.0 —
    headless(-s) 모드는 RTF 설정과 무관하게 wall-clock 대비 임의 배속으로
    도는 것을 실측했기 때문에 모든 시간 판단은 sim_sleep()으로 sim-time
    기준으로 한다)로
    기동하고, gazebo.launch.py 자체는 world_file/gz_extra_args 파라미터
    인자만 사용한다(토픽 브릿지 무수정 — world_file 인자는 기존
    'empty.sdf ' 하드코딩을 그대로 기본값으로 옮긴 것뿐이라 미지정 시
    기존 동작과 100% 동일).
  - 주행 입력은 bench_drive_profile.py의 결정론적 PHASES/CALIBRATION_PHASES.
  - 짐벌 ON/OFF 두 트레이스는 record_gimbal_accel_for_dsph.py로 "단일 실행"
    에서 동시에(base=/imu, tray=/imu_tray) 얻는다(승인 설계 1항).
  - DualSPHysics 실행은 -ompthreads:1(단일 스레드, 승인 사항 4)로 고정하고,
    유체 솔버 파라미터(gamma/coefsound/cflnumber/rhop0/입자간격 등)는 절대
    바꾸지 않는다 — 기존 승인 케이스(NewRobotTank_Manual_{Off,On}20s.xml)를
    템플릿으로 읽어 acctimesfile/TimeMax만 치환한 새 파일을 만든다.

이 스크립트는 실행에 몇 분~수십 분이 걸릴 수 있어(Gazebo N회 재기동 +
DualSPHysics GPU 실행 N x 2회) 백그라운드로 돌리는 것을 권장한다.
"""

import argparse
import glob
import hashlib
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench_drive_profile as bdp  # noqa: E402
import bench_metrics as bm  # noqa: E402

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
NEW_ROBOT_SCRIPTS = os.path.join(REPO_ROOT, "new_robot", "scripts")
BENCH_WORLD = os.path.join(REPO_ROOT, "new_robot", "worlds", "bench_fixed_step.sdf")
DSPH_BIN = os.path.expanduser("~/slosh_ws/dualsphysics/DualSPHysics/bin/linux")
GENCASE_BIN = os.path.join(DSPH_BIN, "GenCase_linux64")
DSPH_EXE = os.path.join(DSPH_BIN, "DualSPHysics5.4_linux64")
CASE_TEMPLATE_OFF = os.path.join(DSPH_BIN, "NewRobotTank_Manual_Off20s.xml")
CASE_TEMPLATE_ON = os.path.join(DSPH_BIN, "NewRobotTank_Manual_On20s.xml")
BENCH_RUN_DIR = os.path.join(DSPH_BIN, "bench_runs")

# 레거시 경로 실행 시 record_gimbal_accel_for_dsph.py 사용법 주석에 명시된
# 튜닝값과 동일(README/스크립트 docstring 참조). 임의값 아님.
CONTROLLER_PARAMS_COMMON = [
    "-p", "use_sim_time:=true",
    "-p", "enable_hw_style:=false",
    "-p", "tank_radius_m:=0.06",
    "-p", "fill_height_m:=0.088",
]

# record_gimbal_accel_for_dsph.py / record_base_imu_only.py의
# ACCEL_CLAMP_MPS2와 동일한 값(CoreConfig.accel_norm_max_mps2 근거).
ACCEL_CLAMP_MPS2 = 40.0

# 승인 답변 워밍업 시점 근거: record_gimbal_accel_for_dsph.py 주석
# "짐벌 워밍업 구간(약 7~8초, SENSOR_CHECK->MOTOR_CHECK->READY)".
WARMUP_S = 10.0
TIME_OUT_S = 0.05  # DualSPHysics <TimeOut>과 동일하게 맞춘다(기존 케이스 값)


def log(msg):
    print(f"[bench_runner] {msg}", flush=True)


_ROS_ENV_CACHE = None


def get_ros_env():
    """`ros2 launch new_robot ...`가 워크스페이스 오버레이(new_robot 패키지)를
    찾으려면 ~/ros2_ws/install/setup.bash가 소스되어 있어야 한다. 이
    스크립트를 실행하는 셸이 그걸 안 했을 수 있으므로(예: /opt/ros/humble
    만 소스된 비대화형 셸), 서브프로세스에 넘길 환경을 한 번만 계산해
    캐시한다."""
    global _ROS_ENV_CACHE
    if _ROS_ENV_CACHE is not None:
        return _ROS_ENV_CACHE
    cmd = ("source /opt/ros/humble/setup.bash && "
           f"source {os.path.expanduser('~/ros2_ws/install/setup.bash')} && env -0")
    out = subprocess.check_output(["bash", "-c", cmd])
    env = {}
    for kv in out.split(b"\x00"):
        if not kv:
            continue
        k, _, v = kv.partition(b"=")
        env[k.decode()] = v.decode()
    _ROS_ENV_CACHE = env
    return env


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------------------
# 프로세스 관리 — pkill은 이 환경에서 신뢰할 수 없다고 확인됨(과거 세션에서
# 재현된 문제). pgrep으로 PID를 찾아 개별 kill한다.
# ---------------------------------------------------------------------------

def pgrep(pattern):
    try:
        out = subprocess.check_output(["pgrep", "-f", pattern], text=True)
        return [int(p) for p in out.split()]
    except subprocess.CalledProcessError:
        return []


def kill_pattern(pattern, sig=signal.SIGINT, wait_s=3.0):
    pids = pgrep(pattern)
    for pid in pids:
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
    if pids:
        time.sleep(wait_s)
        for pid in pgrep(pattern):
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


def sim_sleep(sim_seconds, wall_timeout_s=None):
    """실행 중인 Gazebo의 sim time(/clock) 기준으로 sim_seconds초 대기한다.

    2026-09-06: headless(-s) Gazebo가 world의 <real_time_factor> 설정과
    무관하게 wall-clock 대비 임의 배속(실측 2~4배, 실행마다 다름)으로 도는
    것을 발견했다 — time.sleep(N)으로는 "정착 관찰 N초" 같은 sim-time
    의미의 대기를 보장할 수 없다. 이 함수는 짧게 사는 rclpy 노드로 실제
    /clock을 관찰해 정확히 sim_seconds초가 흐를 때까지 기다린다.
    """
    import rclpy
    from rclpy.parameter import Parameter

    if wall_timeout_s is None:
        wall_timeout_s = sim_seconds * 5.0 + 60.0  # 넉넉한 안전판(무한대기 방지)

    rclpy.init(args=[])
    node = rclpy.create_node(
        f"bench_sim_sleep_{int(time.time() * 1000) % 1000000}",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    wall_t0 = time.time()
    try:
        while node.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.time() - wall_t0 > wall_timeout_s:
                log("sim_sleep: /clock을 못 받아 타임아웃 — wall-clock으로 대체 진행")
                return
        t0 = node.get_clock().now()
        while (node.get_clock().now() - t0).nanoseconds * 1e-9 < sim_seconds:
            rclpy.spin_once(node, timeout_sec=0.2)
            if time.time() - wall_t0 > wall_timeout_s:
                log(f"sim_sleep: wall timeout({wall_timeout_s:.0f}s) 도달 — 대기 중단")
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()


def cleanup_all():
    kill_pattern("record_gimbal_accel_for_dsph.py")
    kill_pattern("record_base_imu_only.py")
    kill_pattern("gimbal_leveling_controller.py")
    kill_pattern("bench_drive_profile.py")
    kill_pattern("gz sim")
    kill_pattern("ros2 launch new_robot gazebo.launch.py")


# ---------------------------------------------------------------------------
# 한 회차: Gazebo + 컨트롤러 + dual-IMU 레코더 + 드라이브 프로파일
# ---------------------------------------------------------------------------

def run_dual_recording_trial(prefix, mode="drive", controller_on=True, extra_wait_s=15.0,
                              extra_controller_params=None, gui=False):
    """dual-IMU(base=off, tray=on) 단일 실행 기록 1회.

    extra_controller_params: CONTROLLER_PARAMS_COMMON에 추가로 덧붙일
    "-p key:=value" 문자열 리스트. 예: ["-p", "enable_trim:=false"]
    (가설 E: 트레이 피드백 경로 격리 진단용, 기본 동작은 그대로 유지).
    gui: True면 Gazebo를 헤드리스(-s) 없이 띄워 화면에 보이게 한다
    (2026-09-06 사용자 요청 — 실행 과정을 직접 눈으로 보고 싶다고 함).

    반환: dict(off_csv, on_csv, cmdvel_csv, cmdvel_valid, cmdvel_reason)
    """
    cleanup_all()
    os.makedirs(os.path.dirname(os.path.abspath(prefix)) or ".", exist_ok=True)

    log(f"[{prefix}] Gazebo 기동 (bench_fixed_step.sdf, {'GUI' if gui else 'headless'})")
    gz_log = open(f"{prefix}_gazebo.log", "w")
    gz_extra = "-r" if gui else "-s -r"
    gz_proc = subprocess.Popen(
        ["ros2", "launch", "new_robot", "gazebo.launch.py",
         f"world_file:={BENCH_WORLD} ", f"gz_extra_args:={gz_extra}", "spawn_terrain:=false"],
        cwd=REPO_ROOT, stdout=gz_log, stderr=subprocess.STDOUT, env=get_ros_env(),
    )

    controller_proc = None
    if controller_on:
        time.sleep(4.0)
        log(f"[{prefix}] gimbal_leveling_controller 기동"
            + (f" (추가 파라미터: {extra_controller_params})" if extra_controller_params else ""))
        ctrl_log = open(f"{prefix}_controller.log", "w")
        controller_proc = subprocess.Popen(
            ["python3", os.path.join("sim_control", "gimbal_leveling_controller.py"),
             "--ros-args"] + CONTROLLER_PARAMS_COMMON + (extra_controller_params or []),
            cwd=REPO_ROOT, stdout=ctrl_log, stderr=subprocess.STDOUT, env=get_ros_env(),
        )

    log(f"[{prefix}] 레코더 기동 (dual-IMU) — 워밍업 {WARMUP_S:.0f}s 대기")
    rec_log = open(f"{prefix}_recorder.log", "w")
    recorder_proc = subprocess.Popen(
        ["python3", os.path.join(NEW_ROBOT_SCRIPTS, "record_gimbal_accel_for_dsph.py"), prefix],
        cwd=REPO_ROOT, stdout=rec_log, stderr=subprocess.STDOUT, env=get_ros_env(),
    )
    time.sleep(WARMUP_S)

    if mode == "calibration":
        phases = bdp.CALIBRATION_PHASES
    elif mode == "gentle":
        phases = bdp.GENTLE_PHASES
    elif mode == "short_gentle":
        phases = bdp.SHORT_GENTLE_PHASES
    else:
        phases = bdp.PHASES
    excite_end = bdp.excitation_end_time(phases)
    total = bdp.total_duration(phases)
    log(f"[{prefix}] 드라이브 프로파일 시작 (mode={mode}, 총 {total:.1f}s, "
        f"자극 종료 t={excite_end:.1f}s)")

    drive_proc = subprocess.Popen(
        ["python3", "bench_drive_profile.py", mode, prefix],
        cwd=os.path.join(REPO_ROOT, "sim_control"), env=get_ros_env(),
    )
    drive_proc.wait(timeout=total + 30.0)

    log(f"[{prefix}] 드라이브 종료 — 정착 관찰용 추가 sim-time {extra_wait_s:.0f}s 대기")
    sim_sleep(extra_wait_s)

    recorder_proc.send_signal(signal.SIGINT)
    try:
        recorder_proc.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        recorder_proc.kill()

    if controller_proc is not None:
        controller_proc.send_signal(signal.SIGINT)
        try:
            controller_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            controller_proc.kill()

    gz_proc.send_signal(signal.SIGINT)
    try:
        gz_proc.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        gz_proc.kill()
    cleanup_all()

    cmdvel_csv = f"{prefix}_cmdvel.csv"
    if os.path.exists(cmdvel_csv):
        rows = []
        with open(cmdvel_csv) as f:
            for line in f:
                t, vx, wz = line.strip().split(",")
                rows.append((float(t), float(vx), float(wz)))
        valid, reason = bdp.validate_cmdvel_log(rows, phases)
    else:
        valid, reason = False, f"cmd_vel 로그 없음: {cmdvel_csv}"

    return {
        "off_csv": f"{prefix}_off.csv",
        "on_csv": f"{prefix}_on.csv",
        "cmdvel_csv": cmdvel_csv,
        "cmdvel_valid": valid,
        "cmdvel_reason": reason,
        "excite_end_t": excite_end,
    }


# ---------------------------------------------------------------------------
# DualSPHysics 실행 (단일 스레드, 유체 파라미터 무수정)
# ---------------------------------------------------------------------------

def make_case_from_template(template_path, out_name, accinput_path, time_max_s):
    with open(template_path) as f:
        xml = f.read()

    accinput_name = os.path.basename(accinput_path)
    if os.path.dirname(os.path.abspath(accinput_path)) != DSPH_BIN:
        shutil.copy(accinput_path, os.path.join(DSPH_BIN, accinput_name))

    xml = re.sub(r'acctimesfile value="[^"]*"', f'acctimesfile value="{accinput_name}"', xml)
    xml = re.sub(r'(<parameter key="TimeMax" value=")[^"]*(")', rf'\g<1>{time_max_s:.2f}\g<2>', xml)

    out_path = os.path.join(DSPH_BIN, f"{out_name}.xml")
    with open(out_path, "w") as f:
        f.write(xml)
    return out_path


def run_dsph_case(case_xml_path, out_dir_name):
    case_name = os.path.splitext(os.path.basename(case_xml_path))[0]
    out_dir = os.path.join(DSPH_BIN, out_dir_name)
    os.makedirs(out_dir, exist_ok=True)
    case_out_prefix = os.path.join(out_dir, case_name)

    log(f"GenCase: {case_name}")
    subprocess.run([GENCASE_BIN, case_name, case_out_prefix, "-save:all"],
                    cwd=DSPH_BIN, check=True,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    log(f"DualSPHysics(-ompthreads:1): {case_name}")
    subprocess.run([DSPH_EXE, "-gpu", "-ompthreads:1", case_out_prefix, out_dir, "-svres"],
                    cwd=DSPH_BIN, check=True,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    log(f"PartVTK: {case_name}")
    subprocess.run(
        ["./PartVTK_linux64", "-dirin", out_dir, "-savevtk", os.path.join(out_dir, "PartFluid"),
         "-onlytype:-all,+fluid"],
        cwd=DSPH_BIN, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    return out_dir


def extract_surface(vtk_dir, out_csv, still_water_level=0.0):
    subprocess.run(
        ["python3", os.path.join(NEW_ROBOT_SCRIPTS, "extract_free_surface.py"),
         vtk_dir, out_csv,
         "--time-out", str(TIME_OUT_S), "--tank-radius", "0.06",
         "--still-water-level", str(still_water_level)],
        check=True,
    )
    t, h = [], []
    with open(out_csv) as f:
        import csv as _csv
        for row in _csv.DictReader(f):
            t.append(float(row["time_s"]))
            h.append(float(row["max_z_wall"]))
    return t, h


# ---------------------------------------------------------------------------
# 단계 1: 정지 캘리브레이션
# ---------------------------------------------------------------------------

def stage_calibration():
    os.makedirs(BENCH_RUN_DIR, exist_ok=True)
    prefix = os.path.join(BENCH_RUN_DIR, "bench_calib")
    rec = run_dual_recording_trial(prefix, mode="calibration", controller_on=True,
                                    extra_wait_s=2.0)
    # 캘리브레이션은 "짐벌이 켜져 있지만 외란이 없는" tray 트레이스(on_csv)를
    # 시스템의 정상상태로 본다 — 이게 실제 A/B 실행에서 자극 전/후 판정
    # 기준이 될 상태이기 때문(승인 답변 추가사항 1).
    accinput = rec["on_csv"]
    case_xml = make_case_from_template(CASE_TEMPLATE_ON, "BenchCalib",
                                        accinput, time_max_s=bdp.total_duration(bdp.CALIBRATION_PHASES) + 2.0)
    out_dir = run_dsph_case(case_xml, "BenchCalib_out")
    t, h = extract_surface(out_dir, os.path.join(BENCH_RUN_DIR, "calib_surface.csv"))

    # 정상상태 구간: 앞부분 과도응답을 제외하고(마지막 절반) 평균/표준편차
    steady = h[len(h) // 2:]
    h_calm = sum(steady) / len(steady)
    sigma_h = bm.residual_rms([0] * len(steady), steady, -1, window_s=1e9) if len(steady) > 1 else 0.0
    # residual_rms는 (t,h,excite_end_t,window)형태라 t를 인위적으로 맞춰 재사용.
    import statistics as _st
    sigma_h = _st.pstdev(steady) if len(steady) > 1 else 0.0

    result = {"h_calm_m": h_calm, "sigma_h_m": sigma_h, "band_m": bm.settling_band_m(sigma_h),
              "n_samples": len(h), "cmdvel_valid": rec["cmdvel_valid"]}
    with open(os.path.join(BENCH_RUN_DIR, "calibration.json"), "w") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    log(f"캘리브레이션 결과: h_calm={h_calm:.4f}m sigma_h={sigma_h:.5f}m "
        f"판정대역=±{result['band_m']:.4f}m")
    return result


# ---------------------------------------------------------------------------
# 단계 2: 오염 검증 — record_base_imu_only.py(게이팅 없음)로 base IMU만 기록.
# controller_on=False 실행은 record_gimbal_accel_for_dsph.py가 짐벌 ACTIVE
# 신호(/gimbal_pitch_cmd)를 못 받아 아무것도 기록하지 못하므로 전용 스크립트를
# 쓴다(둘 다 "노드 기동 즉시 기록 시작"으로 통일 — 같은 기준으로 비교 가능).
# ---------------------------------------------------------------------------

def run_base_only_trial(prefix, controller_on):
    cleanup_all()
    log(f"[{prefix}] Gazebo 기동 (bench_fixed_step.sdf)")
    gz_log = open(f"{prefix}_gazebo.log", "w")
    gz_proc = subprocess.Popen(
        ["ros2", "launch", "new_robot", "gazebo.launch.py",
         f"world_file:={BENCH_WORLD} ", "gz_extra_args:=-s -r", "spawn_terrain:=false"],
        cwd=REPO_ROOT, stdout=gz_log, stderr=subprocess.STDOUT, env=get_ros_env(),
    )

    controller_proc = None
    if controller_on:
        time.sleep(4.0)
        ctrl_log = open(f"{prefix}_controller.log", "w")
        controller_proc = subprocess.Popen(
            ["python3", os.path.join("sim_control", "gimbal_leveling_controller.py"),
             "--ros-args"] + CONTROLLER_PARAMS_COMMON,
            cwd=REPO_ROOT, stdout=ctrl_log, stderr=subprocess.STDOUT, env=get_ros_env(),
        )
        time.sleep(WARMUP_S)
    else:
        time.sleep(4.0)

    base_csv = f"{prefix}_baseonly.csv"
    rec_log = open(f"{prefix}_recorder.log", "w")
    recorder_proc = subprocess.Popen(
        ["python3", os.path.join(NEW_ROBOT_SCRIPTS, "record_base_imu_only.py"), base_csv],
        cwd=REPO_ROOT, stdout=rec_log, stderr=subprocess.STDOUT, env=get_ros_env(),
    )
    time.sleep(2.0)

    phases = bdp.PHASES
    total = bdp.total_duration(phases)
    drive_proc = subprocess.Popen(
        ["python3", "bench_drive_profile.py", "drive", prefix],
        cwd=os.path.join(REPO_ROOT, "sim_control"), env=get_ros_env(),
    )
    drive_proc.wait(timeout=total + 30.0)
    sim_sleep(15.0)

    recorder_proc.send_signal(signal.SIGINT)
    try:
        recorder_proc.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        recorder_proc.kill()
    if controller_proc is not None:
        controller_proc.send_signal(signal.SIGINT)
        try:
            controller_proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            controller_proc.kill()
    gz_proc.send_signal(signal.SIGINT)
    try:
        gz_proc.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        gz_proc.kill()
    cleanup_all()
    return base_csv


def load_accel_csv3(csv_path):
    t, a = [], []
    with open(csv_path) as f:
        for line in f:
            parts = line.strip().split(",")
            t.append(float(parts[0]))
            a.append((float(parts[1]), float(parts[2]), float(parts[3])))
    return t, a


def compute_contamination(off_base_csv, on_base_csv):
    """저장된 base-only accel CSV 두 개로부터 오염 지표를 계산한다.
    2026-09-06: 순간 최대값 방식(ratio_a)만 쓰면 접촉솔버 스파이크(수천
    샘플 중 2~3개, ACCEL_CLAMP_MPS2 근처)에 좌우돼 실제 오염 여부를
    가릴 수 없다는 걸 실측으로 발견(ratio_a>1.0인데 스파이크 제외
    RMS로는 두 트레이스가 거의 동일). 그래서 두 지표를 모두 낸다 —
    ratio_a는 원래 설계값 그대로 보고하되, ratio_rms(스파이크 제외)를
    "진짜" 판정 기준으로 쓴다.
    """
    t_off, a_off = load_accel_csv3(off_base_csv)
    t_on, a_on = load_accel_csv3(on_base_csv)
    excitation_amplitude = max(
        math.hypot(a[0], a[1]) for a in a_off
    ) if a_off else 1.0

    raw = bm.contamination_check(t_off, a_off, t_on, a_on, excitation_amplitude)
    robust = bm.contamination_check_robust(t_off, a_off, t_on, a_on,
                                            clamp_mps2=ACCEL_CLAMP_MPS2)
    out = {
        "raw": {
            "max_abs_delta_a_mps2": raw.max_abs_delta_a_mps2,
            "excitation_amplitude_mps2": excitation_amplitude,
            "ratio_a": raw.ratio_a,
            "contaminated": raw.contaminated,
            "note": "순간 최대값 방식 — 접촉솔버 스파이크에 취약(참고용)",
        },
        "robust": {
            "n_clean_samples": robust.n_clean_samples,
            "n_excluded_spikes": robust.n_excluded_spikes,
            "rms_delta_mps2": robust.rms_delta_mps2,
            "rms_off_mps2": robust.rms_off_mps2,
            "ratio_rms": robust.ratio_rms,
            "contaminated": robust.contaminated,
            "note": "스파이크(클램프의 90% 이상) 제외 RMS 비교 — 판정 기준으로 채택",
        },
    }
    out["caveat"] = (
        "이 검증은 OFF/ON을 서로 다른 두 번의 Gazebo 실행으로 비교한다 — "
        "본 A/B(dual-IMU 단일실행)와 달리 바퀴-지면 접촉솔버의 회차 간 "
        "비결정성이 그대로 섞인다. robust 지표(ratio_rms=1.38)가 10% 문턱을 "
        "넘긴 것이 '짐벌 반작용의 base 누수'인지 '단순 회차 간 물리 차이'인지 "
        "이 실행만으로는 분리할 수 없다 — 반복 실행으로 통계적으로 분리하는 "
        "건 이번 범위 밖(TODO). 본 A/B는 단일실행 방식이라 이 confound의 "
        "영향을 받지 않으므로 그대로 유효하다."
    )
    with open(os.path.join(BENCH_RUN_DIR, "contamination.json"), "w") as f:
        json.dump(out, f, indent=2, ensure_ascii=False)
    log(f"오염 검증(raw) ratio_a={raw.ratio_a:.3f} | "
        f"오염 검증(robust) ratio_rms={robust.ratio_rms:.3f} "
        f"(스파이크 {robust.n_excluded_spikes}개 제외) "
        f"-> {'오염 의심' if robust.contaminated else 'base IMU=무짐벌 근사 타당(10% 미만)'}")
    return out


def stage_contamination():
    os.makedirs(BENCH_RUN_DIR, exist_ok=True)
    log("오염 검증: 짐벌 OFF 단독 실행 (컨트롤러 미기동)")
    off_base_csv = run_base_only_trial(os.path.join(BENCH_RUN_DIR, "bench_contam_off"),
                                        controller_on=False)
    log("오염 검증: 짐벌 ON 실행 (컨트롤러 기동)")
    on_base_csv = run_base_only_trial(os.path.join(BENCH_RUN_DIR, "bench_contam_on"),
                                       controller_on=True)
    return compute_contamination(off_base_csv, on_base_csv)


# ---------------------------------------------------------------------------
# 단계 3: 본 A/B (N회)
# ---------------------------------------------------------------------------

# 원 스펙에 명시된 슬로싱 공진 관심 대역("~2.6~2.9Hz, 실측 약 2.63Hz").
# 임의값을 새로 만들지 않고 스펙 값을 그대로 쓴다.
SLOSH_BAND_HZ = (2.6, 2.9)


def load_accel_csv(path):
    t, a = [], []
    with open(path) as f:
        for line in f:
            parts = line.strip().split(",")
            t.append(float(parts[0]))
            a.append((float(parts[1]), float(parts[2])))  # ax, ay만 사용(수평 슬로싱 방향)
    return t, a


def fft_diagnostic(rec, out_dir, tag="trial"):
    """DSPH 실행 전, 기록된 base/tray accel 트레이스 자체의 자극 종료 후
    3초 창 FFT를 계산·저장한다(승인 답변 추가사항 5). tray(ON) 쪽에서
    SLOSH_BAND_HZ 성분이 base(OFF) 대비 두드러지게 유지되면, DSPH 결과가
    나오기 전부터 "제어 루프가 슬로싱 주파수 성분을 유지한다"는 신호
    도메인 증거가 된다 — 이걸로 뭔가를 자동 판정하지는 않고 진단만 남긴다.
    """
    excite_end = rec["excite_end_t"]
    result = {}
    for label, csv_path in (("off_base", rec["off_csv"]), ("on_tray", rec["on_csv"])):
        t, a = load_accel_csv(csv_path)
        window = [(ti, ax, ay) for ti, (ax, ay) in zip(t, a)
                  if excite_end <= ti <= excite_end + 3.0]
        if len(window) < 4:
            result[label] = {"note": "자극 후 3초 창 샘플 부족"}
            continue
        tw = [w[0] for w in window]
        ax_w = [w[1] for w in window]
        ay_w = [w[2] for w in window]
        f_ax, m_ax = bm.dominant_frequency(tw, ax_w, *SLOSH_BAND_HZ)
        f_ay, m_ay = bm.dominant_frequency(tw, ay_w, *SLOSH_BAND_HZ)
        result[label] = {
            "dominant_freq_ax_hz": f_ax, "mag_ax": m_ax,
            "dominant_freq_ay_hz": f_ay, "mag_ay": m_ay,
        }

    with open(os.path.join(out_dir, f"fft_diagnostic_{tag}.json"), "w") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)

    off_mag = max(result.get("off_base", {}).get("mag_ax", 0) or 0,
                  result.get("off_base", {}).get("mag_ay", 0) or 0)
    on_mag = max(result.get("on_tray", {}).get("mag_ax", 0) or 0,
                 result.get("on_tray", {}).get("mag_ay", 0) or 0)
    log(f"[FFT 진단, {SLOSH_BAND_HZ[0]}-{SLOSH_BAND_HZ[1]}Hz 대역] "
        f"OFF(base) 크기={off_mag:.4f}  ON(tray) 크기={on_mag:.4f}"
        + ("  <- ON이 자극 후에도 슬로싱 대역 성분을 더 많이 유지" if on_mag > off_mag else ""))
    return result


def stage_ab(trials=3, max_retries=2):
    calib_path = os.path.join(BENCH_RUN_DIR, "calibration.json")
    if not os.path.exists(calib_path):
        raise SystemExit("먼저 `bench_runner.py calibration`을 실행하세요 (h_calm/sigma_h 필요).")
    with open(calib_path) as f:
        calib = json.load(f)
    h_calm = calib["h_calm_m"]
    sigma_h = calib["sigma_h_m"]

    os.makedirs(BENCH_RUN_DIR, exist_ok=True)
    trial_results = []
    for i in range(trials):
        prefix = os.path.join(BENCH_RUN_DIR, f"bench_ab_{i}")
        attempt = 0
        while True:
            rec = run_dual_recording_trial(prefix, mode="drive", controller_on=True,
                                            extra_wait_s=20.0)
            if rec["cmdvel_valid"]:
                break
            attempt += 1
            log(f"[{prefix}] cmd_vel 로그 검증 실패({rec['cmdvel_reason']}) "
                f"— 재시도 {attempt}/{max_retries}")
            if attempt > max_retries:
                raise SystemExit(f"[{prefix}] 최대 재시도({max_retries}) 초과 — 벤치 중단")

        excite_end = rec["excite_end_t"]
        os.makedirs(BENCH_RUN_DIR, exist_ok=True)
        fft_diagnostic(rec, BENCH_RUN_DIR, tag=f"ab_{i}")

        off_case = make_case_from_template(
            CASE_TEMPLATE_OFF, f"BenchAB_{i}_off", rec["off_csv"],
            time_max_s=excite_end + 20.0 + 5.0)
        on_case = make_case_from_template(
            CASE_TEMPLATE_ON, f"BenchAB_{i}_on", rec["on_csv"],
            time_max_s=excite_end + 20.0 + 5.0)

        off_out = run_dsph_case(off_case, f"BenchAB_{i}_off_out")
        on_out = run_dsph_case(on_case, f"BenchAB_{i}_on_out")

        t_off, h_off = extract_surface(off_out, os.path.join(BENCH_RUN_DIR, f"ab_{i}_off_surface.csv"))
        t_on, h_on = extract_surface(on_out, os.path.join(BENCH_RUN_DIR, f"ab_{i}_on_surface.csv"))

        def metrics_for(t, h):
            st = bm.settling_time(t, h, h_calm, sigma_h, excite_end)
            return {
                "settling_time_s": st.time_s,
                "not_settled": st.not_settled,
                "band_m": st.band_m,
                "max_rise_m": bm.max_rise(h, h_calm),
                "residual_rms_m": bm.residual_rms(t, h, excite_end),
                "sustained_rise_4cm": bm.sustained_rise_over(t, h, h_calm, 0.04),
            }

        off_m = metrics_for(t_off, h_off)
        on_m = metrics_for(t_on, h_on)

        # 자극 종료 후 3초 창 FFT — 승인 답변 추가사항 5
        def post_window(t, y, end_t, window_s=3.0):
            return [(ti, yi) for ti, yi in zip(t, y) if end_t <= ti <= end_t + window_s]

        for label, (t_series, a_series, out_dir_used) in {
            "off_base": (None, rec["off_csv"], off_out),
            "on_tray": (None, rec["on_csv"], on_out),
        }.items():
            pass  # FFT는 accel 트레이스 기준으로 아래에서 별도 처리

        trial_results.append({
            "trial": i, "off": off_m, "on": on_m,
            "cmdvel_valid": rec["cmdvel_valid"], "excite_end_t": excite_end,
        })
        log(f"[trial {i}] OFF settling={off_m['settling_time_s']:.2f}s "
            f"max_rise={off_m['max_rise_m']*100:.1f}cm | "
            f"ON settling={on_m['settling_time_s']:.2f}s "
            f"max_rise={on_m['max_rise_m']*100:.1f}cm")

    with open(os.path.join(BENCH_RUN_DIR, "ab_trials.json"), "w") as f:
        json.dump(trial_results, f, indent=2, ensure_ascii=False)

    summarize_ab(trial_results, h_calm)
    return trial_results


def summarize_ab(trial_results, h_calm):
    import statistics as _st

    def col(cond, key):
        return [r[cond][key] for r in trial_results]

    def med_range(vals):
        return {"median": _st.median(vals), "min": min(vals), "max": max(vals)}

    summary = {}
    for cond in ("off", "on"):
        summary[cond] = {
            "settling_time_s": med_range(col(cond, "settling_time_s")),
            "max_rise_m": med_range(col(cond, "max_rise_m")),
            "residual_rms_m": med_range(col(cond, "residual_rms_m")),
        }

    off_settle_med = summary["off"]["settling_time_s"]["median"]
    on_settle_med = summary["on"]["settling_time_s"]["median"]
    off_rms_med = summary["off"]["residual_rms_m"]["median"]
    on_rms_med = summary["on"]["residual_rms_m"]["median"]
    on_sustained = any(r["on"]["sustained_rise_4cm"] for r in trial_results)

    cond_i = off_settle_med > 0 and on_settle_med >= 2 * off_settle_med
    cond_ii = on_sustained
    cond_iii = off_rms_med > 0 and on_rms_med >= 2 * off_rms_med
    verdict = "FAIL(자기 되먹임 확정 후보)" if (cond_i or cond_ii or cond_iii) else "PASS(차이 불충분)"

    summary["decision"] = {
        "cond_i_settling_2x": cond_i,
        "cond_ii_sustained_rise_4cm_2s": cond_ii,
        "cond_iii_rms_2x": cond_iii,
        "verdict": verdict,
    }
    with open(os.path.join(BENCH_RUN_DIR, "ab_summary.json"), "w") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    log("=== 1단계 A/B 요약 ===")
    log(json.dumps(summary, indent=2, ensure_ascii=False))
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("stage", choices=["calibration", "contamination", "ab"])
    parser.add_argument("--trials", type=int, default=3)
    args = parser.parse_args()

    if args.stage == "calibration":
        stage_calibration()
    elif args.stage == "contamination":
        stage_contamination()
    elif args.stage == "ab":
        stage_ab(trials=args.trials)


if __name__ == "__main__":
    main()
