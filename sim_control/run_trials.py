#!/usr/bin/env python3
"""run_trials.py — HW 스타일 ON/OFF를 N회 반복 실행해 지표를 평균±표준편차로 집계.

analyze_log.py의 analyze()를 그대로 재사용한다(지표 계산 로직 중복 없음).
매 트라이얼마다 Gazebo를 새로 띄우고, drive_test.py로 같은 주행 자극을 준 뒤
gimbal_leveling_controller.py 로그를 분석한다. 중간에 FAULT로 일찍 끝난
트라이얼(ACTIVE 구간이 짧음)은 자동으로 버리고 재시도한다 — 물리엔진/시스템
자원 문제로 생기는 것이지 HW 스타일 로직과 무관한 잡음이기 때문이다.

실행
    python3 run_trials.py -n 5
    python3 run_trials.py -n 3 --out-dir /tmp/trials --min-active-s 15

전제
    - ROS2/Gazebo가 설치돼 있고 new_robot 패키지가 빌드·소싱돼 있을 것
    - new_robot/scripts/drive_test.py가 같은 저장소에 있을 것
    - 무겁다: 트라이얼당 OFF+ON 합쳐 약 1분. 5회면 5분 내외(재시도 포함하면 더 걸림)
"""

import argparse
import csv
import math
import os
import signal
import statistics
import subprocess
import sys
import time

REPO = os.path.expanduser("~/ros2_ws/src/ros_fluid")
SIM_CONTROL = os.path.join(REPO, "sim_control")
DRIVE_SCRIPT = os.path.join(REPO, "new_robot/scripts/drive_test.py")
ROS_SETUP = "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"

sys.path.insert(0, SIM_CONTROL)
import analyze_log as al  # noqa: E402

GZ_STARTUP_S = 10.0
PRE_DRIVE_S = 2.0
POST_DRIVE_S = 3.0

METRICS = [
    ("tray_roll_rms", "트레이 roll RMS[deg]"),
    ("tray_pitch_rms", "트레이 pitch RMS[deg]"),
    ("reject_roll", "roll 저감비(<1 좋음)"),
    ("reject_pitch", "pitch 저감비(<1 좋음)"),
    ("slosh_x_rms", "슬로싱 x RMS[deg]"),
    ("slosh_y_rms", "슬로싱 y RMS[deg]"),
    ("slosh_x_peak", "슬로싱 x 피크[deg]"),
    ("slosh_y_peak", "슬로싱 y 피크[deg]"),
]


def sh_bg(cmd, log_path):
    """새 프로세스 그룹으로 백그라운드 실행 (그룹째로 죽이기 위해)."""
    log_f = open(log_path, "w")
    return subprocess.Popen(["bash", "-c", cmd], stdout=log_f, stderr=subprocess.STDOUT,
                            preexec_fn=os.setsid), log_f


def kill_group(proc, timeout=6):
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def run_condition(enable_hw_style, out_csv, log_dir, tag):
    """가제보 + 컨트롤러 + 주행 자극 한 번. out_csv에 로그가 남는다."""
    gz_proc, gz_log = sh_bg(
        f"{ROS_SETUP} && cd ~/ros2_ws && exec ros2 launch new_robot gazebo.launch.py "
        f"gz_extra_args:='-s -r'",
        os.path.join(log_dir, f"{tag}_gz.log"))
    try:
        time.sleep(GZ_STARTUP_S)
        flag = "true" if enable_hw_style else "false"
        ctrl_proc, ctrl_log = sh_bg(
            f"{ROS_SETUP} && cd {SIM_CONTROL} && exec python3 gimbal_leveling_controller.py "
            f"--ros-args -p use_sim_time:=true -p enable_hw_style:={flag} "
            f"-p log_csv_path:={out_csv}",
            os.path.join(log_dir, f"{tag}_ctrl.log"))
        try:
            time.sleep(PRE_DRIVE_S)
            subprocess.run(["bash", "-c", f"{ROS_SETUP} && exec python3 {DRIVE_SCRIPT}"],
                          timeout=30)
            time.sleep(POST_DRIVE_S)
        finally:
            kill_group(ctrl_proc)
            ctrl_log.close()
    finally:
        kill_group(gz_proc, timeout=10)
        gz_log.close()
        time.sleep(1.5)  # 다음 트라이얼 전 자원 회수 여유


def try_one(enable_hw_style, out_csv, log_dir, tag, min_active_s, max_attempts):
    """유효한(끝까지 FAULT 없이 ACTIVE인) 로그가 나올 때까지 최대 max_attempts번 시도."""
    for attempt in range(1, max_attempts + 1):
        print(f"    [{tag}] 시도 {attempt}/{max_attempts} ...", flush=True)
        run_condition(enable_hw_style, out_csv, log_dir, f"{tag}_a{attempt}")
        if not os.path.isfile(out_csv):
            print(f"    [{tag}] 로그 파일이 안 생김 — 재시도")
            continue
        res = al.analyze(out_csv)
        if res is None:
            print(f"    [{tag}] ACTIVE 구간 없음 — 재시도")
            continue
        if res["duration_s"] < min_active_s:
            print(f"    [{tag}] ACTIVE {res['duration_s']:.1f}s < {min_active_s}s "
                  f"(중간에 FAULT 의심) — 재시도")
            continue
        print(f"    [{tag}] 유효 (ACTIVE {res['duration_s']:.1f}s)")
        return res
    print(f"    [{tag}] {max_attempts}번 모두 실패 — 이 트라이얼은 건너뜀")
    return None


def mean_std(vals):
    vals = [v for v in vals if v == v]  # NaN 제거
    if not vals:
        return float("nan"), float("nan")
    if len(vals) == 1:
        return vals[0], 0.0
    return statistics.mean(vals), statistics.stdev(vals)


def print_summary(off_results, on_results, paired):
    print("\n" + "=" * 78)
    print(f"집계 결과 (OFF n={len(off_results)}, ON n={len(on_results)}, "
          f"쌍 비교 n={len(paired)})")
    print("=" * 78)
    print(f"{'지표':<24}{'OFF (mean±std)':>22}{'ON (mean±std)':>22}{'평균 변화':>12}")
    for key, label in METRICS:
        off_vals = [r[key] for r in off_results]
        on_vals = [r[key] for r in on_results]
        om, ov = mean_std(off_vals)
        nm, nv = mean_std(on_vals)
        diffs = []
        for o, n in paired:
            a, b = o.get(key), n.get(key)
            if a == a and b == b and abs(a) > 1e-9:
                diffs.append((b / a - 1.0) * 100.0)
        dm, dv = mean_std(diffs)
        off_s = f"{om:.3f}±{ov:.3f}" if om == om else "n/a"
        on_s = f"{nm:.3f}±{nv:.3f}" if nm == nm else "n/a"
        diff_s = f"{dm:+.1f}%±{dv:.1f}" if dm == dm else "n/a"
        print(f"{label:<24}{off_s:>22}{on_s:>22}{diff_s:>12}")
    print("\n(평균 변화는 트라이얼별로 짝지어 계산한 %변화의 평균±표준편차. "
          "표준편차가 크면 물리엔진 실행별 편차가 커서 신뢰도가 낮다는 뜻.)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", "--trials", type=int, default=5, help="트라이얼 수 (기본 5)")
    ap.add_argument("--out-dir", default=os.path.expanduser("~/ros2_ws/trial_logs"))
    ap.add_argument("--min-active-s", type=float, default=15.0,
                    help="이보다 짧으면 중간에 FAULT난 것으로 보고 재시도")
    ap.add_argument("--max-attempts", type=int, default=3,
                    help="트라이얼당(조건별) 최대 시도 횟수")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    if not os.path.isfile(DRIVE_SCRIPT):
        sys.exit(f"주행 스크립트가 없습니다: {DRIVE_SCRIPT}")

    off_results, on_results, paired = [], [], []
    t_start = time.time()

    for i in range(1, args.trials + 1):
        print(f"\n=== 트라이얼 {i}/{args.trials} ===", flush=True)
        off_csv = os.path.join(args.out_dir, f"off_{i}.csv")
        on_csv = os.path.join(args.out_dir, f"on_{i}.csv")

        off_res = try_one(False, off_csv, args.out_dir, f"t{i}_off",
                          args.min_active_s, args.max_attempts)
        on_res = try_one(True, on_csv, args.out_dir, f"t{i}_on",
                         args.min_active_s, args.max_attempts)

        if off_res:
            off_results.append(off_res)
        if on_res:
            on_results.append(on_res)
        if off_res and on_res:
            paired.append((off_res, on_res))

    elapsed = time.time() - t_start
    print(f"\n총 소요 시간: {elapsed/60:.1f}분")
    print_summary(off_results, on_results, paired)
    print(f"\n원본 로그: {args.out_dir}/off_*.csv, on_*.csv")
    print(f"개별 실행 상세 비교: python3 analyze_log.py {args.out_dir}/off_1.csv "
          f"{args.out_dir}/on_1.csv")


if __name__ == "__main__":
    main()
