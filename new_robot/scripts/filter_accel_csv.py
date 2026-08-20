#!/usr/bin/env python3
"""
filter_accel_csv.py

record_tray_accel_for_dsph.py로 뽑은 CSV(time,linx,liny,linz,angx,angy,angz,
헤더 없음)에 아직 남아있는 물리 노이즈 스파이크를 걸러내서 DualSPHysics
accinput에 넣기 적당한 크기로 만든다.

DualSPHysics에서 물 입자가 미친 듯이 튀다가 사라지는 "발산" 현상은
보통 주입되는 가속도가 비현실적으로 클 때(수백 m/s^2 등) 생긴다. 실제
차량 주행 가속도는 급제동/급가속이라 해도 보통 몇 m/s^2 수준이라, 그보다
훨씬 큰 값은 거의 다 아직 못 잡은 시뮬레이션 잔여 노이즈로 보고 걸러낸다.

처리 순서:
  1. 이동평균 저역통과 필터 (고주파 스파이크 완화)
  2. 최대값 클램프 (그래도 남는 극단값 제한)

사용법:
    python3 filter_accel_csv.py new_robot_accel.csv new_robot_accel_filtered.csv
"""

import csv
import sys

LIN_ACC_MAX = 20.0    # m/s^2, 실제 주행에서 나올 법한 최대 가속도보다 넉넉하게
ANG_VEL_MAX = 5.0      # rad/s, 각속도 상한
SMOOTH_WINDOW = 5       # 이동평균 윈도우(샘플 개수, 100Hz 기준 0.05초)


def moving_average(values, window):
    n = len(values)
    half = window // 2
    out = [0.0] * n
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        out[i] = sum(values[lo:hi]) / (hi - lo)
    return out


def clamp(v, limit):
    return max(-limit, min(limit, v))


def main():
    if len(sys.argv) != 3:
        print("사용법: python3 filter_accel_csv.py <입력.csv> <출력.csv>")
        sys.exit(1)

    in_path, out_path = sys.argv[1], sys.argv[2]

    times = []
    cols = [[] for _ in range(6)]  # linx,liny,linz,angx,angy,angz

    with open(in_path) as f:
        for row in csv.reader(f):
            if not row:
                continue
            times.append(float(row[0]))
            for i in range(6):
                cols[i].append(float(row[i + 1]))

    smoothed = [moving_average(c, SMOOTH_WINDOW) for c in cols]

    limits = [LIN_ACC_MAX, LIN_ACC_MAX, LIN_ACC_MAX, ANG_VEL_MAX, ANG_VEL_MAX, ANG_VEL_MAX]
    clamped = [[clamp(v, limits[i]) for v in smoothed[i]] for i in range(6)]

    n_clamped = 0
    for i in range(6):
        for orig, cl in zip(smoothed[i], clamped[i]):
            if abs(orig - cl) > 1e-9:
                n_clamped += 1

    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        for j in range(len(times)):
            writer.writerow(
                [f"{times[j]:.6f}"] + [f"{clamped[i][j]:.6f}" for i in range(6)]
            )

    print(f"{len(times)}개 샘플 처리 완료 -> {out_path}")
    print(f"클램프에 걸린 값: {n_clamped}개 "
          f"(선형 ±{LIN_ACC_MAX}, 각속도 ±{ANG_VEL_MAX} 초과분)")


if __name__ == "__main__":
    main()