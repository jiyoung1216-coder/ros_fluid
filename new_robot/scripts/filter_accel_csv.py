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
  1. 중앙값(median) 필터 — 언덕 충격 등으로 생기는 단일 샘플(10ms) 글리치를
     제거한다. 이동평균만으로는 1~2 샘플짜리 순간 스파이크가 살짝 뭉개진
     채로 남아서, DualSPHysics에 누적 주입되면 SPH가 서서히 불안정해지다가
     입자가 대량으로 이탈(발산)하는 문제가 있었다(2026-08-28, 40초 구간
     재추출 중 t≈20.5s부터 입자 70% 이탈로 실제 발생 확인). 중앙값 필터는
     지속되는 언덕 등반 가속도(수 m/s^2가 1초 이상 유지)는 그대로 보존하면서
     한두 샘플짜리 튀는 값만 제거한다.
  2. 이동평균 저역통과 필터 (중앙값 필터 후 남은 고주파 잔여분 완화)
  3. 최대값 클램프 (그래도 남는 극단값 제한)

사용법:
    python3 filter_accel_csv.py new_robot_accel.csv new_robot_accel_filtered.csv
"""

import csv
import sys

LIN_ACC_MAX = 20.0    # m/s^2, 실제 주행에서 나올 법한 최대 가속도보다 넉넉하게
ANG_VEL_MAX = 5.0      # rad/s, 각속도 상한
MEDIAN_WINDOW = 11      # 중앙값 필터 윈도우(샘플 개수, 100Hz 기준 0.11초).
                         # 5로는 부족했다 — 언덕 진입 순간 바퀴가 모서리에
                         # 부딪히며 매 샘플 부호가 뒤집히는 접촉 채터링(구간
                         # ±60 m/s^2대)이 나오는데, 5샘플 중앙값은 이 정도
                         # 고주파 진동을 못 눌렀다(2026-08-28 실측 확인).
SMOOTH_WINDOW = 7       # 이동평균 윈도우(샘플 개수, 100Hz 기준 0.07초)


def median_filter(values, window):
    n = len(values)
    half = window // 2
    out = [0.0] * n
    for i in range(n):
        lo = max(0, i - half)
        hi = min(n, i + half + 1)
        seg = sorted(values[lo:hi])
        out[i] = seg[len(seg) // 2]
    return out


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

    medianed = [median_filter(c, MEDIAN_WINDOW) for c in cols]
    smoothed = [moving_average(c, SMOOTH_WINDOW) for c in medianed]

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