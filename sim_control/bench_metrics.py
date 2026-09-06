#!/usr/bin/env python3
"""
bench_metrics.py

1단계 벤치마크(짐벌 ON/OFF 슬로싱 비교)에서 쓰는 지표 계산 함수 모음.
순수 함수로만 구성 — Gazebo/ROS2/DualSPHysics 실행과 분리되어 있어
test_core.py에서 오프라인으로 단위시험 가능하다.

extract_free_surface.py(무수정, "그대로 사용")가 뽑아낸 (t, h) 수면 높이
시계열을 입력으로 받아 아래를 계산한다:
  - settling_time  : 자극 종료 후 "정지 캘리브레이션으로 측정한" 대역 안에
                      hold_s초 연속으로 머무를 때까지 걸린 시간
  - max_rise        : 정지 수위(h_calm) 대비 최대 상승량
  - residual_rms     : 자극 종료 후 window_s초 창의 수면 높이 표준편차
  - contamination_ratio : dual-IMU 단일실행 방법의 유일한 가정(짐벌 반작용이
                      base IMU로 새지 않는다)을 실측 검증하기 위한 두 accel
                      트레이스 간 최대 차이 비율
  - fft_spectrum     : 자극 종료 후 신호의 주파수 성분 진단(공진 지속 여부)

2026-09-06: 사용자 승인 설계에 따라 판정 기준을 절대값(예: "7cm")이 아니라
정지 캘리브레이션 실측값(h_calm, sigma_h) 기준으로 정의한다. band는
max(2cm, 3*sigma_h)로 계산하며 2cm는 사용자가 승인 답변에서 명시적으로 준
값이다(임의 상수 아님).
"""

import math
import statistics
from dataclasses import dataclass


DEFAULT_MIN_BAND_M = 0.02  # 사용자 승인 답변에 명시된 값 ("max(2cm, 3σ)")
DEFAULT_SIGMA_MULT = 3.0   # 사용자 승인 답변에 명시된 값 ("3σ")
DEFAULT_HOLD_S = 2.0       # 원 스펙에 명시된 값 ("2초 연속 유지")
DEFAULT_RESIDUAL_WINDOW_S = 3.0  # 원 스펙에 명시된 값 ("3초 창")
MAX_RISE_HOLD_S = 2.0      # 사용자 승인 답변 PASS/FAIL 규칙 (ii)에 명시된 값


@dataclass
class SettlingResult:
    time_s: float          # 자극 종료 시점 기준 상대 시간. not_settled면 관측 구간 끝까지의 값.
    not_settled: bool
    band_m: float           # 실제 사용된 판정 대역(h_calm 기준)


def settling_band_m(sigma_h_m, min_band_m=DEFAULT_MIN_BAND_M, sigma_mult=DEFAULT_SIGMA_MULT):
    """정지 캘리브레이션 실측 sigma_h로부터 판정 대역을 계산한다."""
    return max(min_band_m, sigma_mult * sigma_h_m)


def settling_time(t, h, h_calm_m, sigma_h_m, excite_end_t,
                   hold_s=DEFAULT_HOLD_S, min_band_m=DEFAULT_MIN_BAND_M,
                   sigma_mult=DEFAULT_SIGMA_MULT):
    """자극 종료(excite_end_t) 이후 |h - h_calm| <= band를 "그 이후로 계속"
    유지하기 시작하는 시점까지의 시간을 반환한다.

    2026-09-06: 처음 구현("대역 안에 hold_s초 연속 머무는 첫 시점")은
    실제 A/B 실행에서 심각한 버그로 드러났다 — 자극 직후 수면이 관성으로
    잠깐 h_calm 근처를 스쳐 지나가면(실측: 자극 종료~수 초 뒤에야 진짜
    피크(12cm)가 나타남) 그 우연한 첫 통과를 "정착"으로 오판해
    settling_time=0.00을 반환했다(실제로는 9초 뒤에 12cm까지 치솟는데도).
    표준적인 정착시간 정의(그 이후로 다시는 대역을 벗어나지 않는 시점)를
    쓰도록 고쳤다 — 관측 구간 끝에서부터 거꾸로 "마지막으로 대역을
    벗어난 시점"을 찾고, 그 다음 시점을 정착 시점으로 본다.
    """
    band = settling_band_m(sigma_h_m, min_band_m, sigma_mult)
    samples = [(ti, hi) for ti, hi in zip(t, h) if ti >= excite_end_t]
    if not samples:
        return SettlingResult(0.0, True, band)

    last_violation_idx = None
    for i, (ti, hi) in enumerate(samples):
        if abs(hi - h_calm_m) > band:
            last_violation_idx = i

    last_idx = len(samples) - 1
    if last_violation_idx is None:
        # 자극 종료 이후 대역을 벗어난 적이 없음 -> 즉시 정착
        return SettlingResult(0.0, False, band)
    if last_violation_idx == last_idx:
        # 관측 구간 마지막 샘플까지도 대역 밖 -> 정착 실패
        return SettlingResult(samples[last_idx][0] - excite_end_t, True, band)

    settle_t = samples[last_violation_idx + 1][0]
    remaining = samples[last_idx][0] - settle_t
    not_settled = remaining < hold_s  # 마지막 위반 이후 hold_s만큼의
                                       # "깨끗한" 구간을 실제로 관측했는지
    return SettlingResult(settle_t - excite_end_t, not_settled, band)


def max_rise(h, h_calm_m):
    """정지 수위 대비 최대 상승량(절대 수위가 아니라 h_calm 기준 상승량)."""
    return max(h) - h_calm_m


def sustained_rise_over(t, h, h_calm_m, threshold_above_calm_m, hold_s=MAX_RISE_HOLD_S):
    """h - h_calm >= threshold_above_calm_m 이 hold_s초 연속 유지되는지 여부.
    PASS/FAIL 규칙 (ii) "ON 최대 수면 상승 >= h_calm+4cm를 2초 유지"에 사용.
    """
    streak_start = None
    for ti, hi in zip(t, h):
        if hi - h_calm_m >= threshold_above_calm_m:
            if streak_start is None:
                streak_start = ti
            elif ti - streak_start >= hold_s:
                return True
        else:
            streak_start = None
    return False


def residual_rms(t, h, excite_end_t, window_s=DEFAULT_RESIDUAL_WINDOW_S):
    """자극 종료 후 window_s초 창의 수면 높이 표준편차."""
    window = [hi for ti, hi in zip(t, h)
              if excite_end_t <= ti <= excite_end_t + window_s]
    if len(window) < 2:
        return 0.0
    return statistics.pstdev(window)


def _interp(t_query, t_src, y_src):
    """numpy 없이 쓰는 선형보간 (모듈을 순수-표준라이브러리로 유지)."""
    n = len(t_src)
    if n == 0:
        return 0.0
    if t_query <= t_src[0]:
        return y_src[0]
    if t_query >= t_src[-1]:
        return y_src[-1]
    lo, hi = 0, n - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if t_src[mid] <= t_query:
            lo = mid
        else:
            hi = mid
    span = t_src[hi] - t_src[lo]
    if span <= 0:
        return y_src[lo]
    frac = (t_query - t_src[lo]) / span
    return y_src[lo] + frac * (y_src[hi] - y_src[lo])


@dataclass
class ContaminationResult:
    max_abs_delta_a_mps2: float
    max_abs_delta_w_rads: float
    ratio_a: float   # max_abs_delta_a / excitation_amplitude_mps2
    contaminated: bool  # ratio_a >= 0.10 (사용자 승인 답변의 "10%" 기준)


CONTAMINATION_RATIO_THRESHOLD = 0.10  # 사용자 승인 답변에 명시된 값 ("10% 미만")


def contamination_check(t_off, a_off, t_on, a_on, excitation_amplitude_mps2):
    """짐벌 OFF 단독 실행의 base IMU(a_off)와 ON 실행의 base IMU(a_on)를
    같은 시간축으로 보간해 비교, 짐벌 반작용이 base로 새는 정도를 측정한다.
    a_off/a_on은 (ax,ay,az) 튜플의 시퀀스.
    """
    max_da = 0.0
    for i, ti in enumerate(t_off):
        ax_on = _interp(ti, t_on, [v[0] for v in a_on])
        ay_on = _interp(ti, t_on, [v[1] for v in a_on])
        az_on = _interp(ti, t_on, [v[2] for v in a_on])
        ax_off, ay_off, az_off = a_off[i]
        d = math.sqrt((ax_on - ax_off) ** 2 + (ay_on - ay_off) ** 2 + (az_on - az_off) ** 2)
        if d > max_da:
            max_da = d
    ratio = max_da / excitation_amplitude_mps2 if excitation_amplitude_mps2 > 1e-9 else 0.0
    return ContaminationResult(
        max_abs_delta_a_mps2=max_da,
        max_abs_delta_w_rads=0.0,  # TODO: 실측 필요 — 각속도 채널까지 필요해지면 동일 패턴으로 확장
        ratio_a=ratio,
        contaminated=ratio >= CONTAMINATION_RATIO_THRESHOLD,
    )


# extract_free_surface.py의 --wall-band 0.9 관례(경계 근접 판정 배율)를
# 그대로 재사용 — "클램프 값의 90% 이상"을 접촉솔버 스파이크로 본다.
NEAR_CLAMP_FRACTION = 0.9


@dataclass
class RobustContaminationResult:
    n_clean_samples: int
    n_excluded_spikes: int
    rms_delta_mps2: float
    rms_off_mps2: float
    ratio_rms: float
    contaminated: bool


def contamination_check_robust(t_off, a_off, t_on, a_on, clamp_mps2):
    """contamination_check()의 순간 최대값 방식은, record_gimbal_accel_for_dsph.py
    자체 주석에 문서화된 접촉솔버 스파이크(수천 샘플 중 2~3개, 실측 최대
    420 m/s^2 -> ACCEL_CLAMP_MPS2로 클램프)에 크게 좌우되어 실제 저주파
    반작용 누수와 구분이 안 된다(실측: ratio_a>1.0까지 나왔지만 클램프
    임계값 바로 아래 값끼리의 우연한 차이일 뿐이었음).

    이 함수는 두 트레이스 모두에서 클램프의 NEAR_CLAMP_FRACTION 이상인
    시점을 제외한 뒤, 남은 "정상 주행" 구간의 RMS로 비교한다.
    """
    near_clamp = NEAR_CLAMP_FRACTION * clamp_mps2
    clean_off, clean_on = [], []
    for i, ti in enumerate(t_off):
        mag_off = math.hypot(a_off[i][0], a_off[i][1])
        if mag_off >= near_clamp:
            continue
        ax_on = _interp(ti, t_on, [v[0] for v in a_on])
        ay_on = _interp(ti, t_on, [v[1] for v in a_on])
        mag_on = math.hypot(ax_on, ay_on)
        if mag_on >= near_clamp:
            continue
        clean_off.append(mag_off)
        clean_on.append(mag_on)

    n_excluded = len(t_off) - len(clean_off)
    if len(clean_off) < 2:
        return RobustContaminationResult(len(clean_off), n_excluded, 0.0, 0.0, 0.0, False)

    delta = [b - a for a, b in zip(clean_off, clean_on)]
    rms_delta = statistics.pstdev(delta)
    rms_off = statistics.pstdev(clean_off)
    ratio = rms_delta / rms_off if rms_off > 1e-9 else 0.0
    return RobustContaminationResult(
        n_clean_samples=len(clean_off), n_excluded_spikes=n_excluded,
        rms_delta_mps2=rms_delta, rms_off_mps2=rms_off, ratio_rms=ratio,
        contaminated=ratio >= CONTAMINATION_RATIO_THRESHOLD,
    )


def fft_spectrum(t, y):
    """t가 대략 균일 간격이라고 가정하고(중앙값 dt 사용) 실수 FFT 크기 스펙트럼을 낸다.
    반환: (freqs_hz, magnitude) — DC(0Hz) 제외.
    표준 라이브러리만 사용(analyze_log.py의 "그대로 사용" 원칙과 별개로,
    이 모듈은 신규 파일이라 numpy 의존을 넣어도 되지만, 다른 신규 파일들과
    스타일을 맞추기 위해 순수 파이썬 DFT를 쓴다 — 샘플 수가 수백~수천 개
    수준(진단용 3초 창)이라 O(n^2) DFT로도 충분히 빠르다).
    """
    n = len(y)
    if n < 4:
        return [], []
    dts = [t[i + 1] - t[i] for i in range(n - 1)]
    dt = statistics.median(dts)
    if dt <= 0:
        return [], []
    mean_y = sum(y) / n
    yc = [v - mean_y for v in y]

    freqs = []
    mags = []
    max_k = n // 2
    for k in range(1, max_k):
        re = 0.0
        im = 0.0
        for i, v in enumerate(yc):
            angle = -2.0 * math.pi * k * i / n
            re += v * math.cos(angle)
            im += v * math.sin(angle)
        mag = math.sqrt(re * re + im * im) / n
        freqs.append(k / (n * dt))
        mags.append(mag)
    return freqs, mags


def dominant_frequency(t, y, f_min_hz=0.0, f_max_hz=None):
    """fft_spectrum에서 [f_min_hz, f_max_hz] 범위 안의 최대 성분 주파수를 찾는다."""
    freqs, mags = fft_spectrum(t, y)
    if not freqs:
        return None, 0.0
    best_f, best_m = None, -1.0
    for f, m in zip(freqs, mags):
        if f < f_min_hz:
            continue
        if f_max_hz is not None and f > f_max_hz:
            continue
        if m > best_m:
            best_f, best_m = f, m
    return best_f, best_m
