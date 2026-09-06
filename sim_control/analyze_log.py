#!/usr/bin/env python3
"""
analyze_log.py — gimbal_leveling_controller가 남긴 CSV 분석

보고서에 넣을 지표와 그림을 만든다. 조건을 바꿔 여러 번 기록한 뒤 한꺼번에
비교하는 것이 기본 사용법이다.

기록 예 (조건별로 CSV를 따로 남긴다)
    python3 gimbal_leveling_controller.py --ros-args -p use_sim_time:=true \
        -p enable_zv:=false -p log_csv_path:=/tmp/run_nozv.csv
    python3 gimbal_leveling_controller.py --ros-args -p use_sim_time:=true \
        -p enable_zv:=true  -p log_csv_path:=/tmp/run_zv.csv

분석
    python3 analyze_log.py /tmp/run_nozv.csv /tmp/run_zv.csv
    python3 analyze_log.py /tmp/run_*.csv --plot out.png

matplotlib이 없으면 수치 지표만 출력한다(의존성 아님).
"""

import argparse
import csv
import math
import os
import statistics
import sys


def read_log(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path}: 빈 파일")
    out = {}
    for key in rows[0]:
        col = []
        for r in rows:
            v = r[key]
            try:
                col.append(float(v))
            except (TypeError, ValueError):
                col.append(v)
        out[key] = col
    return out


def active_slice(log):
    """ACTIVE 구간만 잘라낸다. 기동 전 대기 구간이 지표를 흐리는 것을 막는다."""
    idx = [i for i, s in enumerate(log["state"]) if s == "ACTIVE"]
    if not idx:
        return None, None
    return idx[0], idx[-1] + 1


def numeric(seq):
    return [v for v in seq if isinstance(v, float) and v == v]


def rms(seq):
    v = numeric(seq)
    if not v:
        return float("nan")
    return math.sqrt(sum(x * x for x in v) / len(v))


def peak_abs(seq):
    v = [abs(x) for x in numeric(seq)]
    return max(v) if v else float("nan")


def find_peaks(t, y):
    """국소 최대(절대값 기준 포락선용). 인접 3점 비교로 단순 검출."""
    out = []
    for i in range(1, len(y) - 1):
        a, b, c = abs(y[i - 1]), abs(y[i]), abs(y[i + 1])
        if b >= a and b > c and b > 1e-6:
            out.append((t[i], b))
    return out


def decay_rate(t, y):
    """포락선에 지수 감쇠를 맞춰 시간상수를 추정한다.

    ln(peak) 대 t 의 최소자승 기울기 s 에서 tau = -1/s.
    감쇠가 없거나 증가하면 nan을 돌려준다.
    """
    pk = find_peaks(t, y)
    pk = [(ti, vi) for ti, vi in pk if vi > 1e-6]
    if len(pk) < 4:
        return float("nan"), len(pk)
    xs = [p[0] for p in pk]
    ys = [math.log(p[1]) for p in pk]
    n = len(xs)
    mx = sum(xs) / n
    my = sum(ys) / n
    den = sum((x - mx) ** 2 for x in xs)
    if den < 1e-12:
        return float("nan"), n
    slope = sum((xs[i] - mx) * (ys[i] - my) for i in range(n)) / den
    if slope >= 0:
        return float("nan"), n
    return -1.0 / slope, n


def settling_time(t, y, band_deg, hold_s=1.0):
    """|y|가 band 안에 들어와 hold_s 이상 유지되는 첫 시각."""
    n = len(t)
    i = 0
    while i < n:
        if abs(y[i]) <= band_deg:
            j = i
            while j < n and abs(y[j]) <= band_deg:
                if t[j] - t[i] >= hold_s:
                    return t[i]
                j += 1
            i = j
        else:
            i += 1
    return float("nan")


def _dft_mag(t, y, dt):
    """numpy가 없을 때 쓰는 순수 파이썬 대체 DFT(O(n^2)).
    표본 수가 큰 전체 실행 로그에 쓰면 느리므로, numpy 미설치 환경에서만
    폴백으로 쓰인다(이 환경엔 numpy가 있어 평소엔 fft_compare가 이쪽을
    타지 않는다)."""
    n = len(y)
    mean_y = sum(y) / n
    yc = [v - mean_y for v in y]
    freqs, mags = [], []
    for k in range(1, n // 2):
        re = sum(v * math.cos(-2.0 * math.pi * k * i / n) for i, v in enumerate(yc))
        im = sum(v * math.sin(-2.0 * math.pi * k * i / n) for i, v in enumerate(yc))
        freqs.append(k / (n * dt))
        mags.append(math.sqrt(re * re + im * im) / n)
    return freqs, mags


def fft_compare(t, input_y, response_y, f_min_hz=0.0, f_max_hz=None):
    """자극 입력(input_y)과 응답(response_y)의 주파수 성분을 비교한다.
    같은 시간축 t(균일 간격 가정, 중앙값 dt 사용) 위의 두 시계열이 필요하다.

    2단계 가설 검증(짐벌 ON이 슬로싱 공진 주파수 성분을 증폭하는지 확인)의
    진단 보조로 쓴다. 엄밀한 신호처리학적 coherence는 아니고,
    ratio = |FFT(response)| / |FFT(input)| 를 각 주파수에서 계산해 "입력
    대비 응답이 특정 주파수에서 얼마나 부풀었는지"를 보는 간단 지표다.

    반환: dict(freqs, input_mag, response_mag, ratio) — 리스트 4개.
    """
    n = len(t)
    if n < 4 or len(input_y) != n or len(response_y) != n:
        return {"freqs": [], "input_mag": [], "response_mag": [], "ratio": []}

    dts = [t[i + 1] - t[i] for i in range(n - 1)]
    dt = statistics.median(dts)
    if dt <= 0:
        return {"freqs": [], "input_mag": [], "response_mag": [], "ratio": []}

    try:
        import numpy as np
        in_arr = np.asarray(input_y, dtype=float)
        out_arr = np.asarray(response_y, dtype=float)
        freqs_full = np.fft.rfftfreq(n, d=dt)
        in_mag_full = np.abs(np.fft.rfft(in_arr - in_arr.mean())) / n
        out_mag_full = np.abs(np.fft.rfft(out_arr - out_arr.mean())) / n
        freqs = freqs_full[1:].tolist()
        in_mag = in_mag_full[1:].tolist()
        out_mag = out_mag_full[1:].tolist()
    except ImportError:
        freqs, in_mag = _dft_mag(t, input_y, dt)
        _, out_mag = _dft_mag(t, response_y, dt)

    if f_min_hz > 0.0 or f_max_hz is not None:
        keep = [i for i, f in enumerate(freqs)
                if f >= f_min_hz and (f_max_hz is None or f <= f_max_hz)]
        freqs = [freqs[i] for i in keep]
        in_mag = [in_mag[i] for i in keep]
        out_mag = [out_mag[i] for i in keep]

    eps = 1e-12
    ratio = [o / (i + eps) for i, o in zip(in_mag, out_mag)]
    return {"freqs": freqs, "input_mag": in_mag, "response_mag": out_mag, "ratio": ratio}


def plot_fft_compare(result, out_path, title=""):
    """fft_compare()의 결과를 그림으로 저장한다. matplotlib 없으면 건너뜀
    (make_plot과 동일한 선택적 의존성 패턴)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n[fft plot] matplotlib이 없어 그림을 건너뜁니다 "
              "(pip install matplotlib)")
        return

    freqs = result["freqs"]
    if not freqs:
        print("\n[fft plot] 유효한 주파수 성분이 없어 그림을 건너뜁니다")
        return

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(freqs, result["input_mag"], label="input", lw=1.2)
    axes[0].plot(freqs, result["response_mag"], label="response", lw=1.2)
    axes[0].set_ylabel("magnitude")
    axes[0].set_title(title or "입력 vs 응답 스펙트럼")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)

    axes[1].plot(freqs, result["ratio"], color="tab:red", lw=1.2)
    axes[1].set_ylabel("response/input ratio")
    axes[1].set_xlabel("frequency [Hz]")
    axes[1].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    print(f"\n[fft plot] 저장: {out_path}")


def analyze(path):
    log = read_log(path)
    lo, hi = active_slice(log)
    if lo is None:
        print(f"\n### {os.path.basename(path)}  — ACTIVE 구간 없음 (제어 미활성)")
        return None

    def col(name):
        return log[name][lo:hi] if name in log else []

    t0 = col("time_s")[0]
    t = [x - t0 for x in col("time_s")]

    res = {
        "name": os.path.basename(path),
        "duration_s": t[-1] if t else 0.0,
        "samples": len(t),
        "dt_mean_ms": (sum(numeric(col("dt_s"))) / max(1, len(numeric(col("dt_s"))))) * 1000.0,
        "dt_max_ms": peak_abs(col("dt_s")) * 1000.0,
    }

    # 트레이 절대 자세 = 수평 유지 성능 (논문 Fig 5/6 대응)
    for axis in ("roll", "pitch"):
        tray = col(f"tray_{axis}_deg")
        base = col(f"base_{axis}_deg")
        err = col(f"error_{axis}_deg")
        res[f"tray_{axis}_rms"] = rms(tray)
        res[f"tray_{axis}_peak"] = peak_abs(tray)
        res[f"base_{axis}_peak"] = peak_abs(base)
        res[f"err_{axis}_rms"] = rms(err)
        res[f"err_{axis}_peak"] = peak_abs(err)
        # 외란 저감비: 베이스 요동이 트레이로 얼마나 덜 전달됐는가
        bp = res[f"base_{axis}_peak"]
        res[f"reject_{axis}"] = (res[f"tray_{axis}_peak"] / bp) if bp > 1e-6 else float("nan")

    # 슬로싱 지표 = Housner 등가 진자 각도
    for axis, key in (("x", "slosh_x_deg"), ("y", "slosh_y_deg")):
        s = col(key)
        vals = numeric(s)
        if not vals:
            res[f"slosh_{axis}_rms"] = float("nan")
            res[f"slosh_{axis}_peak"] = float("nan")
            res[f"slosh_{axis}_tau"] = float("nan")
            res[f"slosh_{axis}_npk"] = 0
            continue
        res[f"slosh_{axis}_rms"] = rms(s)
        res[f"slosh_{axis}_peak"] = peak_abs(s)
        tt = [t[i] for i in range(len(s)) if isinstance(s[i], float) and s[i] == s[i]]
        tau, npk = decay_rate(tt, vals)
        res[f"slosh_{axis}_tau"] = tau
        res[f"slosh_{axis}_npk"] = npk

    # 논문 기준: 완만한 구간에서 +-5deg 이내 유지
    res["settle_roll_5deg"] = settling_time(t, numeric(col("tray_roll_deg")) or [0.0], 5.0)
    res["settle_pitch_5deg"] = settling_time(t, numeric(col("tray_pitch_deg")) or [0.0], 5.0)

    zv1 = numeric(col("zv_f1_hz"))
    res["zv_f1_hz"] = zv1[-1] if zv1 else float("nan")
    src = col("freq_source")
    res["freq_source"] = src[-1] if src else "?"
    res["_t"] = t
    res["_log"] = log
    res["_lo"] = lo
    res["_hi"] = hi
    return res


def fmt(v, nd=3):
    if isinstance(v, str):
        return v
    if v != v:
        return "  n/a"
    return f"{v:.{nd}f}"


def print_report(results):
    rs = [r for r in results if r]
    if not rs:
        print("분석할 ACTIVE 구간이 없습니다.")
        return

    print("\n" + "=" * 78)
    print("기본 정보")
    print("=" * 78)
    print(f"{'파일':<26}{'길이[s]':>9}{'샘플':>8}{'dt평균[ms]':>12}{'dt최대[ms]':>12}"
          f"{'ZV f1[Hz]':>11}")
    for r in rs:
        print(f"{r['name']:<26}{fmt(r['duration_s'],1):>9}{r['samples']:>8}"
              f"{fmt(r['dt_mean_ms'],2):>12}{fmt(r['dt_max_ms'],2):>12}"
              f"{fmt(r['zv_f1_hz'],3):>11}")

    print("\n" + "=" * 78)
    print("수평 유지 성능 — 트레이 절대 자세 (논문 Fig 5/6 대응)")
    print("=" * 78)
    for axis in ("roll", "pitch"):
        print(f"\n[{axis}]")
        print(f"{'파일':<26}{'트레이RMS':>11}{'트레이피크':>11}{'베이스피크':>11}"
              f"{'저감비':>9}{'+-5deg정착[s]':>14}")
        for r in rs:
            print(f"{r['name']:<26}{fmt(r[f'tray_{axis}_rms']):>11}"
                  f"{fmt(r[f'tray_{axis}_peak']):>11}{fmt(r[f'base_{axis}_peak']):>11}"
                  f"{fmt(r[f'reject_{axis}']):>9}"
                  f"{fmt(r[f'settle_{axis}_5deg'],2):>14}")

    print("\n" + "=" * 78)
    print("슬로싱 — Housner 등가 진자 각도 (URDF에 진자가 있어야 값이 나온다)")
    print("=" * 78)
    print(f"{'파일':<26}{'RMS[deg]':>11}{'피크[deg]':>11}{'감쇠tau[s]':>12}{'피크수':>8}")
    for r in rs:
        for axis in ("x", "y"):
            if r[f"slosh_{axis}_peak"] != r[f"slosh_{axis}_peak"]:
                continue
            print(f"{r['name'] + ' ' + axis:<26}{fmt(r[f'slosh_{axis}_rms']):>11}"
                  f"{fmt(r[f'slosh_{axis}_peak']):>11}"
                  f"{fmt(r[f'slosh_{axis}_tau'],3):>12}{r[f'slosh_{axis}_npk']:>8}")
    if all(r["slosh_x_peak"] != r["slosh_x_peak"] for r in rs):
        print("  (진자각 데이터 없음 — URDF에 Housner 진자와 /joint_states 발행이"
              " 추가되어야 한다)")

    if len(rs) >= 2:
        print("\n" + "=" * 78)
        print(f"비교: {rs[0]['name']} 기준 대비")
        print("=" * 78)
        base = rs[0]
        for r in rs[1:]:
            print(f"\n{r['name']}")
            for key, label in (("tray_roll_rms", "트레이 roll RMS"),
                               ("tray_pitch_rms", "트레이 pitch RMS"),
                               ("slosh_x_rms", "슬로싱 x RMS"),
                               ("slosh_y_rms", "슬로싱 y RMS"),
                               ("slosh_x_peak", "슬로싱 x 피크")):
                a, b = base.get(key), r.get(key)
                if a is None or b is None or a != a or b != b or abs(a) < 1e-9:
                    continue
                print(f"  {label:<20}{fmt(a)} -> {fmt(b)}  ({(b/a-1)*100:+.1f}%)")


def make_plot(results, out_path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("\n[plot] matplotlib이 없어 그림을 건너뜁니다 "
              "(pip install matplotlib)")
        return

    rs = [r for r in results if r]
    if not rs:
        return
    fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)

    for r in rs:
        log, lo, hi, t = r["_log"], r["_lo"], r["_hi"], r["_t"]
        axes[0].plot(t, log["base_pitch_deg"][lo:hi], "--", lw=1, alpha=0.6,
                     label=f"{r['name']} base")
        axes[0].plot(t, log["tray_pitch_deg"][lo:hi], lw=1.4,
                     label=f"{r['name']} tray")
        axes[1].plot(t, log["cmd_pitch_deg"][lo:hi], lw=1.2, label=r["name"])
        if "slosh_x_deg" in log:
            axes[2].plot(t, log["slosh_x_deg"][lo:hi], lw=1.2, label=r["name"])

    axes[0].set_ylabel("pitch [deg]")
    axes[0].set_title("베이스 vs 트레이 절대 자세")
    axes[0].axhspan(-5, 5, color="green", alpha=0.08)
    axes[1].set_ylabel("조인트 명령 [deg]")
    axes[1].set_title("pitch 조인트 명령")
    axes[2].set_ylabel("진자각 [deg]")
    axes[2].set_title("슬로싱 지표 (Housner 등가 진자)")
    axes[2].set_xlabel("time [s]")
    for a in axes:
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    print(f"\n[plot] 저장: {out_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+", help="gimbal_leveling_controller가 남긴 CSV")
    ap.add_argument("--plot", metavar="PNG", help="비교 그림 저장 경로")
    args = ap.parse_args()

    results = []
    for path in args.csv:
        if not os.path.isfile(path):
            print(f"건너뜀(없음): {path}")
            continue
        results.append(analyze(path))

    print_report(results)
    if args.plot:
        make_plot(results, args.plot)
    return 0


if __name__ == "__main__":
    sys.exit(main())
