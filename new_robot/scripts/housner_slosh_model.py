import argparse
import csv
import math

import numpy as np

G = 9.81
LAMBDA1 = 1.8412  # first root of J1'(x) = 0 (upright circular cylinder, mode 1)

TANK_RADIUS = 0.06     # m, outer radius from generate_water_tank_mesh.py
DAMPING_RATIO = 0.01   # typical for low-viscosity liquid sloshing (water)


def sloshing_parameters(radius, fill_height, rho=1000.0):
    """Housner/Abramson equivalent mechanical model for an upright
    circular cylindrical tank (NASA SP-106 style closed-form formulas,
    first sloshing mode only, small-amplitude linear theory)."""
    R, h = radius, fill_height
    m_liquid = rho * math.pi * R * R * h

    omega1 = math.sqrt((LAMBDA1 * G / R) * math.tanh(LAMBDA1 * h / R))

    mass_ratio = 0.318 * (R / h) * math.tanh(1.84 * h / R)
    m1 = m_liquid * mass_ratio
    m0 = m_liquid - m1

    x = LAMBDA1 * h / R
    h1_over_h = 1 - (math.cosh(x) - 1) / (x * math.sinh(x))
    h1 = h1_over_h * h
    h0 = h / 2.0  # rigid-portion centroid, approx for uniform mass below the sloshing layer

    return {
        'm_liquid': m_liquid,
        'omega1': omega1,
        'period1': 2 * math.pi / omega1,
        'm1': m1,
        'm0': m0,
        'h1': h1,
        'h0': h0,
    }


def _deriv(state, a_forcing, omega1, c1):
    x, v = state
    return np.array([v, -omega1 ** 2 * x - c1 * v - a_forcing])


def simulate(times, acc_x, acc_y, params, damping_ratio=DAMPING_RATIO):
    """RK4 integration of a base-excited spring-mass-damper (one per
    horizontal axis) representing the first sloshing mode."""
    m1 = params['m1']
    omega1 = params['omega1']
    c1 = 2 * damping_ratio * omega1

    n = len(times)
    xi = np.zeros((n, 2))       # sloshing-mass displacement relative to tank [x, y]
    xi_dot = np.zeros((n, 2))

    for i in range(1, n):
        dt = times[i] - times[i - 1]
        a = np.array([acc_x[i - 1], acc_y[i - 1]])
        for axis in range(2):
            state = np.array([xi[i - 1, axis], xi_dot[i - 1, axis]])
            k1 = _deriv(state, a[axis], omega1, c1)
            k2 = _deriv(state + 0.5 * dt * k1, a[axis], omega1, c1)
            k3 = _deriv(state + 0.5 * dt * k2, a[axis], omega1, c1)
            k4 = _deriv(state + dt * k3, a[axis], omega1, c1)
            state_next = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
            xi[i, axis], xi_dot[i, axis] = state_next

    xi_ddot = np.gradient(xi_dot, times, axis=0)
    force = m1 * (xi_ddot + np.column_stack([acc_x, acc_y]))
    return xi, xi_dot, force


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input_csv', help='tank_motion.csv from process_odometry.py')
    parser.add_argument('output_csv')
    parser.add_argument('--fill-height', type=float, default=0.09,
                         help='liquid height in tank, meters (default: half of the 0.18m tank)')
    parser.add_argument('--radius', type=float, default=TANK_RADIUS)
    parser.add_argument('--damping-ratio', type=float, default=DAMPING_RATIO)
    args = parser.parse_args()

    with open(args.input_csv) as f:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]

    times = np.array([r['time_s'] for r in rows])
    acc_x = np.array([r['lin_acc_x'] for r in rows])
    acc_y = np.array([r['lin_acc_y'] for r in rows])

    params = sloshing_parameters(args.radius, args.fill_height)
    print(f"liquid mass: {params['m_liquid']:.4f} kg")
    print(f"sloshing mass m1: {params['m1']:.4f} kg "
          f"({params['m1'] / params['m_liquid'] * 100:.1f}% of liquid)")
    print(f"natural frequency: {params['omega1']:.3f} rad/s "
          f"({params['period1']:.3f} s period)")
    print(f"sloshing mass height above tank bottom: {params['h1']:.4f} m")

    xi, xi_dot, force = simulate(times, acc_x, acc_y, params, args.damping_ratio)
    moment = force * params['h1']  # rough overturning-moment estimate (force x lever arm)

    with open(args.output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time_s', 'slosh_disp_x', 'slosh_disp_y',
            'slosh_force_x', 'slosh_force_y',
            'slosh_moment_x', 'slosh_moment_y',
        ])
        for i in range(len(times)):
            writer.writerow([
                times[i], xi[i, 0], xi[i, 1],
                force[i, 0], force[i, 1],
                moment[i, 0], moment[i, 1],
            ])

    peak_force = np.max(np.linalg.norm(force, axis=1))
    peak_moment = np.max(np.linalg.norm(moment, axis=1))
    print(f"peak sloshing force magnitude: {peak_force:.4f} N")
    print(f"peak sloshing moment magnitude: {peak_moment:.4f} N*m")
    print(f"wrote {len(times)} rows to {args.output_csv}")


if __name__ == '__main__':
    main()