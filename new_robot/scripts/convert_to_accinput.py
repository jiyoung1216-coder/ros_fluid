import argparse
import csv

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input_csv', help='tank_motion.csv from process_odometry.py')
    parser.add_argument('output_csv', help='e.g. new_robot_accel.csv')
    args = parser.parse_args()

    with open(args.input_csv) as f:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]

    times = np.array([r['time_s'] for r in rows])
    lin = np.array([[r['lin_acc_x'], r['lin_acc_y'], r['lin_acc_z']] for r in rows])
    ang_vel = np.array([[r['ang_vel_x'], r['ang_vel_y'], r['ang_vel_z']] for r in rows])

    with open(args.output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        for i in range(len(times)):
            writer.writerow([
                f'{times[i]:.6f}',
                f'{lin[i,0]:.6f}', f'{lin[i,1]:.6f}', f'{lin[i,2]:.6f}',
                f'{ang_vel[i,0]:.6f}', f'{ang_vel[i,1]:.6f}', f'{ang_vel[i,2]:.6f}',
            ])

    print(f'wrote {len(times)} rows, duration {times[-1]:.3f}s, to {args.output_csv}')


if __name__ == '__main__':
    main()