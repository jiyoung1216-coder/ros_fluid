import argparse
import csv

import numpy as np

# root -> water_tank 고정 변환 (urdf의 water_tank_joint origin과 동일)
TANK_OFFSET_XYZ = np.array([-0.515032, 0.292627, -0.0950903])
TANK_OFFSET_RPY = np.array([0.0, -1.5708, 0.0])  # roll, pitch, yaw


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


def quat_to_matrix(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def matrix_to_rpy(R):
    pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    cp = np.cos(pitch)
    if abs(cp) > 1e-6:
        roll = np.arctan2(R[2, 1] / cp, R[2, 2] / cp)
        yaw = np.arctan2(R[1, 0] / cp, R[0, 0] / cp)
    else:
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])
    return roll, pitch, yaw


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input_csv')
    parser.add_argument('output_csv')
    args = parser.parse_args()

    with open(args.input_csv) as f:
        rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]

    R_offset = rpy_to_matrix(*TANK_OFFSET_RPY)
    t0 = rows[0]['stamp_sec'] + rows[0]['stamp_nanosec'] * 1e-9

    times, tank_pos, tank_rpy = [], [], []
    for row in rows:
        t = row['stamp_sec'] + row['stamp_nanosec'] * 1e-9 - t0
        root_pos = np.array([row['pos_x'], row['pos_y'], row['pos_z']])
        R_root = quat_to_matrix(row['quat_x'], row['quat_y'], row['quat_z'], row['quat_w'])

        # root의 세계좌표 pose에 고정 오프셋을 합성해서 water_tank의 세계좌표 pose를 구함
        p_tank = root_pos + R_root @ TANK_OFFSET_XYZ
        R_tank = R_root @ R_offset

        times.append(t)
        tank_pos.append(p_tank)
        tank_rpy.append(matrix_to_rpy(R_tank))

    times = np.array(times)
    tank_pos = np.array(tank_pos)
    tank_rpy = np.array(tank_rpy)
    # 각도가 ±pi를 넘어갈 때 미분이 튀지 않도록 unwrap 후 미분
    tank_rpy[:, 0] = np.unwrap(tank_rpy[:, 0])
    tank_rpy[:, 1] = np.unwrap(tank_rpy[:, 1])
    tank_rpy[:, 2] = np.unwrap(tank_rpy[:, 2])

    lin_vel = np.gradient(tank_pos, times, axis=0)
    lin_acc = np.gradient(lin_vel, times, axis=0)
    ang_vel = np.gradient(tank_rpy, times, axis=0)

    with open(args.output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'time_s',
            'tank_x', 'tank_y', 'tank_z',
            'tank_roll', 'tank_pitch', 'tank_yaw',
            'lin_vel_x', 'lin_vel_y', 'lin_vel_z',
            'lin_acc_x', 'lin_acc_y', 'lin_acc_z',
            'ang_vel_x', 'ang_vel_y', 'ang_vel_z',
        ])
        for i in range(len(times)):
            writer.writerow([
                times[i], *tank_pos[i], *tank_rpy[i],
                *lin_vel[i], *lin_acc[i], *ang_vel[i],
            ])

    print(f'wrote {len(times)} rows to {args.output_csv}')


if __name__ == '__main__':
    main()