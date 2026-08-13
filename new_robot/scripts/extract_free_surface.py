import argparse
import csv
import glob
import math
import os
import re

import vtk


def read_points(path):
    reader = vtk.vtkPolyDataReader()
    reader.SetFileName(path)
    reader.Update()
    polydata = reader.GetOutput()
    points = polydata.GetPoints()
    n = points.GetNumberOfPoints()
    return [points.GetPoint(i) for i in range(n)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('vtk_dir', help='folder containing PartFluid_XXXX.vtk files')
    parser.add_argument('output_csv')
    parser.add_argument('--time-out', type=float, default=0.05,
                         help='seconds between PART files (must match TimeOut in the case XML)')
    parser.add_argument('--tank-radius', type=float, default=0.06)
    parser.add_argument('--still-water-level', type=float, default=0.09,
                         help='rest water height used to compute wave height above/below it')
    parser.add_argument('--wall-band', type=float, default=0.9,
                         help='particles with r > wall_band*tank_radius are counted as "near wall"')
    args = parser.parse_args()

    files = sorted(
        glob.glob(os.path.join(args.vtk_dir, 'PartFluid_*.vtk')),
        key=lambda p: int(re.search(r'PartFluid_(\d+)\.vtk', p).group(1)),
    )
    if not files:
        raise SystemExit(f'no PartFluid_*.vtk files found in {args.vtk_dir}')

    rows = []
    for idx, path in enumerate(files):
        pts = read_points(path)
        t = idx * args.time_out

        z_all = [p[2] for p in pts]
        max_z_all = max(z_all)
        min_z_all = min(z_all)

        wall_r = args.wall_band * args.tank_radius
        z_wall = [p[2] for p in pts if math.hypot(p[0], p[1]) >= wall_r]
        max_z_wall = max(z_wall) if z_wall else float('nan')

        rows.append({
            'time_s': t,
            'max_z_all': max_z_all,
            'min_z_all': min_z_all,
            'max_z_wall': max_z_wall,
            'wave_height_all': max_z_all - args.still_water_level,
            'wave_height_wall': max_z_wall - args.still_water_level,
        })

    with open(args.output_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    peak_wave = max(r['wave_height_wall'] for r in rows if not math.isnan(r['wave_height_wall']))
    peak_time = next(r['time_s'] for r in rows if r['wave_height_wall'] == peak_wave)
    print(f'processed {len(files)} PART files')
    print(f'peak wall wave height above still level: {peak_wave:.4f} m at t={peak_time:.2f}s')
    print(f'wrote {args.output_csv}')


if __name__ == '__main__':
    main()