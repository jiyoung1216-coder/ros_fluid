import math
import struct

OUTER_RADIUS = 0.091125
WALL_THICKNESS = 0.01
INNER_RADIUS = OUTER_RADIUS - WALL_THICKNESS
HEIGHT = 0.3645
SEGMENTS = 32
OUTPUT_PATH = "meshes/water_tank.stl"


def vertex(r, theta, z):
    return (r * math.cos(theta), r * math.sin(theta), z)


def normal(v0, v1, v2):
    ux, uy, uz = v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]
    vx, vy, vz = v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]
    nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
    length = math.sqrt(nx * nx + ny * ny + nz * nz)
    if length == 0:
        return (0.0, 0.0, 0.0)
    return (nx / length, ny / length, nz / length)


def build_triangles():
    z_top = HEIGHT / 2
    z_bot = -HEIGHT / 2
    triangles = []

    for i in range(SEGMENTS):
        th0 = 2 * math.pi * i / SEGMENTS
        th1 = 2 * math.pi * (i + 1) / SEGMENTS

        o0t, o1t = vertex(OUTER_RADIUS, th0, z_top), vertex(OUTER_RADIUS, th1, z_top)
        o0b, o1b = vertex(OUTER_RADIUS, th0, z_bot), vertex(OUTER_RADIUS, th1, z_bot)
        triangles.append((o0b, o1b, o1t))
        triangles.append((o0b, o1t, o0t))

        i0t, i1t = vertex(INNER_RADIUS, th0, z_top), vertex(INNER_RADIUS, th1, z_top)
        i0b, i1b = vertex(INNER_RADIUS, th0, z_bot), vertex(INNER_RADIUS, th1, z_bot)
        triangles.append((i0b, i1t, i1b))
        triangles.append((i0b, i0t, i1t))

        triangles.append((o0b, i0b, i1b))
        triangles.append((o0b, i1b, o1b))

    return triangles


def write_binary_stl(path, triangles):
    with open(path, "wb") as f:
        f.write(b"\x00" * 80)
        f.write(struct.pack("<I", len(triangles)))
        for v0, v1, v2 in triangles:
            n = normal(v0, v1, v2)
            f.write(struct.pack("<12fH", *n, *v0, *v1, *v2, 0))


def main():
    triangles = build_triangles()
    write_binary_stl(OUTPUT_PATH, triangles)
    print(f"wrote {len(triangles)} triangles to {OUTPUT_PATH}")


if __name__ == "__main__":
    main()