#!/usr/bin/env python3
"""Remove or sparsify ground points from an ASCII x/y/z PCD file."""

import argparse
import math
import os


def default_output_path(input_path):
    root, ext = os.path.splitext(input_path)
    return root + "_obstacles" + (ext or ".pcd")


def load_ascii_xyz_pcd(path):
    fields = []
    points = []
    with open(path, "r") as f:
        for raw_line in f:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            upper = line.upper()
            if upper.startswith("FIELDS "):
                fields = line.split()[1:]
                continue
            if upper.startswith("DATA "):
                data_mode = line.split()[1].lower()
                if data_mode != "ascii":
                    raise ValueError("only ASCII PCD files are supported: %s" % path)
                break

        if not fields:
            raise ValueError("PCD file has no FIELDS line: %s" % path)
        try:
            x_idx = fields.index("x")
            y_idx = fields.index("y")
            z_idx = fields.index("z")
        except ValueError as exc:
            raise ValueError("PCD file must contain x y z fields: %s" % path) from exc

        for raw_line in f:
            line = raw_line.strip()
            if not line:
                continue
            values = line.split()
            x = float(values[x_idx])
            y = float(values[y_idx])
            z = float(values[z_idx])
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append((x, y, z))
    return points


def write_ascii_xyz_pcd(path, points):
    tmp_path = path + ".tmp"
    with open(tmp_path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write("WIDTH %d\n" % len(points))
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write("POINTS %d\n" % len(points))
        f.write("DATA ascii\n")
        for x, y, z in points:
            f.write("%.6f %.6f %.6f\n" % (x, y, z))
    os.replace(tmp_path, path)


def point_xy_key(point, cell_size):
    inv_cell = 1.0 / cell_size
    return (int(math.floor(point[0] * inv_cell)), int(math.floor(point[1] * inv_cell)))


def ground_percentile(values):
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) <= 2:
        return ordered[0]
    index = int(math.floor(0.20 * float(len(ordered) - 1)))
    return ordered[index]


def sparse_terrain_points(ground_points, terrain_cell_size):
    cells = {}
    for point in ground_points:
        key = point_xy_key(point, terrain_cell_size)
        existing = cells.get(key)
        if existing is None or point[2] < existing[2]:
            cells[key] = point
    return list(cells.values())


def split_z_filtered_points(points, min_z, max_z):
    kept = []
    dropped_z = 0
    for x, y, z in points:
        if z < min_z or z > max_z:
            dropped_z += 1
            continue
        kept.append((x, y, z))
    return kept, dropped_z


def filter_points(
    points,
    mode,
    ground_z,
    ground_clearance,
    obstacle_height_above_ground,
    ground_xy_cell_size,
    keep_terrain,
    terrain_cell_size,
    min_z,
    max_z,
):
    points, dropped_z = split_z_filtered_points(points, min_z, max_z)
    if mode == "none":
        return points, 0, dropped_z, 0

    obstacle_points = []
    ground_points = []
    if mode == "flat":
        ground_limit = ground_z + ground_clearance
        for point in points:
            if point[2] <= ground_limit:
                ground_points.append(point)
            else:
                obstacle_points.append(point)
    else:
        columns = {}
        for point in points:
            columns.setdefault(point_xy_key(point, ground_xy_cell_size), []).append(point[2])
        ground_by_column = {
            key: ground_percentile(values)
            for key, values in columns.items()
        }
        for point in points:
            local_ground_z = ground_by_column.get(point_xy_key(point, ground_xy_cell_size))
            if local_ground_z is None:
                obstacle_points.append(point)
                continue
            if point[2] <= local_ground_z + obstacle_height_above_ground:
                ground_points.append(point)
            else:
                obstacle_points.append(point)

    terrain_points = (
        sparse_terrain_points(ground_points, terrain_cell_size)
        if keep_terrain
        else []
    )
    kept = obstacle_points + terrain_points
    dropped_ground = max(0, len(ground_points) - len(terrain_points))
    kept_terrain = len(terrain_points)
    return kept, dropped_ground, dropped_z, kept_terrain


def main():
    parser = argparse.ArgumentParser(
        description="Remove or sparsify ground points from an ASCII PCD map."
    )
    parser.add_argument(
        "--input",
        default=os.path.expanduser("~/pointcloud_maps/uav_points_map_latest.pcd"),
        help="Input ASCII PCD path.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Output ASCII PCD path. Defaults to INPUT_obstacles.pcd.",
    )
    parser.add_argument("--ground-z", type=float, default=0.0)
    parser.add_argument(
        "--mode",
        choices=("none", "flat", "local"),
        default="local",
        help="Ground filtering mode. 'local' estimates ground per XY cell.",
    )
    parser.add_argument("--ground-clearance", type=float, default=0.25)
    parser.add_argument("--obstacle-height-above-ground", type=float, default=0.25)
    parser.add_argument("--ground-xy-cell-size", type=float, default=0.35)
    parser.add_argument("--keep-terrain", action="store_true")
    parser.add_argument("--terrain-cell-size", type=float, default=0.80)
    parser.add_argument("--min-z", type=float, default=-999.0)
    parser.add_argument("--max-z", type=float, default=999.0)
    args = parser.parse_args()

    input_path = os.path.expanduser(args.input)
    output_path = os.path.expanduser(args.output) if args.output else default_output_path(input_path)

    points = load_ascii_xyz_pcd(input_path)
    kept, dropped_ground, dropped_z, kept_terrain = filter_points(
        points,
        args.mode,
        args.ground_z,
        max(args.ground_clearance, 0.0),
        max(args.obstacle_height_above_ground, 0.0),
        max(args.ground_xy_cell_size, 1e-4),
        args.keep_terrain,
        max(args.terrain_cell_size, max(args.ground_xy_cell_size, 1e-4)),
        args.min_z,
        args.max_z,
    )
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    write_ascii_xyz_pcd(output_path, kept)
    print(
        "input=%s points=%d output=%s kept=%d kept_terrain=%d "
        "dropped_ground=%d dropped_z=%d mode=%s ground_limit=%.3f "
        "obstacle_height_above_ground=%.3f"
        % (
            input_path,
            len(points),
            output_path,
            len(kept),
            kept_terrain,
            dropped_ground,
            dropped_z,
            args.mode,
            args.ground_z + max(args.ground_clearance, 0.0),
            max(args.obstacle_height_above_ground, 0.0),
        )
    )


if __name__ == "__main__":
    main()
