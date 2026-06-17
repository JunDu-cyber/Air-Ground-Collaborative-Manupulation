#!/usr/bin/env python3
"""Project a 3D UAV cloud into an elevation-map-friendly PointCloud2.

The UAV LiDAR map is a full 3D accumulation. This node projects it to one
top-surface height per XY cell so the UGV side can visualize/use a 2.5D
elevation map while keeping the visible obstacle footprint close to the saved
point cloud:

* flat ground cells publish a stable ground height;
* obstacle cells preserve the observed top envelope, including tree canopies;
* obstacle inflation is disabled by default;
* for offline visualization, unseen ground cells inside the observed map bounds
  can be filled as a flat surface so RViz shows a dense elevation map instead
  of a sparse point cloud.
"""

import math
from collections import defaultdict

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point, Pose
from grid_map_msgs.msg import GridMap
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA, Float32MultiArray, Header, MultiArrayDimension
from visualization_msgs.msg import Marker


def bool_param(name, default=False):
    value = rospy.get_param(name, default)
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def clean_frame(frame_id):
    return str(frame_id or "").strip().lstrip("/")


def percentile(sorted_values, fraction):
    if not sorted_values:
        return None
    fraction = min(max(float(fraction), 0.0), 1.0)
    index = int(round(fraction * float(len(sorted_values) - 1)))
    return sorted_values[index]


class UavElevationCloudProjector(object):
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/uav0/mapping/points_saved"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/uav0/mapping/points_elevation"
        )
        self.frame_id = clean_frame(rospy.get_param("~frame_id", ""))
        self.cell_size = max(float(rospy.get_param("~cell_size", 0.10)), 1e-4)
        self.ground_percentile = float(rospy.get_param("~ground_percentile", 0.10))
        self.top_percentile = float(rospy.get_param("~top_percentile", 0.95))
        self.obstacle_height_threshold = max(
            float(rospy.get_param("~obstacle_height_threshold", 0.45)), 0.0
        )
        self.obstacle_min_height_above_ground = max(
            float(
                rospy.get_param(
                    "~obstacle_min_height_above_ground",
                    self.obstacle_height_threshold,
                )
            ),
            0.0,
        )
        self.obstacle_max_height_above_ground = max(
            float(rospy.get_param("~obstacle_max_height_above_ground", 999.0)),
            self.obstacle_min_height_above_ground,
        )
        self.obstacle_output_mode = str(
            rospy.get_param("~obstacle_output_mode", "top")
        ).strip().lower()
        if self.obstacle_output_mode not in ("constant", "band_top", "top"):
            rospy.logwarn(
                "[UavElevationCloudProjector] unknown obstacle_output_mode '%s'; "
                "using 'constant'.",
                self.obstacle_output_mode,
            )
            self.obstacle_output_mode = "constant"
        self.obstacle_output_z = float(rospy.get_param("~obstacle_output_z", 1.20))
        self.min_points_per_cell = max(
            int(rospy.get_param("~min_points_per_cell", 1)), 1
        )
        self.min_obstacle_points = max(
            int(rospy.get_param("~min_obstacle_points", 1)), 1
        )
        self.min_obstacle_fraction = min(
            max(float(rospy.get_param("~min_obstacle_fraction", 0.0)), 0.0), 1.0
        )
        self.obstacle_hole_fill_enabled = bool_param("~obstacle_hole_fill_enabled", False)
        self.obstacle_hole_fill_radius = max(
            int(rospy.get_param("~obstacle_hole_fill_radius", 1)), 1
        )
        self.obstacle_hole_fill_min_neighbors = max(
            int(rospy.get_param("~obstacle_hole_fill_min_neighbors", 5)), 1
        )
        self.fill_ground_holes = bool_param("~fill_ground_holes", True)
        self.unknown_as_nan = bool_param("~unknown_as_nan", False)
        self.flatten_ground = bool_param("~flatten_ground", True)
        self.flat_ground_z = float(rospy.get_param("~flat_ground_z", 0.0))
        self.publish_ground = bool_param("~publish_ground", True)
        self.canopy_shrink_enabled = bool_param("~canopy_shrink_enabled", False)
        self.canopy_min_top_z = float(rospy.get_param("~canopy_min_top_z", 2.50))
        self.canopy_min_component_cells = max(
            int(rospy.get_param("~canopy_min_component_cells", 12)), 1
        )
        self.canopy_max_component_cells = max(
            int(rospy.get_param("~canopy_max_component_cells", 600)),
            self.canopy_min_component_cells,
        )
        self.canopy_min_component_points = max(
            int(rospy.get_param("~canopy_min_component_points", 25)), 1
        )
        self.canopy_footprint_radius = max(
            float(rospy.get_param("~canopy_footprint_radius", 0.45)), self.cell_size
        )
        self.min_z = float(rospy.get_param("~min_z", -999.0))
        self.max_z = float(rospy.get_param("~max_z", 999.0))
        self.min_output_z = float(rospy.get_param("~min_output_z", -999.0))
        self.max_output_z = float(rospy.get_param("~max_output_z", 8.0))
        self.publish_grid_map = bool_param("~publish_grid_map", True)
        self.grid_map_topic = rospy.get_param(
            "~grid_map_topic", "/elevation_mapping/elevation_map_postprocessed"
        )
        self.publish_marker = bool_param("~publish_marker", True)
        self.marker_topic = rospy.get_param(
            "~marker_topic", "/uav0/mapping/elevation_grid_marker"
        )
        self.marker_tile_thickness = max(
            float(rospy.get_param("~marker_tile_thickness", 0.04)), 0.01
        )
        self.marker_decimation = max(int(rospy.get_param("~marker_decimation", 1)), 1)
        self.grid_map_margin_cells = max(
            int(rospy.get_param("~grid_map_margin_cells", 1)), 0
        )
        self.publish_rate = max(float(rospy.get_param("~publish_rate", 1.0)), 0.0)
        self.latch = bool_param("~latch", True)

        self.last_cloud = None
        self.last_grid_map = None
        self.last_marker = None
        self.last_stamp = rospy.Time(0)
        self.inv_cell = 1.0 / self.cell_size

        self.pub = rospy.Publisher(
            self.output_topic, PointCloud2, queue_size=1, latch=self.latch
        )
        self.grid_pub = rospy.Publisher(
            self.grid_map_topic, GridMap, queue_size=1, latch=self.latch
        )
        self.marker_pub = rospy.Publisher(
            self.marker_topic, Marker, queue_size=1, latch=self.latch
        )
        self.sub = rospy.Subscriber(
            self.input_topic, PointCloud2, self.cloud_cb, queue_size=1
        )
        if self.publish_rate > 0.0:
            rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_cb)

        rospy.loginfo(
            "[UavElevationCloudProjector] input=%s output=%s cell=%.3f "
            "ground_p=%.2f top_p=%.2f obstacle_threshold=%.2f "
            "obstacle_band=[%.2f,%.2f] obstacle_output=%s:%.2f "
            "min_points=%d min_obstacle_points=%d min_obstacle_fraction=%.2f "
            "hole_fill=%s radius=%d min_neighbors=%d "
            "fill_ground_holes=%s unknown_as_nan=%s flatten_ground=%s "
            "flat_ground_z=%.2f publish_ground=%s z_filter=[%.2f,%.2f] "
            "canopy_shrink=%s canopy_top=%.2f canopy_cells=[%d,%d] "
            "canopy_points=%d canopy_radius=%.2f "
            "output_z=[%.2f,%.2f] publish_grid_map=%s grid_topic=%s "
            "publish_marker=%s marker_topic=%s marker_decimation=%d "
            "rate=%.2f latch=%s",
            self.input_topic,
            self.output_topic,
            self.cell_size,
            self.ground_percentile,
            self.top_percentile,
            self.obstacle_height_threshold,
            self.obstacle_min_height_above_ground,
            self.obstacle_max_height_above_ground,
            self.obstacle_output_mode,
            self.obstacle_output_z,
            self.min_points_per_cell,
            self.min_obstacle_points,
            self.min_obstacle_fraction,
            self.obstacle_hole_fill_enabled,
            self.obstacle_hole_fill_radius,
            self.obstacle_hole_fill_min_neighbors,
            self.fill_ground_holes,
            self.unknown_as_nan,
            self.flatten_ground,
            self.flat_ground_z,
            self.publish_ground,
            self.min_z,
            self.max_z,
            self.canopy_shrink_enabled,
            self.canopy_min_top_z,
            self.canopy_min_component_cells,
            self.canopy_max_component_cells,
            self.canopy_min_component_points,
            self.canopy_footprint_radius,
            self.min_output_z,
            self.max_output_z,
            self.publish_grid_map,
            self.grid_map_topic,
            self.publish_marker,
            self.marker_topic,
            self.marker_decimation,
            self.publish_rate,
            self.latch,
        )

    def cell_key(self, x, y):
        return (int(math.floor(x * self.inv_cell)), int(math.floor(y * self.inv_cell)))

    def clamp_output_z(self, z):
        if z < self.min_output_z:
            return self.min_output_z
        if z > self.max_output_z:
            return self.max_output_z
        return z

    def cell_center(self, key):
        return ((float(key[0]) + 0.5) * self.cell_size,
                (float(key[1]) + 0.5) * self.cell_size)

    def obstacle_neighbors(self, key, obstacle_by_cell):
        neighbors = []
        radius = self.obstacle_hole_fill_radius
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx == 0 and dy == 0:
                    continue
                point = obstacle_by_cell.get((key[0] + dx, key[1] + dy))
                if point is not None:
                    neighbors.append(point)
        return neighbors

    def neighbor_keys(self, key):
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                yield (key[0] + dx, key[1] + dy)

    def connected_components(self, keys):
        remaining = set(keys)
        components = []
        while remaining:
            start = remaining.pop()
            stack = [start]
            component = [start]
            while stack:
                key = stack.pop()
                for neighbor in self.neighbor_keys(key):
                    if neighbor not in remaining:
                        continue
                    remaining.remove(neighbor)
                    stack.append(neighbor)
                    component.append(neighbor)
            components.append(component)
        return components

    def obstacle_z_from_info(self, info):
        ground_ref_z = info["ground_ref_z"]
        if self.obstacle_output_mode == "constant":
            return self.clamp_output_z(ground_ref_z + self.obstacle_output_z)
        if self.obstacle_output_mode == "band_top":
            return self.clamp_output_z(info["band_top_z"])
        return self.clamp_output_z(info["top_z"])

    def add_canopy_footprints(self, obstacle_by_cell, cell_infos):
        if not self.canopy_shrink_enabled or not cell_infos:
            return set(), 0, 0

        canopy_candidates = []
        for key, info in cell_infos.items():
            if info["top_z"] < info["ground_ref_z"] + self.canopy_min_top_z:
                continue
            if info["high_count"] <= 0:
                continue
            canopy_candidates.append(key)

        compact_cells = 0
        canopy_components = 0
        suppressed_canopy_cells = set()
        radius_cells = int(math.ceil(self.canopy_footprint_radius * self.inv_cell))
        radius_sq = self.canopy_footprint_radius * self.canopy_footprint_radius

        for component in self.connected_components(canopy_candidates):
            component_cells = len(component)
            component_points = sum(cell_infos[key]["high_count"] for key in component)
            if component_cells < self.canopy_min_component_cells:
                continue
            if component_cells > self.canopy_max_component_cells:
                continue
            if component_points < self.canopy_min_component_points:
                continue

            weight_sum = float(component_points)
            center_x = sum(
                cell_infos[key]["center"][0] * cell_infos[key]["high_count"]
                for key in component
            ) / weight_sum
            center_y = sum(
                cell_infos[key]["center"][1] * cell_infos[key]["high_count"]
                for key in component
            ) / weight_sum
            center_key = self.cell_key(center_x, center_y)
            component_top = max(cell_infos[key]["top_z"] for key in component)
            suppressed_canopy_cells.update(component)
            canopy_components += 1

            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    key = (center_key[0] + dx, center_key[1] + dy)
                    cx, cy = self.cell_center(key)
                    if (cx - center_x) ** 2 + (cy - center_y) ** 2 > radius_sq:
                        continue
                    base_info = cell_infos.get(key)
                    ground_ref_z = (
                        base_info["ground_ref_z"] if base_info is not None
                        else self.flat_ground_z
                    )
                    output_z = self.clamp_output_z(
                        ground_ref_z + self.obstacle_output_z
                        if self.obstacle_output_mode == "constant"
                        else component_top
                    )
                    if key not in obstacle_by_cell:
                        compact_cells += 1
                    obstacle_by_cell[key] = (cx, cy, output_z)

        return suppressed_canopy_cells, canopy_components, compact_cells

    def fill_obstacle_holes(self, output_by_cell, obstacle_by_cell, min_key_x,
                            max_key_x, min_key_y, max_key_y):
        if not self.obstacle_hole_fill_enabled or not obstacle_by_cell:
            return 0
        filled = {}
        for ix in range(min_key_x, max_key_x + 1):
            for iy in range(min_key_y, max_key_y + 1):
                key = (ix, iy)
                if key in obstacle_by_cell:
                    continue
                neighbors = self.obstacle_neighbors(key, obstacle_by_cell)
                if len(neighbors) < self.obstacle_hole_fill_min_neighbors:
                    continue
                cx, cy = self.cell_center(key)
                z = max(point[2] for point in neighbors)
                filled[key] = (cx, cy, z)

        output_by_cell.update(filled)
        obstacle_by_cell.update(filled)
        return len(filled)

    def build_multi_array(self, values, rows, cols):
        array = Float32MultiArray()
        array.layout.dim = [
            MultiArrayDimension(label="column_index", size=cols, stride=cols * rows),
            MultiArrayDimension(label="row_index", size=rows, stride=rows),
        ]
        array.layout.data_offset = 0
        # Eigen/GridMap default is column-major.
        data = []
        for col in range(cols):
            for row in range(rows):
                data.append(float(values[row][col]))
        array.data = data
        return array

    def build_grid_map(self, output_by_cell, header, min_key_x, max_key_x,
                       min_key_y, max_key_y):
        if not output_by_cell or min_key_x is None:
            return None

        min_key_x -= self.grid_map_margin_cells
        max_key_x += self.grid_map_margin_cells
        min_key_y -= self.grid_map_margin_cells
        max_key_y += self.grid_map_margin_cells
        cols = max_key_x - min_key_x + 1
        rows = max_key_y - min_key_y + 1
        ground_z = self.clamp_output_z(self.flat_ground_z)
        empty_value = float("nan") if self.unknown_as_nan else ground_z
        elevation = [[empty_value for _col in range(cols)] for _row in range(rows)]
        for (ix, iy), (_x, _y, z) in output_by_cell.items():
            col = ix - min_key_x
            row = iy - min_key_y
            if 0 <= row < rows and 0 <= col < cols:
                elevation[row][col] = self.clamp_output_z(z)

        slope = [[0.0 for _col in range(cols)] for _row in range(rows)]
        for row in range(rows):
            for col in range(cols):
                z = elevation[row][col]
                if not math.isfinite(z):
                    slope[row][col] = float("nan")
                    continue
                max_diff = 0.0
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        if dr == 0 and dc == 0:
                            continue
                        rr = row + dr
                        cc = col + dc
                        if 0 <= rr < rows and 0 <= cc < cols:
                            if not math.isfinite(elevation[rr][cc]):
                                continue
                            max_diff = max(max_diff, abs(z - elevation[rr][cc]))
                slope[row][col] = math.atan2(max_diff, self.cell_size)

        grid = GridMap()
        grid.info.header = header
        grid.info.resolution = self.cell_size
        grid.info.length_x = float(cols) * self.cell_size
        grid.info.length_y = float(rows) * self.cell_size
        pose = Pose()
        min_x = float(min_key_x) * self.cell_size
        max_x = float(max_key_x + 1) * self.cell_size
        min_y = float(min_key_y) * self.cell_size
        max_y = float(max_key_y + 1) * self.cell_size
        pose.position.x = 0.5 * (min_x + max_x)
        pose.position.y = 0.5 * (min_y + max_y)
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        grid.info.pose = pose
        grid.layers = ["elevation", "elevation_inpainted", "slope"]
        grid.basic_layers = ["elevation"]
        elevation_array = self.build_multi_array(elevation, rows, cols)
        grid.data = [
            elevation_array,
            self.build_multi_array(elevation, rows, cols),
            self.build_multi_array(slope, rows, cols),
        ]
        grid.outer_start_index = 0
        grid.inner_start_index = 0
        return grid

    def color_for_height(self, z):
        z = self.clamp_output_z(z)
        low = self.clamp_output_z(self.flat_ground_z)
        high = max(self.max_output_z, low + 1.0)
        t = min(max((z - low) / (high - low), 0.0), 1.0)
        # Blue-green ground, yellow mid heights, red high canopy/obstacles.
        if t < 0.5:
            local = t * 2.0
            r = 0.10 + 0.75 * local
            g = 0.55 + 0.35 * local
            b = 0.95 * (1.0 - local) + 0.12 * local
        else:
            local = (t - 0.5) * 2.0
            r = 0.85 + 0.15 * local
            g = 0.90 * (1.0 - local) + 0.18 * local
            b = 0.12 * (1.0 - local)
        return ColorRGBA(r=r, g=g, b=b, a=0.96)

    def build_marker(self, output_by_cell, header):
        if not output_by_cell:
            return None
        marker = Marker()
        marker.header = header
        marker.ns = "uav_elevation_grid"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.cell_size
        marker.scale.y = self.cell_size
        marker.scale.z = self.marker_tile_thickness
        marker.lifetime = rospy.Duration(0.0)

        for index, (_key, (x, y, z)) in enumerate(sorted(output_by_cell.items())):
            if index % self.marker_decimation != 0:
                continue
            marker.points.append(Point(x=x, y=y, z=z))
            marker.colors.append(self.color_for_height(z))
        return marker

    def project(self, msg):
        cells = defaultdict(list)
        input_points = 0
        dropped_z = 0
        for x, y, z in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            input_points += 1
            if z < self.min_z or z > self.max_z:
                dropped_z += 1
                continue
            cells[self.cell_key(x, y)].append((x, y, z))

        output_by_cell = {}
        obstacle_by_cell = {}
        ground_by_cell = {}
        obstacle_cells = 0
        ground_cells = 0
        sparse_cells = 0
        min_key_x = None
        max_key_x = None
        min_key_y = None
        max_key_y = None
        cell_infos = {}
        for key, points in cells.items():
            if len(points) < self.min_points_per_cell:
                sparse_cells += 1
                continue

            zs = sorted(p[2] for p in points)
            measured_ground_z = percentile(zs, self.ground_percentile)
            top_z = percentile(zs, self.top_percentile)
            if measured_ground_z is None or top_z is None:
                continue

            ground_ref_z = self.flat_ground_z if self.flatten_ground else measured_ground_z
            low_limit = ground_ref_z + self.obstacle_min_height_above_ground
            high_limit = ground_ref_z + self.obstacle_max_height_above_ground
            high_candidate_limit = ground_ref_z + self.obstacle_min_height_above_ground
            band_zs = [z for z in zs if low_limit <= z <= high_limit]
            band_top_z = max(band_zs) if band_zs else top_z
            high_candidate_count = sum(1 for z in zs if z >= high_candidate_limit)
            band_count = len(band_zs)
            band_fraction = float(band_count) / float(len(zs))
            is_obstacle = (
                band_count >= self.min_obstacle_points
                and band_fraction >= self.min_obstacle_fraction
                and band_top_z >= low_limit
            )

            cx, cy = self.cell_center(key)
            cell_infos[key] = {
                "center": (cx, cy),
                "ground_ref_z": ground_ref_z,
                "measured_ground_z": measured_ground_z,
                "top_z": top_z,
                "band_top_z": band_top_z,
                "band_count": band_count,
                "high_count": high_candidate_count,
                "is_obstacle": is_obstacle,
            }
            if min_key_x is None:
                min_key_x = max_key_x = key[0]
                min_key_y = max_key_y = key[1]
            else:
                min_key_x = min(min_key_x, key[0])
                max_key_x = max(max_key_x, key[0])
                min_key_y = min(min_key_y, key[1])
                max_key_y = max(max_key_y, key[1])

        suppressed_canopy_cells, canopy_components, canopy_cells = (
            self.add_canopy_footprints(obstacle_by_cell, cell_infos)
        )

        for key, info in cell_infos.items():
            if key in suppressed_canopy_cells:
                continue
            if info["is_obstacle"]:
                cx, cy = info["center"]
                obstacle_by_cell[key] = (cx, cy, self.obstacle_z_from_info(info))
                obstacle_cells += 1

        for key, info in cell_infos.items():
            if key in obstacle_by_cell:
                continue
            if self.publish_ground:
                cx, cy = info["center"]
                ground_z = (
                    self.flat_ground_z
                    if self.flatten_ground
                    else info["measured_ground_z"]
                )
                ground_by_cell[key] = (cx, cy, self.clamp_output_z(ground_z))
                ground_cells += 1

        output_by_cell.update(ground_by_cell)
        output_by_cell.update(obstacle_by_cell)

        filled_obstacle_cells = 0
        if min_key_x is not None:
            filled_obstacle_cells = self.fill_obstacle_holes(
                output_by_cell,
                obstacle_by_cell,
                min_key_x,
                max_key_x,
                min_key_y,
                max_key_y,
            )

        filled_ground_cells = 0
        if (
            self.fill_ground_holes
            and self.publish_ground
            and min_key_x is not None
            and max_key_x is not None
        ):
            ground_z = self.clamp_output_z(self.flat_ground_z)
            for ix in range(min_key_x, max_key_x + 1):
                for iy in range(min_key_y, max_key_y + 1):
                    key = (ix, iy)
                    if key in output_by_cell:
                        continue
                    cx, cy = self.cell_center(key)
                    output_by_cell[key] = (cx, cy, ground_z)
                    filled_ground_cells += 1

        output_points = list(output_by_cell.values())

        frame_id = self.frame_id or clean_frame(msg.header.frame_id) or "map"
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        header = Header(stamp=stamp, frame_id=frame_id)
        cloud = pc2.create_cloud_xyz32(header, output_points)
        grid_map = None
        if self.publish_grid_map:
            grid_map = self.build_grid_map(
                output_by_cell,
                header,
                min_key_x,
                max_key_x,
                min_key_y,
                max_key_y,
            )
        marker = self.build_marker(output_by_cell, header) if self.publish_marker else None
        rospy.loginfo_throttle(
            5.0,
            "[UavElevationCloudProjector] input_points=%d cells=%d output=%d "
            "obstacle_cells=%d ground_cells=%d filled_obstacle_cells=%d "
            "filled_ground_cells=%d canopy_components=%d canopy_cells=%d "
            "suppressed_canopy_cells=%d sparse_cells=%d dropped_z=%d",
            input_points,
            len(cells),
            len(output_points),
            obstacle_cells,
            ground_cells,
            filled_obstacle_cells,
            filled_ground_cells,
            canopy_components,
            canopy_cells,
            len(suppressed_canopy_cells),
            sparse_cells,
            dropped_z,
        )
        return cloud, grid_map, marker

    def cloud_cb(self, msg):
        self.last_cloud, self.last_grid_map, self.last_marker = self.project(msg)
        self.last_stamp = rospy.Time.now()
        self.pub.publish(self.last_cloud)
        if self.last_grid_map is not None:
            self.grid_pub.publish(self.last_grid_map)
        if self.last_marker is not None:
            self.marker_pub.publish(self.last_marker)

    def timer_cb(self, _event):
        if self.last_cloud is None:
            return
        self.last_cloud.header.stamp = rospy.Time.now()
        self.pub.publish(self.last_cloud)
        if self.last_grid_map is not None:
            self.last_grid_map.info.header.stamp = rospy.Time.now()
            self.grid_pub.publish(self.last_grid_map)
        if self.last_marker is not None:
            self.last_marker.header.stamp = rospy.Time.now()
            self.marker_pub.publish(self.last_marker)


def main():
    rospy.init_node("uav_elevation_cloud_projector")
    UavElevationCloudProjector()
    rospy.spin()


if __name__ == "__main__":
    main()
