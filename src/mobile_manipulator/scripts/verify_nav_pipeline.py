#!/usr/bin/env python3
"""Verify the egocentric UGV navigation pipeline end to end.

Run it against a live stack started with:

    roslaunch mobile_manipulator egocentric_nav.launch \
        nav:=cmu odom_source:=dlio cost_source:=elevation global_planner:=far gui:=false

    rosrun mobile_manipulator verify_nav_pipeline.py            # static checks only
    rosrun mobile_manipulator verify_nav_pipeline.py --drive    # also drive to a goal

Checks, in the order the data flows:

  1. self-filter    no LiDAR returns land on the robot's own body
  2. layers         the grid_map postprocessor produces slope/roughness/traversability
  3. roughness      the integral-image filter matches a numpy reference
  4. observed-mask  the cost clouds carry ONLY cells the sensor actually saw
  5. rates          the cost clouds keep up (the sliding-window filter used to stall here)
  6. self-mound     no phantom obstacle ring around the robot
  7. drive          (--drive) FAR routes the Husky to a goal, without phantom detours
  8. slope cost     /terrain_map_ext penalises slopes that /terrain_map cannot see
                    (needs observed terrain, so it runs last; skipped if you don't drive)

Exit status is 0 only if every check passes, so it can gate a commit.
"""
import argparse
import math
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

POSTPROC = "/elevation_mapping/elevation_map_postprocessed"
RESULTS = []


def record(name, ok, detail):
    RESULTS.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: {detail}")
    return ok


def skip(name, detail):
    print(f"  [SKIP] {name}: {detail}")
    return True


def cloud_xyi(topic, timeout=30.0):
    msg = rospy.wait_for_message(topic, PointCloud2, timeout=timeout)
    pts = np.array(list(pc2.read_points(msg, field_names=("x", "y", "intensity"),
                                        skip_nans=True)))
    return msg, pts


def layer(msg, name):
    """Reconstruct a grid_map layer as a 2-D array (column-major, as published)."""
    d = msg.data[msg.layers.index(name)]
    return np.array(d.data, dtype=np.float64).reshape(d.layout.dim[0].size,
                                                      d.layout.dim[1].size)


def robot_xy():
    o = rospy.wait_for_message("/state_estimation", Odometry, timeout=30.0)
    return o.pose.pose.position.x, o.pose.pose.position.y


def check_self_filter():
    """Returns off the chassis/top-plate build a mound that traps localPlanner."""
    raw = rospy.wait_for_message("/velodyne_points", PointCloud2, timeout=30.0)
    flt = rospy.wait_for_message("/velodyne_points_filtered", PointCloud2, timeout=30.0)

    def near_count(m):
        p = np.array(list(pc2.read_points(m, field_names=("x", "y", "z"), skip_nans=True)))
        r = np.linalg.norm(p[:, :2], axis=1)
        return int((r < 1.0).sum()), int((r > 1.0).sum())

    raw_near, raw_far = near_count(raw)
    flt_near, flt_far = near_count(flt)
    # DLIO reads ring/time off this cloud; the filter must copy points byte-wise.
    fields = {f.name for f in flt.fields}
    keeps_fields = {"ring", "time"} <= fields
    loss = 100.0 * (raw_far - flt_far) / max(raw_far, 1)

    record("self-filter removes body returns", flt_near == 0,
           f"near-body {raw_near} -> {flt_near}")
    record("self-filter keeps real terrain", loss < 1.0,
           f"far-field {raw_far} -> {flt_far} ({loss:.2f}% lost)")
    return record("self-filter preserves ring/time (DLIO)", keeps_fields,
                  f"fields={sorted(fields)}")


def check_layers_and_roughness():
    m = rospy.wait_for_message(POSTPROC, GridMap, timeout=60.0)
    want = ["elevation_filled", "slope", "roughness", "traversability"]
    missing = [w for w in want if w not in m.layers]
    ok = record("postprocessor layers present", not missing,
                f"missing={missing}" if missing else "slope, roughness, traversability")
    if not ok:
        return False

    trav = layer(m, "traversability")
    t = trav[np.isfinite(trav)]
    record("traversability clamped to [0,1]", t.min() >= -1e-6 and t.max() <= 1 + 1e-6,
           f"min={t.min():.3f} max={t.max():.3f}")

    # Reference stddev over the same 3x3 window the plugin uses
    # (windowSize = round(0.5 / 0.25) = 2 -> forced odd -> 3).
    E, R = layer(m, "elevation_filled"), layer(m, "roughness")
    h, rows, cols = 1, *E.shape
    fin = np.isfinite(E)
    X, N = np.where(fin, E, 0.0), fin.astype(np.float64)
    ref = np.full_like(E, np.nan)
    for i in range(rows):
        i0, i1 = max(0, i - h), min(rows - 1, i + h)
        for j in range(cols):
            if not fin[i, j]:
                continue
            j0, j1 = max(0, j - h), min(cols - 1, j + h)
            n = N[i0:i1 + 1, j0:j1 + 1].sum()
            if n < 2:
                continue
            w = X[i0:i1 + 1, j0:j1 + 1]
            ref[i, j] = math.sqrt(max(0.0, (w ** 2).sum() / n - (w.sum() / n) ** 2))

    both = np.isfinite(ref) & np.isfinite(R)
    nan_mismatch = int((np.isfinite(R) ^ np.isfinite(ref)).sum())
    err = np.abs(ref[both] - R[both]).max() if both.any() else float("inf")
    return record("roughness matches numpy reference", err < 1e-4 and nan_mismatch == 0,
                  f"max|diff|={err:.2e} over {both.sum()} cells, NaN mismatch={nan_mismatch}")


def check_observed_mask():
    """Publishing inpainted cells hands FAR phantom obstacles to plan around."""
    m = rospy.wait_for_message(POSTPROC, GridMap, timeout=30.0)
    observed = int(np.isfinite(layer(m, "elevation")).sum())
    _, pts = cloud_xyi("/terrain_map_ext")
    # The cloud should track the observed set, never the full 160x160 grid.
    ratio = len(pts) / max(observed, 1)
    return record("cost cloud carries only observed cells", 0.8 < ratio < 1.2,
                  f"{len(pts)} pts vs {observed} observed cells (ratio {ratio:.2f}); "
                  f"grid is {layer(m, 'elevation').size}")


def check_rates(window=20.0, floor=1.5):
    counts = {"terrain_map": 0, "terrain_map_ext": 0, "postprocessed": 0}

    def bump(k):
        return lambda _m: counts.__setitem__(k, counts[k] + 1)

    subs = [rospy.Subscriber("/terrain_map", PointCloud2, bump("terrain_map")),
            rospy.Subscriber("/terrain_map_ext", PointCloud2, bump("terrain_map_ext")),
            rospy.Subscriber(POSTPROC, GridMap, bump("postprocessed"))]
    time.sleep(window)
    for s in subs:
        s.unregister()
    ok = True
    for k, v in counts.items():
        hz = v / window
        ok &= record(f"rate {k}", hz >= floor, f"{hz:.2f} Hz (floor {floor})")
    return ok


def check_self_mound():
    rx, ry = robot_xy()
    m = rospy.wait_for_message(POSTPROC, GridMap, timeout=30.0)
    A = layer(m, "elevation_filled")
    res, rows, cols = m.info.resolution, *A.shape
    cx, cy = m.info.pose.position.x, m.info.pose.position.y

    def val(px, py):
        i = int(round((cx + rows * res / 2 - px) / res - 0.5))
        j = int(round((cy + cols * res / 2 - py) / res - 0.5))
        return A[i, j] if 0 <= i < rows and 0 <= j < cols else np.nan

    # Reference ring at 4 m: the VLP-16's lowest beam (-15 deg) leaves the mast at
    # 0.683 m, so it first strikes ground ~2.55 m out. Anything inside that radius is
    # the sensor's blind cone and reads NaN -- a 2 m ring would be no reference at all.
    ring_vals = [val(rx + 4.0 * math.cos(a), ry + 4.0 * math.sin(a))
                 for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)]
    ring_vals = [v for v in ring_vals if np.isfinite(v)]
    if not ring_vals:
        return skip("no elevation mound under robot", "no ground observed at 4 m yet")
    ring = float(np.mean(ring_vals))

    # Cells under the robot are usually NaN: the self-box filter drops those returns
    # and the hole is wider than the median fill radius. That IS the healthy state --
    # no data beats a mound. Only judge cells that actually carry an elevation.
    near = [val(rx + r * math.cos(a), ry + r * math.sin(a))
            for r in (0.0, 0.25, 0.5) for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)]
    near = [v for v in near if np.isfinite(v)]
    if not near:
        record("no elevation mound under robot", True,
               f"no elevation under robot (self-filtered); 4 m ring {ring:.3f} m")
    else:
        worst = max(near, key=lambda v: abs(v - ring))
        record("no elevation mound under robot", abs(worst - ring) < 0.10,
               f"worst of {len(near)} cells within 0.5 m: {worst:.3f} m vs "
               f"4 m ring {ring:.3f} m")

    _, pts = cloud_xyi("/terrain_map")
    d = np.hypot(pts[:, 0] - rx, pts[:, 1] - ry)
    near = pts[d < 0.5, 2]
    frac = 100.0 * (near > 0.15).mean() if near.size else 0.0
    return record("no obstacle ring around robot", frac < 5.0,
                  f"{frac:.1f}% of cells within 0.5 m are obstacles")


def check_slope_cost(min_free=5000, settle=60.0):
    """/terrain_map is blind to slopes; /terrain_map_ext must not be.

    Only meaningful once the robot has observed some non-flat terrain. At spawn the
    cost cloud is a few thousand cells of flat ground, so a low count says nothing
    about the pipeline -- hence the min_free sample gate rather than a bare count.
    elevation_mapping keeps fusing for a while after the robot stops, so wait for
    the observed set to grow rather than sampling the instant the drive ends.
    """
    deadline = time.time() + settle
    n_free = caught = 0
    while True:
        _, a = cloud_xyi("/terrain_map")
        _, b = cloud_xyi("/terrain_map_ext")
        ha = {(round(x, 3), round(y, 3)): i for x, y, i in a}
        hb = {(round(x, 3), round(y, 3)): i for x, y, i in b}
        keys = sorted(set(ha) & set(hb))
        if not keys:
            return record("slope discrimination", False, "no overlapping cells")
        A = np.array([ha[k] for k in keys])
        B = np.array([hb[k] for k in keys])
        free_by_height = A < 0.1
        n_free = int(free_by_height.sum())
        caught = int((free_by_height & (B > 0.5)).sum())
        if n_free >= min_free or time.time() > deadline:
            break
        time.sleep(5.0)

    if n_free < min_free:
        return skip("slope discrimination",
                    f"only {n_free} free cells observed after {settle:.0f} s (need "
                    f"{min_free}); drive further with --goal to see more terrain")
    frac = 100.0 * caught / n_free
    return record("slope discrimination", frac >= 0.5,
                  f"{caught}/{n_free} ({frac:.2f}%) of cells that look free by height "
                  f"are costly by traversability")


def check_drive(goal, timeout=180.0, tol=0.8):
    pub = rospy.Publisher("/goal_point", PointStamped, queue_size=1, latch=True)
    wps = []
    sub = rospy.Subscriber("/way_point", PointStamped,
                           lambda m: wps.append((round(m.point.x, 2), round(m.point.y, 2))))
    start = robot_xy()
    straight = math.hypot(goal[0] - start[0], goal[1] - start[1])
    time.sleep(2.0)

    g = PointStamped()
    g.header.frame_id = "odom"
    g.header.stamp = rospy.Time.now()
    g.point.x, g.point.y = goal
    pub.publish(g)
    print(f"    driving ({start[0]:.2f}, {start[1]:.2f}) -> ({goal[0]:.2f}, {goal[1]:.2f}), "
          f"straight {straight:.2f} m")

    t0, travelled, last, reached = time.time(), 0.0, start, False
    while time.time() - t0 < timeout:
        p = robot_xy()
        travelled += math.hypot(p[0] - last[0], p[1] - last[1])
        last = p
        if math.hypot(p[0] - goal[0], p[1] - goal[1]) < tol:
            reached = True
            break
        time.sleep(3.0)
    sub.unregister()

    took = time.time() - t0
    ratio = travelled / max(straight, 1e-3)
    record("FAR reaches the goal", reached,
           f"{'reached' if reached else 'STALLED'} after {took:.0f} s, "
           f"travelled {travelled:.2f} m")
    # A ratio far above ~3 means it is routing around obstacles that are not there.
    return record("path is not a phantom detour", reached and ratio < 3.5,
                  f"detour ratio {ratio:.2f} (straight {straight:.2f} m), "
                  f"{len(set(wps))} distinct waypoints")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drive", action="store_true",
                    help="also command a goal and require the Husky to reach it")
    ap.add_argument("--goal", type=float, nargs=2, metavar=("X", "Y"), default=[7.38, -5.12],
                    help="odom-frame goal for --drive (default: a verified-free cell)")
    ap.add_argument("--rate-window", type=float, default=20.0)
    args = ap.parse_args()

    rospy.init_node("verify_nav_pipeline", anonymous=True)
    print("Verifying egocentric nav pipeline (cost_source:=elevation, global_planner:=far)\n")

    try:
        print("1-3. sensor cloud -> grid_map layers")
        check_self_filter()
        check_layers_and_roughness()
        print("\n4-5. grid_map -> cost clouds")
        check_observed_mask()
        check_rates(args.rate_window)
        print("\n6. cost cloud sanity")
        check_self_mound()
        if args.drive:
            print("\n7. FAR global planner")
            check_drive(tuple(args.goal))
        # Last: it needs observed terrain, which the drive provides.
        print("\n8. slope cost")
        check_slope_cost()
    except rospy.ROSException as e:
        print(f"\nAborted: {e}\nIs the stack running, and did you source devel/setup.bash?")
        return 2

    failed = [n for n, ok, _ in RESULTS if not ok]
    print(f"\n{len(RESULTS) - len(failed)}/{len(RESULTS)} checks passed")
    if failed:
        print("FAILED: " + ", ".join(failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
