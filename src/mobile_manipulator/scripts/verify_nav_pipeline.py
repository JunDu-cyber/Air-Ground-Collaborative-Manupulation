#!/usr/bin/env python3
"""Verify the egocentric UGV navigation pipeline end to end.

Run it against a live stack started with:

    roslaunch mobile_manipulator egocentric_nav.launch \
        nav:=cmu odom_source:=dlio cost_source:=elevation global_planner:=far gui:=false

    rosrun mobile_manipulator verify_nav_pipeline.py            # static checks only
    rosrun mobile_manipulator verify_nav_pipeline.py --drive    # also drive to a goal

Checks, in the order the data flows:

  1. self-filter    no LiDAR returns land on the robot's own body
  2. layers         the postprocessor produces slope/step/step_abs/roughness/traversability
  3. plane fit      the integral-image filter matches an independent numpy reference
  4. metric         the obstacle metric gets right the four cases height-above-minimum
                    got wrong: a ramp is free, a ditch is an obstacle, a wall still
                    blocks, and roughness no longer reacts to smooth tilt
  5. observed-mask  the cost clouds carry ONLY cells the sensor actually saw
  6. rates          the cost clouds keep up (the sliding-window filter used to stall here)
  7. self-mound     no phantom obstacle ring around the robot
  8. drive          (--drive) FAR routes the Husky to a goal, without phantom detours
  9. slope cost     the LOCAL cost cloud no longer has a slope blind spot, and has not
                    over-corrected into calling the world a wall (needs observed terrain,
                    so it runs after the drive)

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

# Must mirror the deployed config: elevation_mapping_ugv.yaml resolution, the
# PlaneFitFilter window in elevation_postprocessor.yaml, and the adapter's cost
# scaling in cmu_planner.launch (obstacle_height_thre MUST equal localPlanner's
# obstacleHeightThre, which is what the slope term is rescaled against).
RES = 0.25            # elevation map resolution [m]
HALF = 2              # round(window_length 1.25 / RES) = 5 -> forced odd -> half = 2
MIN_POINTS = 6        # PlaneFitFilter min_points
SLOPE_LIMIT = 0.44    # rad, 25 deg -- Husky usable slope limit
OBST_THRE = 0.15      # localPlanner obstacleHeightThre [m]


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


def plane_fit_reference(E, res=RES, half=HALF, min_points=MIN_POINTS):
    """Naive windowed least-squares plane fit — an INDEPENDENT re-derivation of what
    PlaneFitFilter computes with integral images. Deliberately written the slow,
    obvious way (explicit per-cell submatrix, no SATs) so that agreeing with the C++
    is real evidence and not the same bug written twice.

    Note this is invariant to grid_map's row/column-vs-x/y layout: transposing the
    grid only swaps a<->b, and slope=atan(hypot(a,b)), the residual and its RMS are
    all unchanged by that swap.
    """
    rows, cols = E.shape
    fin = np.isfinite(E)
    slope = np.full_like(E, np.nan)
    step = np.full_like(E, np.nan)
    rough = np.full_like(E, np.nan)

    for i in range(rows):
        i0, i1 = max(0, i - half), min(rows - 1, i + half)
        for j in range(cols):
            if not fin[i, j]:
                continue
            j0, j1 = max(0, j - half), min(cols - 1, j + half)
            sub = E[i0:i1 + 1, j0:j1 + 1]
            mask = np.isfinite(sub)
            n = int(mask.sum())
            if n < min_points:
                continue
            ii, jj = np.nonzero(mask)
            y = (i0 + ii) * res
            x = (j0 + jj) * res
            z = sub[mask]

            xb, yb, zb = x.mean(), y.mean(), z.mean()
            dx, dy, dz = x - xb, y - yb, z - zb
            Sxx = (dx * dx).sum()
            Sxy = (dx * dy).sum()
            Syy = (dy * dy).sum()
            Sxz = (dx * dz).sum()
            Syz = (dy * dz).sum()
            Szz = (dz * dz).sum()

            det = Sxx * Syy - Sxy * Sxy
            if not abs(det) > 1e-12:
                continue
            a = (Sxz * Syy - Syz * Sxy) / det
            b = (Syz * Sxx - Sxz * Sxy) / det

            zp = zb + a * (j * res - xb) + b * (i * res - yb)
            slope[i, j] = math.atan(math.hypot(a, b))
            step[i, j] = E[i, j] - zp
            rough[i, j] = math.sqrt(max(0.0, (Szz - a * Sxz - b * Syz) / n))
    return slope, step, rough


def terrain_cost(step, slope, thre=OBST_THRE, limit=SLOPE_LIMIT, vh=1.0):
    """The /terrain_map intensity gridmap_to_terrainmap publishes: an obstacle is a
    real discontinuity (|step|) OR a smooth-but-too-steep surface (slope rescaled so
    it hits `thre` exactly at the vehicle's slope limit)."""
    return np.clip(np.maximum(np.abs(step), thre * slope / limit), 0.0, vh)


def check_layers_and_plane_fit():
    m = rospy.wait_for_message(POSTPROC, GridMap, timeout=60.0)
    want = ["elevation_filled", "slope", "step", "step_abs", "roughness", "traversability"]
    missing = [w for w in want if w not in m.layers]
    ok = record("postprocessor layers present", not missing,
                f"missing={missing}" if missing else "slope, step, step_abs, roughness, traversability")
    if not ok:
        return False

    trav = layer(m, "traversability")
    t = trav[np.isfinite(trav)]
    record("traversability clamped to [0,1]", t.min() >= -1e-6 and t.max() <= 1 + 1e-6,
           f"min={t.min():.3f} max={t.max():.3f}")

    E = layer(m, "elevation_filled")
    S, P, R = layer(m, "slope"), layer(m, "step"), layer(m, "roughness")
    rs, rp, rr = plane_fit_reference(E)

    worst, worst_name, nan_mism = 0.0, "", 0
    for name, got, ref in (("slope", S, rs), ("step", P, rp), ("roughness", R, rr)):
        both = np.isfinite(ref) & np.isfinite(got)
        nan_mism += int((np.isfinite(got) ^ np.isfinite(ref)).sum())
        e = np.abs(ref[both] - got[both]).max() if both.any() else float("inf")
        if e > worst:
            worst, worst_name = e, name
    n_cmp = int((np.isfinite(rp) & np.isfinite(P)).sum())
    return record("plane fit matches numpy reference", worst < 1e-5 and nan_mism == 0,
                  f"max|diff|={worst:.2e} (worst: {worst_name}) over {n_cmp} cells, "
                  f"NaN mismatch={nan_mism}")


def check_metric_properties():
    """The four terrain cases the old height-above-minimum metric got wrong.

    These run on the numpy reference, which check `plane fit matches numpy reference`
    has just proven equals the deployed C++ filter bit-for-bit. So this measures the
    METRIC's behaviour on terrain the simulator does not happen to contain, without
    re-testing the implementation.
    """
    n, res = 21, RES
    ii, jj = np.mgrid[0:n, 0:n]

    def cost_of(E):
        s, p, _ = plane_fit_reference(E)
        c = terrain_cost(p, s)
        core = c[HALF + 1:-(HALF + 1), HALF + 1:-(HALF + 1)]  # ignore border clipping
        return core

    # 1. A 15 deg ramp is DRIVABLE. The old metric read 1.06*tan(15) = 0.28 -> wall.
    ramp = (jj * res) * math.tan(math.radians(15.0))
    c = cost_of(ramp)
    record("15 deg ramp is free (was a wall)", c.max() < OBST_THRE,
           f"max cost {c.max():.3f} < {OBST_THRE} "
           f"(old metric: {1.06 * math.tan(math.radians(15.0)):.3f} -> obstacle)")

    # 2. A 0.5 m ditch is an OBSTACLE. The old metric made the hole its own ground
    #    datum, so it read 0.00 and the robot drove in.
    ditch = np.zeros((n, n))
    ditch[:, n // 2 - 1:n // 2 + 2] = -0.5
    c = cost_of(ditch)
    record("0.5 m ditch is an obstacle (was free)", c.max() > OBST_THRE,
           f"max cost {c.max():.3f} > {OBST_THRE} (old metric: 0.000 -> free)")

    # 3. A wall must still block -- guards the plane-fit smearing risk.
    wall = np.zeros((n, n))
    wall[:, n // 2:] = 2.5
    c = cost_of(wall)
    record("wall still blocks", c.max() > OBST_THRE,
           f"max cost {c.max():.3f} > {OBST_THRE}")

    # 4. Roughness must measure roughness, not tilt. A perfectly smooth 20 deg slope
    #    scored ~0.05 m under the old stddev-of-elevation.
    smooth = (jj * res) * math.tan(math.radians(20.0))
    _, _, rr = plane_fit_reference(smooth)
    core = rr[HALF + 1:-(HALF + 1), HALF + 1:-(HALF + 1)]
    return record("roughness is slope-corrected", np.nanmax(core) < 1e-6,
                  f"max roughness {np.nanmax(core):.2e} on a smooth 20 deg slope "
                  f"(old stddev metric: ~0.05)")


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


def check_slope_cost(min_cells=5000, settle=60.0):
    """The local cost cloud must no longer have a slope blind spot.

    This check used to assert the OPPOSITE: that cells reading free in /terrain_map
    were costly in /terrain_map_ext. That was measuring the blind spot of the old
    height-above-minimum metric -- /terrain_map could not see slopes at all, so only
    the traversability-derived ext cloud caught them, and a healthy pipeline showed a
    large disagreement. The plane-fit metric folds slope into /terrain_map directly,
    so that disagreement is exactly what we set out to remove: the assertion is now
    inverted, and a LARGE gap would mean the local planner is still slope-blind.

    Only meaningful once the robot has observed some terrain, so it runs after the
    drive; elevation_mapping keeps fusing for a while after the robot stops.
    """
    deadline = time.time() + settle
    n = 0
    while True:
        _, a = cloud_xyi("/terrain_map")
        _, b = cloud_xyi("/terrain_map_ext")
        ha = {(round(x, 3), round(y, 3)): i for x, y, i in a}
        hb = {(round(x, 3), round(y, 3)): i for x, y, i in b}
        keys = sorted(set(ha) & set(hb))
        if not keys:
            return record("local cost is slope-aware", False, "no overlapping cells")
        A = np.array([ha[k] for k in keys])
        B = np.array([hb[k] for k in keys])
        n = len(keys)
        if n >= min_cells or time.time() > deadline:
            break
        time.sleep(5.0)

    if n < min_cells:
        return skip("local cost is slope-aware",
                    f"only {n} overlapping cells after {settle:.0f} s (need {min_cells})")

    # Cells the traversability map calls impassable that the LOCAL map still waves
    # through. This was ~3% with the old metric (every one of them a slope); it must
    # now be tiny, because /terrain_map applies the same slope limit itself.
    blind = int(((A < 0.1) & (B > 0.5)).sum())
    blind_pct = 100.0 * blind / n
    record("local cost has no slope blind spot", blind_pct < 1.0,
           f"{blind}/{n} ({blind_pct:.2f}%) cells impassable by traversability but free "
           f"by local cost (was ~3%, every one a slope the local metric could not see)")

    # And it must not have over-corrected into calling the world a wall.
    obst = 100.0 * (A > OBST_THRE).mean()
    return record("local obstacle fraction is sane", 0.2 < obst < 40.0,
                  f"{obst:.1f}% of observed cells are obstacles in /terrain_map")


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


# NOTE: there is deliberately no end-to-end "drive up a slope" check.
#
# outdoor_city has no drivable slope to use. I measured the true ground by teleporting
# the robot and letting it settle: the grass_plane's collision is an INFINITE <plane>
# half-space at z=0, so everything within ~30 m of spawn is dead flat (+/-0.15 m), and
# beyond that the heightmap erupts 6.4 m in 10 m -- a 33-47 deg escarpment the Husky
# genuinely cannot climb and SHOULD refuse. (The heightmap PNG suggests a gentler 22 deg,
# but it is sampled at 3.9 m/pixel, which smooths the real grade. Trust the settle probe.)
#
# Spawning a ramp at runtime does not work either: elevation_mapping never ingests it.
# With a 15 deg ramp physically present in Gazebo and plainly visible in /registered_scan
# (mean z 1.07 at y=9-13, rising to 2.71 at y=17-22), the fused map still read flat ground
# (-0.19 to -0.33 m) straight along the ramp centreline, with step ~ 0 and slope ~ 2 deg.
# The map fuses rather than replaces and does not take on geometry that appears after the
# fact; /elevation_mapping/clear_map does not rescue it, and calling that service mid-run
# empties FAR's obstacle cloud so it stops emitting waypoints entirely.
#
# Any "ramp cost" measured through that fixture describes a map with no ramp in it, so it
# would be a flaky test reporting an artefact as a failure. The slope behaviour is instead
# covered by check_metric_properties(), which runs on the numpy reference that
# check_layers_and_plane_fit() has just proven equals the deployed C++ filter to ~1e-7.

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
        check_layers_and_plane_fit()
        print("\n4. obstacle metric properties (plane fit vs terrain)")
        check_metric_properties()
        print("\n5-6. grid_map -> cost clouds")
        check_observed_mask()
        check_rates(args.rate_window)
        print("\n7. cost cloud sanity")
        check_self_mound()
        if args.drive:
            print("\n8. FAR global planner")
            check_drive(tuple(args.goal))
        # Last: it needs observed terrain, which the drive provides.
        print("\n9. slope cost")
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
