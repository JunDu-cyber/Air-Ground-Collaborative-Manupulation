# Patches to third-party packages

`src/elevation_mapping/` is a **separate clone** of
[ANYbotics/elevation_mapping](https://github.com/ANYbotics/elevation_mapping) and is listed
in `.gitignore` (line 34). Anything we change in it therefore lives **outside this repo**
and is lost on a fresh clone unless it is captured here.

That had already bitten us once: the UAV altitude-variance change in
`PerfectSensorProcessor.cpp` was sitting uncommitted in that ignored clone with nothing
recording its existence.

## elevation_mapping-airground.patch

Against upstream `f4b082c` (`Merge pull request #269 ... maintenance-discontinued`).

Three changes, all additive and all no-ops for stock single-robot configs:

1. **`InputSourceManager.cpp` — `robot_base_frame_id` becomes PER INPUT SOURCE.**
   Upstream reads it once, node-wide, and hands the same `GeneralParameters` to every
   source. But that frame is the *pivot of the pose-covariance lever arm*:
   `SensorProcessorBase` looks up `base->sensor` and `map->base` and propagates the body's
   pose covariance through that arm into each point's height variance. One node-wide pivot
   cannot serve two sensors carried by two different bodies — our UGV Velodyne pivots at
   `base_link`, the UAV's at `uav0/base_link`. Each source may now override it in its own
   namespace; omitted, it falls back to the node-wide value.

2. **`SensorProcessorBase.{hpp,cpp}` — OPTIONAL per-source pose covariance.**
   For the same reason, a source may declare its own
   `robot_pose_with_covariance_topic`. When set, that processor subscribes itself and
   `process()` weights its points by *its own body's* covariance at the cloud timestamp,
   ignoring the node-wide one. When unset, the node-wide covariance is used exactly as
   before. Without this, the UGV's points would be weighted by the UAV's covariance.

3. **`PerfectSensorProcessor.cpp` — UAV altitude variance folded into height variance.**
   (Pre-existing project change, captured here for the first time.) The map-frame z element
   of the robot-pose translation covariance adds directly to height variance; horizontal
   x/y is deliberately routed through `min_horizontal_variance` + `ElevationMap::fuse()`
   slope coupling instead, to avoid double-counting.

Together these are what let ONE `elevation_mapping` node fuse the UGV's ground LiDAR and
the UAV's aerial LiDAR into one map with each source's variance modelled correctly. See
`config/elevation_mapping_uav_prior.yaml` and `uav_prior:=true`.

### Applying it to a fresh clone

```bash
cd src/elevation_mapping
git checkout f4b082c            # the base this was written against
git apply ../../patches/elevation_mapping-airground.patch
cd ../.. && catkin build elevation_mapping
```

### Regenerating it after further edits

```bash
git -C src/elevation_mapping diff > patches/elevation_mapping-airground.patch
```
