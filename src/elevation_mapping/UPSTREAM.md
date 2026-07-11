# Vendored fork of ANYbotics/elevation_mapping

This directory is **not** a pristine copy. It is vendored into this repository (plain
source, no nested `.git`) and carries local changes required by the air-ground stack.

| | |
|---|---|
| Upstream | https://github.com/ANYbotics/elevation_mapping |
| Forked at | `f4b082c` — *Merge pull request #269 from ANYbotics/Magnusgaertner-maintenance-discontinued* |
| Upstream status | **Discontinued** by ANYbotics (see that very commit), so there is nothing to track |

## Why it is vendored rather than a clone

It used to be a separate clone listed in `.gitignore`. That meant every change we made to it
lived outside version control and vanished on a fresh clone — and it was already happening:
the `PerfectSensorProcessor` UAV-altitude-variance change sat uncommitted in that ignored
tree for weeks with nothing recording its existence.

## Local changes

All of them are additive and are no-ops for stock single-robot configurations. The
equivalent diff against `f4b082c` is kept at `patches/elevation_mapping-airground.patch`
for reference (it is what these changes were before vendoring).

1. **`InputSourceManager.cpp` — `robot_base_frame_id` is PER INPUT SOURCE.**
   That frame is the *pivot of the pose-covariance lever arm*: `SensorProcessorBase` looks up
   `base->sensor` and `map->base` from it and propagates the body's pose covariance through
   that arm into each point's height variance. Upstream reads it once, node-wide, and gives
   every source the same `GeneralParameters`. One pivot cannot serve two bodies — our UGV's
   Velodyne pivots at `base_link` (arm ~0.7 m), the UAV's at `uav0/base_link` (arm ~0.1 m).
   Each source may now override it in its own namespace; omitted, it falls back to the
   node-wide value.

2. **`SensorProcessorBase.{hpp,cpp}` — OPTIONAL per-source pose covariance.**
   For the same reason a source may declare its own `robot_pose_with_covariance_topic`. When
   set, that processor subscribes itself and `process()` weights its points by *its own
   body's* covariance at the cloud timestamp, ignoring the node-wide one. Without this the
   UGV's points would be weighted by the UAV's pose error.

3. **`PerfectSensorProcessor.cpp` — UAV altitude variance into height variance.**
   The map-frame z element of the robot-pose translation covariance adds directly to height
   variance. Horizontal x/y is deliberately routed through `min_horizontal_variance` +
   `ElevationMap::fuse()` slope coupling instead, to avoid double-counting.

Together these let ONE `elevation_mapping` node fuse the UGV's ground LiDAR and the UAV's
aerial LiDAR into one map with each source's variance modelled correctly. See
`mobile_manipulator/config/elevation_mapping_uav_prior.yaml` and `uav_prior:=true`.

## Comparing against upstream later

The nested `.git` is gone, so `git diff` here shows changes against *our* history. To diff
against pristine upstream:

```bash
git clone https://github.com/ANYbotics/elevation_mapping /tmp/em-upstream
git -C /tmp/em-upstream checkout f4b082c
diff -ru /tmp/em-upstream src/elevation_mapping -x .git -x UPSTREAM.md
```
