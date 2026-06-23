# Patches for vendored (vcs-imported) dependencies

`src/elevation_mapping/` and the other ETH repos are **gitignored** and reproduced
with `vcs import src < src/elevation_mapping.repos`. Edits to them are therefore
lost whenever they are re-imported. The patches here persist the edits we depend
on. Re-apply them after every `vcs import`.

## perfect_sensor_processor_altitude.patch

Adds the UAV **altitude** (vertical-position) uncertainty term to the per-point
height variance in `PerfectSensorProcessor::computeVariances`. The stock processor
uses only the rotation block of the robot-pose covariance and ignores translation;
this adds the map-frame `z` element so that the UAV's altitude error published on
`/uav/pose_cov` reaches the elevation map. Part of the egocentric UAV pose-error
propagation (see `config/elevation_mapping.yaml` and
`scripts/uav_pose_cov_publisher.py`).

Apply from the elevation_mapping repo root:

```bash
cd src/elevation_mapping
git apply ../mobile_manipulator/patches/perfect_sensor_processor_altitude.patch
# or, without git:
#   patch -p1 < ../mobile_manipulator/patches/perfect_sensor_processor_altitude.patch
```

Then rebuild: `catkin build elevation_mapping`.
