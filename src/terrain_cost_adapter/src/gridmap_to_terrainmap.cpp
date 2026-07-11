// ============================================================================
// gridmap_to_terrainmap — Phase B cost-source adapter (§1h).
//
// Converts an elevation_mapping grid_map into the CMU local_planner's
// /terrain_map (sensor_msgs/PointCloud2, PointXYZI) where intensity = terrain
// COST = height above the local ground. This lets elevation_mapping (which can
// fuse the UAV aerial prior beyond the UGV LiDAR horizon) be the SINGLE cost
// source feeding the planner, replacing terrain_analysis in Phase B.
//
//   in : grid_map_msgs/GridMap  (elevation_mapping, layer `height_layer`, odom)
//   out: sensor_msgs/PointCloud2 /terrain_map     (PointXYZI, intensity=cost)
//        sensor_msgs/PointCloud2 /terrain_map_ext (PointXYZI, intensity=cost)
//
// /terrain_map cost(cell) = clamp(elevation - localGround, 0, vehicleHeight),
// where localGround = min elevation in a `ground_radius` window (a flat patch ->
// ~0 cost = traversable; a step/rock/wall -> cost = its height -> obstacle once
// it exceeds the planner's obstacleHeightThre). This is the same metric the CMU
// terrain_analysis produces, so the planner's thresholds carry over.
//
// /terrain_map_ext cost(cell) = (1 - traversability) * vehicleHeight, from the
// postprocessor's `trav_layer` (slope+roughness, 0..1). Unlike height-above-
// ground it penalizes steep slopes, so the FAR global planner routes around
// terrain the local metric cannot see. Same [0, vehicleHeight] range, so FAR's
// obstacle thresholds match the local planner's. Only published when the layer
// exists (i.e. the extended postprocessor pipeline is loaded).
// ============================================================================
#include <algorithm>
#include <cmath>
#include <string>

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace {
ros::Publisher g_pub;
ros::Publisher g_pub_ext;
std::string g_height_layer;
std::string g_trav_layer;
std::string g_validity_layer;
std::string g_step_layer;
std::string g_slope_layer;
double g_ground_radius;
double g_vehicle_height;
double g_slope_limit;
double g_obstacle_height_thre;
}  // namespace

void gridMapCallback(const grid_map_msgs::GridMap& message) {
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(message, map);

  std::string layer = g_height_layer;
  if (!map.exists(layer)) {
    // fall back to the raw elevation layer if the hole-filled one is absent
    if (map.exists("elevation")) {
      layer = "elevation";
    } else {
      ROS_WARN_THROTTLE(5.0, "[gridmap_to_terrainmap] layer '%s' (and 'elevation') missing",
                        g_height_layer.c_str());
      return;
    }
  }

  const grid_map::Matrix& elev = map[layer];
  const grid_map::Size size = map.getSize();
  const double res = map.getResolution();
  const int win = std::max(1, static_cast<int>(std::round(g_ground_radius / res)));

  // Emit only cells the sensor actually observed. `height_layer` is the HOLE-FILLED
  // elevation, which is finite everywhere -- great for computing normals/roughness
  // over small holes, but if we published all of it we would hand the planners
  // ~89% invented terrain, ~16% of which scores as obstacle. Those phantom walls
  // fragment FAR's visibility graph and send it on huge detours around nothing.
  // The raw `elevation` layer is NaN wherever nothing was ever measured, so it is
  // the validity mask. This matches terrain_analysis_ext, which only ever emits
  // observed terrain; unobserved space stays UNKNOWN (not free, not obstacle) and
  // FAR's attemptable navigation handles it.
  const bool use_mask = !g_validity_layer.empty() && map.exists(g_validity_layer);
  if (!use_mask) {
    ROS_WARN_THROTTLE(10.0,
                      "[gridmap_to_terrainmap] validity layer '%s' missing; publishing "
                      "inpainted cells as if observed (planners will see phantom terrain)",
                      g_validity_layer.c_str());
  }
  const grid_map::Matrix& valid = use_mask ? map[g_validity_layer] : elev;
  auto observed = [&](const grid_map::Index& idx) {
    return std::isfinite(valid(idx(0), idx(1)));
  };

  const bool has_trav = map.exists(g_trav_layer);
  if (!has_trav) {
    ROS_WARN_THROTTLE(10.0,
                      "[gridmap_to_terrainmap] no '%s' layer in grid_map; "
                      "/terrain_map_ext idle (extended postprocessor not loaded?)",
                      g_trav_layer.c_str());
  }
  const grid_map::Matrix& trav = has_trav ? map[g_trav_layer] : elev;

  // Plane-fit cost source. `step` is the SIGNED residual from the locally fitted
  // plane, so it is slope-invariant (a smooth ramp of any angle reads ~0) and it
  // catches negative obstacles (a ditch reads negative; the old height-above-MINIMUM
  // metric made the ditch its own ground datum and called the hole free ground).
  const bool has_plane = map.exists(g_step_layer) && map.exists(g_slope_layer);
  if (!has_plane) {
    ROS_WARN_THROTTLE(10.0,
                      "[gridmap_to_terrainmap] no '%s'/'%s' layers; falling back to the "
                      "height-above-minimum metric, which reads every slope past ~8 deg as "
                      "a wall and is blind to ditches (is PlaneFitFilter in the chain?)",
                      g_step_layer.c_str(), g_slope_layer.c_str());
  }
  const grid_map::Matrix& step = has_plane ? map[g_step_layer] : elev;
  const grid_map::Matrix& slope = has_plane ? map[g_slope_layer] : elev;

  pcl::PointCloud<pcl::PointXYZI> cloud, ext;
  const size_t cells = static_cast<size_t>(size(0)) * static_cast<size_t>(size(1));
  cloud.reserve(cells);
  ext.reserve(cells);

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index idx(*it);
    const float e = elev(idx(0), idx(1));
    if (!std::isfinite(e) || !observed(idx)) {
      continue;
    }

    // Local ground datum. Only needed for the legacy fallback metric and to drape the
    // FAR cloud at ground level; the plane-fit cost below does not use it.
    float ground = e;
    for (int di = -win; di <= win; ++di) {
      const int i = idx(0) + di;
      if (i < 0 || i >= size(0)) continue;
      for (int dj = -win; dj <= win; ++dj) {
        const int j = idx(1) + dj;
        if (j < 0 || j >= size(1)) continue;
        const float v = elev(i, j);
        if (std::isfinite(v) && v < ground) ground = v;
      }
    }

    float cost;
    if (has_plane) {
      const float s = step(idx(0), idx(1));
      const float sl = slope(idx(0), idx(1));
      if (!std::isfinite(s) || !std::isfinite(sl)) continue;  // degenerate fit -> unknown

      // Two independent ways for a cell to be impassable, combined into the ONE scalar
      // localPlanner gates on (obstacle iff intensity > obstacleHeightThre):
      //
      //   |step|          a wall, curb, rock or ditch -- an actual discontinuity, in
      //                   metres, and already comparable to obstacleHeightThre.
      //   slope penalty   a SMOOTH but too-steep surface, which has step ~ 0 and would
      //                   otherwise sail through. Rescaled into a pseudo-height that
      //                   hits exactly obstacleHeightThre at slope_limit, so a slope at
      //                   the Husky's 25 deg limit lands precisely on the gate: below it
      //                   the term stays under the threshold and |step| dominates; a
      //                   smooth 45 deg cliff trips the gate on slope alone.
      //
      // Taking the max (not a sum) keeps each term's physical meaning and keeps the
      // CMU-side obstacleHeightThre tuning untouched.
      const float slope_pseudo_height =
          static_cast<float>(g_obstacle_height_thre) * sl / static_cast<float>(g_slope_limit);
      cost = std::max(std::fabs(s), slope_pseudo_height);
    } else {
      cost = e - ground;  // legacy: height above the local minimum
    }
    if (cost < 0.0f) cost = 0.0f;
    if (cost > static_cast<float>(g_vehicle_height)) cost = static_cast<float>(g_vehicle_height);

    grid_map::Position pos;
    map.getPosition(idx, pos);

    pcl::PointXYZI p;
    p.x = static_cast<float>(pos.x());
    p.y = static_cast<float>(pos.y());
    p.z = e;
    p.intensity = cost;
    cloud.push_back(p);

    if (!has_trav) continue;
    const float t = trav(idx(0), idx(1));
    if (!std::isfinite(t)) continue;

    // Drape the FAR cloud on the GROUND, not on top of the obstacle.
    //
    // FAR crops /terrain_cloud to |z - robot_z| < kTolerZ (~1.85 m) before splitting
    // it into free/obstacle. Upstream terrain_analysis_ext feeds it raw LiDAR points,
    // so a wall contributes returns all the way down to ground level and survives that
    // crop. Our 2.5D grid has ONE point per cell carrying the elevation of the obstacle
    // TOP (a building reads z ~= 2.8 m), so every obstacle would be cropped away --
    // leaving FAR's surrounding obstacle cloud empty, `is_cloud_init_` false, and its
    // whole planning loop inert. The cost already lives in `intensity`; z only has to
    // say where on the terrain surface this cell is.
    pcl::PointXYZI q;
    q.x = p.x;
    q.y = p.y;
    q.z = ground;
    q.intensity = std::max(0.0f, std::min(1.0f, 1.0f - t)) *
                  static_cast<float>(g_vehicle_height);
    ext.push_back(q);
  }

  sensor_msgs::PointCloud2 out;
  pcl::toROSMsg(cloud, out);
  out.header.frame_id = map.getFrameId();        // == elevation_mapping map_frame (odom)
  out.header.stamp = message.info.header.stamp;
  g_pub.publish(out);

  if (has_trav) {
    sensor_msgs::PointCloud2 out_ext;
    pcl::toROSMsg(ext, out_ext);
    out_ext.header = out.header;
    g_pub_ext.publish(out_ext);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gridmap_to_terrainmap");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string grid_map_topic, terrain_map_topic, terrain_map_ext_topic;
  pnh.param<std::string>("grid_map_topic", grid_map_topic,
                         "/elevation_mapping/elevation_map_postprocessed");
  pnh.param<std::string>("terrain_map_topic", terrain_map_topic, "/terrain_map");
  pnh.param<std::string>("terrain_map_ext_topic", terrain_map_ext_topic, "/terrain_map_ext");
  pnh.param<std::string>("height_layer", g_height_layer, "elevation_filled");
  pnh.param<std::string>("trav_layer", g_trav_layer, "traversability");
  // NaN wherever the sensor never measured; set to "" to publish inpainted cells too.
  pnh.param<std::string>("validity_layer", g_validity_layer, "elevation");
  // Plane-fit layers (PlaneFitFilter). Absent -> legacy height-above-minimum metric.
  pnh.param<std::string>("step_layer", g_step_layer, "step");
  pnh.param<std::string>("slope_layer", g_slope_layer, "slope");
  pnh.param<double>("ground_radius", g_ground_radius, 0.75);
  pnh.param<double>("vehicle_height", g_vehicle_height, 1.0);
  // Slope at which terrain becomes impassable: 0.44 rad = 25 deg (Husky limit).
  pnh.param<double>("slope_limit", g_slope_limit, 0.44);
  // MUST match localPlanner's obstacleHeightThre -- it is the gate this cost is
  // scaled against, so that a slope at slope_limit lands exactly on the threshold.
  pnh.param<double>("obstacle_height_thre", g_obstacle_height_thre, 0.15);

  g_pub = nh.advertise<sensor_msgs::PointCloud2>(terrain_map_topic, 2);
  g_pub_ext = nh.advertise<sensor_msgs::PointCloud2>(terrain_map_ext_topic, 2);
  ros::Subscriber sub = nh.subscribe(grid_map_topic, 1, gridMapCallback);

  ROS_INFO("[gridmap_to_terrainmap] %s (layer %s) -> %s  (ground_radius=%.2f, vehicleHeight=%.2f); "
           "(layer %s) -> %s",
           grid_map_topic.c_str(), g_height_layer.c_str(), terrain_map_topic.c_str(),
           g_ground_radius, g_vehicle_height,
           g_trav_layer.c_str(), terrain_map_ext_topic.c_str());
  ros::spin();
  return 0;
}
