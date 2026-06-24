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
//   out: sensor_msgs/PointCloud2 (PointXYZI, intensity=cost, frame = map frame)
//
// cost(cell) = clamp(elevation - localGround, 0, vehicleHeight), where
// localGround = min elevation in a `ground_radius` window (a flat patch -> ~0
// cost = traversable; a step/rock/wall -> cost = its height -> obstacle once it
// exceeds the planner's obstacleHeightThre). This is the same metric the CMU
// terrain_analysis produces, so the planner's thresholds carry over.
// ============================================================================
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
std::string g_height_layer;
double g_ground_radius;
double g_vehicle_height;
}  // namespace

void gridMapCallback(const grid_map_msgs::GridMap& message) {
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(message, map);

  std::string layer = g_height_layer;
  if (!map.exists(layer)) {
    // fall back to the raw elevation layer if the inpainted one is absent
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

  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.reserve(static_cast<size_t>(size(0)) * static_cast<size_t>(size(1)));

  for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    const grid_map::Index idx(*it);
    const float e = elev(idx(0), idx(1));
    if (!std::isfinite(e)) {
      continue;
    }

    // local ground = minimum elevation in a (2*win+1)^2 window
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

    float cost = e - ground;
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
  }

  sensor_msgs::PointCloud2 out;
  pcl::toROSMsg(cloud, out);
  out.header.frame_id = map.getFrameId();        // == elevation_mapping map_frame (odom)
  out.header.stamp = message.info.header.stamp;
  g_pub.publish(out);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gridmap_to_terrainmap");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string grid_map_topic, terrain_map_topic;
  pnh.param<std::string>("grid_map_topic", grid_map_topic,
                         "/elevation_mapping/elevation_map_postprocessed");
  pnh.param<std::string>("terrain_map_topic", terrain_map_topic, "/terrain_map");
  pnh.param<std::string>("height_layer", g_height_layer, "elevation_inpainted");
  pnh.param<double>("ground_radius", g_ground_radius, 0.75);
  pnh.param<double>("vehicle_height", g_vehicle_height, 1.0);

  g_pub = nh.advertise<sensor_msgs::PointCloud2>(terrain_map_topic, 2);
  ros::Subscriber sub = nh.subscribe(grid_map_topic, 1, gridMapCallback);

  ROS_INFO("[gridmap_to_terrainmap] %s (layer %s) -> %s  (ground_radius=%.2f, vehicleHeight=%.2f)",
           grid_map_topic.c_str(), g_height_layer.c_str(), terrain_map_topic.c_str(),
           g_ground_radius, g_vehicle_height);
  ros::spin();
  return 0;
}
