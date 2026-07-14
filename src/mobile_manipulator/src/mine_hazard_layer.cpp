#include <algorithm>
#include <cmath>
#include <limits>

#include <costmap_2d/cost_values.h>
#include <geometry_msgs/PointStamped.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mobile_manipulator/mine_hazard_layer.h>

namespace mobile_manipulator
{

MineHazardLayer::MineHazardLayer()
  : lethal_radius_(0.10),
    disposed_lethal_radius_(0.10),
    transform_tolerance_(0.5)
{
}

void MineHazardLayer::onInitialize()
{
  ros::NodeHandle private_nh("~" + name_);
  private_nh.param("enabled", enabled_, true);
  private_nh.param("mine_map_topic", mine_map_topic_,
                   std::string("/mine_detection/map"));
  private_nh.param("mission_topic", mission_topic_,
                   std::string("/mine_mission/status"));
  private_nh.param("disposed_topic", disposed_topic_,
                   std::string("/mine_disposal/poses"));
  private_nh.param("reset_topic", reset_topic_,
                   std::string("/mine_hazard/reset"));
  private_nh.param("map_frame", fallback_map_frame_, std::string("map"));
  private_nh.param("lethal_radius", lethal_radius_, 0.10);
  private_nh.param("disposed_lethal_radius", disposed_lethal_radius_, 0.10);
  private_nh.param("transform_tolerance", transform_tolerance_, 0.5);

  lethal_radius_ = std::max(0.01, lethal_radius_);
  disposed_lethal_radius_ = std::max(0.01, disposed_lethal_radius_);
  transform_tolerance_ = std::max(0.0, transform_tolerance_);
  current_ = true;
  matchSize();

  mine_map_sub_ = nh_.subscribe(mine_map_topic_, 5,
                                &MineHazardLayer::mineMapCallback, this);
  mission_sub_ = nh_.subscribe(mission_topic_, 5,
                               &MineHazardLayer::missionCallback, this);
  disposed_sub_ = nh_.subscribe(disposed_topic_, 2,
                                &MineHazardLayer::disposedCallback, this);
  reset_sub_ = nh_.subscribe(reset_topic_, 2,
                             &MineHazardLayer::resetCallback, this);

  ROS_INFO("[%s] mine hazard layer ready: input=%s radius=%.2fm frame=%s",
           name_.c_str(), mine_map_topic_.c_str(), lethal_radius_,
           layered_costmap_->getGlobalFrameID().c_str());
}

void MineHazardLayer::disposedCallback(
    const geometry_msgs::PoseArrayConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const std::string frame = msg->header.frame_id.empty()
                                ? fallback_map_frame_
                                : msg->header.frame_id;
  disposed_hazards_.clear();
  disposed_hazards_.reserve(msg->poses.size());
  for (const auto& pose : msg->poses)
    disposed_hazards_.push_back(Hazard{pose.position, frame});
}

void MineHazardLayer::matchSize()
{
  CostmapLayer::matchSize();
  // Unknown means this layer has no opinion; updateWithMax then preserves the
  // static/obstacle layers while applying only our lethal cells.
  setDefaultValue(costmap_2d::NO_INFORMATION);
  resetMaps();
}

void MineHazardLayer::mineMapCallback(
    const uav_truth_tracker::MineMapConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const std::string frame = msg->header.frame_id.empty()
                                ? fallback_map_frame_
                                : msg->header.frame_id;
  for (const auto& mine : msg->mines)
  {
    if (!mine.confirmed)
      continue;
    const Hazard hazard{mine.position, frame};
    observed_hazards_[mine.id] = hazard;
    // Continue remembering the source coordinate so reset or a lost physical
    // lock can restore it, but never let a fresh UAV map packet repaint the
    // stale source while that exact mine is attached to the UGV.
    if (cleared_ids_.count(mine.id) != 0 ||
        carried_ids_.count(mine.id) != 0)
      continue;
    hazards_[mine.id] = hazard;
  }
  // Intentionally do not erase IDs absent from a later perception message.
  // A temporary UAV dropout must never make a known mine traversable.
}

void MineHazardLayer::missionCallback(
    const mobile_manipulator::MineMissionConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::set<uint32_t> next_carried_ids;
  for (const auto& entry : msg->entries)
  {
    if (entry.state == mobile_manipulator::MineMissionEntry::CLEARED)
    {
      cleared_ids_.insert(entry.id);
      hazards_.erase(entry.id);
      transformed_cache_.erase(entry.id);
      continue;
    }

    const bool physically_carried =
        entry.state == mobile_manipulator::MineMissionEntry::CARRYING ||
        entry.state == mobile_manipulator::MineMissionEntry::RETURNING_HOME ||
        entry.state == mobile_manipulator::MineMissionEntry::AT_DROPOFF ||
        entry.state == mobile_manipulator::MineMissionEntry::PLACING;
    if (!physically_carried)
      continue;

    next_carried_ids.insert(entry.id);
    hazards_.erase(entry.id);
    transformed_cache_.erase(entry.id);
    if (carried_ids_.count(entry.id) == 0)
      ROS_WARN("[%s] source hazard M%03u suspended after physical PICK lock",
               name_.c_str(), entry.id);
  }

  // If a carrying state disappears without CLEARED (for example the grasp
  // lock was lost and the mission entered MANUAL_REQUIRED), restore the last
  // confirmed source cell immediately and keep the base safety lock effective.
  for (const uint32_t id : carried_ids_)
  {
    if (next_carried_ids.count(id) != 0 || cleared_ids_.count(id) != 0)
      continue;
    const auto observed = observed_hazards_.find(id);
    if (observed != observed_hazards_.end())
    {
      hazards_[id] = observed->second;
      ROS_ERROR("[%s] source hazard M%03u restored after carrying state ended",
                name_.c_str(), id);
    }
  }
  carried_ids_.swap(next_carried_ids);
}

void MineHazardLayer::resetCallback(const std_msgs::EmptyConstPtr&)
{
  std::lock_guard<std::mutex> lock(mutex_);
  hazards_.clear();
  transformed_cache_.clear();
  cleared_ids_.clear();
  carried_ids_.clear();
  rebuildFromLastMapLocked();
  ROS_WARN("[%s] hazard state reset; confirmed mines were restored from the latest map",
           name_.c_str());
}

void MineHazardLayer::rebuildFromLastMapLocked()
{
  hazards_.clear();
  for (const auto& item : observed_hazards_)
  {
    if (cleared_ids_.count(item.first) == 0 &&
        carried_ids_.count(item.first) == 0)
      hazards_[item.first] = item.second;
  }
}

bool MineHazardLayer::transformHazard(const Hazard& hazard,
                                      double& x, double& y) const
{
  const std::string target_frame = layered_costmap_->getGlobalFrameID();
  const std::string source_frame = hazard.frame_id.empty()
                                       ? fallback_map_frame_
                                       : hazard.frame_id;
  if (source_frame == target_frame)
  {
    x = hazard.point.x;
    y = hazard.point.y;
    return true;
  }

  geometry_msgs::PointStamped input;
  geometry_msgs::PointStamped output;
  input.header.frame_id = source_frame;
  input.header.stamp = ros::Time(0);
  input.point = hazard.point;
  try
  {
    output = tf_->transform(input, target_frame,
                            ros::Duration(transform_tolerance_));
    x = output.point.x;
    y = output.point.y;
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(2.0, "[%s] cannot transform mine %s -> %s: %s",
                      name_.c_str(), source_frame.c_str(), target_frame.c_str(),
                      ex.what());
    return false;
  }
}

void MineHazardLayer::touchDisk(double x, double y, double radius,
                                double* min_x, double* min_y,
                                double* max_x, double* max_y)
{
  touch(x - radius, y - radius, min_x, min_y, max_x, max_y);
  touch(x + radius, y + radius, min_x, min_y, max_x, max_y);
}

void MineHazardLayer::paintDisk(double x, double y, double radius)
{
  if (getSizeInCellsX() == 0 || getSizeInCellsY() == 0)
    return;

  int min_mx, min_my, max_mx, max_my;
  worldToMapEnforceBounds(x - radius, y - radius, min_mx, min_my);
  worldToMapEnforceBounds(x + radius, y + radius, max_mx, max_my);
  const double radius_sq = radius * radius;
  for (int my = min_my; my <= max_my; ++my)
  {
    for (int mx = min_mx; mx <= max_mx; ++mx)
    {
      double wx, wy;
      mapToWorld(static_cast<unsigned int>(mx), static_cast<unsigned int>(my), wx, wy);
      if ((wx - x) * (wx - x) + (wy - y) * (wy - y) <= radius_sq)
        setCost(static_cast<unsigned int>(mx), static_cast<unsigned int>(my),
                costmap_2d::LETHAL_OBSTACLE);
    }
  }
}

void MineHazardLayer::updateBounds(double, double, double,
                                   double* min_x, double* min_y,
                                   double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  useExtraBounds(min_x, min_y, max_x, max_y);
  std::lock_guard<std::mutex> lock(mutex_);

  // Ensure cells occupied in the previous cycle are part of this update, so a
  // SUCCESS result really clears them from the master costmap.
  for (const auto& mine : previous_rendered_)
    touchDisk(mine.x, mine.y, mine.radius, min_x, min_y, max_x, max_y);

  std::vector<RenderedHazard> rendered;
  rendered.reserve(hazards_.size() + disposed_hazards_.size());
  for (const auto& item : hazards_)
  {
    double x = 0.0;
    double y = 0.0;
    if (transformHazard(item.second, x, y))
      transformed_cache_[item.first] = RenderedHazard{item.first, x, y, lethal_radius_};
    else
    {
      const auto cached = transformed_cache_.find(item.first);
      if (cached == transformed_cache_.end())
        continue;
      x = cached->second.x;
      y = cached->second.y;
    }
    rendered.push_back(RenderedHazard{item.first, x, y, lethal_radius_});
  }
  uint32_t disposed_id = 0x80000000u;
  for (const auto& hazard : disposed_hazards_)
  {
    double x = 0.0;
    double y = 0.0;
    if (transformHazard(hazard, x, y))
      rendered.push_back(RenderedHazard{disposed_id++, x, y,
                                        disposed_lethal_radius_});
  }

  resetMaps();
  for (const auto& mine : rendered)
  {
    paintDisk(mine.x, mine.y, mine.radius);
    touchDisk(mine.x, mine.y, mine.radius, min_x, min_y, max_x, max_y);
  }
  previous_rendered_.swap(rendered);
  current_ = true;
}

void MineHazardLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                  int min_i, int min_j, int max_i, int max_j)
{
  if (enabled_)
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void MineHazardLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  resetMaps();
  current_ = true;
}

}  // namespace mobile_manipulator

PLUGINLIB_EXPORT_CLASS(mobile_manipulator::MineHazardLayer, costmap_2d::Layer)
