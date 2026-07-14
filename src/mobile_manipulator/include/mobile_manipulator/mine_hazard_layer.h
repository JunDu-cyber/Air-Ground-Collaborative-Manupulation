#ifndef MOBILE_MANIPULATOR_MINE_HAZARD_LAYER_H_
#define MOBILE_MANIPULATOR_MINE_HAZARD_LAYER_H_

#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <costmap_2d/costmap_layer.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <mobile_manipulator/MineMission.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <uav_truth_tracker/MineMap.h>

namespace mobile_manipulator
{

class MineHazardLayer : public costmap_2d::CostmapLayer
{
public:
  MineHazardLayer();

  void onInitialize() override;
  void matchSize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y,
                    double* max_x, double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid,
                   int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;

private:
  struct Hazard
  {
    geometry_msgs::Point point;
    std::string frame_id;
  };

  struct RenderedHazard
  {
    uint32_t id;
    double x;
    double y;
    double radius;
  };

  void mineMapCallback(const uav_truth_tracker::MineMapConstPtr& msg);
  void missionCallback(const mobile_manipulator::MineMissionConstPtr& msg);
  void disposedCallback(const geometry_msgs::PoseArrayConstPtr& msg);
  void resetCallback(const std_msgs::EmptyConstPtr& msg);
  void rebuildFromLastMapLocked();
  bool transformHazard(const Hazard& hazard, double& x, double& y) const;
  void paintDisk(double x, double y, double radius);
  void touchDisk(double x, double y, double radius, double* min_x, double* min_y,
                 double* max_x, double* max_y);

  ros::NodeHandle nh_;
  ros::Subscriber mine_map_sub_;
  ros::Subscriber mission_sub_;
  ros::Subscriber disposed_sub_;
  ros::Subscriber reset_sub_;

  mutable std::mutex mutex_;
  std::map<uint32_t, Hazard> hazards_;
  std::map<uint32_t, Hazard> observed_hazards_;
  std::vector<Hazard> disposed_hazards_;
  std::map<uint32_t, RenderedHazard> transformed_cache_;
  std::set<uint32_t> cleared_ids_;
  // A retained mine is no longer at its original map coordinate.  Keep its
  // observation for recovery/reset, but do not paint that stale source cell
  // while the mission reports CARRYING/RETURNING/PLACING.
  std::set<uint32_t> carried_ids_;
  std::vector<RenderedHazard> previous_rendered_;

  std::string mine_map_topic_;
  std::string mission_topic_;
  std::string disposed_topic_;
  std::string reset_topic_;
  std::string fallback_map_frame_;
  double lethal_radius_;
  double disposed_lethal_radius_;
  double transform_tolerance_;
};

}  // namespace mobile_manipulator

#endif  // MOBILE_MANIPULATOR_MINE_HAZARD_LAYER_H_
