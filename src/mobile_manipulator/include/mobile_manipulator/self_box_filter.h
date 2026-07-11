// ============================================================================
// SelfBoxFilterPointCloud2 — drop LiDAR returns that land inside an axis-aligned
// box fixed to the robot base.
//
// Why this exists alongside robot_body_filter: that filter tests points against
// the robot's COLLISION geometry, but husky_description declares top_chassis_link
// and user_rail_link visual-only. Returns off them therefore survive, and
// elevation_mapping accumulates them into a ~0.73 m mound centred on the robot,
// which the CMU localPlanner reads as an obstacle ring that blocks all 343 of its
// motion primitives.
//
// Division of labour: robot_body_filter handles the ARTICULATED UR5 arm and
// gripper (they move, and they do have collision meshes); this box handles the
// RIGID base, top plate, rails and sensor mast. Both run, in that order.
//
// Points are only TESTED in the target frame; the cloud is not reframed, and
// surviving points are copied byte-for-byte so non-XYZ fields (DLIO reads `ring`
// and `time` off this same cloud) pass through untouched.
// ============================================================================
#pragma once

#include <memory>
#include <string>

#include <filters/filter_base.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace mobile_manipulator {

class SelfBoxFilterPointCloud2 : public filters::FilterBase<sensor_msgs::PointCloud2> {
 public:
  SelfBoxFilterPointCloud2();
  virtual ~SelfBoxFilterPointCloud2();

  virtual bool configure();

  virtual bool update(const sensor_msgs::PointCloud2& input,
                      sensor_msgs::PointCloud2& output);

 private:
  std::string targetFrame_;
  double boxMin_[3];
  double boxMax_[3];

  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;

  //! cloud frame -> target frame. Static in practice (velodyne -> base_link), so
  //! it is looked up once and reused.
  bool haveTransform_{false};
  std::string cachedSourceFrame_;
  double tf_[3][4];  // row-major 3x4 [R|t]
};

}  // namespace mobile_manipulator
