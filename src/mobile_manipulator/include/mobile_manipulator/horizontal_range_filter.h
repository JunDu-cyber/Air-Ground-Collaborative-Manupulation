#pragma once

#include <memory>
#include <string>

#include <filters/filter_base.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace mobile_manipulator {

// Elevation-only range gate. Points are tested in base_link but copied in their
// original layout/frame so elevation_mapping still receives every auxiliary
// field and performs its normal sensor transform.
class HorizontalRangeFilterPointCloud2
    : public filters::FilterBase<sensor_msgs::PointCloud2> {
 public:
  HorizontalRangeFilterPointCloud2();
  virtual ~HorizontalRangeFilterPointCloud2();

  virtual bool configure();
  virtual bool update(const sensor_msgs::PointCloud2& input,
                      sensor_msgs::PointCloud2& output);

 private:
  std::string targetFrame_;
  double maxDistance_{20.0};
  double maxDistanceSquared_{400.0};
  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;
  bool haveTransform_{false};
  std::string cachedSourceFrame_;
  double tf_[3][4]{};
};

}  // namespace mobile_manipulator
