#include "mobile_manipulator/horizontal_range_filter.h"

#include <cmath>
#include <cstring>

#include <geometry_msgs/TransformStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace mobile_manipulator {

HorizontalRangeFilterPointCloud2::HorizontalRangeFilterPointCloud2() = default;
HorizontalRangeFilterPointCloud2::~HorizontalRangeFilterPointCloud2() = default;

bool HorizontalRangeFilterPointCloud2::configure() {
  if (!getParam(std::string("target_frame"), targetFrame_)) {
    targetFrame_ = "base_link";
  }
  if (!getParam(std::string("max_xy_distance"), maxDistance_) ||
      !std::isfinite(maxDistance_) || maxDistance_ <= 0.0) {
    ROS_ERROR("HorizontalRangeFilter: max_xy_distance must be finite and > 0.");
    return false;
  }
  maxDistanceSquared_ = maxDistance_ * maxDistance_;
  tfBuffer_.reset(new tf2_ros::Buffer());
  tfListener_.reset(new tf2_ros::TransformListener(*tfBuffer_));
  ROS_INFO("HorizontalRangeFilter: retaining points within %.2f m XY of %s.",
           maxDistance_, targetFrame_.c_str());
  return true;
}

bool HorizontalRangeFilterPointCloud2::update(
    const sensor_msgs::PointCloud2& input, sensor_msgs::PointCloud2& output) {
  if (!haveTransform_ || cachedSourceFrame_ != input.header.frame_id) {
    geometry_msgs::TransformStamped ts;
    try {
      ts = tfBuffer_->lookupTransform(targetFrame_, input.header.frame_id,
                                      input.header.stamp, ros::Duration(0.2));
    } catch (const tf2::TransformException& ex) {
      // This branch feeds only elevation_mapping. Withholding one scan is safer
      // than silently violating the configured range bound.
      ROS_WARN_THROTTLE(5.0,
                        "HorizontalRangeFilter: no transform %s <- %s (%s); dropping scan.",
                        targetFrame_.c_str(), input.header.frame_id.c_str(), ex.what());
      output = input;
      output.height = 1;
      output.width = 0;
      output.row_step = 0;
      output.data.clear();
      output.is_dense = true;
      return true;
    }
    const auto& q = ts.transform.rotation;
    const auto& t = ts.transform.translation;
    tf2::Matrix3x3 R(tf2::Quaternion(q.x, q.y, q.z, q.w));
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) tf_[r][c] = R[r][c];
    }
    tf_[0][3] = t.x;
    tf_[1][3] = t.y;
    tf_[2][3] = t.z;
    cachedSourceFrame_ = input.header.frame_id;
    haveTransform_ = true;
  }

  sensor_msgs::PointCloud2ConstIterator<float> itX(input, "x");
  sensor_msgs::PointCloud2ConstIterator<float> itY(input, "y");
  sensor_msgs::PointCloud2ConstIterator<float> itZ(input, "z");
  const size_t count = static_cast<size_t>(input.width) * input.height;
  const size_t step = input.point_step;

  output.header = input.header;
  output.fields = input.fields;
  output.is_bigendian = input.is_bigendian;
  output.point_step = input.point_step;
  output.height = 1;
  output.data.resize(input.data.size());

  size_t kept = 0;
  for (size_t i = 0; i < count; ++i, ++itX, ++itY, ++itZ) {
    const float x = *itX, y = *itY, z = *itZ;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    const double bx = tf_[0][0] * x + tf_[0][1] * y + tf_[0][2] * z + tf_[0][3];
    const double by = tf_[1][0] * x + tf_[1][1] * y + tf_[1][2] * z + tf_[1][3];
    if (bx * bx + by * by > maxDistanceSquared_) continue;
    std::memcpy(&output.data[kept * step], &input.data[i * step], step);
    ++kept;
  }
  output.width = static_cast<uint32_t>(kept);
  output.row_step = static_cast<uint32_t>(kept * step);
  output.data.resize(kept * step);
  output.is_dense = true;
  return true;
}

}  // namespace mobile_manipulator

PLUGINLIB_EXPORT_CLASS(mobile_manipulator::HorizontalRangeFilterPointCloud2,
                       filters::FilterBase<sensor_msgs::PointCloud2>)
