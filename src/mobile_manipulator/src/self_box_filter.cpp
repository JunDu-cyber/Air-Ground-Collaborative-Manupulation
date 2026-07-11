#include "mobile_manipulator/self_box_filter.h"

#include <cmath>
#include <cstring>
#include <vector>

#include <geometry_msgs/TransformStamped.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace mobile_manipulator {

SelfBoxFilterPointCloud2::SelfBoxFilterPointCloud2() = default;
SelfBoxFilterPointCloud2::~SelfBoxFilterPointCloud2() = default;

namespace {
bool readVec3(const std::vector<double>& v, const char* name, double out[3]) {
  if (v.size() != 3) {
    ROS_ERROR("SelfBoxFilter: '%s' must have exactly 3 elements (got %zu).", name, v.size());
    return false;
  }
  for (int i = 0; i < 3; ++i) out[i] = v[i];
  return true;
}
}  // namespace

bool SelfBoxFilterPointCloud2::configure() {
  if (!getParam(std::string("frames/target"), targetFrame_)) {
    targetFrame_ = "base_link";
  }

  std::vector<double> vmin, vmax;
  if (!getParam(std::string("box/min"), vmin)) {
    ROS_ERROR("SelfBoxFilter did not find parameter 'box/min'.");
    return false;
  }
  if (!getParam(std::string("box/max"), vmax)) {
    ROS_ERROR("SelfBoxFilter did not find parameter 'box/max'.");
    return false;
  }
  if (!readVec3(vmin, "box/min", boxMin_) || !readVec3(vmax, "box/max", boxMax_)) {
    return false;
  }
  for (int i = 0; i < 3; ++i) {
    if (boxMin_[i] >= boxMax_[i]) {
      ROS_ERROR("SelfBoxFilter: box/min[%d] (%f) must be < box/max[%d] (%f).",
                i, boxMin_[i], i, boxMax_[i]);
      return false;
    }
  }

  tfBuffer_.reset(new tf2_ros::Buffer());
  tfListener_.reset(new tf2_ros::TransformListener(*tfBuffer_));

  ROS_INFO("SelfBoxFilter: dropping points inside [%.2f %.2f %.2f]..[%.2f %.2f %.2f] of frame %s.",
           boxMin_[0], boxMin_[1], boxMin_[2], boxMax_[0], boxMax_[1], boxMax_[2],
           targetFrame_.c_str());
  return true;
}

bool SelfBoxFilterPointCloud2::update(const sensor_msgs::PointCloud2& input,
                                      sensor_msgs::PointCloud2& output) {
  if (!haveTransform_ || cachedSourceFrame_ != input.header.frame_id) {
    geometry_msgs::TransformStamped ts;
    try {
      ts = tfBuffer_->lookupTransform(targetFrame_, input.header.frame_id,
                                      ros::Time(0), ros::Duration(0.2));
    } catch (const tf2::TransformException& ex) {
      // Passing the cloud through unfiltered is safer than dropping it: DLIO
      // would lose odometry entirely, and this resolves on the next scan.
      ROS_WARN_THROTTLE(5.0, "SelfBoxFilter: no transform %s <- %s (%s); passing cloud through.",
                        targetFrame_.c_str(), input.header.frame_id.c_str(), ex.what());
      output = input;
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

  const size_t numPoints = static_cast<size_t>(input.width) * input.height;
  const size_t step = input.point_step;

  // Same layout as the input; only the surviving points are copied, so `ring`,
  // `time`, `intensity` and anything else ride along untouched.
  output.header = input.header;
  output.fields = input.fields;
  output.is_bigendian = input.is_bigendian;
  output.point_step = input.point_step;
  output.height = 1;
  output.data.resize(input.data.size());

  size_t kept = 0;
  for (size_t i = 0; i < numPoints; ++i, ++itX, ++itY, ++itZ) {
    const float x = *itX, y = *itY, z = *itZ;
    bool drop = false;
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
      const double px = tf_[0][0] * x + tf_[0][1] * y + tf_[0][2] * z + tf_[0][3];
      const double py = tf_[1][0] * x + tf_[1][1] * y + tf_[1][2] * z + tf_[1][3];
      const double pz = tf_[2][0] * x + tf_[2][1] * y + tf_[2][2] * z + tf_[2][3];
      drop = px >= boxMin_[0] && px <= boxMax_[0] &&
             py >= boxMin_[1] && py <= boxMax_[1] &&
             pz >= boxMin_[2] && pz <= boxMax_[2];
    }
    if (!drop) {
      std::memcpy(&output.data[kept * step], &input.data[i * step], step);
      ++kept;
    }
  }

  output.width = static_cast<uint32_t>(kept);
  output.row_step = static_cast<uint32_t>(kept * step);
  output.data.resize(kept * step);
  output.is_dense = input.is_dense;
  return true;
}

}  // namespace mobile_manipulator

PLUGINLIB_EXPORT_CLASS(mobile_manipulator::SelfBoxFilterPointCloud2,
                       filters::FilterBase<sensor_msgs::PointCloud2>)
