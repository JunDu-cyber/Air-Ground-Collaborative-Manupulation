#include "terrain_cost_adapter/roughness_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace terrain_cost_adapter {

template <typename T>
RoughnessFilter<T>::RoughnessFilter() = default;

template <typename T>
RoughnessFilter<T>::~RoughnessFilter() = default;

template <typename T>
bool RoughnessFilter<T>::configure() {
  if (!filters::FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("RoughnessFilter did not find parameter 'input_layer'.");
    return false;
  }
  if (!filters::FilterBase<T>::getParam(std::string("output_layer"), outputLayer_)) {
    ROS_ERROR("RoughnessFilter did not find parameter 'output_layer'.");
    return false;
  }
  if (!filters::FilterBase<T>::getParam(std::string("window_length"), windowLength_)) {
    ROS_ERROR("RoughnessFilter did not find parameter 'window_length'.");
    return false;
  }
  if (windowLength_ <= 0.0) {
    ROS_ERROR("RoughnessFilter: 'window_length' must be positive (got %f).", windowLength_);
    return false;
  }
  ROS_INFO("RoughnessFilter: %s -> %s, window %.2f m (integral image).",
           inputLayer_.c_str(), outputLayer_.c_str(), windowLength_);
  return true;
}

template <typename T>
bool RoughnessFilter<T>::update(const T& mapIn, T& mapOut) {
  mapOut = mapIn;

  // The layers live in a circular buffer: raw Eigen indices only coincide with
  // spatial indices once the start index is (0, 0). Without this, the integral
  // image would sum across the buffer seam and corrupt a band of the map.
  mapOut.convertToDefaultStartIndex();

  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR("RoughnessFilter: input layer '%s' does not exist.", inputLayer_.c_str());
    return false;
  }

  const grid_map::Matrix& H = mapOut[inputLayer_];
  const int rows = static_cast<int>(H.rows());
  const int cols = static_cast<int>(H.cols());

  // Match grid_map::SlidingWindowIterator::setWindowLength: round to cells, then
  // force odd so the window is centred on the cell.
  int windowSize = static_cast<int>(std::round(windowLength_ / mapOut.getResolution()));
  if (windowSize % 2 == 0) ++windowSize;
  const int half = std::max(1, windowSize / 2);

  // Summed-area tables, padded by one row/col of zeros so the 4-corner lookup
  // needs no bounds checks. double: float accumulation over 25k cells of
  // squared elevations loses too much precision for a stddev.
  Eigen::MatrixXd sumN = Eigen::MatrixXd::Zero(rows + 1, cols + 1);
  Eigen::MatrixXd sum1 = Eigen::MatrixXd::Zero(rows + 1, cols + 1);
  Eigen::MatrixXd sum2 = Eigen::MatrixXd::Zero(rows + 1, cols + 1);

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      const float v = H(i, j);
      const bool finite = std::isfinite(v);
      const double x = finite ? static_cast<double>(v) : 0.0;
      const double c = finite ? 1.0 : 0.0;
      sumN(i + 1, j + 1) = c         + sumN(i, j + 1) + sumN(i + 1, j) - sumN(i, j);
      sum1(i + 1, j + 1) = x         + sum1(i, j + 1) + sum1(i + 1, j) - sum1(i, j);
      sum2(i + 1, j + 1) = x * x     + sum2(i, j + 1) + sum2(i + 1, j) - sum2(i, j);
    }
  }

  mapOut.add(outputLayer_);
  grid_map::Matrix& R = mapOut[outputLayer_];
  const float nan = std::numeric_limits<float>::quiet_NaN();

  auto rect = [](const Eigen::MatrixXd& S, int i0, int j0, int i1, int j1) {
    return S(i1 + 1, j1 + 1) - S(i0, j1 + 1) - S(i1 + 1, j0) + S(i0, j0);
  };

  for (int i = 0; i < rows; ++i) {
    const int i0 = std::max(0, i - half);
    const int i1 = std::min(rows - 1, i + half);
    for (int j = 0; j < cols; ++j) {
      if (!std::isfinite(H(i, j))) {
        R(i, j) = nan;
        continue;
      }
      const int j0 = std::max(0, j - half);
      const int j1 = std::min(cols - 1, j + half);

      const double n = rect(sumN, i0, j0, i1, j1);
      if (n < 2.0) {
        R(i, j) = nan;
        continue;
      }
      const double s1 = rect(sum1, i0, j0, i1, j1);
      const double s2 = rect(sum2, i0, j0, i1, j1);
      const double mean = s1 / n;
      const double var = std::max(0.0, s2 / n - mean * mean);
      R(i, j) = static_cast<float>(std::sqrt(var));
    }
  }

  return true;
}

}  // namespace terrain_cost_adapter

template class terrain_cost_adapter::RoughnessFilter<grid_map::GridMap>;

PLUGINLIB_EXPORT_CLASS(terrain_cost_adapter::RoughnessFilter<grid_map::GridMap>,
                       filters::FilterBase<grid_map::GridMap>)
