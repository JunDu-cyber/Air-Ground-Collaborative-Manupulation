#include "terrain_cost_adapter/plane_fit_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace terrain_cost_adapter {

template <typename T>
PlaneFitFilter<T>::PlaneFitFilter() = default;

template <typename T>
PlaneFitFilter<T>::~PlaneFitFilter() = default;

template <typename T>
bool PlaneFitFilter<T>::configure() {
  if (!filters::FilterBase<T>::getParam(std::string("input_layer"), inputLayer_)) {
    ROS_ERROR("PlaneFitFilter did not find parameter 'input_layer'.");
    return false;
  }
  // Output layer names default to the ones the postprocessor chain and the adapter
  // already expect, so the yaml only has to name them when overriding.
  if (!filters::FilterBase<T>::getParam(std::string("slope_layer"), slopeLayer_)) {
    slopeLayer_ = "slope";
  }
  if (!filters::FilterBase<T>::getParam(std::string("step_layer"), stepLayer_)) {
    stepLayer_ = "step";
  }
  if (!filters::FilterBase<T>::getParam(std::string("step_abs_layer"), stepAbsLayer_)) {
    stepAbsLayer_ = "step_abs";
  }
  if (!filters::FilterBase<T>::getParam(std::string("roughness_layer"), roughnessLayer_)) {
    roughnessLayer_ = "roughness";
  }
  if (!filters::FilterBase<T>::getParam(std::string("window_length"), windowLength_)) {
    ROS_ERROR("PlaneFitFilter did not find parameter 'window_length'.");
    return false;
  }
  if (windowLength_ <= 0.0) {
    ROS_ERROR("PlaneFitFilter: 'window_length' must be positive (got %f).", windowLength_);
    return false;
  }
  double minPoints = 6.0;
  if (filters::FilterBase<T>::getParam(std::string("min_points"), minPoints)) {
    minPoints_ = static_cast<int>(minPoints);
  }
  if (minPoints_ < 3) {
    ROS_ERROR("PlaneFitFilter: 'min_points' must be >= 3 (a plane needs 3 points), got %d.",
              minPoints_);
    return false;
  }
  ROS_INFO("PlaneFitFilter: %s -> %s / %s / %s / %s, window %.2f m, min_points %d (integral image).",
           inputLayer_.c_str(), slopeLayer_.c_str(), stepLayer_.c_str(), stepAbsLayer_.c_str(),
           roughnessLayer_.c_str(), windowLength_, minPoints_);
  return true;
}

template <typename T>
bool PlaneFitFilter<T>::update(const T& mapIn, T& mapOut) {
  const ros::WallTime t_start = ros::WallTime::now();
  mapOut = mapIn;

  // The layers live in a circular buffer: raw Eigen indices only coincide with
  // spatial indices once the start index is (0, 0). Without this, the integral
  // images would sum across the buffer seam and corrupt a band of the map.
  mapOut.convertToDefaultStartIndex();

  if (!mapOut.exists(inputLayer_)) {
    ROS_ERROR("PlaneFitFilter: input layer '%s' does not exist.", inputLayer_.c_str());
    return false;
  }

  const grid_map::Matrix& H = mapOut[inputLayer_];
  const int rows = static_cast<int>(H.rows());
  const int cols = static_cast<int>(H.cols());
  const double res = mapOut.getResolution();

  int windowSize = static_cast<int>(std::round(windowLength_ / res));
  if (windowSize % 2 == 0) ++windowSize;
  const int half = std::max(1, windowSize / 2);

  // Ten NaN-masked summed-area tables, padded by one row/col of zeros so the
  // 4-corner lookup needs no bounds checks. double throughout: the second moments
  // of metric coordinates are large, and a stddev built from them in float loses
  // far too much precision.
  //
  // x, y are in METRES (index * resolution) with an arbitrary origin. The plane's
  // gradient (a, b) and the residuals are both invariant to that origin because we
  // solve in CENTRAL moments below, so the choice of origin does not matter.
  using Mat = Eigen::MatrixXd;
  Mat sN = Mat::Zero(rows + 1, cols + 1);
  Mat sX = Mat::Zero(rows + 1, cols + 1);
  Mat sY = Mat::Zero(rows + 1, cols + 1);
  Mat sZ = Mat::Zero(rows + 1, cols + 1);
  Mat sXX = Mat::Zero(rows + 1, cols + 1);
  Mat sXY = Mat::Zero(rows + 1, cols + 1);
  Mat sYY = Mat::Zero(rows + 1, cols + 1);
  Mat sXZ = Mat::Zero(rows + 1, cols + 1);
  Mat sYZ = Mat::Zero(rows + 1, cols + 1);
  Mat sZZ = Mat::Zero(rows + 1, cols + 1);

  for (int i = 0; i < rows; ++i) {
    const double y = static_cast<double>(i) * res;
    for (int j = 0; j < cols; ++j) {
      const double x = static_cast<double>(j) * res;
      const float v = H(i, j);
      const bool finite = std::isfinite(v);
      const double z = finite ? static_cast<double>(v) : 0.0;
      const double c = finite ? 1.0 : 0.0;
      const double xf = finite ? x : 0.0;
      const double yf = finite ? y : 0.0;

      // S(i+1, j+1) = value + S(i, j+1) + S(i+1, j) - S(i, j)
      sN(i + 1, j + 1)  = c           + sN(i, j + 1)  + sN(i + 1, j)  - sN(i, j);
      sX(i + 1, j + 1)  = xf          + sX(i, j + 1)  + sX(i + 1, j)  - sX(i, j);
      sY(i + 1, j + 1)  = yf          + sY(i, j + 1)  + sY(i + 1, j)  - sY(i, j);
      sZ(i + 1, j + 1)  = z           + sZ(i, j + 1)  + sZ(i + 1, j)  - sZ(i, j);
      sXX(i + 1, j + 1) = xf * xf     + sXX(i, j + 1) + sXX(i + 1, j) - sXX(i, j);
      sXY(i + 1, j + 1) = xf * yf     + sXY(i, j + 1) + sXY(i + 1, j) - sXY(i, j);
      sYY(i + 1, j + 1) = yf * yf     + sYY(i, j + 1) + sYY(i + 1, j) - sYY(i, j);
      sXZ(i + 1, j + 1) = xf * z      + sXZ(i, j + 1) + sXZ(i + 1, j) - sXZ(i, j);
      sYZ(i + 1, j + 1) = yf * z      + sYZ(i, j + 1) + sYZ(i + 1, j) - sYZ(i, j);
      sZZ(i + 1, j + 1) = z * z       + sZZ(i, j + 1) + sZZ(i + 1, j) - sZZ(i, j);
    }
  }

  mapOut.add(slopeLayer_);
  mapOut.add(stepLayer_);
  mapOut.add(stepAbsLayer_);
  mapOut.add(roughnessLayer_);
  grid_map::Matrix& Slope = mapOut[slopeLayer_];
  grid_map::Matrix& Step = mapOut[stepLayer_];
  grid_map::Matrix& StepAbs = mapOut[stepAbsLayer_];
  grid_map::Matrix& Rough = mapOut[roughnessLayer_];

  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto rect = [](const Mat& S, int i0, int j0, int i1, int j1) {
    return S(i1 + 1, j1 + 1) - S(i0, j1 + 1) - S(i1 + 1, j0) + S(i0, j0);
  };
  auto setNaN = [&](int i, int j) {
    Slope(i, j) = nan;
    Step(i, j) = nan;
    StepAbs(i, j) = nan;
    Rough(i, j) = nan;
  };

  for (int i = 0; i < rows; ++i) {
    const int i0 = std::max(0, i - half);
    const int i1 = std::min(rows - 1, i + half);
    const double yc = static_cast<double>(i) * res;
    for (int j = 0; j < cols; ++j) {
      const float zc = H(i, j);
      if (!std::isfinite(zc)) {
        setNaN(i, j);
        continue;
      }
      const int j0 = std::max(0, j - half);
      const int j1 = std::min(cols - 1, j + half);

      const double n = rect(sN, i0, j0, i1, j1);
      if (n < static_cast<double>(minPoints_)) {
        setNaN(i, j);
        continue;
      }

      const double Sx = rect(sX, i0, j0, i1, j1);
      const double Sy = rect(sY, i0, j0, i1, j1);
      const double Sz = rect(sZ, i0, j0, i1, j1);

      // Central moments. Solving the normal equations in raw global coordinates
      // would be badly conditioned (x, y run to tens of metres, so Sxx dwarfs the
      // signal); centring makes the 2x2 system well behaved.
      const double Sxx = rect(sXX, i0, j0, i1, j1) - Sx * Sx / n;
      const double Sxy = rect(sXY, i0, j0, i1, j1) - Sx * Sy / n;
      const double Syy = rect(sYY, i0, j0, i1, j1) - Sy * Sy / n;
      const double Sxz = rect(sXZ, i0, j0, i1, j1) - Sx * Sz / n;
      const double Syz = rect(sYZ, i0, j0, i1, j1) - Sy * Sz / n;
      const double Szz = rect(sZZ, i0, j0, i1, j1) - Sz * Sz / n;

      const double det = Sxx * Syy - Sxy * Sxy;
      // Degenerate support (all finite cells collinear): no unique plane.
      if (!(std::fabs(det) > 1e-12)) {
        setNaN(i, j);
        continue;
      }

      const double a = (Sxz * Syy - Syz * Sxy) / det;
      const double b = (Syz * Sxx - Sxz * Sxy) / det;
      const double xbar = Sx / n, ybar = Sy / n, zbar = Sz / n;
      const double xc = static_cast<double>(j) * res;

      // Plane height at this cell, written about the centroid so `c` never appears.
      const double zPlane = zbar + a * (xc - xbar) + b * (yc - ybar);
      const double step = static_cast<double>(zc) - zPlane;

      // RMS residual about the fitted plane, closed form from the same moments:
      //   sum((z - plane)^2)/n = (Szz - a*Sxz - b*Syz)/n
      const double var = std::max(0.0, (Szz - a * Sxz - b * Syz) / n);

      Slope(i, j) = static_cast<float>(std::atan(std::hypot(a, b)));
      Step(i, j) = static_cast<float>(step);
      StepAbs(i, j) = static_cast<float>(std::fabs(step));
      Rough(i, j) = static_cast<float>(std::sqrt(var));
    }
  }

  // Measured 0.79-1.50 ms on a full 160x160 map -- the ten integral images are O(N) to
  // build and O(1) per cell, so the 5x5 window costs no more than a 3x3 would. Against a
  // ~330 ms budget at the observed 3 Hz this is free, and it replaces three filters.
  ROS_DEBUG_THROTTLE(10.0, "[PlaneFitFilter] %.2f ms (%dx%d, window %d cells)",
                     (ros::WallTime::now() - t_start).toSec() * 1e3, rows, cols, windowSize);
  return true;
}

}  // namespace terrain_cost_adapter

template class terrain_cost_adapter::PlaneFitFilter<grid_map::GridMap>;

PLUGINLIB_EXPORT_CLASS(terrain_cost_adapter::PlaneFitFilter<grid_map::GridMap>,
                       filters::FilterBase<grid_map::GridMap>)
