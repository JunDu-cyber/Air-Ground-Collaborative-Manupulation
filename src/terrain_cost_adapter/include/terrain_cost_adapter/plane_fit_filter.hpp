// ============================================================================
// PlaneFitFilter — slope, step and roughness from a local least-squares plane.
//
// Replaces the "height above the local minimum" obstacle metric, which never
// subtracts the terrain slope and therefore cannot tell a ramp from a wall.
// With resolution 0.25 m and a 0.75 m ground radius the old window was 7x7
// cells (half-diagonal 1.06 m), so on a plane of slope t the window minimum sat
// at the downhill corner and the metric read
//
//     cost ~= 1.06 * tan(t)   ->   > obstacleHeightThre (0.15 m) once t > 8.1 deg
//
// i.e. EVERY slope past ~8 deg was a solid wall, though the Husky climbs 25-30.
// A min-filter is also blind to negative obstacles: a ditch's own cells become
// the ground datum, so a hole reads as perfectly flat and traversable.
//
// Here we fit z = a*x + b*y + c over the window and measure against THAT plane:
//
//   slope     = atan(sqrt(a^2 + b^2))            exact plane slope
//   step      = z(centre) - plane(centre)        SIGNED residual: + = protrusion
//                                                (wall, rock, curb), - = depression
//                                                (ditch, drop-off). Slope-invariant
//                                                by construction, so a smooth ramp
//                                                of ANY angle has step ~ 0.
//   roughness = sqrt((Szz' - a*Sxz' - b*Syz')/n) RMS residual ABOUT THE PLANE, i.e.
//                                                slope-corrected. The old
//                                                stddev-of-elevation reported ~0.05 m
//                                                on a perfectly smooth 20 deg slope,
//                                                purely from the tilt.
//
// All three come out of ONE fit, so this single filter replaces the previous
// NormalVectorsFilter + MathExpressionFilter(slope) + RoughnessFilter chain.
//
// Cost: ten NaN-masked summed-area tables (integral images) — O(N) to build,
// O(1) per cell, independent of window size, no allocation, no expression
// parsing. Same structure as the RoughnessFilter it supersedes.
// ============================================================================
#pragma once

#include <string>

#include <filters/filter_base.hpp>
#include <grid_map_core/grid_map_core.hpp>

namespace terrain_cost_adapter {

template <typename T>
class PlaneFitFilter : public filters::FilterBase<T> {
 public:
  PlaneFitFilter();
  virtual ~PlaneFitFilter();

  virtual bool configure();

  //! Fits a plane per cell over a window_length square of `input_layer` and writes
  //! the slope / step / step_abs / roughness layers. A cell is NaN in every output
  //! when its centre is NaN, when the window holds fewer than min_points finite
  //! cells, or when the fit is degenerate (collinear support). The adapter already
  //! skips NaN cells as unobserved, so those cases fall through as "unknown"
  //! rather than as free space.
  virtual bool update(const T& mapIn, T& mapOut);

 private:
  std::string inputLayer_;
  std::string slopeLayer_;
  std::string stepLayer_;
  std::string stepAbsLayer_;
  std::string roughnessLayer_;

  //! Side length of the square fitting window, in metres.
  double windowLength_{0.75};
  //! Minimum finite cells in a window for the fit to be trusted. 3 is the algebraic
  //! minimum for a plane; 6 keeps it from chasing noise on sparse support.
  int minPoints_{6};
};

}  // namespace terrain_cost_adapter
