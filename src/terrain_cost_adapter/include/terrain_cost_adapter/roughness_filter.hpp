// ============================================================================
// RoughnessFilter — local surface roughness (stddev of elevation) for grid_map.
//
// Drop-in replacement for
//   gridMapFilters/SlidingWindowMathExpressionFilter
//     expression: sqrt(sumOfFinites(square(x - meanOfFinites(x))) / numberOfFinites(x))
// which re-parses that expression string and allocates a submap FOR EVERY CELL.
// On a filled 160x160 map that collapsed elevation_mapping's postprocessor from
// 4 Hz to 0.03 Hz and starved the CMU localPlanner of /terrain_map.
//
// Here the same population stddev is computed from three NaN-masked summed-area
// tables (integral images): O(N) to build, O(1) per cell, independent of window
// size, with no allocation and no expression parsing.
//
//   n  = count of finite cells in the window
//   S1 = sum of finite elevations
//   S2 = sum of squared finite elevations
//   var = S2/n - (S1/n)^2      -> roughness = sqrt(var)
// ============================================================================
#pragma once

#include <string>

#include <filters/filter_base.hpp>
#include <grid_map_core/grid_map_core.hpp>

namespace terrain_cost_adapter {

template <typename T>
class RoughnessFilter : public filters::FilterBase<T> {
 public:
  RoughnessFilter();
  virtual ~RoughnessFilter();

  virtual bool configure();

  //! Writes `output_layer` = stddev of `input_layer` over a window_length square.
  //! Cells whose centre is NaN stay NaN (matches compute_empty_cells: false);
  //! windows are clipped at the map border (matches edge_handling: crop).
  virtual bool update(const T& mapIn, T& mapOut);

 private:
  std::string inputLayer_;
  std::string outputLayer_;
  //! Side length of the square window, in metres.
  double windowLength_{0.5};
};

}  // namespace terrain_cost_adapter
