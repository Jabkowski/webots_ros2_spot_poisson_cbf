#pragma once

// ROS-free helpers between Nav2 map/costmap geometry and the pixel-based
// Safe CBF-RRT* core. Kept free of ROS types so they are unit-testable.
//
// Pixel convention of the core (MATLAB): 1-based, cell (row r, col c) has centre
// (x, y) = (c + 1, r + 1). World -> pixel for a grid with origin (ox, oy) and
// resolution res:  px = (wx - ox) / res + 0.5.

#include <cstdint>
#include <vector>

#include "safe_cbf_rrt/grid.hpp"
#include "safe_cbf_rrt/planner.hpp"

namespace webots_spot_rrt_cbf {

using safe_cbf_rrt::Grid2D;
using safe_cbf_rrt::Point;

struct GridGeometry {
  int nx = 0;
  int ny = 0;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
};

Point worldToPixel(const GridGeometry& g, double wx, double wy);
void pixelToWorld(const GridGeometry& g, const Point& p, double& wx, double& wy);

/// Raw safety field as received from pinn_safety (row-major, row = y).
struct FieldData {
  GridGeometry geometry;
  std::vector<float> h;
  std::vector<float> dhdx;  ///< per metre
  std::vector<float> dhdy;  ///< per metre
  double h_max = 0.0;

  bool valid() const {
    const std::size_t n = static_cast<std::size_t>(geometry.nx) * geometry.ny;
    return n > 0 && h.size() == n && dhdx.size() == n && dhdy.size() == n;
  }
};

/// Bilinear resample of the field onto the target grid (by world coordinates) and
/// divide h, dhdx, dhdy by h_scale. Target cells outside the field get h = 0.
void resampleField(const FieldData& field, const GridGeometry& target, double h_scale, Grid2D& h,
                   Grid2D& dhdx, Grid2D& dhdy);

/// occupancy = 1 for costmap cells with cost >= occupied_threshold (and for
/// NO_INFORMATION=255 unless allow_unknown).
Grid2D buildOccupancy(const std::uint8_t* costmap_chars, const GridGeometry& g,
                      std::uint8_t occupied_threshold, bool allow_unknown);

/// Set occupancy to 0 inside a disc (world metres) around a world point.
void clearDisc(Grid2D& occupancy, const GridGeometry& g, double wx, double wy, double radius_m);

/// Greedy shortcutting. A shortcut is accepted when the straight edge passes the
/// planner's own validity check (collision + CBF) and, with CBF on, its
/// length-weighted mean h is at least `h_tolerance` times that of the replaced part.
std::vector<Point> shortcutPath(const std::vector<Point>& path,
                                const safe_cbf_rrt::EdgeChecker& checker, bool do_cbf,
                                double h_tolerance);

/// Resample a polyline to (at most) `step` spacing, keeping all original vertices.
std::vector<Point> resamplePath(const std::vector<Point>& path, double step);

}  // namespace webots_spot_rrt_cbf
