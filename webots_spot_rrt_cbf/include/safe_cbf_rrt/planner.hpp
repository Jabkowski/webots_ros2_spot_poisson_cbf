#pragma once

// Vendored from rrt_star_cbf/cpp_version (see VENDORED_SOURCES.md5 for the origin files).
// Local changes (all additive, planning behaviour unchanged when new fields keep defaults):
//   - Params::maxTimeMs   wall-clock planning budget (0 = unlimited)
//   - Params::recordTree  store accepted tree edges in Result::treeEdges for RViz
//   - EdgeChecker moved from the .cpp into this header so post-processing
//     (shortcutting) can reuse the exact collision/CBF/cost rules of the planner.

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

#include "safe_cbf_rrt/grid.hpp"

namespace safe_cbf_rrt {

using Point = std::array<double, 2>;  // [x, y] in 1-based pixel coordinates

struct Params {
  double c = 0.5;              ///< weight between length and safety (do_cbf only)
  int maxIter = 10000;
  double stepSize = 2.0;
  double neighborRadius = 10.0;
  int samplesPerEdge = 20;     ///< samples for collision & CBF checks
  double kappa = 5.0;          ///< CBF class-K gain
  double v = 0.0;              ///< speed used in the CBF derivative term
  bool doCbf = true;
  double goalBias = 0.05;
  std::uint32_t seed = 0;
  double maxTimeMs = 0.0;      ///< stop searching after this many ms (0 = unlimited)
  bool recordTree = false;     ///< fill Result::treeEdges
};

struct Result {
  std::vector<Point> path;     ///< start -> goal, empty if not found
  double pathLength = 0.0;
  double meanHPath = 0.0;      ///< length-weighted mean h, NaN if no path
  double rejectionRatio = 0.0;
  int iterations = 0;
  int numNodes = 0;
  bool found = false;
  bool timedOut = false;
  double runtimeMs = 0.0;
  std::vector<std::pair<Point, Point>> treeEdges;  ///< parent -> child, if recordTree
};

/// Edge validity (collision + CBF) and cost, exactly as used inside safeCbfRrtStar.
/// Holds references to the grids and params; they must outlive the checker.
class EdgeChecker {
 public:
  EdgeChecker(const Grid2D& occ, const Grid2D& h, const Grid2D& dhdx, const Grid2D& dhdy,
              const Params& p)
      : focc_(occ), fh_(h), fdhx_(dhdx), fdhy_(dhdy), p_(p) {}

  bool collisionFree(const Point& a, const Point& b) const;
  bool cbfSafe(const Point& a, const Point& b) const;
  bool valid(const Point& a, const Point& b) const { return collisionFree(a, b) && cbfSafe(a, b); }
  double meanSampledH(const Point& a, const Point& b) const;
  double edgeCost(const Point& a, const Point& b) const;
  double rawH(double x, double y) const { return fh_(x, y); }

 private:
  BilinearInterpolant focc_, fh_, fdhx_, fdhy_;
  const Params& p_;
};

/// Port of SafeCBFRRTStar.m. Grids are Ny x Nx (row = y); occupancy uses
/// 1 = occupied. dhdx/dhdy are the gradient grids of h.
Result safeCbfRrtStar(const Point& start, const Point& goal, const Grid2D& occupancy,
                      const Grid2D& h, const Grid2D& dhdx, const Grid2D& dhdy,
                      const Params& params);

}  // namespace safe_cbf_rrt
