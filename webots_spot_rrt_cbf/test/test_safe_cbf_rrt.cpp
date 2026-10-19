#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "safe_cbf_rrt/grid.hpp"
#include "safe_cbf_rrt/matlab_rng.hpp"
#include "safe_cbf_rrt/planner.hpp"
#include "webots_spot_rrt_cbf/field_adapter.hpp"

using namespace safe_cbf_rrt;
using namespace webots_spot_rrt_cbf;

namespace {

// 100 x 100 grid, obstacle block in the middle with a narrow gap near the bottom and a wide
// corridor at the top. h = clamp(distance to nearest obstacle or border / 15, 0, 1) with exact
// finite-difference gradients (per pixel).
struct Scene {
  Grid2D occ, h, dhdx, dhdy;
};

Scene makeScene() {
  const int n = 100;
  Scene s{Grid2D(n, n, 0.0), Grid2D(n, n, 0.0), Grid2D(n, n, 0.0), Grid2D(n, n, 0.0)};
  // wall at columns 48..52 (x = 49..53) from y = 1..70, open above y = 70
  for (int r = 0; r < 70; ++r)
    for (int c = 47; c < 52; ++c) s.occ.at(r, c) = 1.0;
  // distance transform (brute force, fine for 100x100)
  std::vector<std::pair<int, int>> obst;
  for (int r = 0; r < n; ++r)
    for (int c = 0; c < n; ++c)
      if (s.occ.at(r, c) > 0.5) obst.emplace_back(r, c);
  for (int r = 0; r < n; ++r) {
    for (int c = 0; c < n; ++c) {
      double d = std::min({static_cast<double>(r + 1), static_cast<double>(n - r),
                           static_cast<double>(c + 1), static_cast<double>(n - c)});
      for (const auto& o : obst) d = std::min(d, std::hypot(r - o.first, c - o.second));
      s.h.at(r, c) = s.occ.at(r, c) > 0.5 ? 0.0 : std::clamp(d / 15.0, 0.0, 1.0);
    }
  }
  const auto xs = linspace(1.0, n, n);
  matlabGradient(s.h, xs, xs, s.dhdx, s.dhdy);
  return s;
}

double minHAlong(const std::vector<Point>& path, const Grid2D& h) {
  BilinearInterpolant f(h);
  double m = 1e9;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    for (const double t : linspace(0.0, 1.0, 20)) {
      m = std::min(m, f(path[i][0] + t * (path[i + 1][0] - path[i][0]),
                        path[i][1] + t * (path[i + 1][1] - path[i][1])));
    }
  }
  return m;
}

}  // namespace

TEST(MatlabRand, MatchesNumpyRandomState) {
  MatlabRand r2(2);
  EXPECT_NEAR(r2(), 0.43599490214200376, 1e-15);
  EXPECT_NEAR(r2(), 0.025926231827891333, 1e-15);
}

TEST(Grid, BilinearMatchesLinearFunctionAndClamps) {
  Grid2D g(5, 7);
  for (int r = 0; r < g.ny; ++r)
    for (int c = 0; c < g.nx; ++c) g.at(r, c) = 2.0 * (c + 1) + 3.0 * (r + 1);
  BilinearInterpolant f(g);
  EXPECT_NEAR(f(2.5, 3.25), 2.0 * 2.5 + 3.0 * 3.25, 1e-12);
  EXPECT_NEAR(f(-3.0, 2.0), 2.0 * 1 + 3.0 * 2, 1e-12);
}

TEST(Adapter, WorldPixelRoundTripAndCellCentres) {
  GridGeometry g{361, 202, 0.05, -1.22, -4.98};
  // centre of cell (row 0, col 0) is pixel (1, 1)
  const Point p = worldToPixel(g, -1.22 + 0.025, -4.98 + 0.025);
  EXPECT_NEAR(p[0], 1.0, 1e-12);
  EXPECT_NEAR(p[1], 1.0, 1e-12);
  double wx, wy;
  pixelToWorld(g, {361.0, 202.0}, wx, wy);
  EXPECT_NEAR(wx, -1.22 + 360.5 * 0.05, 1e-12);
  EXPECT_NEAR(wy, -4.98 + 201.5 * 0.05, 1e-12);
  const Point q = worldToPixel(g, 3.3, 2.2);
  pixelToWorld(g, q, wx, wy);
  EXPECT_NEAR(wx, 3.3, 1e-12);
  EXPECT_NEAR(wy, 2.2, 1e-12);
}

TEST(Adapter, ResampleIdentityAndScale) {
  FieldData f;
  f.geometry = {4, 3, 0.1, 0.0, 0.0};
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 4; ++c) {
      f.h.push_back(2.0f * (c + 1) + (r + 1));
      f.dhdx.push_back(2.0f);
      f.dhdy.push_back(1.0f);
    }
  f.h_max = 2.0 * 4 + 3;
  Grid2D h, dx, dy;
  resampleField(f, f.geometry, 11.0, h, dx, dy);
  EXPECT_NEAR(h.at(2, 3), 11.0 / 11.0, 1e-6);
  EXPECT_NEAR(dx.at(1, 1), 2.0 / 11.0, 1e-6);
  // target grid shifted: cells outside the field are 0, inside interpolated
  GridGeometry t{3, 3, 0.1, 0.25, 0.0};  // centres at x = 0.3, 0.4, 0.5 (0.5 is outside 0..0.4)
  resampleField(f, t, 1.0, h, dx, dy);
  EXPECT_NEAR(h.at(0, 0), 8.0, 1e-6);  // x=0.3 -> halfway between cols 2 and 3 (7 and 9)
  EXPECT_NEAR(h.at(0, 2), 0.0, 1e-12);             // x = 0.5 outside
}

TEST(Adapter, OccupancyAndClearDisc) {
  GridGeometry g{20, 20, 0.1, 0.0, 0.0};
  std::vector<std::uint8_t> costs(400, 0);
  costs[5 * 20 + 5] = 253;
  costs[6 * 20 + 6] = 255;
  costs[7 * 20 + 7] = 100;
  Grid2D occ = buildOccupancy(costs.data(), g, 253, false);
  EXPECT_EQ(occ.at(5, 5), 1.0);
  EXPECT_EQ(occ.at(6, 6), 1.0);  // unknown blocked
  EXPECT_EQ(occ.at(7, 7), 0.0);
  occ = buildOccupancy(costs.data(), g, 253, true);
  EXPECT_EQ(occ.at(6, 6), 0.0);
  clearDisc(occ, g, 0.55, 0.55, 0.15);
  EXPECT_EQ(occ.at(5, 5), 0.0);
}

TEST(Planner, FindsCollisionFreePathAndTreeIsRecorded) {
  const Scene s = makeScene();
  Params p;
  p.doCbf = false;
  p.c = 1.0;
  p.stepSize = 3.0;
  p.neighborRadius = 8.0;
  p.maxIter = 8000;
  p.seed = 1;
  p.recordTree = true;
  const Result r = safeCbfRrtStar({10, 10}, {90, 10}, s.occ, s.h, s.dhdx, s.dhdy, p);
  ASSERT_TRUE(r.found);
  EXPECT_GE(r.path.size(), 2u);
  EXPECT_FALSE(r.treeEdges.empty());
  EXPECT_NEAR(r.path.front()[0], 10.0, 1e-9);
  EXPECT_NEAR(r.path.back()[0], 90.0, 1e-9);
  EdgeChecker chk(s.occ, s.h, s.dhdx, s.dhdy, p);
  for (std::size_t i = 0; i + 1 < r.path.size(); ++i) EXPECT_TRUE(chk.collisionFree(r.path[i], r.path[i + 1]));
}

TEST(Planner, TimeBudgetStopsSearch) {
  const Scene s = makeScene();
  Params p;
  p.doCbf = false;
  p.stepSize = 0.5;  // tiny steps: cannot reach in time
  p.maxIter = 100000000;
  p.maxTimeMs = 20.0;
  const Result r = safeCbfRrtStar({10, 10}, {90, 90}, s.occ, s.h, s.dhdx, s.dhdy, p);
  EXPECT_FALSE(r.found);
  EXPECT_TRUE(r.timedOut);
  EXPECT_LT(r.runtimeMs, 500.0);
}

// CBF edges enforce v/kappa clearance, so the minimum h along the path must be larger.
TEST(Planner, CbfPathKeepsMoreClearanceThanBase) {
  const Scene s = makeScene();
  double base_sum = 0.0, cbf_sum = 0.0;
  int n = 0;
  for (std::uint32_t seed = 1; seed <= 6; ++seed) {
    Params base;
    base.doCbf = false;
    base.c = 1.0;
    base.stepSize = 3.0;
    base.neighborRadius = 8.0;
    base.maxIter = 8000;
    base.seed = seed;
    Params cbf = base;
    cbf.doCbf = true;
    cbf.c = 0.2;
    cbf.v = 1.0;      // pixel-normalised fields: v [px/s], kappa [1/s]
    cbf.kappa = 0.1;  // keeps roughly v / kappa = 10 px from obstacles
    const Result rb = safeCbfRrtStar({10, 10}, {90, 10}, s.occ, s.h, s.dhdx, s.dhdy, base);
    const Result rc = safeCbfRrtStar({10, 10}, {90, 10}, s.occ, s.h, s.dhdx, s.dhdy, cbf);
    if (!rb.found || !rc.found) continue;
    base_sum += minHAlong(rb.path, s.h);
    cbf_sum += minHAlong(rc.path, s.h);
    ++n;
  }
  ASSERT_GE(n, 4);
  EXPECT_GT(cbf_sum / n, base_sum / n);
}

TEST(Planner, ShortcutKeepsValidityAndShortens) {
  const Scene s = makeScene();
  Params p;
  p.doCbf = true;
  p.c = 0.2;
  p.v = 1.0;
  p.kappa = 0.3;
  p.stepSize = 3.0;
  p.neighborRadius = 8.0;
  p.maxIter = 8000;
  p.seed = 3;
  const Result r = safeCbfRrtStar({10, 10}, {90, 10}, s.occ, s.h, s.dhdx, s.dhdy, p);
  ASSERT_TRUE(r.found);
  EdgeChecker chk(s.occ, s.h, s.dhdx, s.dhdy, p);
  const auto sc = shortcutPath(r.path, chk, true, 0.9);
  EXPECT_LE(sc.size(), r.path.size());
  EXPECT_EQ(sc.front(), r.path.front());
  EXPECT_EQ(sc.back(), r.path.back());
  for (std::size_t i = 0; i + 1 < sc.size(); ++i) EXPECT_TRUE(chk.valid(sc[i], sc[i + 1]));
  const auto dense = resamplePath(sc, 1.0);
  for (std::size_t i = 0; i + 1 < dense.size(); ++i)
    EXPECT_LE(std::hypot(dense[i + 1][0] - dense[i][0], dense[i + 1][1] - dense[i][1]), 1.0 + 1e-9);
}
