#include "safe_cbf_rrt/planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "safe_cbf_rrt/matlab_rng.hpp"

namespace safe_cbf_rrt {

namespace {

struct Node {
  Point pos;
  int parent;  // -1 for root
  double cost;
};

double dist(const Point& a, const Point& b) { return std::hypot(b[0] - a[0], b[1] - a[1]); }

}  // namespace

bool EdgeChecker::collisionFree(const Point& a, const Point& b) const {
  const auto xs = linspace(a[0], b[0], p_.samplesPerEdge);
  const auto ys = linspace(a[1], b[1], p_.samplesPerEdge);
  for (std::size_t k = 0; k < xs.size(); ++k) {
    if (focc_(xs[k], ys[k]) > 0.5) return false;
  }
  return true;
}

bool EdgeChecker::cbfSafe(const Point& a, const Point& b) const {
  if (!p_.doCbf) return true;
  if (dist(a, b) < 1e-9) return true;
  const double theta = std::atan2(b[1] - a[1], b[0] - a[0]);
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  const auto xs = linspace(a[0], b[0], p_.samplesPerEdge);
  const auto ys = linspace(a[1], b[1], p_.samplesPerEdge);
  for (std::size_t k = 0; k < xs.size(); ++k) {
    const double h_prim = p_.v * (fdhx_(xs[k], ys[k]) * ct + fdhy_(xs[k], ys[k]) * st);
    if (!(h_prim + p_.kappa * fh_(xs[k], ys[k]) > 0.0)) return false;
  }
  return true;
}

double EdgeChecker::meanSampledH(const Point& a, const Point& b) const {
  const auto xs = linspace(a[0], b[0], p_.samplesPerEdge);
  const auto ys = linspace(a[1], b[1], p_.samplesPerEdge);
  double sum = 0.0;
  for (std::size_t k = 0; k < xs.size(); ++k) sum += fh_(xs[k], ys[k]);
  return std::clamp(sum / static_cast<double>(xs.size()), 0.0, 1.0);
}

double EdgeChecker::edgeCost(const Point& a, const Point& b) const {
  const double L = dist(a, b);
  if (!p_.doCbf) return L;
  const double mean_h = meanSampledH(a, b);
  const double safety = mean_h > 0.0 ? 1.0 / mean_h : std::numeric_limits<double>::infinity();
  return p_.c * L + (1.0 - p_.c) * safety;
}

namespace {

bool outOfBounds(const Point& p, int nx, int ny) {
  return p[0] < 1.0 || p[1] < 1.0 || p[0] > nx || p[1] > ny;
}

}  // namespace

Result safeCbfRrtStar(const Point& start, const Point& goal, const Grid2D& occupancy,
                      const Grid2D& h, const Grid2D& dhdx, const Grid2D& dhdy,
                      const Params& params) {
  const auto t0 = std::chrono::steady_clock::now();
  const int nx = occupancy.nx;
  const int ny = occupancy.ny;
  for (const Grid2D* g : {&h, &dhdx, &dhdy}) {
    if (g->nx != nx || g->ny != ny) throw std::invalid_argument("grid size mismatch");
  }
  if (outOfBounds(start, nx, ny)) throw std::invalid_argument("Start out of grid bounds");
  if (outOfBounds(goal, nx, ny)) throw std::invalid_argument("Goal out of grid bounds");

  EdgeChecker check(occupancy, h, dhdx, dhdy, params);
  MatlabRand rand(params.seed);

  std::vector<Node> nodes;
  nodes.reserve(static_cast<std::size_t>(params.maxIter) + 2);
  nodes.push_back({start, -1, 0.0});

  bool found = false;
  int goal_idx = -1;
  int accepted = 0;
  int iter = 0;
  std::vector<int> neighbors;

  bool timed_out = false;
  for (iter = 1; iter <= params.maxIter; ++iter) {
    if (params.maxTimeMs > 0.0) {
      const double elapsed_ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      if (elapsed_ms > params.maxTimeMs) {
        timed_out = true;
        break;
      }
    }
    Point p_rand;
    if (rand() < params.goalBias) {
      p_rand = goal;
    } else {
      const double rx = rand();
      const double ry = rand();
      p_rand = {1.0 + rx * (nx - 1), 1.0 + ry * (ny - 1)};
    }

    int idx_near = 0;
    double best_d = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      const double d = dist(nodes[i].pos, p_rand);
      if (d < best_d) {
        best_d = d;
        idx_near = static_cast<int>(i);
      }
    }
    const Point p_near = nodes[idx_near].pos;

    const double nd = dist(p_near, p_rand);
    if (nd < 1e-9) continue;
    const double step = std::min(params.stepSize, nd);
    const Point p_new = {p_near[0] + step * (p_rand[0] - p_near[0]) / nd,
                         p_near[1] + step * (p_rand[1] - p_near[1]) / nd};

    if (!check.valid(p_near, p_new)) continue;

    // Neighbor set is computed before inserting p_new (as in the MATLAB code).
    neighbors.clear();
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (dist(nodes[i].pos, p_new) <= params.neighborRadius) neighbors.push_back(static_cast<int>(i));
    }

    int best_parent = idx_near;
    double best_cost = nodes[idx_near].cost + check.edgeCost(p_near, p_new);
    for (int j : neighbors) {
      const Point& pj = nodes[j].pos;
      if (!check.valid(pj, p_new)) continue;
      const double cost = nodes[j].cost + check.edgeCost(pj, p_new);
      if (cost < best_cost) {
        best_cost = cost;
        best_parent = j;
      }
    }

    ++accepted;
    const int new_idx = static_cast<int>(nodes.size());
    nodes.push_back({p_new, best_parent, best_cost});

    // Rewire without propagating cost changes to descendants (MATLAB parity).
    for (int j : neighbors) {
      const Point pj = nodes[j].pos;
      if (!check.valid(p_new, pj)) continue;
      const double cost = nodes[new_idx].cost + check.edgeCost(p_new, pj);
      if (cost < nodes[j].cost) {
        nodes[j].parent = new_idx;
        nodes[j].cost = cost;
      }
    }

    if (dist(p_new, goal) <= params.stepSize && check.valid(p_new, goal)) {
      nodes.push_back({goal, new_idx, nodes[new_idx].cost + check.edgeCost(p_new, goal)});
      goal_idx = static_cast<int>(nodes.size()) - 1;
      found = true;
      break;
    }
  }
  if (iter > params.maxIter) iter = params.maxIter;

  Result res;
  res.iterations = iter;
  res.rejectionRatio = iter > 0 ? static_cast<double>(iter - accepted) / iter : 0.0;
  res.numNodes = static_cast<int>(nodes.size());
  res.found = found;
  res.timedOut = timed_out && !found;
  if (params.recordTree) {
    res.treeEdges.reserve(nodes.size());
    for (const Node& n : nodes) {
      if (n.parent >= 0) res.treeEdges.emplace_back(nodes[n.parent].pos, n.pos);
    }
  }

  if (!found) {
    res.meanHPath = std::numeric_limits<double>::quiet_NaN();
  } else {
    for (int i = goal_idx; i >= 0; i = nodes[i].parent) res.path.push_back(nodes[i].pos);
    std::reverse(res.path.begin(), res.path.end());

    double weighted = 0.0;
    for (std::size_t i = 0; i + 1 < res.path.size(); ++i) {
      const Point& a = res.path[i];
      const Point& b = res.path[i + 1];
      const double seg = dist(a, b);
      res.pathLength += seg;
      if (seg > 0.0) {
        const auto xs = linspace(a[0], b[0], params.samplesPerEdge);
        const auto ys = linspace(a[1], b[1], params.samplesPerEdge);
        double sum = 0.0;
        for (std::size_t k = 0; k < xs.size(); ++k) sum += check.rawH(xs[k], ys[k]);
        weighted += sum / static_cast<double>(xs.size()) * seg;
      }
    }
    if (res.path.size() < 2) {
      res.meanHPath = std::clamp(check.rawH(res.path[0][0], res.path[0][1]), 0.0, 1.0);
    } else if (res.pathLength > 0.0) {
      res.meanHPath = std::clamp(weighted / res.pathLength, 0.0, 1.0);
    } else {
      res.meanHPath = std::numeric_limits<double>::quiet_NaN();
    }
  }

  res.runtimeMs =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return res;
}

}  // namespace safe_cbf_rrt
