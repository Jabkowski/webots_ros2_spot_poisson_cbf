#include "webots_spot_rrt_cbf/field_adapter.hpp"

#include <algorithm>
#include <cmath>

namespace webots_spot_rrt_cbf {

namespace {

double dist(const Point& a, const Point& b) { return std::hypot(b[0] - a[0], b[1] - a[1]); }

// Bilinear sample of a row-major float grid at continuous 0-based cell coordinates.
double sampleBilinear(const std::vector<float>& d, int nx, int ny, double fx, double fy) {
  fx = std::clamp(fx, 0.0, static_cast<double>(nx - 1));
  fy = std::clamp(fy, 0.0, static_cast<double>(ny - 1));
  const int c0 = std::min(static_cast<int>(std::floor(fx)), std::max(nx - 2, 0));
  const int r0 = std::min(static_cast<int>(std::floor(fy)), std::max(ny - 2, 0));
  const int c1 = std::min(c0 + 1, nx - 1);
  const int r1 = std::min(r0 + 1, ny - 1);
  const double tx = fx - c0;
  const double ty = fy - r0;
  const auto at = [&](int r, int c) {
    return static_cast<double>(d[static_cast<std::size_t>(r) * nx + c]);
  };
  return (1.0 - ty) * ((1.0 - tx) * at(r0, c0) + tx * at(r0, c1)) +
         ty * ((1.0 - tx) * at(r1, c0) + tx * at(r1, c1));
}

}  // namespace

Point worldToPixel(const GridGeometry& g, double wx, double wy) {
  return {(wx - g.origin_x) / g.resolution + 0.5, (wy - g.origin_y) / g.resolution + 0.5};
}

void pixelToWorld(const GridGeometry& g, const Point& p, double& wx, double& wy) {
  wx = g.origin_x + (p[0] - 0.5) * g.resolution;
  wy = g.origin_y + (p[1] - 0.5) * g.resolution;
}

void resampleField(const FieldData& field, const GridGeometry& target, double h_scale, Grid2D& h,
                   Grid2D& dhdx, Grid2D& dhdy) {
  h = Grid2D(target.ny, target.nx, 0.0);
  dhdx = Grid2D(target.ny, target.nx, 0.0);
  dhdy = Grid2D(target.ny, target.nx, 0.0);
  if (!field.valid() || h_scale <= 0.0) return;

  const GridGeometry& fg = field.geometry;
  const double inv_scale = 1.0 / h_scale;
  for (int r = 0; r < target.ny; ++r) {
    const double wy = target.origin_y + (r + 0.5) * target.resolution;
    const double fy = (wy - fg.origin_y) / fg.resolution - 0.5;
    if (fy < -0.5 || fy > fg.ny - 0.5) continue;
    for (int c = 0; c < target.nx; ++c) {
      const double wx = target.origin_x + (c + 0.5) * target.resolution;
      const double fx = (wx - fg.origin_x) / fg.resolution - 0.5;
      if (fx < -0.5 || fx > fg.nx - 0.5) continue;
      h.at(r, c) = std::max(0.0, sampleBilinear(field.h, fg.nx, fg.ny, fx, fy)) * inv_scale;
      dhdx.at(r, c) = sampleBilinear(field.dhdx, fg.nx, fg.ny, fx, fy) * inv_scale;
      dhdy.at(r, c) = sampleBilinear(field.dhdy, fg.nx, fg.ny, fx, fy) * inv_scale;
    }
  }
}

Grid2D buildOccupancy(const std::uint8_t* costmap_chars, const GridGeometry& g,
                      std::uint8_t occupied_threshold, bool allow_unknown) {
  Grid2D occ(g.ny, g.nx, 0.0);
  const std::size_t n = static_cast<std::size_t>(g.nx) * g.ny;
  for (std::size_t i = 0; i < n; ++i) {
    const std::uint8_t cost = costmap_chars[i];
    const bool unknown = cost == 255;
    if (unknown) {
      occ.data[i] = allow_unknown ? 0.0 : 1.0;
    } else {
      occ.data[i] = cost >= occupied_threshold ? 1.0 : 0.0;
    }
  }
  return occ;
}

void clearDisc(Grid2D& occupancy, const GridGeometry& g, double wx, double wy, double radius_m) {
  if (radius_m <= 0.0) return;
  const Point p = worldToPixel(g, wx, wy);
  const double rpx = radius_m / g.resolution;
  const int c_lo = std::max(0, static_cast<int>(std::floor(p[0] - 1.0 - rpx)));
  const int c_hi = std::min(g.nx - 1, static_cast<int>(std::ceil(p[0] - 1.0 + rpx)));
  const int r_lo = std::max(0, static_cast<int>(std::floor(p[1] - 1.0 - rpx)));
  const int r_hi = std::min(g.ny - 1, static_cast<int>(std::ceil(p[1] - 1.0 + rpx)));
  for (int r = r_lo; r <= r_hi; ++r) {
    for (int c = c_lo; c <= c_hi; ++c) {
      if (std::hypot((c + 1.0) - p[0], (r + 1.0) - p[1]) <= rpx) occupancy.at(r, c) = 0.0;
    }
  }
}

std::vector<Point> shortcutPath(const std::vector<Point>& path,
                                const safe_cbf_rrt::EdgeChecker& checker, bool do_cbf,
                                double h_tolerance) {
  if (path.size() < 3) return path;
  std::vector<Point> out;
  out.push_back(path.front());
  std::size_t i = 0;
  const std::size_t last = path.size() - 1;
  while (i < last) {
    std::size_t best = i + 1;
    for (std::size_t j = last; j > i + 1; --j) {
      if (!checker.valid(path[i], path[j])) continue;
      if (do_cbf) {
        double len = 0.0;
        double weighted = 0.0;
        for (std::size_t k = i; k < j; ++k) {
          const double seg = dist(path[k], path[k + 1]);
          len += seg;
          weighted += checker.meanSampledH(path[k], path[k + 1]) * seg;
        }
        const double old_h = len > 0.0 ? weighted / len : 0.0;
        const double new_h = checker.meanSampledH(path[i], path[j]);
        if (new_h < h_tolerance * old_h) continue;
      }
      best = j;
      break;
    }
    out.push_back(path[best]);
    i = best;
  }
  return out;
}

std::vector<Point> resamplePath(const std::vector<Point>& path, double step) {
  if (path.size() < 2 || step <= 0.0) return path;
  std::vector<Point> out;
  out.push_back(path.front());
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Point& a = path[i];
    const Point& b = path[i + 1];
    const double len = dist(a, b);
    const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / n;
      out.push_back({a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])});
    }
  }
  return out;
}

}  // namespace webots_spot_rrt_cbf
