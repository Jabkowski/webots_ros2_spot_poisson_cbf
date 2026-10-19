#include "safe_cbf_rrt/grid.hpp"

#include <algorithm>
#include <cmath>

namespace safe_cbf_rrt {

double BilinearInterpolant::operator()(double x, double y) const {
  const Grid2D& g = *grid_;
  // 0-based continuous indices, clamped ('nearest' extrapolation).
  double cx = std::clamp(x - 1.0, 0.0, static_cast<double>(g.nx - 1));
  double cy = std::clamp(y - 1.0, 0.0, static_cast<double>(g.ny - 1));

  int c0 = std::min(static_cast<int>(std::floor(cx)), std::max(g.nx - 2, 0));
  int r0 = std::min(static_cast<int>(std::floor(cy)), std::max(g.ny - 2, 0));
  int c1 = std::min(c0 + 1, g.nx - 1);
  int r1 = std::min(r0 + 1, g.ny - 1);
  double tx = cx - c0;
  double ty = cy - r0;

  double v00 = g.at(r0, c0);
  double v01 = g.at(r0, c1);
  double v10 = g.at(r1, c0);
  double v11 = g.at(r1, c1);
  return (1.0 - ty) * ((1.0 - tx) * v00 + tx * v01) + ty * ((1.0 - tx) * v10 + tx * v11);
}

void matlabGradient(const Grid2D& f, const std::vector<double>& xs,
                    const std::vector<double>& ys, Grid2D& dfdx, Grid2D& dfdy) {
  const int ny = f.ny;
  const int nx = f.nx;
  dfdx = Grid2D(ny, nx, 0.0);
  dfdy = Grid2D(ny, nx, 0.0);

  for (int r = 0; r < ny; ++r) {
    for (int c = 0; c < nx; ++c) {
      if (nx < 2) {
        dfdx.at(r, c) = 0.0;
      } else if (c == 0) {
        dfdx.at(r, c) = (f.at(r, 1) - f.at(r, 0)) / (xs[1] - xs[0]);
      } else if (c == nx - 1) {
        dfdx.at(r, c) = (f.at(r, nx - 1) - f.at(r, nx - 2)) / (xs[nx - 1] - xs[nx - 2]);
      } else {
        dfdx.at(r, c) = (f.at(r, c + 1) - f.at(r, c - 1)) / (xs[c + 1] - xs[c - 1]);
      }

      if (ny < 2) {
        dfdy.at(r, c) = 0.0;
      } else if (r == 0) {
        dfdy.at(r, c) = (f.at(1, c) - f.at(0, c)) / (ys[1] - ys[0]);
      } else if (r == ny - 1) {
        dfdy.at(r, c) = (f.at(ny - 1, c) - f.at(ny - 2, c)) / (ys[ny - 1] - ys[ny - 2]);
      } else {
        dfdy.at(r, c) = (f.at(r + 1, c) - f.at(r - 1, c)) / (ys[r + 1] - ys[r - 1]);
      }
    }
  }
}

std::vector<double> linspace(double a, double b, int n) {
  std::vector<double> out;
  if (n <= 0) return out;
  if (n == 1) return {b};
  out.resize(n);
  const double step = (b - a) / (n - 1);
  for (int i = 0; i < n; ++i) out[i] = a + i * step;
  out[n - 1] = b;
  return out;
}

}  // namespace safe_cbf_rrt
