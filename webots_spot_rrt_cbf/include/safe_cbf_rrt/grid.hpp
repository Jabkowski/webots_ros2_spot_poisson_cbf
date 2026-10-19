#pragma once

#include <cstddef>
#include <vector>

namespace safe_cbf_rrt {

/// Dense row-major grid with Ny rows (y) and Nx columns (x), matching a MATLAB
/// Ny x Nx matrix. Row r corresponds to pixel coordinate y = r + 1 and column c
/// to x = c + 1 (MATLAB 1-based pixel coordinates).
struct Grid2D {
  int ny = 0;
  int nx = 0;
  std::vector<double> data;

  Grid2D() = default;
  Grid2D(int ny_, int nx_, double value = 0.0)
      : ny(ny_), nx(nx_), data(static_cast<std::size_t>(ny_) * nx_, value) {}

  double& at(int r, int c) { return data[static_cast<std::size_t>(r) * nx + c]; }
  double at(int r, int c) const { return data[static_cast<std::size_t>(r) * nx + c]; }
};

/// Equivalent of griddedInterpolant({1:Ny, 1:Nx}, grid, 'linear', 'nearest'):
/// bilinear inside the grid, clamped to the border outside.
class BilinearInterpolant {
 public:
  explicit BilinearInterpolant(const Grid2D& grid) : grid_(&grid) {}

  /// Evaluate at 1-based pixel coordinates (x, y).
  double operator()(double x, double y) const;

 private:
  const Grid2D* grid_;
};

/// MATLAB gradient(F, xs, ys) for a grid sampled at coordinate vectors xs (size
/// Nx) and ys (size Ny): central differences inside, one-sided at the edges.
/// NaNs propagate.
void matlabGradient(const Grid2D& f, const std::vector<double>& xs,
                    const std::vector<double>& ys, Grid2D& dfdx, Grid2D& dfdy);

/// MATLAB linspace(a, b, n).
std::vector<double> linspace(double a, double b, int n);

}  // namespace safe_cbf_rrt
