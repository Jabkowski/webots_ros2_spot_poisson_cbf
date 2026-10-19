#pragma once

// Vendored (trimmed) from rrt_star_cbf/cpp_version/include/safe_cbf_rrt/matlab_rng.hpp.
// The obstacle generator used by the original example is not needed here.

#include <cstdint>
#include <random>

namespace safe_cbf_rrt {

/// Reproduces MATLAB `rng(seed, 'twister'); rand` (and numpy
/// `RandomState(seed).rand()`): MT19937 seeded with init_genrand, doubles
/// built with genrand_res53.
class MatlabRand {
 public:
  explicit MatlabRand(std::uint32_t seed) : gen_(seed) {}

  double operator()() {
    const std::uint32_t a = gen_() >> 5;
    const std::uint32_t b = gen_() >> 6;
    return (a * 67108864.0 + b) * (1.0 / 9007199254740992.0);
  }

 private:
  std::mt19937 gen_;
};

}  // namespace safe_cbf_rrt
