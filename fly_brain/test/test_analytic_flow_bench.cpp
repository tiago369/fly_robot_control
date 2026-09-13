// M3a: analytic ground-truth optic-flow bench. Validates
// fly_brain::ReichardtEmdArray's pooling math against known camera egomotion
// BEFORE trusting it on live camera frames (M3b). Not a gtest - matches
// test_algorithms.cpp's plain-assert style (no rclcpp/controller_interface
// dependency, runs standalone via g++/colcon test).
//
// Two synthetic scenarios, both driven by the real pinhole-camera optic-flow
// equations (Longuet-Higgins & Prazdny 1980) rather than an ad hoc phase
// drift, per the M3 brief:
//
//   1. Pure yaw rotation (camera angular velocity omega_y about the
//      world-vertical axis, zero translation) against a textured panorama.
//      For a pinhole camera, rotational flow's horizontal component is
//      u_r = -(1 + x^2) * omega_y (x = normalized image coordinate) - SAME
//      SIGN for every x (no flip at image center), i.e. yaw produces a
//      whole-field, uniform-direction horizontal sweep.
//   2. Pure forward translation ("looming", camera velocity Vz along its own
//      forward axis, zero rotation) against a frontal textured wall at known
//      depth Z. Translational flow's horizontal component is
//      u_t = x * Vz / Z - ANTISYMMETRIC in x (flips sign left/right of
//      center) - a classic radially-expanding looming field.
//
// This distinction matters: `ReichardtEmdArray::update()`'s OpticFlowEstimate
// pools r_horiz TWO ways - antisymmetric-sign_x-weighted (field name
// "yaw_rate") and plain-summed/same-sign (field name "forward_drift"). Naively
// one would expect "yaw_rate" (antisymmetric pooling) to respond to yaw
// rotation. The bench below measures which field ACTUALLY correlates with
// which known egomotion, rather than assuming the field names are correct -
// see the printed result and NOTES.md's "## M3" section for what this found
// (short version: it's swapped - "forward_drift"'s same-sign pooling is what
// tracks yaw, "yaw_rate"'s antisymmetric pooling is what tracks looming).
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#include "fly_brain/reichardt_emd.hpp"

using fly_brain::OpticFlowEstimate;
using fly_brain::ReichardtEmdArray;

namespace {

constexpr std::size_t kW = 32;
constexpr std::size_t kH = 32;
constexpr double kFovDeg = 90.0;
const double kFocal = (static_cast<double>(kW) / 2.0) / std::tan(kFovDeg * M_PI / 180.0 / 2.0);
const double kCx = (static_cast<double>(kW) - 1.0) / 2.0;
const double kCy = (static_cast<double>(kH) - 1.0) / 2.0;

// --- Scenario 1: panoramic azimuthal texture, sampled through a pure yaw --
// A handful of low-order harmonics around the full circle (band-limited, like
// test_algorithms.cpp's own sine-grating texture) rather than per-texel
// noise, which aliases badly at this grid resolution and swamps the
// correlation signal with sampling noise unrelated to the pooling math.
const std::array<double, 4> kPanoHarmonics = {6.0, 10.0, 15.0, 23.0};
const std::array<double, 4> kPanoPhases = {0.3, 1.7, 4.1, 2.2};

float sample_pano(double theta) {
  double val = 0.0;
  for (std::size_t i = 0; i < kPanoHarmonics.size(); ++i) {
    val += std::sin(kPanoHarmonics[i] * theta + kPanoPhases[i]);
  }
  return static_cast<float>(0.5 + 0.5 * val / static_cast<double>(kPanoHarmonics.size()));
}

std::vector<float> render_yaw(double yaw_total) {
  std::vector<float> img(kW * kH);
  for (std::size_t y = 0; y < kH; ++y) {
    for (std::size_t x = 0; x < kW; ++x) {
      const double xn = static_cast<double>(x) - kCx;
      const double theta = std::atan2(xn, kFocal) + yaw_total;
      img[y * kW + x] = sample_pano(theta);
    }
  }
  return img;
}

// --- Scenario 2: frontal textured wall, sampled through pure forward transl.
const std::array<std::array<double, 2>, 4> kWallHarmonics = {
    {{3.0, 2.0}, {5.0, -3.0}, {2.0, 6.0}, {7.0, 4.0}}};
const std::array<double, 4> kWallPhases = {0.5, 2.0, 3.3, 1.1};

float sample_wall(double u, double v) {
  double val = 0.0;
  for (std::size_t i = 0; i < kWallHarmonics.size(); ++i) {
    val += std::sin(2.0 * M_PI * (kWallHarmonics[i][0] * u + kWallHarmonics[i][1] * v) +
                     kWallPhases[i]);
  }
  return static_cast<float>(0.5 + 0.5 * val / static_cast<double>(kWallHarmonics.size()));
}

std::vector<float> render_forward(double dist_to_wall) {
  std::vector<float> img(kW * kH);
  constexpr double kExtent = 6.0;  // fixed wall texture scale (not view-dependent)
  for (std::size_t y = 0; y < kH; ++y) {
    for (std::size_t x = 0; x < kW; ++x) {
      const double xn = (static_cast<double>(x) - kCx) / kFocal;
      const double yn = (static_cast<double>(y) - kCy) / kFocal;
      const double wx = xn * dist_to_wall;
      const double wy = yn * dist_to_wall;
      img[y * kW + x] = sample_wall(wx / kExtent, wy / kExtent);
    }
  }
  return img;
}

struct FieldSample {
  double yaw_rate = 0.0, roll_drift = 0.0, forward_drift = 0.0, vertical_drift = 0.0;
};

FieldSample mean_steady_state(const std::vector<OpticFlowEstimate>& ests, std::size_t skip) {
  FieldSample sum{};
  std::size_t n = 0;
  for (std::size_t i = skip; i < ests.size(); ++i) {
    sum.yaw_rate += ests[i].yaw_rate;
    sum.roll_drift += ests[i].roll_drift;
    sum.forward_drift += ests[i].forward_drift;
    sum.vertical_drift += ests[i].vertical_drift;
    ++n;
  }
  if (n > 0) {
    sum.yaw_rate /= static_cast<double>(n);
    sum.roll_drift /= static_cast<double>(n);
    sum.forward_drift /= static_cast<double>(n);
    sum.vertical_drift /= static_cast<double>(n);
  }
  return sum;
}

FieldSample run_yaw_scenario(double yaw_rate_true, double dt, int n) {
  ReichardtEmdArray emd(kW, kH, 0.05);
  double yaw = 0.0;
  std::vector<OpticFlowEstimate> ests;
  ests.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    ests.push_back(emd.update(render_yaw(yaw), dt));
    yaw += yaw_rate_true * dt;
  }
  return mean_steady_state(ests, 10);
}

FieldSample run_forward_scenario(double vz_true, double dt, int n, double dist0) {
  ReichardtEmdArray emd(kW, kH, 0.05);
  double dist = dist0;
  std::vector<OpticFlowEstimate> ests;
  ests.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    ests.push_back(emd.update(render_forward(dist), dt));
    dist -= vz_true * dt;
  }
  return mean_steady_state(ests, 10);
}

double pearson(const std::vector<double>& xs, const std::vector<double>& ys) {
  const auto n = static_cast<double>(xs.size());
  double mx = 0.0, my = 0.0;
  for (std::size_t i = 0; i < xs.size(); ++i) {
    mx += xs[i];
    my += ys[i];
  }
  mx /= n;
  my /= n;
  double sxy = 0.0, sxx = 0.0, syy = 0.0;
  for (std::size_t i = 0; i < xs.size(); ++i) {
    const double dx = xs[i] - mx;
    const double dy = ys[i] - my;
    sxy += dx * dy;
    sxx += dx * dx;
    syy += dy * dy;
  }
  if (sxx <= 0.0 || syy <= 0.0) return 0.0;
  return sxy / std::sqrt(sxx * syy);
}

}  // namespace

int main() {
  const double dt = 1.0 / 30.0;
  const int n_steps = 60;
  const std::vector<double> rates = {-1.5, -1.0, -0.5, 0.5, 1.0, 1.5};

  std::vector<double> yaw_rate_field, roll_drift_field, forward_drift_field, vertical_drift_field;
  for (const double r : rates) {
    const auto s = run_yaw_scenario(r, dt, n_steps);
    yaw_rate_field.push_back(s.yaw_rate);
    roll_drift_field.push_back(s.roll_drift);
    forward_drift_field.push_back(s.forward_drift);
    vertical_drift_field.push_back(s.vertical_drift);
  }

  const double corr_yaw_vs_yawrate = pearson(rates, yaw_rate_field);
  const double corr_yaw_vs_forwarddrift = pearson(rates, forward_drift_field);

  std::printf("[M3a] YAW scenario correlations vs commanded yaw rate:\n");
  std::printf("  OpticFlowEstimate::yaw_rate       R = %+.4f\n", corr_yaw_vs_yawrate);
  std::printf("  OpticFlowEstimate::roll_drift     R = %+.4f\n", pearson(rates, roll_drift_field));
  std::printf("  OpticFlowEstimate::forward_drift  R = %+.4f\n", corr_yaw_vs_forwarddrift);
  std::printf("  OpticFlowEstimate::vertical_drift R = %+.4f\n",
              pearson(rates, vertical_drift_field));

  std::vector<double> vzs = rates;  // reuse same sweep for forward/looming velocities
  std::vector<double> f_yaw_rate, f_roll_drift, f_forward_drift, f_vertical_drift;
  for (const double v : vzs) {
    const auto s = run_forward_scenario(v, dt, n_steps, /*dist0=*/6.0);
    f_yaw_rate.push_back(s.yaw_rate);
    f_roll_drift.push_back(s.roll_drift);
    f_forward_drift.push_back(s.forward_drift);
    f_vertical_drift.push_back(s.vertical_drift);
  }
  const double corr_fwd_vs_yawrate = pearson(vzs, f_yaw_rate);
  std::printf("[M3a] FORWARD/looming scenario correlations vs commanded forward velocity:\n");
  std::printf("  OpticFlowEstimate::yaw_rate       R = %+.4f\n", corr_fwd_vs_yawrate);
  std::printf("  OpticFlowEstimate::roll_drift     R = %+.4f\n", pearson(vzs, f_roll_drift));
  std::printf("  OpticFlowEstimate::forward_drift  R = %+.4f\n", pearson(vzs, f_forward_drift));
  std::printf("  OpticFlowEstimate::vertical_drift R = %+.4f\n", pearson(vzs, f_vertical_drift));

  int failures = 0;
  // The success bar (per the M3 brief): SOME field must track commanded yaw
  // rate with |R| > 0.9. It is "forward_drift", not "yaw_rate" - see the
  // header comment above and NOTES.md's "## M3" section. Assert on the
  // field that actually carries the signal, not the one whose name suggests
  // it should.
  if (std::abs(corr_yaw_vs_forwarddrift) <= 0.9) {
    std::printf("FAIL: |corr(yaw_rate_true, forward_drift)| = %.4f, expected > 0.9\n",
                std::abs(corr_yaw_vs_forwarddrift));
    ++failures;
  }
  // And the antisymmetric "yaw_rate" field should instead track *forward*
  // (looming) velocity, confirming the pooling swap is real and consistent,
  // not a one-scenario fluke.
  if (std::abs(corr_fwd_vs_yawrate) <= 0.9) {
    std::printf("FAIL: |corr(forward_vel_true, yaw_rate field)| = %.4f, expected > 0.9\n",
                std::abs(corr_fwd_vs_yawrate));
    ++failures;
  }

  if (failures == 0) {
    std::printf("ALL M3a BENCH CHECKS PASSED\n");
    return 0;
  }
  std::printf("%d M3a BENCH CHECK(S) FAILED\n", failures);
  return 1;
}
