#pragma once
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include "fly_brain/lptc_pooling.hpp"

namespace fly_brain {

struct OpticFlowEstimate {
  double yaw_rate = 0.0;      // antisymmetric horizontal pooling (HS-cell-like)
  double roll_drift = 0.0;    // antisymmetric vertical pooling (VS-cell-like)
  double forward_drift = 0.0; // symmetric horizontal pooling (bulk/looming proxy)
  double vertical_drift = 0.0;// symmetric vertical pooling
};

// Grid of Hassenstein-Reichardt elementary motion detectors on a coarse
// grayscale grid (real ommatidia are far coarser than camera pixel counts).
// Each detector delay-and-correlates two neighboring luminance signals and
// mirror-subtracts to cancel non-motion flicker, producing a direction-
// opponent response. yaw_rate/roll_drift are pooled by fly_brain::
// LptcPopulation into named HSN/HSE/HSS and VS1-VS6 units (see
// lptc_pooling.hpp for the anatomical grounding); forward_drift/
// vertical_drift remain a plain symmetric whole-grid average (no
// well-characterized public LPTC mapping was used for those two).
class ReichardtEmdArray {
public:
  ReichardtEmdArray(std::size_t width, std::size_t height, double tau)
      : w_(width),
        h_(height),
        tau_(tau),
        lp_(width * height, 0.0f),
        last_activity_(width * height, 0.0f),
        initialized_(false) {}

  // image: row-major grayscale grid, size w*h, values in [0,1].
  OpticFlowEstimate update(const std::vector<float>& image, double dt) {
    if (!initialized_) {
      lp_ = image;
      initialized_ = true;
      return {};
    }

    std::vector<float> r_horiz(w_ * h_, 0.0f);
    std::vector<float> r_vert(w_ * h_, 0.0f);

    for (std::size_t y = 0; y < h_; ++y) {
      for (std::size_t x = 0; x < w_; ++x) {
        const std::size_t idx = y * w_ + x;
        if (x + 1 < w_) {
          const std::size_t idx_r = idx + 1;
          r_horiz[idx] = lp_[idx] * image[idx_r] - lp_[idx_r] * image[idx];
        }
        if (y + 1 < h_) {
          const std::size_t idx_d = idx + w_;
          r_vert[idx] = lp_[idx] * image[idx_d] - lp_[idx_d] * image[idx];
        }
      }
    }

    const double alpha = tau_ > 0.0 ? dt / tau_ : 1.0;
    for (std::size_t i = 0; i < lp_.size(); ++i) {
      lp_[i] = static_cast<float>(lp_[i] + alpha * (image[i] - lp_[i]));
    }

    // Per-cell combined response magnitude, kept around purely for
    // visualization (a "which units are firing right now" readout) - not
    // used by the pooling/estimate math above, which already consumed
    // r_horiz/r_vert directly.
    for (std::size_t i = 0; i < last_activity_.size(); ++i) {
      last_activity_[i] = std::abs(r_horiz[i]) + std::abs(r_vert[i]);
    }

    OpticFlowEstimate est;
    est.yaw_rate = LptcPopulation::pool_hs(r_horiz, w_, h_, hs_activity_);
    est.roll_drift = LptcPopulation::pool_vs(r_vert, w_, h_, vs_activity_);

    double norm = 0.0;
    for (std::size_t y = 0; y < h_; ++y) {
      for (std::size_t x = 0; x < w_; ++x) {
        const std::size_t idx = y * w_ + x;
        est.forward_drift += r_horiz[idx];
        est.vertical_drift += r_vert[idx];
        norm += 1.0;
      }
    }
    if (norm > 0.0) {
      est.forward_drift /= norm;
      est.vertical_drift /= norm;
    }
    return est;
  }

  // Per-cell HS (HSN/HSE/HSS) and VS (VS1-VS6) activations from the most
  // recent update() call, in fly_brain::LptcPopulation::hs_names()/
  // vs_names() order. See lptc_pooling.hpp.
  const std::array<double, LptcPopulation::kNumHs>& hs_activity() const { return hs_activity_; }
  const std::array<double, LptcPopulation::kNumVs>& vs_activity() const { return vs_activity_; }

  // Per-cell |horizontal EMD response| + |vertical EMD response| from the
  // most recent update() call, row-major, size width()*height(). For
  // visualization only (see fly_brain::OpticFlowController).
  const std::vector<float>& last_activity() const { return last_activity_; }
  std::size_t width() const { return w_; }
  std::size_t height() const { return h_; }

private:
  std::size_t w_, h_;
  double tau_;
  std::vector<float> lp_;
  std::vector<float> last_activity_;
  bool initialized_;
  std::array<double, LptcPopulation::kNumHs> hs_activity_{};
  std::array<double, LptcPopulation::kNumVs> vs_activity_{};
};

}  // namespace fly_brain
