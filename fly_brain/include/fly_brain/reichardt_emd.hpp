#pragma once
#include <cmath>
#include <cstddef>
#include <vector>

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
// opponent response. Pooled with antisymmetric/symmetric spatial weights to
// mimic LPTC (HS/VS-cell) self-motion estimators.
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
    const double cx = (static_cast<double>(w_) - 1.0) / 2.0;
    const double cy = (static_cast<double>(h_) - 1.0) / 2.0;
    double norm = 0.0;
    for (std::size_t y = 0; y < h_; ++y) {
      for (std::size_t x = 0; x < w_; ++x) {
        const std::size_t idx = y * w_ + x;
        const double sign_x = (static_cast<double>(x) - cx) >= 0 ? 1.0 : -1.0;
        const double sign_y = (static_cast<double>(y) - cy) >= 0 ? 1.0 : -1.0;
        est.yaw_rate += sign_x * r_horiz[idx];
        est.forward_drift += r_horiz[idx];
        est.roll_drift += sign_y * r_vert[idx];
        est.vertical_drift += r_vert[idx];
        norm += 1.0;
      }
    }
    if (norm > 0.0) {
      est.yaw_rate /= norm;
      est.forward_drift /= norm;
      est.roll_drift /= norm;
      est.vertical_drift /= norm;
    }
    return est;
  }

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
};

}  // namespace fly_brain
