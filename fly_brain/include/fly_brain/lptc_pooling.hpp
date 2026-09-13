// Connectome-informed lobula plate tangential cell (LPTC) pooling.
//
// Real Drosophila HS (Horizontal System) and VS (Vertical System) neurons
// each pool hundreds of retinotopic T4/T5 elementary-motion-detector (EMD)
// columns into wide-field self-rotation estimators (Hausen 1982; Krapp &
// Hengstenberg, Nature 1996, "Estimation of self-motion by optic flow
// processing in single visual interneurons"). The 2024 eLife connectomic
// survey of the full Drosophila LPTC population ("A comprehensive
// neuroanatomical survey of the Drosophila Lobula Plate Tangential Neurons
// with predictions for their optic flow sensitivity", Braun et al.) counts
// 58 LPTs per hemisphere, including 3 HS cells (HSN/HSE/HSS) and 8 VS cells,
// and confirms HS/VS are the primary rotation-selective population, each
// cell's receptive field tiling a distinct band of the eye.
//
// This is a literature-informed RATE-BASED approximation, not a per-synapse
// reconstruction pulled from the live FlyWire/hemibrain connectome (no API
// query was made - that requires the account credentials of whoever wants
// the raw EM data) and not a spiking simulation. What it borrows from the
// real circuit is the STRUCTURE: named cells, each with its own retinotopic
// receptive-field band, rather than one anonymous whole-eye average. See
// NOTES.md's "## Post-M7: connectome-informed LPTC pooling" section.
//
// Anatomical grounding used for the band layout:
//   - HS cells stratify the lobula plate DORSAL-TO-VENTRAL: this model uses
//     3 named units (HSN dorsal, HSE equatorial, HSS ventral - matching the
//     real cell count and naming), each pooling horizontal-motion EMD
//     columns across its own elevation (row) band only.
//   - VS cells tile the lobula plate FRONT-TO-BACK (retinotopic azimuth):
//     this model uses 6 named units (VS1 most frontal through VS6 most
//     posterior/lateral - Drosophila has 6-8 depending on the source; 6 is
//     used here for a clean division of a 32-column grid), each pooling
//     vertical-motion EMD columns across its own azimuth (column) band only.
//
// Backward-compatibility guarantee: each population's pooled/aggregate value
// (returned by pool_hs()/pool_vs()) is computed as a per-pixel weighted
// combination of the same underlying signal ReichardtEmdArray always pooled
// (sign_x-weighted r_horiz for HS, sign_y-weighted r_vert for VS) - so it is
// numerically equivalent (to floating-point summation-order noise) to the
// original whole-grid average, regardless of how unevenly the grid divides
// into bands. This means M3/M4's already-tuned closed-loop gains and the
// M3a analytic bench's correlation checks (test_analytic_flow_bench.cpp)
// are unaffected by decomposing the pooling into named per-cell units.
#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace fly_brain {

class LptcPopulation {
public:
  static constexpr std::size_t kNumHs = 3;
  static constexpr std::size_t kNumVs = 6;

  static const std::array<const char*, kNumHs>& hs_names() {
    static const std::array<const char*, kNumHs> kNames{"HSN", "HSE", "HSS"};
    return kNames;
  }

  static const std::array<const char*, kNumVs>& vs_names() {
    static const std::array<const char*, kNumVs> kNames{"VS1", "VS2", "VS3",
                                                          "VS4", "VS5", "VS6"};
    return kNames;
  }

  // Pools r_horiz (row-major, width w x height h) into kNumHs HS-cell
  // activations, one per dorsal-to-ventral elevation band (a contiguous
  // range of rows), each the mean of sign_x-weighted r_horiz over just that
  // band. hs_out is resized/filled with the kNumHs per-cell activations, in
  // hs_names() order. Returns the population-average activation (see the
  // class-level comment for why this equals ReichardtEmdArray's original
  // whole-grid "yaw_rate" pooling).
  static double pool_hs(const std::vector<float>& r_horiz, std::size_t w, std::size_t h,
                         std::array<double, kNumHs>& hs_out) {
    const double cx = (static_cast<double>(w) - 1.0) / 2.0;
    double total = 0.0;
    std::size_t total_n = 0;
    for (std::size_t b = 0; b < kNumHs; ++b) {
      const std::size_t y0 = b * h / kNumHs;
      const std::size_t y1 = (b + 1) * h / kNumHs;
      double sum = 0.0;
      std::size_t n = 0;
      for (std::size_t y = y0; y < y1; ++y) {
        for (std::size_t x = 0; x < w; ++x) {
          const double sign_x = (static_cast<double>(x) - cx) >= 0 ? 1.0 : -1.0;
          sum += sign_x * static_cast<double>(r_horiz[y * w + x]);
          ++n;
        }
      }
      hs_out[b] = n > 0 ? sum / static_cast<double>(n) : 0.0;
      total += sum;
      total_n += n;
    }
    return total_n > 0 ? total / static_cast<double>(total_n) : 0.0;
  }

  // Pools r_vert (row-major, width w x height h) into kNumVs VS-cell
  // activations, one per front-to-back azimuth band (a contiguous range of
  // columns), each the mean of sign_y-weighted r_vert over just that band.
  // vs_out is filled with the kNumVs per-cell activations, in vs_names()
  // order. Returns the population-average activation (equals the original
  // whole-grid "roll_drift" pooling - see class-level comment).
  static double pool_vs(const std::vector<float>& r_vert, std::size_t w, std::size_t h,
                         std::array<double, kNumVs>& vs_out) {
    const double cy = (static_cast<double>(h) - 1.0) / 2.0;
    double total = 0.0;
    std::size_t total_n = 0;
    for (std::size_t b = 0; b < kNumVs; ++b) {
      const std::size_t x0 = b * w / kNumVs;
      const std::size_t x1 = (b + 1) * w / kNumVs;
      double sum = 0.0;
      std::size_t n = 0;
      for (std::size_t x = x0; x < x1; ++x) {
        for (std::size_t y = 0; y < h; ++y) {
          const double sign_y = (static_cast<double>(y) - cy) >= 0 ? 1.0 : -1.0;
          sum += sign_y * static_cast<double>(r_vert[y * w + x]);
          ++n;
        }
      }
      vs_out[b] = n > 0 ? sum / static_cast<double>(n) : 0.0;
      total += sum;
      total_n += n;
    }
    return total_n > 0 ? total / static_cast<double>(total_n) : 0.0;
  }
};

}  // namespace fly_brain
