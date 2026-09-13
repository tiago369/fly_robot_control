// Connectome-informed lobula plate tangential cell (LPTC) pooling.
//
// Real Drosophila HS (Horizontal System) and VS (Vertical System) neurons
// each pool hundreds of retinotopic T4/T5 elementary-motion-detector (EMD)
// columns into wide-field self-rotation estimators (Hausen 1982; Krapp &
// Hengstenberg, Nature 1996, "Estimation of self-motion by optic flow
// processing in single visual interneurons").
//
// The cell counts and the HS<->horizontal / VS<->vertical routing below are
// no longer just a literature paraphrase - they were checked against REAL
// FlyWire FAFB connectome data (v783, public Zenodo release, CC-BY 4.0, no
// account needed: github.com/flyconnectome/flywire_annotations for cell-type
// labels + zenodo.org/records/10676866's proofread_connections_783.feather
// for real per-neuron-pair synapse counts). Querying that data on
// 2026-09-13 found, per hemisphere:
//   - Exactly 3 HS cells (HSN, HSE, HSS) and exactly 8 VS cells (VS1-VS8) -
//     matching the 2024 eLife connectomic survey (Braun et al., "A
//     comprehensive neuroanatomical survey of the Drosophila Lobula Plate
//     Tangential Neurons with predictions for their optic flow
//     sensitivity"), which also counts 58 LPTs per hemisphere in total.
//   - HSN/HSE/HSS each receive the overwhelming majority of their measured
//     T4/T5 synapses from the T4a/T5a subtype specifically (e.g. HSE_right:
//     3921 T5a + 3317 T4a synapses out of 7242 total measured - 99.97%)
//     - T4a/T5a is the subtype tuned to front-to-back (horizontal) local
//     motion, confirming HS cells should pool the HORIZONTAL EMD signal.
//   - VS1-VS8 each receive the overwhelming majority of their measured T4/T5
//     synapses from the T4d/T5d subtype (with a T4b/T5b contribution for
//     VS1/VS2 specifically) - T4d/T5d is the vertical-motion subtype,
//     confirming VS cells should pool the VERTICAL EMD signal. Full
//     per-cell breakdown in NOTES.md's "## Post-M7" section.
// This is real evidence for a design choice this file already made (HS
// pools r_horiz, VS pools r_vert) before the connectome was queried - it
// wasn't guessed backwards from the data.
//
// This remains a RATE-BASED approximation, not a spiking simulation, and
// the band layout (which row/column range feeds which named unit) is still
// a simplified stand-in for the real dendritic receptive fields (the real
// per-synapse 3D coordinates needed to reconstruct actual receptive-field
// shapes live in a separate ~9.5GB per-synapse table that was not pulled
// down - see NOTES.md for why). What's grounded in real data here is the
// cell inventory (exact names/counts) and the horizontal/vertical routing;
// the specific band boundaries below remain an anatomically-motivated
// simplification, not measured receptive-field maps.
//
// Band layout:
//   - HS cells stratify the lobula plate DORSAL-TO-VENTRAL: this model uses
//     3 named units (HSN dorsal, HSE equatorial, HSS ventral - matching the
//     real cell count and naming), each pooling horizontal-motion EMD
//     columns across its own elevation (row) band only.
//   - VS cells tile the lobula plate FRONT-TO-BACK (retinotopic azimuth):
//     this model uses 8 named units (VS1 most frontal through VS8 most
//     posterior/lateral - matching the real, confirmed cell count), each
//     pooling vertical-motion EMD columns across its own azimuth (column)
//     band only.
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
  static constexpr std::size_t kNumVs = 8;

  static const std::array<const char*, kNumHs>& hs_names() {
    static const std::array<const char*, kNumHs> kNames{"HSN", "HSE", "HSS"};
    return kNames;
  }

  static const std::array<const char*, kNumVs>& vs_names() {
    static const std::array<const char*, kNumVs> kNames{"VS1", "VS2", "VS3", "VS4",
                                                          "VS5", "VS6", "VS7", "VS8"};
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
