#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <vector>

namespace fly_brain {

// M5: phototaxis-analog color-based target detector. A DIFFERENT vision
// algorithm from fly_brain::ReichardtEmdArray (M3) - that one detects
// change/motion (frame-to-frame luminance correlation) and is blind to a
// static, non-moving colored object; a real fly's phototaxis response is
// driven by the STATIC appearance (color/brightness) of a salient target in
// the visual field, not its motion. This is a simple per-pixel color
// threshold + centroid (a standard, well-understood blob-detection method -
// deliberately not anything fancier, matching this project's "small,
// hand-designed, biologically-loose" scope), operating directly on an rgb8
// buffer so fly_brain::PhototaxisController can run it without any
// OpenCV/vision-library dependency (consistent with OpticFlowController's own
// dependency-free `downsample_grayscale()`).
//
// Real-data check (2026-09-13, see NOTES.md's "## Post-M7" section):
// checked whether this module could get the same real-connectome grounding
// the optic-flow/haltere modules got above, and concluded honestly that it
// shouldn't be forced. The real fly circuit this is loosely analogous to
// (R7/R8 photoreceptors -> Dm8 -> Tm5/Tm9, well documented: Dm8 pools
// 13-16 R7 inputs per Karuppudurai et al. 2014/Ramos-Traslosheros &
// Silies 2021) is a SPECTRAL-OPPONENCY circuit - it compares "pale" vs
// "yellow" ommatidia (different UV/blue-vs-green sensitivities) to
// discriminate hue, on a UV-sensitive retina this project's RGB camera
// can't replicate. This class does something categorically different:
// template-matching a single fixed target RGB triple. Retrofitting a
// "Dm8-inspired pooling radius" onto an RGB threshold would be a cosmetic
// parallel, not a real grounding - the two circuits solve different
// problems on different sensor hardware - so `min_pixels` below is left as
// the honestly-labeled hand-tuned floor it always was, not given a fake
// citation.
struct ColorBlobResult {
  bool visible = false;
  // Normalized image coordinates of the matching-pixel centroid, in [-1, 1]:
  // 0 = image center, +1 = right/bottom edge, -1 = left/top edge. Only
  // meaningful when `visible` is true (left at 0.0 otherwise).
  double centroid_x = 0.0;
  double centroid_y = 0.0;
  // Fraction of the frame's pixels that matched the color threshold, in
  // [0, 1] - a simple, correspondence-free "apparent size" / distance proxy
  // (bigger blob = closer target). Valid even when `visible` is false (it is
  // simply below the min_pixels bar in that case, not meaningless).
  double area_fraction = 0.0;
  // Raw matching-pixel count, mainly for tests/telemetry.
  int pixel_count = 0;
};

struct ColorThreshold {
  // Target color to detect (rgb8, 0-255). Default: a bright orange/red, far
  // from this world's grayscale checker ground plane and blue-gradient sky
  // (see models/skydio_x2/drone_scene.xml's "attractant_target" body/material
  // for the exact rgba the MJCF renders - kept in agreement with these
  // defaults, see PhototaxisController's own parameter declarations).
  int target_r = 235;
  int target_g = 70;
  int target_b = 20;
  // Max allowed per-channel absolute difference from the target color.
  int tolerance = 70;
  // Reject low-saturation (grayish) pixels outright, regardless of hue match -
  // this is what actually keeps the detector from firing on the ground
  // plane's white/near-black checker squares or the sky gradient, both of
  // which are intentionally desaturated (see drone_scene.xml's own M3
  // comment on the groundplane texture).
  int min_channel_spread = 50;
};

class ColorBlobDetector {
public:
  explicit ColorBlobDetector(ColorThreshold thresh = ColorThreshold{}) : t_(thresh) {}

  // rgb: row-major rgb8 buffer, width*height*3 bytes (no stride padding -
  // caller is responsible for de-striding, same contract
  // OpticFlowController::downsample_grayscale() expects of its own caller).
  // min_pixels: minimum matching-pixel count to report `visible=true` (a
  // small floor against single-pixel/anti-aliasing noise, not a real
  // detection-confidence model).
  ColorBlobResult detect(const uint8_t* rgb, int width, int height, int min_pixels = 12) const {
    ColorBlobResult res;
    if (rgb == nullptr || width <= 0 || height <= 0) {
      return res;
    }
    long sum_x = 0, sum_y = 0;
    int count = 0;
    for (int y = 0; y < height; ++y) {
      const std::size_t row_base = static_cast<std::size_t>(y) * static_cast<std::size_t>(width) * 3;
      for (int x = 0; x < width; ++x) {
        const std::size_t idx = row_base + static_cast<std::size_t>(x) * 3;
        if (matches(rgb[idx], rgb[idx + 1], rgb[idx + 2])) {
          sum_x += x;
          sum_y += y;
          ++count;
        }
      }
    }
    res.pixel_count = count;
    res.area_fraction =
        static_cast<double>(count) / (static_cast<double>(width) * static_cast<double>(height));
    if (count >= min_pixels) {
      res.visible = true;
      const double cx = static_cast<double>(sum_x) / static_cast<double>(count);
      const double cy = static_cast<double>(sum_y) / static_cast<double>(count);
      const double half_w = (static_cast<double>(width) - 1.0) / 2.0;
      const double half_h = (static_cast<double>(height) - 1.0) / 2.0;
      res.centroid_x = half_w > 0.0 ? (cx - half_w) / half_w : 0.0;
      res.centroid_y = half_h > 0.0 ? (cy - half_h) / half_h : 0.0;
    }
    return res;
  }

private:
  bool matches(uint8_t r8, uint8_t g8, uint8_t b8) const {
    const int r = r8, g = g8, b = b8;
    if (std::abs(r - t_.target_r) > t_.tolerance) return false;
    if (std::abs(g - t_.target_g) > t_.tolerance) return false;
    if (std::abs(b - t_.target_b) > t_.tolerance) return false;
    const int mx = std::max({r, g, b});
    const int mn = std::min({r, g, b});
    if (mx - mn < t_.min_channel_spread) return false;
    return true;
  }

  ColorThreshold t_;
};

}  // namespace fly_brain
