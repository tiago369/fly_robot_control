#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "fly_brain/cascaded_pid.hpp"
#include "fly_brain/color_blob_detector.hpp"
#include "fly_brain/complementary_filter.hpp"
#include "fly_brain/descending_fusion.hpp"
#include "fly_brain/haltere_reflex.hpp"
#include "fly_brain/mixer.hpp"
#include "fly_brain/reichardt_emd.hpp"

using namespace fly_brain;

static int g_failures = 0;
#define CHECK(cond)                                                                    \
  do {                                                                                  \
    if (!(cond)) {                                                                      \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                       \
      ++g_failures;                                                                     \
    }                                                                                    \
  } while (0)
#define NEAR(a, b, eps) CHECK(std::abs((a) - (b)) < (eps))

void test_mixer() {
  QuadMixer mixer(0.2, 0.02);

  auto f_hover = mixer.allocate(8.0, 0.0, 0.0, 0.0);
  for (double f : f_hover) NEAR(f, 2.0, 1e-9);

  // pure roll torque should reconstruct via the same forward geometry used inside QuadMixer
  const double arm = 0.2, kdrag = 0.02;
  constexpr double deg45 = M_PI / 4.0;
  const std::array<double, 4> angles = {deg45, 3 * deg45, 5 * deg45, 7 * deg45};
  const std::array<double, 4> spin = {+1.0, -1.0, +1.0, -1.0};

  auto forward = [&](const std::array<double, 4>& f) {
    double T = 0, tx = 0, ty = 0, tz = 0;
    for (int i = 0; i < 4; ++i) {
      const double x = arm * std::cos(angles[i]);
      const double y = arm * std::sin(angles[i]);
      T += f[i];
      tx += y * f[i];
      ty += -x * f[i];
      tz += spin[i] * kdrag * f[i];
    }
    return std::array<double, 4>{T, tx, ty, tz};
  };

  auto f_roll = mixer.allocate(8.0, 0.5, 0.0, 0.0);
  auto recon = forward(f_roll);
  NEAR(recon[0], 8.0, 1e-6);
  NEAR(recon[1], 0.5, 1e-6);
  NEAR(recon[2], 0.0, 1e-6);
  NEAR(recon[3], 0.0, 1e-6);

  auto f_yaw = mixer.allocate(8.0, 0.0, 0.0, 0.1);
  auto recon_yaw = forward(f_yaw);
  NEAR(recon_yaw[3], 0.1, 1e-6);

  std::printf("test_mixer done\n");
}

void test_complementary_filter() {
  ComplementaryFilter filt(0.98);
  ImuSample level{0, 0, 0, 0, 0, 9.81};
  Attitude att;
  for (int i = 0; i < 200; ++i) att = filt.update(level, 0.002);
  NEAR(att.roll, 0.0, 1e-3);
  NEAR(att.pitch, 0.0, 1e-3);

  // constant ~10deg roll tilt reading should converge to ~10deg estimate
  const double true_roll = 10.0 * M_PI / 180.0;
  ImuSample tilted{0, 0, 0, 0, 9.81 * std::sin(true_roll), 9.81 * std::cos(true_roll)};
  ComplementaryFilter filt2(0.98);
  for (int i = 0; i < 2000; ++i) att = filt2.update(tilted, 0.002);
  NEAR(att.roll, true_roll, 2e-2);

  std::printf("test_complementary_filter done\n");
}

void test_haltere_reflex() {
  HaltereReflex reflex;
  AttitudeSetpoint sp;
  sp.thrust = 8.0;
  Attitude level_att{};
  ImuSample spinning_right{0.5, 0.0, 0.0, 0, 0, 9.81};  // rolling right (+wx), no setpoint
  auto cmd = reflex.step(sp, level_att, spinning_right, 0.002);
  CHECK(cmd.tau_x < 0.0);  // reflex should oppose the unwanted roll rate
  CHECK(cmd.thrust == 8.0);

  std::printf("test_haltere_reflex done\n");
}

void test_cascaded_pid() {
  CascadedPid::Gains g;
  g.hover_thrust = 8.0;
  CascadedPid pid(g);
  Attitude level_att{};
  ImuSample still{0, 0, 0, 0, 0, 9.81};
  auto cmd = pid.step(/*z_sp=*/1.0, /*z=*/0.5, 0, 0, 0, level_att, still, 0.002);
  CHECK(cmd.thrust > g.hover_thrust);  // below target altitude -> more thrust

  std::printf("test_cascaded_pid done\n");
}

void test_reichardt_emd() {
  const std::size_t w = 16, h = 16;
  ReichardtEmdArray emd(w, h, 0.05);
  const double dt = 1.0 / 60.0;

  auto render_grating = [&](double phase) {
    std::vector<float> img(w * h);
    for (std::size_t y = 0; y < h; ++y)
      for (std::size_t x = 0; x < w; ++x)
        img[y * w + x] = static_cast<float>(
            0.5 + 0.5 * std::sin(2.0 * M_PI * (static_cast<double>(x) / 4.0) + phase));
    return img;
  };

  // rightward-drifting grating: phase decreases with time (pattern moves +x)
  double phase = 0.0;
  double speed = 6.0;  // rad/s of phase drift
  OpticFlowEstimate last{};
  for (int i = 0; i < 30; ++i) {
    auto img = render_grating(phase);
    last = emd.update(img, dt);
    phase -= speed * dt;
  }
  const double rightward_forward = last.forward_drift;
  CHECK(rightward_forward != 0.0);

  // reverse direction: leftward-drifting grating should flip the sign
  ReichardtEmdArray emd2(w, h, 0.05);
  phase = 0.0;
  OpticFlowEstimate last2{};
  for (int i = 0; i < 30; ++i) {
    auto img = render_grating(phase);
    last2 = emd2.update(img, dt);
    phase += speed * dt;
  }
  CHECK(last2.forward_drift * rightward_forward < 0.0);

  std::printf("test_reichardt_emd done (fwd_right=%.6f fwd_left=%.6f)\n", rightward_forward,
              last2.forward_drift);
}

void test_descending_fusion() {
  // M4: DescendingFusion::fuse()'s optomotor term reads flow.roll_drift, NOT
  // flow.yaw_rate - see descending_fusion.hpp's own doc comment (and
  // NOTES.md's "## M3"/"## M4" sections) for the full field-naming
  // resolution: despite the name, "yaw_rate" doesn't track camera yaw
  // rotation on either the idealized bench (M3a: "forward_drift" does) or
  // the real tilted nose_cam (M3b, empirically: "roll_drift" does, and is
  // what M4 actually wires up here).
  DescendingFusion fusion;
  TaskCommand task;
  task.desired_yaw_rate = 0.0;
  OpticFlowEstimate flow;
  flow.roll_drift = 0.3;  // sensed self-rotation (e.g. yawing right), real-camera field
  flow.yaw_rate = 0.3;    // deliberately also set, to confirm this field is now IGNORED
  auto sp = fusion.fuse(task, flow);
  CHECK(sp.yaw_rate < 0.0);  // optomotor reflex should command counter-rotation

  std::printf("test_descending_fusion done\n");
}

void test_color_blob_detector() {
  // M5: fly_brain::ColorBlobDetector - a DIFFERENT algorithm from
  // ReichardtEmdArray (motion) - validated on synthetic rgb8 buffers before
  // ever being wired to a live camera topic, same discipline
  // test_analytic_flow_bench.cpp established for the optic-flow pipeline
  // (NOTES.md's "## M3" section).
  const int w = 64, h = 48;
  auto make_gray_image = [&](uint8_t level) {
    std::vector<uint8_t> img(static_cast<std::size_t>(w) * h * 3, level);
    return img;
  };
  auto paint_rect = [&](std::vector<uint8_t>& img, int x0, int y0, int rw, int rh, uint8_t r,
                        uint8_t g, uint8_t b) {
    for (int y = y0; y < y0 + rh; ++y) {
      for (int x = x0; x < x0 + rw; ++x) {
        if (x < 0 || x >= w || y < 0 || y >= h) continue;
        const std::size_t idx = (static_cast<std::size_t>(y) * w + x) * 3;
        img[idx] = r;
        img[idx + 1] = g;
        img[idx + 2] = b;
      }
    }
  };

  ColorBlobDetector det;  // library defaults: bright orange/red target color

  // 1. A featureless mid-gray frame (stand-in for the desaturated
  // groundplane/sky) must NOT be detected as visible - this is what actually
  // matters for the real world (rejecting the ground plane), not just "some
  // color check fires".
  {
    auto img = make_gray_image(128);
    auto res = det.detect(img.data(), w, h);
    CHECK(!res.visible);
    NEAR(res.area_fraction, 0.0, 1e-9);
  }

  // 2. A bright orange/red rectangle dead-center should be detected, with a
  // near-zero centroid offset and a positive area fraction.
  {
    auto img = make_gray_image(30);  // dark background, not just mid-gray
    paint_rect(img, w / 2 - 4, h / 2 - 4, 8, 8, 235, 70, 20);
    auto res = det.detect(img.data(), w, h);
    CHECK(res.visible);
    NEAR(res.centroid_x, 0.0, 0.05);
    NEAR(res.centroid_y, 0.0, 0.05);
    CHECK(res.area_fraction > 0.0);
  }

  // 3. Left vs right placement must produce the correspondingly-signed
  // centroid_x - this is the exact signal PhototaxisController turns into a
  // yaw correction, so getting its SIGN right synthetically, before trusting
  // it live, is the whole point of this test (same motivation as M3a's
  // bench catching the yaw_rate/forward_drift field-naming bug).
  {
    auto img_left = make_gray_image(30);
    paint_rect(img_left, 2, h / 2 - 3, 6, 6, 235, 70, 20);
    auto res_left = det.detect(img_left.data(), w, h);
    CHECK(res_left.visible);
    CHECK(res_left.centroid_x < -0.5);  // near the left edge

    auto img_right = make_gray_image(30);
    paint_rect(img_right, w - 8, h / 2 - 3, 6, 6, 235, 70, 20);
    auto res_right = det.detect(img_right.data(), w, h);
    CHECK(res_right.visible);
    CHECK(res_right.centroid_x > 0.5);  // near the right edge
  }

  // 4. Top vs bottom placement must produce the correspondingly-signed
  // centroid_y.
  {
    auto img_top = make_gray_image(30);
    paint_rect(img_top, w / 2 - 3, 1, 6, 5, 235, 70, 20);
    auto res_top = det.detect(img_top.data(), w, h);
    CHECK(res_top.visible);
    CHECK(res_top.centroid_y < -0.4);

    auto img_bottom = make_gray_image(30);
    paint_rect(img_bottom, w / 2 - 3, h - 6, 6, 5, 235, 70, 20);
    auto res_bottom = det.detect(img_bottom.data(), w, h);
    CHECK(res_bottom.visible);
    CHECK(res_bottom.centroid_y > 0.4);
  }

  // 5. A bigger blob must report a larger area_fraction than a smaller one
  // (the "apparent size = distance proxy" PhototaxisController uses to
  // decide how hard to approach) - and a too-small blob (below min_pixels)
  // must NOT register as visible at all (rejects single-pixel noise).
  {
    auto img_small = make_gray_image(30);
    paint_rect(img_small, w / 2 - 2, h / 2 - 2, 4, 4, 235, 70, 20);
    auto res_small = det.detect(img_small.data(), w, h);

    auto img_big = make_gray_image(30);
    paint_rect(img_big, w / 2 - 10, h / 2 - 10, 20, 20, 235, 70, 20);
    auto res_big = det.detect(img_big.data(), w, h);

    CHECK(res_small.visible);
    CHECK(res_big.visible);
    CHECK(res_big.area_fraction > res_small.area_fraction);

    auto img_tiny = make_gray_image(30);
    paint_rect(img_tiny, w / 2, h / 2, 1, 1, 235, 70, 20);  // 1 pixel, below min_pixels
    auto res_tiny = det.detect(img_tiny.data(), w, h);
    CHECK(!res_tiny.visible);
  }

  // 6. A DIFFERENT bright, saturated color (e.g. a hypothetical blue marker)
  // must NOT match the default orange/red threshold - confirms this is a
  // real color-selective detector, not just a generic "anything colorful"
  // motion-blind blob finder.
  {
    auto img = make_gray_image(30);
    paint_rect(img, w / 2 - 4, h / 2 - 4, 8, 8, 20, 70, 235);  // saturated blue
    auto res = det.detect(img.data(), w, h);
    CHECK(!res.visible);
  }

  std::printf("test_color_blob_detector done\n");
}

int main() {
  test_mixer();
  test_complementary_filter();
  test_haltere_reflex();
  test_cascaded_pid();
  test_reichardt_emd();
  test_descending_fusion();
  test_color_blob_detector();

  if (g_failures == 0) {
    std::printf("ALL TESTS PASSED\n");
    return 0;
  }
  std::printf("%d CHECK(S) FAILED\n", g_failures);
  return 1;
}
