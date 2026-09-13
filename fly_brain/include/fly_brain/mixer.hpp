#pragma once
#include <array>
#include <cmath>
#include <stdexcept>

namespace fly_brain {

// Maps {total_thrust, roll_torque, pitch_torque, yaw_torque} to 4 per-rotor
// thrusts for an X-configuration quadrotor. Body frame: x forward, y left,
// z up (REP103). Rotors numbered 0..3 at angles 45/135/225/315 deg from +x,
// spin direction alternates per diagonal pair (0,2 vs 1,3).
class QuadMixer {
public:
  // arm_length: rotor distance from body origin (m).
  // drag_to_thrust: ratio of reaction yaw torque to thrust for each rotor (1/m units
  //   collapse into a dimensionless coefficient here since thrust is already a force).
  QuadMixer(double arm_length, double drag_to_thrust) {
    constexpr double deg45 = M_PI / 4.0;
    // rotor angles and spin signs (+1 = CCW produces -yaw reaction torque convention below)
    const std::array<double, 4> angles = {deg45, 3 * deg45, 5 * deg45, 7 * deg45};
    const std::array<double, 4> spin = {+1.0, -1.0, +1.0, -1.0};

    std::array<std::array<double, 4>, 4> A{};
    for (int i = 0; i < 4; ++i) {
      const double x = arm_length * std::cos(angles[i]);
      const double y = arm_length * std::sin(angles[i]);
      A[0][i] = 1.0;                       // thrust row
      A[1][i] = y;                         // roll torque row: tau_x = sum(y_i * f_i)
      A[2][i] = -x;                        // pitch torque row: tau_y = -sum(x_i * f_i)
      A[3][i] = spin[i] * drag_to_thrust;  // yaw torque row
    }
    inv_ = invert4x4(A);
  }

  // One rotor's position (body frame, REP103: x fwd, y left, z up - only x/y
  // matter, see below) and spin-direction sign. `spin` must match the sign of
  // the real airframe's yaw-reaction-torque-to-thrust ratio for that rotor
  // (e.g., read off a MuJoCo <motor> actuator's gear vector: gear[5] > 0 =>
  // spin = +1).
  struct RotorGeometry {
    double x, y;
    double spin;
  };

  // General (possibly-asymmetric) rotor layout constructor. Use this instead
  // of the symmetric-45-degree-X constructor above whenever the real
  // airframe's rotor positions aren't a perfect square. Ours isn't: the
  // vendored skydio_x2 MJCF (models/skydio_x2/x2.xml) places thrust1/2 at
  // x=-0.14 and thrust3/4 at x=+0.14, but y is +-0.18 - i.e. a 0.14x0.18
  // rectangular X (~52 deg from +x), not a 45 deg square X, so a single
  // `arm_length` can't represent it correctly. z is irrelevant here: for a
  // rotor thrust purely along body +z, torque = r x F = (y*Fz, -x*Fz, 0), so
  // only each rotor's x/y offset (not its mounting height) affects roll/pitch
  // authority.
  QuadMixer(const std::array<RotorGeometry, 4>& rotors, double drag_to_thrust) {
    std::array<std::array<double, 4>, 4> A{};
    for (int i = 0; i < 4; ++i) {
      A[0][i] = 1.0;
      A[1][i] = rotors[i].y;
      A[2][i] = -rotors[i].x;
      A[3][i] = rotors[i].spin * drag_to_thrust;
    }
    inv_ = invert4x4(A);
  }

  // Returns 4 rotor thrust commands (may be negative if infeasible upstream clipping is skipped).
  std::array<double, 4> allocate(double thrust, double tau_x, double tau_y, double tau_z) const {
    const std::array<double, 4> b = {thrust, tau_x, tau_y, tau_z};
    std::array<double, 4> f{};
    for (int i = 0; i < 4; ++i) {
      double sum = 0.0;
      for (int j = 0; j < 4; ++j) sum += inv_[i][j] * b[j];
      f[i] = sum;
    }
    return f;
  }

private:
  using Mat4 = std::array<std::array<double, 4>, 4>;

  static Mat4 invert4x4(Mat4 m) {
    Mat4 inv{};
    for (int i = 0; i < 4; ++i) inv[i][i] = 1.0;

    for (int col = 0; col < 4; ++col) {
      int pivot = col;
      for (int row = col + 1; row < 4; ++row) {
        if (std::abs(m[row][col]) > std::abs(m[pivot][col])) pivot = row;
      }
      if (std::abs(m[pivot][col]) < 1e-12) {
        throw std::runtime_error("QuadMixer: singular allocation matrix (check geometry)");
      }
      std::swap(m[col], m[pivot]);
      std::swap(inv[col], inv[pivot]);

      const double d = m[col][col];
      for (int j = 0; j < 4; ++j) {
        m[col][j] /= d;
        inv[col][j] /= d;
      }
      for (int row = 0; row < 4; ++row) {
        if (row == col) continue;
        const double factor = m[row][col];
        for (int j = 0; j < 4; ++j) {
          m[row][j] -= factor * m[col][j];
          inv[row][j] -= factor * inv[col][j];
        }
      }
    }
    return inv;
  }

  Mat4 inv_{};
};

}  // namespace fly_brain
