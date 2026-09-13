#pragma once
#include <cmath>

namespace fly_brain {

struct ImuSample {
  double wx, wy, wz;  // body angular velocity, rad/s (gyro)
  double ax, ay, az;  // body specific force, m/s^2 (accelerometer)
};

struct Attitude {
  double roll = 0.0;   // phi, rad
  double pitch = 0.0;  // theta, rad
  double yaw = 0.0;    // psi, rad (gyro-integrated only, no yaw reference sensor)
};

// Fuses gyro (fast, drift-prone) with accelerometer tilt (slow, noise-prone but
// drift-free) the way halteres' fast mechanosensory signal needs a slower
// reference to stay bounded over a flight. alpha close to 1 trusts the gyro
// short-term and lets the accelerometer correct long-term drift.
class ComplementaryFilter {
public:
  explicit ComplementaryFilter(double alpha = 0.98) : alpha_(alpha) {}

  const Attitude& update(const ImuSample& imu, double dt) {
    // body-rate -> Euler-rate kinematics (REP103: x fwd, y left, z up)
    const double sphi = std::sin(att_.roll), cphi = std::cos(att_.roll);
    const double cth = std::cos(att_.pitch);
    const double tth = std::tan(att_.pitch);

    const double roll_dot = imu.wx + imu.wy * sphi * tth + imu.wz * cphi * tth;
    const double pitch_dot = imu.wy * cphi - imu.wz * sphi;
    const double safe_cth = std::abs(cth) < 1e-6 ? (cth < 0 ? -1e-6 : 1e-6) : cth;
    const double yaw_dot = (imu.wy * sphi + imu.wz * cphi) / safe_cth;

    const double roll_gyro = att_.roll + roll_dot * dt;
    const double pitch_gyro = att_.pitch + pitch_dot * dt;

    const double roll_acc = std::atan2(imu.ay, imu.az);
    const double norm_xz = std::sqrt(imu.ay * imu.ay + imu.az * imu.az);
    const double pitch_acc = std::atan2(-imu.ax, norm_xz);

    att_.roll = alpha_ * roll_gyro + (1.0 - alpha_) * roll_acc;
    att_.pitch = alpha_ * pitch_gyro + (1.0 - alpha_) * pitch_acc;
    att_.yaw = att_.yaw + yaw_dot * dt;  // no absolute yaw reference available
    return att_;
  }

  const Attitude& attitude() const { return att_; }

private:
  double alpha_;
  Attitude att_{};
};

}  // namespace fly_brain
