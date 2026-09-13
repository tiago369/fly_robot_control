#pragma once
#include "fly_brain/complementary_filter.hpp"

namespace fly_brain {

struct MotorCommand {
  double thrust = 0.0, tau_x = 0.0, tau_y = 0.0, tau_z = 0.0;
};

struct AttitudeSetpoint {
  double roll = 0.0, pitch = 0.0;   // absolute angles, rad
  double yaw_rate = 0.0;            // commanded rate, rad/s (yaw has no absolute reference)
  double thrust = 0.0;              // total thrust, N
};

// nested classes' default member initializers aren't in the enclosing class's
// complete-class context until the enclosing class finishes, so a member
// function default argument of `Gains{}` fails to compile if Gains is nested;
// keeping it at namespace scope avoids that.
struct HaltereGains {
  double kp_att_roll = 6.0, kp_att_pitch = 6.0;
  double kp_rate_x = 0.08, kd_rate_x = 0.004;
  double kp_rate_y = 0.08, kd_rate_y = 0.004;
  double kp_rate_z = 0.06, kd_rate_z = 0.003;
};

// Haltere-analog: the fast (runs every physics/control step) reflex loop.
// Outer attitude loop is P-only; the rate loop is the actual "haltere" reflex,
// since real halteres primarily encode angular *rate* via Coriolis forces.
class HaltereReflex {
public:
  using Gains = HaltereGains;

  explicit HaltereReflex(Gains gains = Gains{}) : g_(gains) {}

  MotorCommand step(const AttitudeSetpoint& sp, const Attitude& att, const ImuSample& imu,
                     double dt) {
    const double rate_sp_x = g_.kp_att_roll * (sp.roll - att.roll);
    const double rate_sp_y = g_.kp_att_pitch * (sp.pitch - att.pitch);
    const double rate_sp_z = sp.yaw_rate;

    const double err_x = rate_sp_x - imu.wx;
    const double err_y = rate_sp_y - imu.wy;
    const double err_z = rate_sp_z - imu.wz;

    const double dedt_x = dt > 0.0 ? (err_x - prev_err_x_) / dt : 0.0;
    const double dedt_y = dt > 0.0 ? (err_y - prev_err_y_) / dt : 0.0;
    const double dedt_z = dt > 0.0 ? (err_z - prev_err_z_) / dt : 0.0;
    prev_err_x_ = err_x;
    prev_err_y_ = err_y;
    prev_err_z_ = err_z;

    MotorCommand cmd;
    cmd.thrust = sp.thrust;
    cmd.tau_x = g_.kp_rate_x * err_x + g_.kd_rate_x * dedt_x;
    cmd.tau_y = g_.kp_rate_y * err_y + g_.kd_rate_y * dedt_y;
    cmd.tau_z = g_.kp_rate_z * err_z + g_.kd_rate_z * dedt_z;
    return cmd;
  }

private:
  Gains g_;
  double prev_err_x_ = 0.0, prev_err_y_ = 0.0, prev_err_z_ = 0.0;
};

}  // namespace fly_brain
