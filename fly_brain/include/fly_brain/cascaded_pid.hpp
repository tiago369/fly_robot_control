#pragma once
#include "fly_brain/haltere_reflex.hpp"

namespace fly_brain {

// Conventional (non-bio-inspired) baseline: altitude PID feeding thrust, plus
// an independent attitude->rate PD stage (deliberately separate from
// HaltereReflex, even though structurally similar) so it stays a fair,
// unmodified comparison point for the bio-inspired controllers.
class CascadedPid {
public:
  struct Gains {
    double kp_z = 12.0, ki_z = 2.0, kd_z = 8.0;
    double hover_thrust = 0.0;  // feedforward, set to vehicle weight (N)
    double kp_att_roll = 6.0, kp_att_pitch = 6.0;
    double kp_rate_x = 0.08, kd_rate_x = 0.004;
    double kp_rate_y = 0.08, kd_rate_y = 0.004;
    double kp_rate_z = 0.06, kd_rate_z = 0.003;
  };

  explicit CascadedPid(Gains gains) : g_(gains) {}

  MotorCommand step(double z_setpoint, double z, double roll_setpoint, double pitch_setpoint,
                     double yaw_rate_setpoint, const Attitude& att, const ImuSample& imu,
                     double dt) {
    const double err_z = z_setpoint - z;
    z_integral_ += err_z * dt;
    const double z_dot = dt > 0.0 ? (err_z - prev_err_z_) / dt : 0.0;
    prev_err_z_ = err_z;
    const double thrust =
        g_.hover_thrust + g_.kp_z * err_z + g_.ki_z * z_integral_ + g_.kd_z * z_dot;

    const double rate_sp_x = g_.kp_att_roll * (roll_setpoint - att.roll);
    const double rate_sp_y = g_.kp_att_pitch * (pitch_setpoint - att.pitch);
    const double rate_sp_z = yaw_rate_setpoint;

    const double err_x = rate_sp_x - imu.wx;
    const double err_y = rate_sp_y - imu.wy;
    const double err_yaw = rate_sp_z - imu.wz;
    const double dedt_x = dt > 0.0 ? (err_x - prev_err_x_) / dt : 0.0;
    const double dedt_y = dt > 0.0 ? (err_y - prev_err_y_) / dt : 0.0;
    const double dedt_yaw = dt > 0.0 ? (err_yaw - prev_err_yaw_) / dt : 0.0;
    prev_err_x_ = err_x;
    prev_err_y_ = err_y;
    prev_err_yaw_ = err_yaw;

    MotorCommand cmd;
    cmd.thrust = thrust;
    cmd.tau_x = g_.kp_rate_x * err_x + g_.kd_rate_x * dedt_x;
    cmd.tau_y = g_.kp_rate_y * err_y + g_.kd_rate_y * dedt_y;
    cmd.tau_z = g_.kp_rate_z * err_yaw + g_.kd_rate_z * dedt_yaw;
    return cmd;
  }

private:
  Gains g_;
  double z_integral_ = 0.0;
  double prev_err_z_ = 0.0, prev_err_x_ = 0.0, prev_err_y_ = 0.0, prev_err_yaw_ = 0.0;
};

}  // namespace fly_brain
