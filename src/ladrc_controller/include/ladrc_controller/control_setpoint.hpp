#ifndef LADRC_CONTROLLER__CONTROL_SETPOINT_HPP_
#define LADRC_CONTROLLER__CONTROL_SETPOINT_HPP_

#include <array>
#include <limits>
#include <stdexcept>
#include <string>

#include "ladrc_controller/transport_samples.hpp"
#include "ladrc_controller/transport_samples.hpp"

namespace ladrc_controller
{

enum class ControlMode
{
  PX4_POSITION,
  LADRC_ACCELERATION
};

inline ControlMode parseControlMode(const std::string & value)
{
  if (value == "px4_position") {
    return ControlMode::PX4_POSITION;
  }
  if (value == "ladrc_acceleration") {
    return ControlMode::LADRC_ACCELERATION;
  }
  throw std::invalid_argument(
          "control_mode must be 'px4_position' or 'ladrc_acceleration'");
}

inline const char * toString(ControlMode mode)
{
  return mode == ControlMode::LADRC_ACCELERATION ?
         "ladrc_acceleration" : "px4_position";
}

inline std::array<float, 3> enuToNed(double x_enu, double y_enu, double z_enu)
{
  return {
    static_cast<float>(y_enu),
    static_cast<float>(x_enu),
    static_cast<float>(-z_enu)};
}

inline transport_samples::OffboardControlMode makeOffboardControlMode(
  ControlMode mode, uint64_t timestamp)
{
  transport_samples::OffboardControlMode msg{};
  msg.timestamp = timestamp;
  msg.position = mode == ControlMode::PX4_POSITION;
  msg.velocity = false;
  msg.acceleration = mode == ControlMode::LADRC_ACCELERATION;
  msg.attitude = false;
  msg.body_rate = false;
  return msg;
}

inline transport_samples::TrajectorySetpoint makeTrajectorySetpoint(
  ControlMode mode,
  double px_enu, double py_enu, double pz_enu,
  double ax_enu, double ay_enu, double az_enu,
  double yaw_ref, bool position_mode_acceleration_feedforward,
  uint64_t timestamp)
{
  transport_samples::TrajectorySetpoint msg{};
  msg.timestamp = timestamp;
  const float nan = std::numeric_limits<float>::quiet_NaN();

  if (mode == ControlMode::LADRC_ACCELERATION) {
    msg.position = {nan, nan, nan};
    msg.velocity = {nan, nan, nan};
    msg.acceleration = enuToNed(ax_enu, ay_enu, az_enu);
  } else {
    msg.position = enuToNed(px_enu, py_enu, pz_enu);
    msg.velocity = {nan, nan, nan};
    msg.acceleration = position_mode_acceleration_feedforward ?
      enuToNed(ax_enu, ay_enu, az_enu) : std::array<float, 3>{nan, nan, nan};
  }

  msg.yaw = static_cast<float>(yaw_ref);
  return msg;
}

}  // namespace ladrc_controller

#endif  // LADRC_CONTROLLER__CONTROL_SETPOINT_HPP_
