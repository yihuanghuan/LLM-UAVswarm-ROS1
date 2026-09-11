#ifndef LADRC_CONTROLLER__EXECUTION_PROFILE_GUARD_HPP_
#define LADRC_CONTROLLER__EXECUTION_PROFILE_GUARD_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace ladrc_controller
{

struct ExecutionProfileValues
{
  double duration{0.0};
  std::array<double, 3> omega_c{};
  std::array<double, 3> omega_o{};
  double velocity_limit{0.0};
  double acceleration_limit{0.0};
  double jerk_limit{0.0};
  double iapf_enter_distance{0.0};
  double iapf_exit_distance{0.0};
  double iapf_repulsion_scale{0.0};
  double style_gain{0.0};
  double task_gain{0.0};
};

struct ExecutionProfileLimits
{
  std::array<double, 3> omega_c_min{};
  std::array<double, 3> omega_c_max{};
  std::array<double, 3> omega_o_min{};
  std::array<double, 3> omega_o_max{};
  double velocity_max{0.0};
  double acceleration_max{0.0};
  double jerk_max{0.0};
  double iapf_violation_distance{0.0};
  double iapf_enter_min{0.0};
  double iapf_enter_max{0.0};
  double iapf_exit_max{0.0};
  double iapf_repulsion_max{0.0};
};

inline bool validLimits(const ExecutionProfileLimits & limits)
{
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (!(limits.omega_c_min[axis] > 0.0 &&
      limits.omega_c_max[axis] >= limits.omega_c_min[axis] &&
      limits.omega_o_min[axis] > 0.0 &&
      limits.omega_o_max[axis] >= limits.omega_o_min[axis]))
    {
      return false;
    }
  }
  return limits.velocity_max > 0.0 && limits.acceleration_max > 0.0 &&
    limits.jerk_max > 0.0 && limits.iapf_violation_distance > 0.0 &&
    limits.iapf_enter_min > limits.iapf_violation_distance &&
    limits.iapf_enter_max >= limits.iapf_enter_min &&
    limits.iapf_exit_max > limits.iapf_enter_min &&
    limits.iapf_repulsion_max > 0.0;
}

inline bool validateAndClampExecutionProfile(
  ExecutionProfileValues & profile,
  const ExecutionProfileLimits & limits,
  std::string * error = nullptr)
{
  if (!validLimits(limits)) {
    if (error != nullptr) *error = "invalid configured execution profile limits";
    return false;
  }
  const auto finite_positive = [](double value) {
      return std::isfinite(value) && value > 0.0;
    };
  if (!finite_positive(profile.duration) ||
    !finite_positive(profile.velocity_limit) ||
    !finite_positive(profile.acceleration_limit) ||
    !finite_positive(profile.jerk_limit) ||
    !finite_positive(profile.iapf_enter_distance) ||
    !finite_positive(profile.iapf_exit_distance) ||
    !finite_positive(profile.style_gain) ||
    !finite_positive(profile.task_gain))
  {
    if (error != nullptr) *error = "profile contains non-finite or non-positive values";
    return false;
  }
  if (!std::isfinite(profile.iapf_repulsion_scale) ||
    profile.iapf_repulsion_scale < 0.0)
  {
    if (error != nullptr) *error = "profile repulsion scale is invalid";
    return false;
  }
  for (std::size_t axis = 0; axis < 3; ++axis) {
    if (!finite_positive(profile.omega_c[axis]) ||
      !finite_positive(profile.omega_o[axis]))
    {
      if (error != nullptr) *error = "profile bandwidth is invalid";
      return false;
    }
    profile.omega_c[axis] = std::clamp(
      profile.omega_c[axis], limits.omega_c_min[axis], limits.omega_c_max[axis]);
    profile.omega_o[axis] = std::clamp(
      profile.omega_o[axis], limits.omega_o_min[axis], limits.omega_o_max[axis]);
  }
  profile.velocity_limit = std::min(profile.velocity_limit, limits.velocity_max);
  profile.acceleration_limit = std::min(
    profile.acceleration_limit, limits.acceleration_max);
  profile.jerk_limit = std::min(profile.jerk_limit, limits.jerk_max);
  profile.iapf_enter_distance = std::clamp(
    profile.iapf_enter_distance, limits.iapf_enter_min, limits.iapf_enter_max);
  profile.iapf_exit_distance = std::min(
    profile.iapf_exit_distance, limits.iapf_exit_max);
  profile.iapf_repulsion_scale = std::min(
    profile.iapf_repulsion_scale, limits.iapf_repulsion_max);
  if (profile.iapf_enter_distance <= limits.iapf_violation_distance ||
    profile.iapf_exit_distance <= profile.iapf_enter_distance)
  {
    if (error != nullptr) {
      *error = "clamped IAPF distances violate hard threshold or hysteresis";
    }
    return false;
  }
  return true;
}

inline double smoothProfileValue(double previous, double requested, double alpha)
{
  if (!(std::isfinite(alpha) && alpha > 0.0 && alpha <= 1.0)) {
    return requested;
  }
  if (!std::isfinite(previous) || previous <= 0.0) {
    return requested;
  }
  return alpha * requested + (1.0 - alpha) * previous;
}

}  // namespace ladrc_controller

#endif  // LADRC_CONTROLLER__EXECUTION_PROFILE_GUARD_HPP_
