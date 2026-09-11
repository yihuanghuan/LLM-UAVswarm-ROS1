#include "ladrc_controller/iapf_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ladrc_controller
{
namespace
{

double clamp01(double value)
{
  return std::max(0.0, std::min(1.0, value));
}

Eigen::Vector3d deterministicEscapeDirection(
  const Eigen::Vector3d & own_position,
  const Eigen::Vector3d & own_velocity,
  const NeighborSample & neighbor,
  uint8_t self_uav_id)
{
  const bool self_is_low = self_uav_id < neighbor.id;
  const Eigen::Vector3d canonical_position = self_is_low
    ? neighbor.position - own_position
    : own_position - neighbor.position;
  const Eigen::Vector3d canonical_velocity = self_is_low
    ? neighbor.velocity - own_velocity
    : own_velocity - neighbor.velocity;

  Eigen::Vector3d escape = canonical_position.cross(canonical_velocity);
  if (escape.norm() <= 1e-9) {
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    if (
      canonical_position.norm() <= 1e-9 ||
      std::abs(canonical_position.normalized().dot(axis)) > 0.9)
    {
      axis = Eigen::Vector3d::UnitX();
    }
    escape = canonical_position.cross(axis);
  }
  if (escape.norm() <= 1e-9) {
    escape = Eigen::Vector3d::UnitY();
  } else {
    escape.normalize();
  }
  return (self_is_low ? 1.0 : -1.0) * escape;
}

}  // namespace

AvoidanceMode parseAvoidanceMode(const std::string & value)
{
  if (value == "off") {
    return AvoidanceMode::OFF;
  }
  if (value == "classic_position") {
    return AvoidanceMode::CLASSIC_POSITION;
  }
  if (value == "iapf_position") {
    return AvoidanceMode::IAPF_POSITION;
  }
  if (value == "iapf_dual") {
    return AvoidanceMode::IAPF_DUAL;
  }
  throw std::invalid_argument("invalid avoidance_mode: " + value);
}

EscapeMode parseEscapeMode(const std::string & value)
{
  if (value == "none") {
    return EscapeMode::NONE;
  }
  if (value == "fixed_positive_z") {
    return EscapeMode::FIXED_POSITIVE_Z;
  }
  if (value == "id_order") {
    return EscapeMode::ID_ORDER;
  }
  throw std::invalid_argument("invalid iapf_escape_mode: " + value);
}

std::string toString(AvoidanceMode mode)
{
  switch (mode) {
    case AvoidanceMode::OFF:
      return "off";
    case AvoidanceMode::CLASSIC_POSITION:
      return "classic_position";
    case AvoidanceMode::IAPF_POSITION:
      return "iapf_position";
    case AvoidanceMode::IAPF_DUAL:
      return "iapf_dual";
  }
  throw std::invalid_argument("unknown avoidance mode");
}

Eigen::Vector3d clampNorm(
  const Eigen::Vector3d & value, double limit, bool * saturated)
{
  const double norm = value.norm();
  const bool should_saturate = limit > 0.0 && norm > limit;
  if (saturated != nullptr) {
    *saturated = should_saturate;
  }
  if (!should_saturate) {
    return value;
  }
  return value * (limit / norm);
}

Eigen::Vector3d smoothOffset(
  const Eigen::Vector3d & desired,
  const Eigen::Vector3d & previous,
  double alpha)
{
  if (!(alpha > 0.0 && alpha <= 1.0)) {
    throw std::invalid_argument("IAPF filter alpha must be in (0, 1]");
  }
  return alpha * desired + (1.0 - alpha) * previous;
}

IAPFResult computeIAPF(
  const Eigen::Vector3d & own_position,
  const Eigen::Vector3d & own_velocity,
  uint8_t self_uav_id,
  const std::vector<NeighborSample> & neighbors,
  AvoidanceMode avoidance_mode,
  EscapeMode escape_mode,
  const IAPFParameters & parameters)
{
  if (
    parameters.violation_distance <= 0.0 ||
    parameters.enter_distance <= parameters.violation_distance ||
    parameters.exit_distance <= parameters.enter_distance ||
    parameters.distance_epsilon <= 0.0 ||
    parameters.distance_epsilon >= parameters.violation_distance ||
    parameters.repulsion_gain < 0.0 ||
    parameters.position_gain < 0.0 ||
    parameters.position_limit < 0.0 ||
    parameters.acceleration_gain < 0.0 ||
    parameters.acceleration_limit < 0.0 ||
    parameters.escape_gain < 0.0)
  {
    throw std::invalid_argument("invalid IAPF parameter range");
  }
  IAPFResult result;
  for (const auto & neighbor : neighbors) {
    if (!neighbor.fresh) {
      ++result.stale_neighbor_count;
      continue;
    }
    ++result.valid_neighbor_count;
    const Eigen::Vector3d separation = own_position - neighbor.position;
    const Eigen::Vector3d relative_velocity =
      own_velocity - neighbor.velocity;
    const double distance = separation.norm();
    const double closing_speed = distance > parameters.distance_epsilon
      ? -separation.dot(relative_velocity) / distance
      : 0.0;
    if (!result.has_nearest_neighbor || distance < result.nearest_neighbor_distance) {
      result.has_nearest_neighbor = true;
      result.nearest_neighbor_id = neighbor.id;
      result.nearest_neighbor_distance = distance;
      result.nearest_neighbor_closing_speed = closing_speed;
    }

    if (avoidance_mode == AvoidanceMode::OFF) {
      continue;
    }
    const bool active = neighbor.was_active
      ? distance < parameters.exit_distance
      : distance < parameters.enter_distance &&
      (closing_speed > 0.0 || distance < parameters.violation_distance);
    if (!active) {
      continue;
    }

    result.active = true;
    result.hysteresis_active = true;
    result.active_neighbor_ids.push_back(neighbor.id);
    const double effective_distance =
      std::max(distance, parameters.distance_epsilon);
    Eigen::Vector3d direction;
    if (distance > std::numeric_limits<double>::epsilon()) {
      direction = separation / distance;
    } else {
      const double sign = self_uav_id < neighbor.id ? 1.0 : -1.0;
      direction = Eigen::Vector3d(sign, 0.0, 0.0);
    }

    const double magnitude =
      parameters.repulsion_gain *
      (1.0 / effective_distance - 1.0 / parameters.exit_distance) /
      (effective_distance * effective_distance);
    double attenuation = 1.0;
    if (
      closing_speed <= 0.0 &&
      distance > parameters.violation_distance)
    {
      attenuation = clamp01(
        (parameters.exit_distance - distance) /
        (parameters.exit_distance - parameters.violation_distance));
    }
    Eigen::Vector3d force = direction * magnitude * attenuation;

    if (avoidance_mode != AvoidanceMode::CLASSIC_POSITION) {
      if (escape_mode == EscapeMode::FIXED_POSITIVE_Z) {
        force.z() += magnitude * parameters.escape_gain * attenuation;
      } else if (escape_mode == EscapeMode::ID_ORDER) {
        force += deterministicEscapeDirection(
          own_position, own_velocity, neighbor, self_uav_id) *
          magnitude * parameters.escape_gain * attenuation;
      }
    }
    result.raw_repulsion += force;
  }

  if (avoidance_mode == AvoidanceMode::OFF) {
    result.active = false;
    result.raw_repulsion.setZero();
  }
  result.position_offset = clampNorm(
    parameters.position_gain * result.raw_repulsion,
    parameters.position_limit, &result.position_saturated);
  if (avoidance_mode == AvoidanceMode::IAPF_DUAL) {
    result.acceleration_offset = clampNorm(
      parameters.acceleration_gain * result.raw_repulsion,
      parameters.acceleration_limit, &result.acceleration_saturated);
  }
  return result;
}

}  // namespace ladrc_controller
