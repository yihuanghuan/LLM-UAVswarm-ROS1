#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <Eigen/Dense>

namespace ladrc_controller
{

class PositionVelocityFilter
{
public:
  explicit PositionVelocityFilter(double time_constant = 0.5)
  : time_constant_(std::max(time_constant, 1e-3)) {}

  void setTimeConstant(double time_constant)
  {
    time_constant_ = std::max(time_constant, 1e-3);
  }

  void reset(double timestamp, uint8_t reset_counter,
             const Eigen::Vector3d & position)
  {
    previous_timestamp_ = timestamp;
    previous_position_ = position;
    reset_counter_ = reset_counter;
    velocity_.setZero();
    initialized_ = true;
    valid_ = false;
  }

  void update(double timestamp, uint8_t reset_counter,
              const Eigen::Vector3d & position)
  {
    if (!initialized_ || reset_counter != reset_counter_ ||
        !std::isfinite(timestamp) || !position.allFinite())
    {
      reset(timestamp, reset_counter, position);
      return;
    }

    const double dt = timestamp - previous_timestamp_;
    if (dt <= 1e-4 || dt > 0.5)
    {
      reset(timestamp, reset_counter, position);
      return;
    }

    const Eigen::Vector3d raw_velocity =
      (position - previous_position_) / dt;
    const double alpha = 1.0 - std::exp(-dt / time_constant_);
    velocity_ += alpha * (raw_velocity - velocity_);
    previous_timestamp_ = timestamp;
    previous_position_ = position;
    valid_ = true;
  }

  const Eigen::Vector3d & velocity() const { return velocity_; }
  double speed() const { return velocity_.norm(); }
  bool valid() const { return valid_; }

private:
  double time_constant_{0.5};
  double previous_timestamp_{0.0};
  uint8_t reset_counter_{0};
  Eigen::Vector3d previous_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_{Eigen::Vector3d::Zero()};
  bool initialized_{false};
  bool valid_{false};
};

}  // namespace ladrc_controller
