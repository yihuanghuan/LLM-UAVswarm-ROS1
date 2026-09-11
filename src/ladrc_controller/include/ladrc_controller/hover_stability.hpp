#pragma once

#include <cstdint>
#include <optional>

namespace ladrc_controller
{

struct HoverStabilityThresholds
{
  double position_enter{0.40};
  double velocity_enter{0.30};
  double position_exit{0.50};
  double velocity_exit{0.40};
  double hold_time{1.0};
};

struct HoverStabilityState
{
  uint8_t state{0};
  std::optional<double> candidate_since;

  void reset()
  {
    state = 0;
    candidate_since.reset();
  }

  bool confirmed() const { return state == 2; }
};

inline void updateHoverStability(
  HoverStabilityState & stability,
  double position_error,
  double speed,
  double now_seconds,
  const HoverStabilityThresholds & thresholds)
{
  const bool should_exit =
    position_error > thresholds.position_exit ||
    speed > thresholds.velocity_exit;
  const bool should_enter =
    position_error <= thresholds.position_enter &&
    speed <= thresholds.velocity_enter;

  if (should_exit)
  {
    stability.reset();
  }
  else if (stability.state == 0 && should_enter)
  {
    stability.state = 1;
    stability.candidate_since = now_seconds;
  }
  else if (
    stability.state == 1 && stability.candidate_since.has_value() &&
    now_seconds - stability.candidate_since.value() >= thresholds.hold_time)
  {
    stability.state = 2;
  }
}

}  // namespace ladrc_controller
