#include <gtest/gtest.h>

#include "ladrc_controller/hover_stability.hpp"

using ladrc_controller::HoverStabilityState;
using ladrc_controller::HoverStabilityThresholds;
using ladrc_controller::updateHoverStability;

TEST(HoverStability, AcceptanceBoundaryEntersAndConfirms)
{
  HoverStabilityState state;
  const HoverStabilityThresholds thresholds;
  updateHoverStability(state, 0.40, 0.30, 5.0, thresholds);
  EXPECT_EQ(state.state, 1);
  updateHoverStability(state, 0.45, 0.35, 6.0, thresholds);
  EXPECT_EQ(state.state, 2);
  EXPECT_TRUE(state.confirmed());
}

TEST(HoverStability, HysteresisBandCannotEnterFromUnstable)
{
  HoverStabilityState state;
  const HoverStabilityThresholds thresholds;
  updateHoverStability(state, 0.45, 0.35, 1.0, thresholds);
  EXPECT_EQ(state.state, 0);
}

TEST(HoverStability, ConfirmedStateExitsAndCanReenter)
{
  HoverStabilityState state;
  const HoverStabilityThresholds thresholds;
  updateHoverStability(state, 0.30, 0.20, 1.0, thresholds);
  updateHoverStability(state, 0.45, 0.35, 2.0, thresholds);
  ASSERT_TRUE(state.confirmed());
  updateHoverStability(state, 0.51, 0.20, 2.1, thresholds);
  EXPECT_EQ(state.state, 0);
  updateHoverStability(state, 0.40, 0.30, 3.0, thresholds);
  updateHoverStability(state, 0.40, 0.30, 4.0, thresholds);
  EXPECT_TRUE(state.confirmed());
}

TEST(HoverStability, ResetClearsCandidateAndConfirmedState)
{
  HoverStabilityState state;
  const HoverStabilityThresholds thresholds;
  updateHoverStability(state, 0.30, 0.20, 1.0, thresholds);
  ASSERT_TRUE(state.candidate_since.has_value());
  state.reset();
  EXPECT_EQ(state.state, 0);
  EXPECT_FALSE(state.candidate_since.has_value());
  EXPECT_FALSE(state.confirmed());
}
