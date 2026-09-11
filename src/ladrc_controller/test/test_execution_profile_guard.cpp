#include <gtest/gtest.h>

#include <limits>
#include <string>

#include "ladrc_controller/execution_profile_guard.hpp"

namespace
{

ladrc_controller::ExecutionProfileLimits limits()
{
  return {
    {1.0, 1.0, 1.0}, {10.0, 10.0, 10.0},
    {2.0, 2.0, 2.0}, {30.0, 30.0, 30.0},
    5.0, 4.0, 8.0, 0.75, 1.0, 3.0, 4.0, 2.0};
}

ladrc_controller::ExecutionProfileValues profile()
{
  return {
    5.0, {3.0, 3.0, 3.5}, {10.0, 10.0, 15.0},
    2.0, 1.5, 3.0, 1.5, 1.8, 1.0, 1.0, 1.0};
}

}  // namespace

TEST(ExecutionProfileGuard, RejectsNonFiniteProfile)
{
  auto values = profile();
  values.duration = std::numeric_limits<double>::quiet_NaN();
  std::string error;
  EXPECT_FALSE(ladrc_controller::validateAndClampExecutionProfile(
    values, limits(), &error));
  EXPECT_NE(error.find("non-finite"), std::string::npos);
}

TEST(ExecutionProfileGuard, ClampsToInjectedHardLimits)
{
  auto values = profile();
  values.omega_c[0] = 100.0;
  values.velocity_limit = 20.0;
  values.iapf_repulsion_scale = 5.0;
  EXPECT_TRUE(ladrc_controller::validateAndClampExecutionProfile(values, limits()));
  EXPECT_DOUBLE_EQ(values.omega_c[0], 10.0);
  EXPECT_DOUBLE_EQ(values.velocity_limit, 5.0);
  EXPECT_DOUBLE_EQ(values.iapf_repulsion_scale, 2.0);
}

TEST(ExecutionProfileGuard, RejectsNonFiniteProvenanceGain)
{
  auto values = profile();
  values.task_gain = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(ladrc_controller::validateAndClampExecutionProfile(values, limits()));
}

TEST(ExecutionProfileGuard, RejectsBrokenHysteresisAfterClamp)
{
  auto values = profile();
  values.iapf_enter_distance = 3.0;
  values.iapf_exit_distance = 2.0;
  EXPECT_FALSE(ladrc_controller::validateAndClampExecutionProfile(values, limits()));
}

TEST(ExecutionProfileGuard, AcceptsZeroAndRejectsNegativeRepulsionScale)
{
  auto values = profile();
  values.iapf_repulsion_scale = 0.0;
  EXPECT_TRUE(ladrc_controller::validateAndClampExecutionProfile(values, limits()));
  values.iapf_repulsion_scale = -0.1;
  EXPECT_FALSE(ladrc_controller::validateAndClampExecutionProfile(values, limits()));
}

TEST(ExecutionProfileGuard, RejectsEnterDistanceAtHardThreshold)
{
  auto configured_limits = limits();
  auto values = profile();
  configured_limits.iapf_enter_min = configured_limits.iapf_violation_distance;
  EXPECT_FALSE(ladrc_controller::validateAndClampExecutionProfile(
    values, configured_limits));
}

TEST(ExecutionProfileGuard, SmoothApplyUsesConfiguredAlpha)
{
  EXPECT_DOUBLE_EQ(ladrc_controller::smoothProfileValue(2.0, 4.0, 0.25), 2.5);
  EXPECT_DOUBLE_EQ(ladrc_controller::smoothProfileValue(2.0, 4.0, -1.0), 4.0);
}
