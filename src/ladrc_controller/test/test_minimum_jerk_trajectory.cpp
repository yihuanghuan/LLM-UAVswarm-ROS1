#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "ladrc_controller/minimum_jerk_trajectory.hpp"


using ladrc_controller::MinimumJerkTrajectory;


TEST(MinimumJerkTrajectory, RejectsInvalidDuration)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  for (const double duration : {0.0, -1.0, nan, infinity})
  {
    MinimumJerkTrajectory trajectory;
    EXPECT_THROW(trajectory.initialize(0.0, 1.0, duration), std::invalid_argument);
    EXPECT_FALSE(trajectory.isInitialized());
  }
}


TEST(MinimumJerkTrajectory, SatisfiesEndpointBoundaryConditions)
{
  MinimumJerkTrajectory trajectory;
  trajectory.initialize(-2.5, 4.0, 3.0);

  const auto start = trajectory.evaluate(0.0);
  const auto end = trajectory.evaluate(3.0);
  EXPECT_DOUBLE_EQ(start.position, -2.5);
  EXPECT_DOUBLE_EQ(start.velocity, 0.0);
  EXPECT_DOUBLE_EQ(start.acceleration, 0.0);
  EXPECT_NEAR(end.position, 4.0, 1e-12);
  EXPECT_NEAR(end.velocity, 0.0, 1e-12);
  EXPECT_NEAR(end.acceleration, 0.0, 1e-12);
}
