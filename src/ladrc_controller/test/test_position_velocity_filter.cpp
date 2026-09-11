#include <gtest/gtest.h>

#include "ladrc_controller/position_velocity_filter.hpp"

TEST(PositionVelocityFilter, RejectsRawVelocityBiasWhenPositionIsStationary)
{
  ladrc_controller::PositionVelocityFilter filter(0.2);
  for (int index = 0; index <= 100; ++index)
  {
    filter.update(index * 0.01, 0, Eigen::Vector3d(1.0, 2.0, 5.0));
  }
  ASSERT_TRUE(filter.valid());
  EXPECT_NEAR(filter.speed(), 0.0, 1e-9);
}

TEST(PositionVelocityFilter, TracksPositionMotion)
{
  ladrc_controller::PositionVelocityFilter filter(0.1);
  for (int index = 0; index <= 200; ++index)
  {
    const double time = index * 0.01;
    filter.update(time, 0, Eigen::Vector3d(time, 0.0, 0.0));
  }
  ASSERT_TRUE(filter.valid());
  EXPECT_NEAR(filter.velocity().x(), 1.0, 1e-3);
}

TEST(PositionVelocityFilter, ResetsAcrossEstimatorFrameReset)
{
  ladrc_controller::PositionVelocityFilter filter(0.1);
  filter.update(0.0, 0, Eigen::Vector3d::Zero());
  filter.update(0.1, 0, Eigen::Vector3d(0.1, 0.0, 0.0));
  ASSERT_TRUE(filter.valid());
  filter.update(0.2, 1, Eigen::Vector3d(10.0, 0.0, 0.0));
  EXPECT_FALSE(filter.valid());
  EXPECT_NEAR(filter.speed(), 0.0, 1e-9);
}
