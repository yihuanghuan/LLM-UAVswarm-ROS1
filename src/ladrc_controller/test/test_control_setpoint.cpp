#include <gtest/gtest.h>

#include <cmath>

#include "ladrc_controller/control_setpoint.hpp"
#include "ladrc_controller/ladrc_core.hpp"

using ladrc_controller::ControlMode;

TEST(ControlSetpoint, ParsesSupportedModesAndRejectsUnknown)
{
  EXPECT_EQ(
    ladrc_controller::parseControlMode("px4_position"),
    ControlMode::PX4_POSITION);
  EXPECT_EQ(
    ladrc_controller::parseControlMode("ladrc_acceleration"),
    ControlMode::LADRC_ACCELERATION);
  EXPECT_THROW(
    ladrc_controller::parseControlMode("auto"), std::invalid_argument);
}

TEST(ControlSetpoint, SelectsCorrectOffboardLevel)
{
  const auto position = ladrc_controller::makeOffboardControlMode(
    ControlMode::PX4_POSITION, 10);
  EXPECT_TRUE(position.position);
  EXPECT_FALSE(position.acceleration);

  const auto acceleration = ladrc_controller::makeOffboardControlMode(
    ControlMode::LADRC_ACCELERATION, 20);
  EXPECT_FALSE(acceleration.position);
  EXPECT_FALSE(acceleration.velocity);
  EXPECT_TRUE(acceleration.acceleration);
  EXPECT_FALSE(acceleration.attitude);
  EXPECT_FALSE(acceleration.body_rate);
}

TEST(ControlSetpoint, LadrcModePublishesOnlyConvertedAcceleration)
{
  const auto msg = ladrc_controller::makeTrajectorySetpoint(
    ControlMode::LADRC_ACCELERATION,
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
    0.0, true, 30);
  EXPECT_TRUE(std::isnan(msg.position[0]));
  EXPECT_TRUE(std::isnan(msg.position[1]));
  EXPECT_TRUE(std::isnan(msg.position[2]));
  EXPECT_TRUE(std::isnan(msg.velocity[0]));
  EXPECT_FLOAT_EQ(msg.acceleration[0], 5.0F);
  EXPECT_FLOAT_EQ(msg.acceleration[1], 4.0F);
  EXPECT_FLOAT_EQ(msg.acceleration[2], -6.0F);
}

TEST(ControlSetpoint, PositionBaselineKeepsPositionAndOptionalFeedforward)
{
  const auto without_feedforward = ladrc_controller::makeTrajectorySetpoint(
    ControlMode::PX4_POSITION,
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
    0.0, false, 40);
  EXPECT_FLOAT_EQ(without_feedforward.position[0], 2.0F);
  EXPECT_FLOAT_EQ(without_feedforward.position[1], 1.0F);
  EXPECT_FLOAT_EQ(without_feedforward.position[2], -3.0F);
  EXPECT_TRUE(std::isnan(without_feedforward.acceleration[0]));

  const auto with_feedforward = ladrc_controller::makeTrajectorySetpoint(
    ControlMode::PX4_POSITION,
    1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
    0.0, true, 50);
  EXPECT_FLOAT_EQ(with_feedforward.acceleration[0], 5.0F);
  EXPECT_FLOAT_EQ(with_feedforward.acceleration[1], 4.0F);
  EXPECT_FLOAT_EQ(with_feedforward.acceleration[2], -6.0F);
}

TEST(ControlSetpoint, LadrcUsesSafeAccelerationOnceAndSaturatesOutput)
{
  ladrc_controller::LADRCParams parameters;
  parameters.kp = 0.0;
  parameters.kd = 0.0;
  parameters.b0 = 1.0;
  parameters.min_output = -3.0;
  parameters.max_output = 3.0;
  ladrc_controller::LADRCController controller(parameters);
  controller.setObserverInitialState(0.0, 0.0, 0.0);

  const double nominal_acceleration = 1.25;
  const double iapf_acceleration_offset = 0.50;
  const double safe_acceleration =
    nominal_acceleration + iapf_acceleration_offset;
  EXPECT_DOUBLE_EQ(controller.update(0.0, 0.0, safe_acceleration, 0.0), 1.75);
  EXPECT_DOUBLE_EQ(controller.update(0.0, 0.0, 20.0, 0.0), 3.0);
}
