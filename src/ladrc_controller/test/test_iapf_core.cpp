#include <gtest/gtest.h>

#include "ladrc_controller/iapf_core.hpp"

using ladrc_controller::AvoidanceMode;
using ladrc_controller::EscapeMode;
using ladrc_controller::IAPFParameters;
using ladrc_controller::NeighborSample;

TEST(IAPFCore, ParsesOnlySupportedModes)
{
  EXPECT_EQ(ladrc_controller::parseAvoidanceMode("off"), AvoidanceMode::OFF);
  EXPECT_THROW(ladrc_controller::parseAvoidanceMode("unknown"), std::invalid_argument);
  EXPECT_THROW(ladrc_controller::parseEscapeMode("unknown"), std::invalid_argument);
}

TEST(IAPFCore, ClassicHasNoEscapeOrAcceleration)
{
  IAPFParameters parameters;
  const auto result = ladrc_controller::computeIAPF(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1,
    {{2, Eigen::Vector3d(0.9, 0.0, 0.0), Eigen::Vector3d::Zero(), true, false}},
    AvoidanceMode::CLASSIC_POSITION, EscapeMode::ID_ORDER, parameters);
  EXPECT_TRUE(result.active);
  EXPECT_DOUBLE_EQ(result.raw_repulsion.z(), 0.0);
  EXPECT_DOUBLE_EQ(result.acceleration_offset.norm(), 0.0);
}

TEST(IAPFCore, IdOrderEscapeIsPairwiseOpposite)
{
  IAPFParameters parameters;
  const auto first = ladrc_controller::computeIAPF(
    Eigen::Vector3d(-0.5, 0.0, 0.0), Eigen::Vector3d::UnitX(), 1,
    {{2, Eigen::Vector3d(0.5, 0.0, 0.0), -Eigen::Vector3d::UnitX(), true, false}},
    AvoidanceMode::IAPF_DUAL, EscapeMode::ID_ORDER, parameters);
  const auto second = ladrc_controller::computeIAPF(
    Eigen::Vector3d(0.5, 0.0, 0.0), -Eigen::Vector3d::UnitX(), 2,
    {{1, Eigen::Vector3d(-0.5, 0.0, 0.0), Eigen::Vector3d::UnitX(), true, false}},
    AvoidanceMode::IAPF_DUAL, EscapeMode::ID_ORDER, parameters);
  EXPECT_NEAR((first.raw_repulsion + second.raw_repulsion).norm(), 0.0, 1e-9);
}

TEST(IAPFCore, CoincidentNeighborsRemainFiniteAndSaturate)
{
  IAPFParameters parameters;
  const auto result = ladrc_controller::computeIAPF(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1,
    {{2, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), true, false}},
    AvoidanceMode::IAPF_DUAL, EscapeMode::ID_ORDER, parameters);
  EXPECT_TRUE(result.raw_repulsion.allFinite());
  EXPECT_TRUE(result.position_saturated);
  EXPECT_TRUE(result.acceleration_saturated);
  EXPECT_NEAR(result.position_offset.norm(), parameters.position_limit, 1e-9);
  EXPECT_NEAR(result.acceleration_offset.norm(), parameters.acceleration_limit, 1e-9);
}

TEST(IAPFCore, StaleNeighborsDoNotRepel)
{
  IAPFParameters parameters;
  const auto result = ladrc_controller::computeIAPF(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1,
    {{2, Eigen::Vector3d(0.5, 0.0, 0.0), Eigen::Vector3d::Zero(), false, false}},
    AvoidanceMode::IAPF_DUAL, EscapeMode::ID_ORDER, parameters);
  EXPECT_FALSE(result.active);
  EXPECT_EQ(result.valid_neighbor_count, 0);
  EXPECT_EQ(result.stale_neighbor_count, 1);
  EXPECT_DOUBLE_EQ(result.raw_repulsion.norm(), 0.0);
}

TEST(IAPFCore, RejectsUnsafeParameterRanges)
{
  IAPFParameters parameters;
  parameters.distance_epsilon = parameters.violation_distance;
  EXPECT_THROW(
    ladrc_controller::computeIAPF(
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1, {},
      AvoidanceMode::IAPF_DUAL,
      EscapeMode::ID_ORDER, parameters),
    std::invalid_argument);
}

TEST(IAPFCore, HysteresisAndClosingSpeedGateActivation)
{
  IAPFParameters parameters;
  const NeighborSample receding{
    2, Eigen::Vector3d(1.4, 0.0, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0), true, false};
  const auto inactive = ladrc_controller::computeIAPF(
    Eigen::Vector3d::Zero(), Eigen::Vector3d(-1.0, 0.0, 0.0), 1,
    {receding}, AvoidanceMode::IAPF_DUAL, EscapeMode::ID_ORDER,
    parameters);
  EXPECT_FALSE(inactive.active);
  EXPECT_LT(inactive.nearest_neighbor_closing_speed, 0.0);

  auto hysteresis_neighbor = receding;
  hysteresis_neighbor.was_active = true;
  const auto retained = ladrc_controller::computeIAPF(
    Eigen::Vector3d::Zero(), Eigen::Vector3d(-1.0, 0.0, 0.0), 1,
    {hysteresis_neighbor}, AvoidanceMode::IAPF_DUAL, EscapeMode::ID_ORDER,
    parameters);
  EXPECT_TRUE(retained.active);
  EXPECT_GT(retained.raw_repulsion.norm(), 0.0);
}

TEST(IAPFCore, RecedingRepulsionFadesBetweenViolationAndExit)
{
  IAPFParameters parameters;
  const auto near = ladrc_controller::computeIAPF(
    Eigen::Vector3d::Zero(), -Eigen::Vector3d::UnitX(), 1,
    {{2, Eigen::Vector3d(1.1, 0.0, 0.0), Eigen::Vector3d::UnitX(), true, true}},
    AvoidanceMode::IAPF_DUAL, EscapeMode::ID_ORDER, parameters);
  const auto far = ladrc_controller::computeIAPF(
    Eigen::Vector3d::Zero(), -Eigen::Vector3d::UnitX(), 1,
    {{2, Eigen::Vector3d(1.6, 0.0, 0.0), Eigen::Vector3d::UnitX(), true, true}},
    AvoidanceMode::IAPF_DUAL, EscapeMode::ID_ORDER, parameters);
  EXPECT_TRUE(near.active);
  EXPECT_TRUE(far.active);
  EXPECT_GT(near.raw_repulsion.norm(), far.raw_repulsion.norm());
}

TEST(IAPFCore, RepulsionStrengthComesOnlyFromCompiledGain)
{
  IAPFParameters normal;
  IAPFParameters safer = normal;
  safer.repulsion_gain = 1.5 * normal.repulsion_gain;
  const std::vector<NeighborSample> neighbors{{
    2, Eigen::Vector3d(0.9, 0.0, 0.0), Eigen::Vector3d::Zero(), true, false}};
  const auto normal_result = ladrc_controller::computeIAPF(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1, neighbors,
    AvoidanceMode::CLASSIC_POSITION, EscapeMode::NONE, normal);
  const auto safer_result = ladrc_controller::computeIAPF(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 1, neighbors,
    AvoidanceMode::CLASSIC_POSITION, EscapeMode::NONE, safer);
  EXPECT_NEAR(
    safer_result.raw_repulsion.norm(),
    1.5 * normal_result.raw_repulsion.norm(), 1e-9);
}

TEST(IAPFCore, LowPassFilterSmoothsAndValidatesAlpha)
{
  const Eigen::Vector3d desired(1.0, -1.0, 0.5);
  const Eigen::Vector3d previous = Eigen::Vector3d::Zero();
  EXPECT_TRUE(
    ladrc_controller::smoothOffset(desired, previous, 0.2)
      .isApprox(0.2 * desired));
  EXPECT_TRUE(
    ladrc_controller::smoothOffset(desired, previous, 1.0)
      .isApprox(desired));
  EXPECT_THROW(
    ladrc_controller::smoothOffset(desired, previous, 0.0),
    std::invalid_argument);
}
