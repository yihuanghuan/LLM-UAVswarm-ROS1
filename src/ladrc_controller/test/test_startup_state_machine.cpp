#include <gtest/gtest.h>
#include "ladrc_controller/startup_state_machine.hpp"

using ladrc_controller::StartupConfig;
using ladrc_controller::StartupInputs;
using ladrc_controller::StartupState;
using ladrc_controller::StartupStateMachine;

namespace
{
StartupConfig fastConfig()
{
  StartupConfig config;
  config.estimator_settle_s = 1.0;
  config.prestream_s = 1.5;
  config.request_retry_s = 1.0;
  config.takeoff_hold_s = 0.5;
  config.runtime_fault_debounce_s = 0.5;
  config.total_timeout_s = 30.0;
  config.max_request_attempts = 5;
  return config;
}
}

TEST(StartupStateMachine, RequiresContinuousEstimatorSettleAndPrestream)
{
  StartupStateMachine machine(fastConfig());
  StartupInputs inputs;
  EXPECT_EQ(machine.state(), StartupState::WAIT_ESTIMATOR_READY);
  inputs.estimator_ready = true;
  machine.update(0.0, inputs);
  inputs.estimator_ready = false;
  machine.update(0.8, inputs);
  inputs.estimator_ready = true;
  machine.update(1.0, inputs);
  auto actions = machine.update(2.0, inputs);
  EXPECT_EQ(machine.state(), StartupState::PRESTREAM);
  EXPECT_TRUE(actions.capture_ground_hold);
  EXPECT_FALSE(machine.update(3.49, inputs).send_arm);
  actions = machine.update(3.50, inputs);
  EXPECT_TRUE(actions.send_arm);
  EXPECT_EQ(machine.state(), StartupState::ARMING);
}

TEST(StartupStateMachine, ConfirmsFeedbackThenTakeoffHold)
{
  StartupStateMachine machine(fastConfig());
  StartupInputs inputs{true, false, false, false, true, false};
  machine.update(0.0, inputs);
  machine.update(1.0, inputs);
  machine.update(2.5, inputs);
  inputs.armed = true;
  EXPECT_TRUE(machine.update(2.6, inputs).send_offboard);
  EXPECT_EQ(machine.state(), StartupState::SETTING_OFFBOARD);
  inputs.offboard = true;
  inputs.runtime_healthy = true;
  auto actions = machine.update(2.7, inputs);
  EXPECT_TRUE(actions.capture_takeoff_reference);
  EXPECT_EQ(machine.state(), StartupState::TAKING_OFF);
  inputs.takeoff_stable = true;
  EXPECT_FALSE(machine.update(3.0, inputs).became_ready);
  EXPECT_TRUE(machine.update(3.5, inputs).became_ready);
  EXPECT_EQ(machine.state(), StartupState::READY);
}

TEST(StartupStateMachine, LatchesRuntimeFaultWithoutRecoveryActions)
{
  StartupConfig config = fastConfig();
  config.estimator_settle_s = 0.0;
  config.prestream_s = 0.0;
  config.takeoff_hold_s = 0.0;
  StartupStateMachine machine(config);
  StartupInputs inputs{true, false, false, false, true, false};
  machine.update(0.0, inputs);
  machine.update(0.0, inputs);
  machine.update(0.0, inputs);
  inputs.armed = true;
  machine.update(0.1, inputs);
  inputs.offboard = true;
  inputs.runtime_healthy = true;
  machine.update(0.2, inputs);
  inputs.takeoff_stable = true;
  EXPECT_TRUE(machine.update(0.3, inputs).became_ready);
  inputs.runtime_healthy = false;
  EXPECT_FALSE(machine.update(0.4, inputs).became_failed);
  auto actions = machine.update(0.9, inputs);
  EXPECT_TRUE(actions.became_failed);
  EXPECT_FALSE(actions.send_arm);
  EXPECT_FALSE(actions.send_offboard);
  EXPECT_EQ(machine.state(), StartupState::FAILED);
  inputs.runtime_healthy = true;
  EXPECT_EQ(machine.update(2.0, inputs).state_changed, false);
  EXPECT_EQ(machine.state(), StartupState::FAILED);
}

TEST(StartupStateMachine, LatchesArmLossWhileWaitingForOffboard)
{
  StartupConfig config = fastConfig();
  config.estimator_settle_s = 0.0;
  config.prestream_s = 0.0;
  StartupStateMachine machine(config);
  StartupInputs inputs{true, false, false, false, true, false};
  machine.update(0.0, inputs);
  machine.update(0.0, inputs);
  machine.update(0.0, inputs);
  inputs.armed = true;
  machine.update(0.1, inputs);
  EXPECT_EQ(machine.state(), StartupState::SETTING_OFFBOARD);
  inputs.armed = false;
  EXPECT_FALSE(machine.update(0.2, inputs).became_failed);
  auto actions = machine.update(0.71, inputs);
  EXPECT_TRUE(actions.became_failed);
  EXPECT_FALSE(actions.send_arm);
  EXPECT_FALSE(actions.send_offboard);
  EXPECT_EQ(machine.state(), StartupState::FAILED);
}

TEST(StartupStateMachine, StopsAfterFiniteArmRetries)
{
  StartupConfig config = fastConfig();
  config.estimator_settle_s = 0.0;
  config.prestream_s = 0.0;
  StartupStateMachine machine(config);
  StartupInputs inputs{true, false, false, false, true, false};
  machine.update(0.0, inputs);
  for (int second = 0; second < 5; ++second) {
    machine.update(static_cast<double>(second), inputs);
  }
  EXPECT_EQ(machine.armAttempts(), 5);
  EXPECT_TRUE(machine.update(5.0, inputs).became_failed);
  EXPECT_EQ(machine.state(), StartupState::FAILED);
}

TEST(StartupStateMachine, EnforcesTotalTimeout)
{
  StartupStateMachine machine(fastConfig());
  StartupInputs inputs;
  machine.update(0.0, inputs);
  EXPECT_TRUE(machine.update(30.0, inputs).became_failed);
  EXPECT_EQ(machine.state(), StartupState::FAILED);
}
