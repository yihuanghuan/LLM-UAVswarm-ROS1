#ifndef LADRC_CONTROLLER__STARTUP_STATE_MACHINE_HPP_
#define LADRC_CONTROLLER__STARTUP_STATE_MACHINE_HPP_

#include <algorithm>
#include <cstdint>

namespace ladrc_controller
{

enum class StartupState : uint8_t
{
  WAIT_ESTIMATOR_READY = 0,
  PRESTREAM = 1,
  ARMING = 2,
  SETTING_OFFBOARD = 3,
  TAKING_OFF = 4,
  READY = 5,
  FAILED = 6
};

struct StartupConfig
{
  double estimator_settle_s{10.0};
  double prestream_s{1.5};
  double request_retry_s{1.0};
  double takeoff_hold_s{0.5};
  double runtime_fault_debounce_s{0.5};
  double total_timeout_s{60.0};
  int max_request_attempts{20};
};

struct StartupInputs
{
  bool estimator_ready{false};
  bool armed{false};
  bool offboard{false};
  bool takeoff_stable{false};
  bool feedback_healthy{false};
  bool runtime_healthy{false};
};

struct StartupActions
{
  bool send_arm{false};
  bool send_offboard{false};
  bool capture_ground_hold{false};
  bool capture_takeoff_reference{false};
  bool became_ready{false};
  bool became_failed{false};
  bool state_changed{false};
};

class StartupStateMachine
{
public:
  explicit StartupStateMachine(const StartupConfig & config = StartupConfig())
  : config_(config)
  {
  }

  StartupActions update(double now_s, const StartupInputs & inputs)
  {
    StartupActions actions;
    if (!started_) {
      started_ = true;
      started_s_ = now_s;
    }
    if (state_ == StartupState::FAILED) return actions;

    if (state_ == StartupState::READY) {
      if (inputs.runtime_healthy) {
        unhealthy_since_s_ = -1.0;
        return actions;
      }
      if (unhealthy_since_s_ < 0.0) unhealthy_since_s_ = now_s;
      if (now_s - unhealthy_since_s_ >= config_.runtime_fault_debounce_s) {
        transition(StartupState::FAILED, actions);
        actions.became_failed = true;
      }
      return actions;
    }

    if (now_s - started_s_ >= config_.total_timeout_s) {
      transition(StartupState::FAILED, actions);
      actions.became_failed = true;
      return actions;
    }

    switch (state_) {
      case StartupState::WAIT_ESTIMATOR_READY:
        if (!inputs.estimator_ready) {
          condition_since_s_ = -1.0;
          return actions;
        }
        if (condition_since_s_ < 0.0) condition_since_s_ = now_s;
        if (now_s - condition_since_s_ >= config_.estimator_settle_s) {
          transition(StartupState::PRESTREAM, actions);
          condition_since_s_ = now_s;
          actions.capture_ground_hold = true;
        }
        return actions;

      case StartupState::PRESTREAM:
        if (!inputs.estimator_ready) {
          transition(StartupState::WAIT_ESTIMATOR_READY, actions);
          condition_since_s_ = -1.0;
          return actions;
        }
        if (now_s - condition_since_s_ >= config_.prestream_s) {
          transition(StartupState::ARMING, actions);
          last_request_s_ = now_s - config_.request_retry_s;
        }
        break;

      case StartupState::ARMING:
        if (!inputs.estimator_ready) {
          transition(StartupState::WAIT_ESTIMATOR_READY, actions);
          condition_since_s_ = -1.0;
          return actions;
        }
        if (inputs.armed) {
          transition(StartupState::SETTING_OFFBOARD, actions);
          last_request_s_ = now_s - config_.request_retry_s;
        }
        break;

      case StartupState::SETTING_OFFBOARD:
        if (!inputs.feedback_healthy || !inputs.armed) {
          if (unhealthy_since_s_ < 0.0) unhealthy_since_s_ = now_s;
          if (now_s - unhealthy_since_s_ >= config_.runtime_fault_debounce_s) {
            transition(StartupState::FAILED, actions);
            actions.became_failed = true;
          }
          return actions;
        }
        unhealthy_since_s_ = -1.0;
        if (inputs.offboard) {
          transition(StartupState::TAKING_OFF, actions);
          condition_since_s_ = -1.0;
          actions.capture_takeoff_reference = true;
          return actions;
        }
        break;

      case StartupState::TAKING_OFF:
        if (!inputs.runtime_healthy) {
          if (unhealthy_since_s_ < 0.0) unhealthy_since_s_ = now_s;
          if (now_s - unhealthy_since_s_ >= config_.runtime_fault_debounce_s) {
            transition(StartupState::FAILED, actions);
            actions.became_failed = true;
          }
          return actions;
        }
        unhealthy_since_s_ = -1.0;
        if (!inputs.takeoff_stable) {
          condition_since_s_ = -1.0;
          return actions;
        }
        if (condition_since_s_ < 0.0) condition_since_s_ = now_s;
        if (now_s - condition_since_s_ >= config_.takeoff_hold_s) {
          transition(StartupState::READY, actions);
          actions.became_ready = true;
        }
        return actions;

      case StartupState::READY:
      case StartupState::FAILED:
        return actions;
    }

    if (state_ == StartupState::ARMING && requestDue(now_s)) {
      if (arm_attempts_ >= config_.max_request_attempts) {
        transition(StartupState::FAILED, actions);
        actions.became_failed = true;
      } else {
        actions.send_arm = true;
        ++arm_attempts_;
        last_request_s_ = now_s;
      }
    } else if (state_ == StartupState::SETTING_OFFBOARD && requestDue(now_s)) {
      if (offboard_attempts_ >= config_.max_request_attempts) {
        transition(StartupState::FAILED, actions);
        actions.became_failed = true;
      } else {
        actions.send_offboard = true;
        ++offboard_attempts_;
        last_request_s_ = now_s;
      }
    }
    return actions;
  }

  StartupState state() const {return state_;}
  int armAttempts() const {return arm_attempts_;}
  int offboardAttempts() const {return offboard_attempts_;}

private:
  bool requestDue(double now_s) const
  {
    return now_s - last_request_s_ >= config_.request_retry_s;
  }

  void transition(StartupState next, StartupActions & actions)
  {
    if (state_ == next) return;
    state_ = next;
    actions.state_changed = true;
  }

  StartupConfig config_;
  StartupState state_{StartupState::WAIT_ESTIMATOR_READY};
  bool started_{false};
  double started_s_{0.0};
  double condition_since_s_{-1.0};
  double unhealthy_since_s_{-1.0};
  double last_request_s_{-1.0};
  int arm_attempts_{0};
  int offboard_attempts_{0};
};

inline const char * toString(StartupState state)
{
  switch (state) {
    case StartupState::WAIT_ESTIMATOR_READY: return "WAIT_ESTIMATOR_READY";
    case StartupState::PRESTREAM: return "PRESTREAM";
    case StartupState::ARMING: return "ARMING";
    case StartupState::SETTING_OFFBOARD: return "SETTING_OFFBOARD";
    case StartupState::TAKING_OFF: return "TAKING_OFF";
    case StartupState::READY: return "READY";
    case StartupState::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

}  // namespace ladrc_controller
#endif
