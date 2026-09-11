#include "ladrc_controller/ros1_node.hpp"
#include "ladrc_controller/transport_samples.hpp"
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <uav_swarm_interfaces/UAVExecutionCommand.h>
#include <uav_swarm_interfaces/UAVStatus.h>
#include <uav_swarm_interfaces/StartupEvent.h>
#include <uav_swarm_interfaces/TrajectoryMetrics.h>
#include <uav_swarm_interfaces/ControlAdaptationLog.h>
#include <uav_swarm_interfaces/IAPFDebug.h>
#include <uav_swarm_interfaces/ControlTrackingDebug.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include "ladrc_controller/ladrc_core.hpp"
#include "ladrc_controller/control_setpoint.hpp"
#include "ladrc_controller/iapf_core.hpp"
#include "ladrc_controller/minimum_jerk_trajectory.hpp"
#include "ladrc_controller/execution_profile_guard.hpp"
#include "ladrc_controller/swarm_state_builder.hpp"
#include "ladrc_controller/startup_state_machine.hpp"
#include "ladrc_controller/hover_stability.hpp"
#include "ladrc_controller/position_velocity_filter.hpp"
#include <cmath>
#include <chrono>
#include <atomic>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <unordered_map>
#include <Eigen/Dense>

using namespace std::chrono_literals;

// 自动起飞状态机
class LADRCPositionControllerNode : public ros1_support::Node
{
  struct NeighborState
  {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    ros::Time receive_time;
    bool iapf_active{false};
  };

public:
  LADRCPositionControllerNode()
      : Node("ladrc_position_controller")
  {
    // 声明参数
    this->declare_parameter("control_frequency", 50.0);
    this->declare_parameter("control_mode", "px4_position");
    this->declare_parameter<double>("omega_o_x");
    this->declare_parameter<double>("omega_o_y");
    this->declare_parameter<double>("omega_o_z");
    this->declare_parameter<double>("omega_c_x");
    this->declare_parameter<double>("omega_c_y");
    this->declare_parameter<double>("omega_c_z");
    this->declare_parameter("b0_x", 1.0);
    this->declare_parameter("b0_y", 1.0);
    this->declare_parameter("b0_z", 1.0);
    this->declare_parameter("max_velocity", 5.0);
    this->declare_parameter("max_acceleration_x", 3.0);
    this->declare_parameter("max_acceleration_y", 3.0);
    this->declare_parameter("max_acceleration_z", 3.0);
    this->declare_parameter("hover_position_enter_tolerance", 0.40);
    this->declare_parameter("hover_velocity_enter_tolerance", 0.30);
    this->declare_parameter("hover_position_exit_tolerance", 0.50);
    this->declare_parameter("hover_velocity_exit_tolerance", 0.40);
    this->declare_parameter("hover_stable_hold_time", 1.0);
    this->declare_parameter("hover_velocity_filter_tau", 0.5);
    this->declare_parameter("startup_settle_time", 10.0);
    this->declare_parameter("startup_speed_tolerance", 0.15);
    this->declare_parameter("startup_odom_timeout", 0.5);
    this->declare_parameter("startup_status_timeout", 2.0);
    this->declare_parameter("startup_prestream_time", 1.5);
    this->declare_parameter("startup_command_retry_interval", 1.0);
    this->declare_parameter("startup_takeoff_altitude", 1.5);
    this->declare_parameter("startup_takeoff_position_tolerance", 0.25);
    this->declare_parameter("startup_takeoff_hold_time", 0.5);
    this->declare_parameter("startup_runtime_fault_debounce", 0.5);
    this->declare_parameter("startup_total_timeout", 60.0);
    this->declare_parameter("startup_max_request_attempts", 20);
    this->declare_parameter("enable_execution_profiles", false);
    this->declare_parameter("execution_profile_smoothing_alpha", -1.0);
    this->declare_parameter("execution_profile_omega_c_min", std::vector<double>{});
    this->declare_parameter("execution_profile_omega_c_max", std::vector<double>{});
    this->declare_parameter("execution_profile_omega_o_min", std::vector<double>{});
    this->declare_parameter("execution_profile_omega_o_max", std::vector<double>{});
    this->declare_parameter("execution_profile_velocity_max", -1.0);
    this->declare_parameter("execution_profile_acceleration_max", -1.0);
    this->declare_parameter("execution_profile_jerk_max", -1.0);
    this->declare_parameter("execution_profile_iapf_enter_min", -1.0);
    this->declare_parameter("execution_profile_iapf_enter_max", -1.0);
    this->declare_parameter("execution_profile_iapf_exit_max", -1.0);
    this->declare_parameter("execution_profile_iapf_repulsion_max", -1.0);

    // Gazebo 多机 spawn 偏移量（sitl_multiple_run.sh 默认 Y=3*instance）
    this->declare_parameter("enu_offset_x", 0.0);
    this->declare_parameter("enu_offset_y", 0.0);
    this->declare_parameter("enu_offset_z", 0.0);
    this->declare_parameter("px4_target_system", 0);

    // [Phase 4] IAPF 避障参数
    this->declare_parameter("avoidance_mode", "iapf_dual");
    this->declare_parameter("iapf_violation_distance", 1.0);
    this->declare_parameter("iapf_enter_distance", 1.5);
    this->declare_parameter("iapf_exit_distance", 1.65);
    this->declare_parameter("iapf_filter_alpha", 0.20);
    this->declare_parameter("iapf_repulsion_gain", 1.0);
    this->declare_parameter("iapf_escape_mode", "id_order");
    this->declare_parameter("iapf_escape_gain", 0.05);
    this->declare_parameter("iapf_distance_epsilon", 0.10);
    this->declare_parameter("iapf_position_gain", 0.05);
    this->declare_parameter("iapf_position_limit", 0.50);
    this->declare_parameter("iapf_accel_gain", 0.3);
    this->declare_parameter("iapf_accel_limit", 2.0);
    this->declare_parameter("neighbor_timeout", 0.20);
    this->declare_parameter("neighbor_uav_ids", std::vector<int64_t>{});
    this->declare_parameter(
        "control_adaptation_log_path",
        defaultControlAdaptationLogPath());

    (void)currentAvoidanceMode();
    (void)ladrc_controller::parseEscapeMode(
      this->get_parameter("iapf_escape_mode").as_string());

    // 获取参数
    double control_freq = this->get_parameter("control_frequency").as_double();
    dt_ = 1.0 / control_freq;
    control_mode_ = ladrc_controller::parseControlMode(
      this->get_parameter("control_mode").as_string());
    // 初始化 LADRC 控制器
    initializeControllers();

    execution_command_sub_ =
      this->create_subscription<uav_swarm_interfaces::UAVExecutionCommand>(
        "execution_command", ros1_support::QoS(10),
        std::bind(
          &LADRCPositionControllerNode::executionCommandCallback,
          this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::Odometry>(
        "mavros/local_position/odom",
        ros1_support::SensorDataQoS(),
        std::bind(&LADRCPositionControllerNode::odomCallback, this, std::placeholders::_1));
    vehicle_status_sub_ = this->create_subscription<mavros_msgs::State>(
        "mavros/state", ros1_support::SensorDataQoS(),
        std::bind(
          &LADRCPositionControllerNode::vehicleStatusCallback,
          this, std::placeholders::_1));

    // 从命名空间提取自身 UAV ID（例如 /uav3 → 3）
    std::string ns = this->get_namespace();
    size_t uav_pos = ns.find("/uav");
    if (uav_pos != std::string::npos)
    {
      std::string id_str = ns.substr(uav_pos + 4);  // "/uav" 后 4 个字符
      // 去掉可能的尾部斜杠
      while (!id_str.empty() && id_str.back() == '/') id_str.pop_back();
      try { self_uav_id_ = static_cast<uint8_t>(std::stoi(id_str)); }
      catch (...) { self_uav_id_ = 0; }
    }

    // --- [Phase 4] 邻居无人机 Odometry 订阅 ---
    auto neighbor_ids = this->get_parameter("neighbor_uav_ids").as_integer_array();
    for (auto id : neighbor_ids)
    {
      uint8_t neighbor_id = static_cast<uint8_t>(id);
      if (neighbor_id == 0 || neighbor_id == self_uav_id_) continue;  // 跳过无效 ID 和自身
      configured_neighbor_ids_.push_back(neighbor_id);

      auto callback = [this, neighbor_id](const nav_msgs::Odometry::ConstPtr ros_msg) {
        const auto sample = toInternalOdometry(*ros_msg);
        const auto* msg = &sample;
        // 存入邻居位置 map：全局 ENU（本地 + spawn 偏移 Y=3*id）
        const auto previous = neighbor_states_.find(neighbor_id);
        const bool was_active =
          previous != neighbor_states_.end() && previous->second.iapf_active;
        neighbor_states_[neighbor_id] = NeighborState{
          Eigen::Vector3d(
            msg->position[1],
            msg->position[0] + 3.0 * neighbor_id,
            -msg->position[2]),
          Eigen::Vector3d(
            msg->velocity[1],
            msg->velocity[0],
            -msg->velocity[2]),
          this->now(),
          was_active};
      };

      auto sub = this->create_subscription<nav_msgs::Odometry>(
          "/uav" + std::to_string(neighbor_id) + "/mavros/local_position/odom",
          ros1_support::SensorDataQoS(),
          callback);
      neighbor_subs_.push_back(sub);
    }
    RCLCPP_INFO(this->get_logger(), "已创建 %zu 个邻居 Odom 订阅", neighbor_subs_.size());

    // --- [Phase 1] 新增状态发布器 ---
    status_pub_ = this->create_publisher<uav_swarm_interfaces::UAVStatus>(
        "status", 10);
    startup_event_pub_ =
      this->create_publisher<uav_swarm_interfaces::StartupEvent>(
        "startup_event", ros1_support::QoS(20).transient_local());

    // 低频 ENU 位置发布器（供调度层获取真实坐标）
    swarm_state_pub_ = this->create_publisher<nav_msgs::Odometry>(
        "swarm_state", ros1_support::SensorDataQoS());

    // 低频轨迹指标发布器（供外部订阅查看 Minimum Jerk 编译结果）
    trajectory_metrics_pub_ =
        this->create_publisher<uav_swarm_interfaces::TrajectoryMetrics>(
            "trajectory_metrics", 10);

    control_adaptation_pub_ =
        this->create_publisher<uav_swarm_interfaces::ControlAdaptationLog>(
            "control_adaptation", 10);
    iapf_debug_pub_ =
        this->create_publisher<uav_swarm_interfaces::IAPFDebug>(
            "iapf_debug", 10);
    control_tracking_debug_pub_ =
        this->create_publisher<uav_swarm_interfaces::ControlTrackingDebug>(
            "control_tracking_debug", ros1_support::SensorDataQoS());

    // Publishers — [Phase 1] 使用相对话题以支持命名空间
    // 必须使用 SensorDataQoS (Best Effort)，PX4 XRCE-DDS 桥接器默认使用 Best Effort 订阅
    // 使用默认 Reliable QoS 会导致静默无法匹配，收不到数据
    trajectory_pub_ = this->create_publisher<mavros_msgs::PositionTarget>(
        "mavros/setpoint_raw/local", 1);
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 控制循环定时器
    auto control_timer_period = std::chrono::duration<double>(dt_);
    control_timer_ = this->create_wall_timer(
        control_timer_period,
        std::bind(&LADRCPositionControllerNode::controlLoop, this));

    // 状态机定时器 (10 Hz)
    auto command_timer_period = std::chrono::milliseconds(100);
    command_timer_ = this->create_wall_timer(
        command_timer_period,
        std::bind(&LADRCPositionControllerNode::stateMachine, this));
    position_velocity_filter_.setTimeConstant(
      this->get_parameter("hover_velocity_filter_tau").as_double());
    ladrc_controller::StartupConfig startup_config;
    startup_config.estimator_settle_s =
      this->get_parameter("startup_settle_time").as_double();
    startup_config.prestream_s =
      this->get_parameter("startup_prestream_time").as_double();
    startup_config.request_retry_s =
      this->get_parameter("startup_command_retry_interval").as_double();
    startup_config.takeoff_hold_s =
      this->get_parameter("startup_takeoff_hold_time").as_double();
    startup_config.runtime_fault_debounce_s =
      this->get_parameter("startup_runtime_fault_debounce").as_double();
    startup_config.total_timeout_s =
      this->get_parameter("startup_total_timeout").as_double();
    startup_config.max_request_attempts = static_cast<int>(
      this->get_parameter("startup_max_request_attempts").as_int());
    startup_ = ladrc_controller::StartupStateMachine(startup_config);

    RCLCPP_INFO(this->get_logger(), "LADRC 集群执行节点已初始化 (命名空间: %s), ENU偏移=[%.1f, %.1f, %.1f]",
        this->get_namespace(),
        this->get_parameter("enu_offset_x").as_double(),
        this->get_parameter("enu_offset_y").as_double(),
        this->get_parameter("enu_offset_z").as_double());
    RCLCPP_INFO(this->get_logger(), "等待 execution_command 和 MAVROS odometry 消息...");
    RCLCPP_INFO(this->get_logger(), "控制模式: %s",
        ladrc_controller::toString(control_mode_));
  }

private:
  static transport_samples::Odometry toInternalOdometry(const nav_msgs::Odometry& message) {
    transport_samples::Odometry sample;
    sample.position = {message.pose.pose.position.y, message.pose.pose.position.x, -message.pose.pose.position.z};
    // MAVROS local_position/odom twist is expressed in child (body) frame.
    const auto& q = message.pose.pose.orientation;
    Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    const auto& v = message.twist.twist.linear;
    Eigen::Vector3d enu = rotation * Eigen::Vector3d(v.x,v.y,v.z);
    sample.velocity = {enu.y(), enu.x(), -enu.z()};
    sample.timestamp_sample = message.header.stamp.toNSec()/1000;
    return sample;
  }
  void initializeControllers()
  {
    ladrc_controller::LADRCParams params_x, params_y, params_z;

    double max_acc_x = this->get_parameter("max_acceleration_x").as_double();
    double max_acc_y = this->get_parameter("max_acceleration_y").as_double();
    double max_acc_z = this->get_parameter("max_acceleration_z").as_double();

    // X-axis controller
    params_x.omega_o = this->get_parameter("omega_o_x").as_double();
    params_x.omega_c = this->get_parameter("omega_c_x").as_double();
    params_x.kp = params_x.omega_c * params_x.omega_c;
    params_x.kd = 2.0 * params_x.omega_c;
    params_x.b0 = this->get_parameter("b0_x").as_double();
    params_x.dt = dt_;
    params_x.max_output = max_acc_x;
    params_x.min_output = -max_acc_x;

    // Y-axis controller
    params_y.omega_o = this->get_parameter("omega_o_y").as_double();
    params_y.omega_c = this->get_parameter("omega_c_y").as_double();
    params_y.kp = params_y.omega_c * params_y.omega_c;
    params_y.kd = 2.0 * params_y.omega_c;
    params_y.b0 = this->get_parameter("b0_y").as_double();
    params_y.dt = dt_;
    params_y.max_output = max_acc_y;
    params_y.min_output = -max_acc_y;

    // Z-axis controller
    params_z.omega_o = this->get_parameter("omega_o_z").as_double();
    params_z.omega_c = this->get_parameter("omega_c_z").as_double();
    params_z.kp = params_z.omega_c * params_z.omega_c;
    params_z.kd = 2.0 * params_z.omega_c;
    params_z.b0 = this->get_parameter("b0_z").as_double();
    params_z.dt = dt_;
    params_z.max_output = max_acc_z;
    params_z.min_output = -max_acc_z;

    ladrc_x_ = std::make_unique<ladrc_controller::LADRCController>(params_x);
    ladrc_y_ = std::make_unique<ladrc_controller::LADRCController>(params_y);
    ladrc_z_ = std::make_unique<ladrc_controller::LADRCController>(params_z);
  }

  bool readyForCommand(uint8_t message_uav_id)
  {
    if (startup_.state() == ladrc_controller::StartupState::READY && odomFresh())
    {
      return true;
    }
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "UAV%d 尚未就绪（状态=%d, odom=%d），缓存最新合法命令", message_uav_id,
        static_cast<int>(startup_.state()), has_odom_);
    return false;
  }

  bool loadExecutionProfileLimits(
      ladrc_controller::ExecutionProfileLimits & limits)
  {
    const auto to_array = [this](
      const std::string & name, std::array<double, 3> & result) {
        const auto values = this->get_parameter(name).as_double_array();
        if (values.size() != 3) return false;
        std::copy(values.begin(), values.end(), result.begin());
        return true;
      };
    if (!to_array("execution_profile_omega_c_min", limits.omega_c_min) ||
      !to_array("execution_profile_omega_c_max", limits.omega_c_max) ||
      !to_array("execution_profile_omega_o_min", limits.omega_o_min) ||
      !to_array("execution_profile_omega_o_max", limits.omega_o_max))
    {
      return false;
    }
    limits.velocity_max =
      this->get_parameter("execution_profile_velocity_max").as_double();
    limits.acceleration_max =
      this->get_parameter("execution_profile_acceleration_max").as_double();
    limits.jerk_max =
      this->get_parameter("execution_profile_jerk_max").as_double();
    limits.iapf_violation_distance =
      this->get_parameter("iapf_violation_distance").as_double();
    limits.iapf_enter_min =
      this->get_parameter("execution_profile_iapf_enter_min").as_double();
    limits.iapf_enter_max =
      this->get_parameter("execution_profile_iapf_enter_max").as_double();
    limits.iapf_exit_max =
      this->get_parameter("execution_profile_iapf_exit_max").as_double();
    limits.iapf_repulsion_max =
      this->get_parameter("execution_profile_iapf_repulsion_max").as_double();
    return ladrc_controller::validLimits(limits);
  }

  void initializeAcceptedCommand(
      uint32_t mission_id, uint8_t uav_id,
      double global_x, double global_y, double global_z,
      double duration, const std::string & style)
  {
    if (has_command_) writeControlAdaptationCsvRow();
    resetIAPFState();
    mission_id_ = mission_id;
    uav_id_ = uav_id;
    target_duration_ = duration;
    motion_style_ = style;
    has_command_ = true;

    const double off_x = this->get_parameter("enu_offset_x").as_double();
    const double off_y = this->get_parameter("enu_offset_y").as_double();
    const double off_z = this->get_parameter("enu_offset_z").as_double();
    target_pos_x_ = global_x - off_x;
    target_pos_y_ = global_y - off_y;
    target_pos_z_ = global_z - off_z;

    const double p0_x = current_odom_.position[1];
    const double p0_y = current_odom_.position[0];
    const double p0_z = -current_odom_.position[2];
    const double dx = target_pos_x_ - p0_x;
    const double dy = target_pos_y_ - p0_y;
    const double dz = target_pos_z_ - p0_z;
    target_distance_ = std::sqrt(dx * dx + dy * dy + dz * dz);
    average_speed_ = target_distance_ / target_duration_;

    traj_x_.initialize(p0_x, target_pos_x_, target_duration_);
    traj_y_.initialize(p0_y, target_pos_y_, target_duration_);
    traj_z_.initialize(p0_z, target_pos_z_, target_duration_);
    initializeTrajectoryMetrics(
      p0_x, p0_y, p0_z, global_x, global_y, global_z);
    command_start_time_ = this->now();
    is_hover_stable_ = false;
    hover_stability_.reset();
    latest_position_error_ = std::numeric_limits<double>::infinity();
    latest_raw_ekf_speed_ = std::numeric_limits<double>::infinity();
    latest_position_derived_speed_ = std::numeric_limits<double>::infinity();
    arrival_time_recorded_ = false;
    arrival_time_error_ = std::numeric_limits<double>::quiet_NaN();
    resetControlAdaptationRuntimeMetrics();
    trajectory_metrics_pub_counter_ = 0;
  }

  void executionCommandCallback(
      const uav_swarm_interfaces::UAVExecutionCommand::ConstPtr msg)
  {
    if (!this->get_parameter("enable_execution_profiles").as_bool())
    {
      RCLCPP_ERROR(this->get_logger(),
        "收到 UAVExecutionCommand，但 enable_execution_profiles=false");
      return;
    }
    ladrc_controller::ExecutionProfileLimits limits;
    ladrc_controller::ExecutionProfileValues values{
      msg->profile.duration,
      {msg->profile.omega_c[0], msg->profile.omega_c[1], msg->profile.omega_c[2]},
      {msg->profile.omega_o[0], msg->profile.omega_o[1], msg->profile.omega_o[2]},
      msg->profile.velocity_limit, msg->profile.acceleration_limit,
      msg->profile.jerk_limit, msg->profile.iapf_enter_distance,
      msg->profile.iapf_exit_distance, msg->profile.iapf_repulsion_scale,
      msg->profile.style_gain, msg->profile.task_gain};
    std::string validation_error;
    const bool valid =
      msg->uav_id == static_cast<uint8_t>(self_uav_id_) &&
      std::isfinite(msg->target_pos.x) &&
      std::isfinite(msg->target_pos.y) &&
      std::isfinite(msg->target_pos.z) &&
      !msg->profile.style.empty() &&
      !msg->profile.configuration_id.empty() &&
      loadExecutionProfileLimits(limits) &&
      ladrc_controller::validateAndClampExecutionProfile(
        values, limits, &validation_error);
    if (!valid) {
      RCLCPP_ERROR(
        this->get_logger(), "拒绝非法 Execution Profile，未缓存: %s",
        validation_error.c_str());
      return;
    }
    if (!readyForCommand(msg->uav_id)) {
      pending_execution_command_ = msg;
      publishStartupEvent("pending_command_received");
      return;
    }
    applyExecutionCommand(msg);
  }

  void applyExecutionCommand(
      const uav_swarm_interfaces::UAVExecutionCommand::ConstPtr msg)
  {

    ladrc_controller::ExecutionProfileLimits limits;
    if (!loadExecutionProfileLimits(limits))
    {
      RCLCPP_ERROR(this->get_logger(),
        "Execution Profile hard limits 未完整配置，拒绝新命令");
      return;
    }
    ladrc_controller::ExecutionProfileValues values{
      msg->profile.duration,
      {msg->profile.omega_c[0], msg->profile.omega_c[1], msg->profile.omega_c[2]},
      {msg->profile.omega_o[0], msg->profile.omega_o[1], msg->profile.omega_o[2]},
      msg->profile.velocity_limit,
      msg->profile.acceleration_limit,
      msg->profile.jerk_limit,
      msg->profile.iapf_enter_distance,
      msg->profile.iapf_exit_distance,
      msg->profile.iapf_repulsion_scale,
      msg->profile.style_gain,
      msg->profile.task_gain};
    if (msg->uav_id != static_cast<uint8_t>(self_uav_id_) ||
      !std::isfinite(msg->target_pos.x) ||
      !std::isfinite(msg->target_pos.y) ||
      !std::isfinite(msg->target_pos.z) ||
      msg->profile.style.empty() || msg->profile.configuration_id.empty())
    {
      RCLCPP_ERROR(this->get_logger(),
        "拒绝 Execution Profile: command metadata is incomplete or non-finite");
      return;
    }
    std::string error;
    if (!ladrc_controller::validateAndClampExecutionProfile(values, limits, &error))
    {
      RCLCPP_ERROR(this->get_logger(),
        "拒绝 Execution Profile: %s", error.c_str());
      return;
    }

    const double alpha =
      this->get_parameter("execution_profile_smoothing_alpha").as_double();
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
      const std::array<double, 3> previous_c{omega_c_x_, omega_c_y_, omega_c_z_};
      const std::array<double, 3> previous_o{omega_o_x_, omega_o_y_, omega_o_z_};
      values.omega_c[axis] = ladrc_controller::smoothProfileValue(
        previous_c[axis], values.omega_c[axis], alpha);
      values.omega_o[axis] = ladrc_controller::smoothProfileValue(
        previous_o[axis], values.omega_o[axis], alpha);
    }

    initializeAcceptedCommand(
      msg->mission_id, msg->uav_id,
      msg->target_pos.x, msg->target_pos.y, msg->target_pos.z,
      values.duration, msg->profile.style);
    omega_c_x_ = values.omega_c[0];
    omega_c_y_ = values.omega_c[1];
    omega_c_z_ = values.omega_c[2];
    omega_o_x_ = values.omega_o[0];
    omega_o_y_ = values.omega_o[1];
    omega_o_z_ = values.omega_o[2];
    ladrc_x_->setControllerBandwidth(omega_c_x_);
    ladrc_y_->setControllerBandwidth(omega_c_y_);
    ladrc_z_->setControllerBandwidth(omega_c_z_);
    ladrc_x_->setObserverBandwidth(omega_o_x_);
    ladrc_y_->setObserverBandwidth(omega_o_y_);
    ladrc_z_->setObserverBandwidth(omega_o_z_);
    ladrc_x_->setOutputLimits(-values.acceleration_limit, values.acceleration_limit);
    ladrc_y_->setOutputLimits(-values.acceleration_limit, values.acceleration_limit);
    ladrc_z_->setOutputLimits(-values.acceleration_limit, values.acceleration_limit);
    gain_multiplier_ = values.style_gain * values.task_gain;
    profile_iapf_enter_distance_ = values.iapf_enter_distance;
    profile_iapf_exit_distance_ = values.iapf_exit_distance;
    profile_iapf_repulsion_scale_ = values.iapf_repulsion_scale;
    profile_soft_safety_active_ = true;
    active_profile_configuration_id_ = msg->profile.configuration_id;
    // A fresh false sample acknowledges the new command generation. The
    // Candidate FSM will not accept a stale true from the preceding task.
    publishUAVStatus();
    RCLCPP_INFO(this->get_logger(),
      "应用 Execution Profile mission=%u task=%u config=%s T_exec=%.3f",
      msg->mission_id, msg->task_id,
      active_profile_configuration_id_.c_str(), target_duration_);
    publishStartupEvent("command_accepted");
    publishStartupEvent("mission_trajectory_started");
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr ros_msg)
  {
    const auto sample = toInternalOdometry(*ros_msg);
    const auto* msg = &sample;
    RCLCPP_INFO_ONCE(this->get_logger(), "已接收到 vehicle_odometry 消息");
    const bool first_valid_odom = !has_odom_;
    current_odom_ = *msg;
    has_odom_ = true;
    last_odom_receive_time_s_ = this->now().toSec();
    const Eigen::Vector3d position_enu(
      msg->position[1], msg->position[0], -msg->position[2]);
    position_velocity_filter_.update(
      static_cast<double>(msg->timestamp_sample) * 1e-6,
      msg->reset_counter, position_enu);
    if (first_valid_odom) {
      ladrc_x_->setObserverInitialState(
        msg->position[1], msg->velocity[1], 0.0);
      ladrc_y_->setObserverInitialState(
        msg->position[0], msg->velocity[0], 0.0);
      ladrc_z_->setObserverInitialState(
        -msg->position[2], -msg->velocity[2], 0.0);
    }

    // Standardized Candidate planning state. PX4 timestamps are boot-clock
    // microseconds, so the normalization boundary stamps the received sample
    // with this node's ROS clock instead of pretending it is ROS epoch time.
    auto state = ladrc_controller::buildSwarmState(
      *msg, this->now(), self_uav_id_,
      this->get_parameter("enu_offset_x").as_double(),
      this->get_parameter("enu_offset_y").as_double(),
      this->get_parameter("enu_offset_z").as_double());
    swarm_state_pub_->publish(state);
  }

  void vehicleStatusCallback(const mavros_msgs::State::ConstPtr msg)
  {
    // MAVROS State has no PX4 preflight bit. Connection + fresh finite odometry
    // gates requests; PX4 arming service and armed feedback remain authoritative.
    current_vehicle_status_.failsafe = !msg->connected || msg->system_status == 5 || msg->system_status == 6;
    current_vehicle_status_.pre_flight_checks_pass = msg->connected;
    has_vehicle_status_ = true;
    last_vehicle_status_receive_time_s_ = this->now().toSec();
    px4_armed_ = msg->armed;
    px4_offboard_ = msg->mode == "OFFBOARD";
  }

  bool odomFresh() const
  {
    return has_odom_ &&
      this->now().toSec() - last_odom_receive_time_s_ <=
      this->get_parameter("startup_odom_timeout").as_double();
  }

  bool vehicleStatusFresh() const
  {
    return has_vehicle_status_ &&
      this->now().toSec() - last_vehicle_status_receive_time_s_ <=
      this->get_parameter("startup_status_timeout").as_double();
  }

  bool estimatorReady() const
  {
    return odomFresh() && vehicleStatusFresh() &&
      !current_vehicle_status_.failsafe &&
      current_vehicle_status_.pre_flight_checks_pass &&
      position_velocity_filter_.valid() &&
      position_velocity_filter_.speed() <=
      this->get_parameter("startup_speed_tolerance").as_double();
  }

  bool runtimeHealthy() const
  {
    return feedbackHealthy() && px4_armed_ && px4_offboard_;
  }

  bool feedbackHealthy() const
  {
    return odomFresh() && vehicleStatusFresh() &&
      !current_vehicle_status_.failsafe;
  }

  bool takeoffStable() const
  {
    if (!odomFresh() || !takeoff_reference_set_ ||
      !position_velocity_filter_.valid())
    {
      return false;
    }
    const Eigen::Vector3d position(
      current_odom_.position[1], current_odom_.position[0],
      -current_odom_.position[2]);
    const Eigen::Vector3d reference(
      hover_hold_x_, hover_hold_y_, hover_hold_z_);
    return (reference - position).norm() <=
      this->get_parameter("startup_takeoff_position_tolerance").as_double() &&
      position_velocity_filter_.speed() <=
      this->get_parameter("startup_speed_tolerance").as_double();
  }

  // --- 状态机逻辑 ---
  void publishVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param7 = 0.0)
  {
    if (command == 400) {
      mavros_msgs::CommandBool service; service.request.value = param1 != 0.0;
      arming_client_.call(service);
    } else if (command == 176) {
      mavros_msgs::SetMode service; service.request.custom_mode = "OFFBOARD";
      mode_client_.call(service);
    }
  }

  void activatePendingCommand()
  {
    if (pending_execution_command_) {
      auto command = pending_execution_command_;
      pending_execution_command_.reset();
      applyExecutionCommand(command);
    }
  }

  void publishStartupEvent(const std::string & event)
  {
    uav_swarm_interfaces::StartupEvent msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "local_enu";
    msg.uav_id = self_uav_id_;
    msg.mission_id = mission_id_;
    msg.event = event;
    msg.startup_state = static_cast<uint8_t>(startup_.state());
    msg.system_ready =
      startup_.state() == ladrc_controller::StartupState::READY;
    msg.armed = px4_armed_;
    msg.offboard = px4_offboard_;
    msg.failsafe = has_vehicle_status_ && current_vehicle_status_.failsafe;
    msg.pre_flight_checks_pass = has_vehicle_status_ &&
      current_vehicle_status_.pre_flight_checks_pass;
    msg.altitude = has_odom_ ? static_cast<float>(-current_odom_.position[2]) :
      std::numeric_limits<float>::quiet_NaN();
    msg.position_derived_speed = position_velocity_filter_.valid() ?
      static_cast<float>(position_velocity_filter_.speed()) :
      std::numeric_limits<float>::quiet_NaN();
    msg.has_pending_command =
      static_cast<bool>(pending_execution_command_);
    startup_event_pub_->publish(msg);
  }

  void stateMachine()
  {
    const auto previous_state = startup_.state();
    const ladrc_controller::StartupInputs inputs{
      estimatorReady(), px4_armed_, px4_offboard_, takeoffStable(),
      feedbackHealthy(), runtimeHealthy()};
    const auto actions = startup_.update(this->now().toSec(), inputs);
    if (actions.capture_ground_hold && odomFresh()) {
      hover_hold_x_ = current_odom_.position[1];
      hover_hold_y_ = current_odom_.position[0];
      hover_hold_z_ = -current_odom_.position[2];
      hover_hold_set_ = true;
      publishStartupEvent("estimator_ready");
      publishStartupEvent("prestream_started");
    }
    if (actions.capture_takeoff_reference && odomFresh()) {
      hover_hold_x_ = current_odom_.position[1];
      hover_hold_y_ = current_odom_.position[0];
      hover_hold_z_ =
        this->get_parameter("startup_takeoff_altitude").as_double();
      hover_hold_set_ = true;
      takeoff_reference_set_ = true;
      publishStartupEvent("offboard_confirmed");
      publishStartupEvent("takeoff_started");
    }
    if (actions.send_arm) {
      RCLCPP_INFO(this->get_logger(), "请求 ARM（第 %d 次）", startup_.armAttempts());
      publishVehicleCommand(
        400, 1.0);
      publishStartupEvent("arm_requested");
    }
    if (actions.send_offboard) {
      RCLCPP_INFO(this->get_logger(), "请求 OFFBOARD（第 %d 次）", startup_.offboardAttempts());
      publishVehicleCommand(
        176, 1.0, 6.0);
      publishStartupEvent("offboard_requested");
    }
    if (actions.became_ready) {
      RCLCPP_INFO(
        this->get_logger(),
        "自动起飞稳定保持完成，进入 READY");
      publishStartupEvent("takeoff_stable");
      publishStartupEvent("system_ready");
      activatePendingCommand();
    }
    if (actions.became_failed) {
      has_command_ = false;
      pending_execution_command_.reset();
      is_hover_stable_ = false;
      if (has_odom_) {
        hover_hold_x_ = current_odom_.position[1];
        hover_hold_y_ = current_odom_.position[0];
        hover_hold_z_ = -current_odom_.position[2];
        hover_hold_set_ = true;
      }
      RCLCPP_ERROR(
        this->get_logger(),
        "STARTUP_FAILED 已锁止；不会自动 DISARM、重新 ARM 或重新进入 Offboard "
        "(armed=%d offboard=%d arm_attempts=%d offboard_attempts=%d)",
        px4_armed_, px4_offboard_, startup_.armAttempts(),
        startup_.offboardAttempts());
      publishStartupEvent("startup_failed_latched");
    } else if (actions.state_changed && previous_state != startup_.state()) {
      if (startup_.state() == ladrc_controller::StartupState::SETTING_OFFBOARD) {
        publishStartupEvent("arm_confirmed");
      }
    }
    publishUAVStatus();
  }

  void controlLoop()
  {
    // 持续发布 offboard 模式
    publishOffboardControlMode();

    if (!odomFresh())
    {
      publishUAVStatus();
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "等待新鲜 VehicleOdometry（startup=%s armed=%d offboard=%d）",
        ladrc_controller::toString(startup_.state()), px4_armed_, px4_offboard_);
      return;
    }

    // 1. 获取测量值 (Odom) 并转换为 ENU
    double x_meas = current_odom_.position[1];
    double y_meas = current_odom_.position[0];
    double z_meas = -current_odom_.position[2];

    // Prestream/arming/offboard confirmation uses a frozen, valid setpoint.
    // Once TAKING_OFF starts, the reference below is executed by the selected
    // backend: PX4 position mode or the unchanged LADRC acceleration pipeline.
    if (startup_.state() == ladrc_controller::StartupState::WAIT_ESTIMATOR_READY ||
      startup_.state() == ladrc_controller::StartupState::PRESTREAM ||
      startup_.state() == ladrc_controller::StartupState::ARMING ||
      startup_.state() == ladrc_controller::StartupState::SETTING_OFFBOARD)
    {
      if (!hover_hold_set_) {
        hover_hold_x_ = x_meas;
        hover_hold_y_ = y_meas;
        hover_hold_z_ = z_meas;
        hover_hold_set_ = true;
      }
      publishTrajectorySetpoint(
        hover_hold_x_, hover_hold_y_, hover_hold_z_,
        0.0, 0.0, 0.0, 0.0);
      publishUAVStatus();
      return;
    }

    // 2. 生成标称参考。无任务时锁定首次测量位置，仍然执行完整闭环。
    double elapsed = 0.0;
    bool all_finished = false;
    Eigen::Vector3d nominal_reference;
    Eigen::Vector3d nominal_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d nominal_acceleration = Eigen::Vector3d::Zero();
    if (!has_command_)
    {
      if (!hover_hold_set_)
      {
        hover_hold_x_ = x_meas;
        hover_hold_y_ = y_meas;
        hover_hold_z_ = z_meas;
        hover_hold_set_ = true;
        RCLCPP_INFO(this->get_logger(),
            "UAV%d 悬停保持锁定: [%.2f, %.2f, %.2f]", self_uav_id_, x_meas, y_meas, z_meas);
      }
      nominal_reference = Eigen::Vector3d(
        hover_hold_x_, hover_hold_y_, hover_hold_z_);
    }
    else
    {
      hover_hold_set_ = false;
      elapsed = (this->now() - command_start_time_).toSec();
      all_finished =
        traj_x_.isFinished(elapsed) &&
        traj_y_.isFinished(elapsed) &&
        traj_z_.isFinished(elapsed);
      const auto ref_x = traj_x_.evaluate(elapsed);
      const auto ref_y = traj_y_.evaluate(elapsed);
      const auto ref_z = traj_z_.evaluate(elapsed);
      nominal_reference = Eigen::Vector3d(
        ref_x.position, ref_y.position, ref_z.position);
      nominal_velocity = Eigen::Vector3d(
        ref_x.velocity, ref_y.velocity, ref_z.velocity);
      nominal_acceleration = Eigen::Vector3d(
        ref_x.acceleration, ref_y.acceleration, ref_z.acceleration);

      updateControlAdaptationRuntimeMetrics(
        elapsed, nominal_reference.x(), nominal_reference.y(), nominal_reference.z(),
        x_meas, y_meas, z_meas);

      // 轨迹结束后 Minimum Jerk 返回终点、零速度和零加速度，继续位置调节。
      if (all_finished)
      {
        const double pos_err = (nominal_reference -
          Eigen::Vector3d(x_meas, y_meas, z_meas)).norm();
        const double raw_ekf_speed = std::sqrt(
          current_odom_.velocity[0] * current_odom_.velocity[0] +
          current_odom_.velocity[1] * current_odom_.velocity[1] +
          current_odom_.velocity[2] * current_odom_.velocity[2]);
        const double position_derived_speed = position_velocity_filter_.valid()
          ? position_velocity_filter_.speed()
          : std::numeric_limits<double>::infinity();
        latest_position_error_ = pos_err;
        latest_raw_ekf_speed_ = raw_ekf_speed;
        latest_position_derived_speed_ = position_derived_speed;
        const bool was_confirmed = hover_stability_.confirmed();
        ladrc_controller::updateHoverStability(
          hover_stability_, pos_err, position_derived_speed,
          this->now().toSec(),
          {
            this->get_parameter(
              "hover_position_enter_tolerance").as_double(),
            this->get_parameter(
              "hover_velocity_enter_tolerance").as_double(),
            this->get_parameter(
              "hover_position_exit_tolerance").as_double(),
            this->get_parameter(
              "hover_velocity_exit_tolerance").as_double(),
            this->get_parameter("hover_stable_hold_time").as_double()
          });
        is_hover_stable_ = hover_stability_.confirmed();
        if (!was_confirmed && is_hover_stable_)
        {
          if (!arrival_time_recorded_)
          {
            arrival_time_error_ = elapsed - target_duration_;
            arrival_time_recorded_ = true;
          }
          settling_time_ = elapsed;
          writeControlAdaptationCsvRow();
          RCLCPP_INFO(this->get_logger(),
              "悬停稳定确认! pos_err=%.2fm, pos_vel=%.2fm/s, "
              "raw_ekf_vel=%.2fm/s → is_hover_stable=true",
              pos_err, position_derived_speed, raw_ekf_speed);
        }
      }

      if (++trajectory_metrics_pub_counter_ >= 5)
      {
        trajectory_metrics_pub_counter_ = 0;
        publishTrajectoryMetrics(elapsed, x_meas, y_meas, z_meas, all_finished);
        publishControlAdaptationLog();
      }
    }

    // 3. IAPF 先修正参考，再由 LADRC 计算唯一的最终加速度指令。
    const auto iapf = computeAvoidance(x_meas, y_meas, z_meas);
    const Eigen::Vector3d safe_reference =
      nominal_reference + iapf.position_offset;
    const Eigen::Vector3d safe_acceleration =
      nominal_acceleration + iapf.acceleration_offset;
    const Eigen::Vector3d ladrc_output(
      ladrc_x_->update(
        safe_reference.x(), nominal_velocity.x(), safe_acceleration.x(), x_meas),
      ladrc_y_->update(
        safe_reference.y(), nominal_velocity.y(), safe_acceleration.y(), y_meas),
      ladrc_z_->update(
        safe_reference.z(), nominal_velocity.z(), safe_acceleration.z(), z_meas));

    const Eigen::Vector3d actual_position(x_meas, y_meas, z_meas);
    const Eigen::Vector3d actual_velocity(
      current_odom_.velocity[1], current_odom_.velocity[0],
      -current_odom_.velocity[2]);
    if (!nominal_reference.allFinite() || !nominal_velocity.allFinite() ||
      !safe_reference.allFinite() || !safe_acceleration.allFinite() ||
      !ladrc_output.allFinite() || !actual_position.allFinite() ||
      !actual_velocity.allFinite())
    {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "检测到非有限控制量，已阻止 TrajectorySetpoint 发布");
      return;
    }

    // 4. position 基线发布安全位置；LADRC 模式只发布 LADRC 加速度。
    publishUAVStatus();
    const bool position_mode_acceleration_feedforward =
      currentAvoidanceMode() == ladrc_controller::AvoidanceMode::IAPF_DUAL;
    const Eigen::Vector3d published_acceleration =
      control_mode_ == ladrc_controller::ControlMode::LADRC_ACCELERATION ?
      ladrc_output : safe_acceleration;
    const auto px4_setpoint = publishTrajectorySetpoint(
      safe_reference.x(), safe_reference.y(), safe_reference.z(),
      published_acceleration.x(), published_acceleration.y(),
      published_acceleration.z(), 0.0,
      position_mode_acceleration_feedforward);
    const Eigen::Vector3d global_offset(
      this->get_parameter("enu_offset_x").as_double(),
      this->get_parameter("enu_offset_y").as_double(),
      this->get_parameter("enu_offset_z").as_double());
    publishIAPFDebug(
      iapf, nominal_reference + global_offset,
      safe_reference + global_offset,
      nominal_acceleration, safe_acceleration);
    publishControlTrackingDebug(
      nominal_reference, safe_reference, nominal_velocity,
      nominal_acceleration, safe_acceleration, ladrc_output,
      actual_position, actual_velocity, px4_setpoint);

    // 日志中的 Cmd 即 LADRC 输出；加速度模式下与发布值完全一致。
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "UAV%d mode=%s Ref[%.1f,%.1f,%.1f] Safe[%.1f,%.1f,%.1f] "
        "Pos[%.2f,%.2f,%.2f] Cmd[%.1f,%.1f,%.1f]%s",
        self_uav_id_, ladrc_controller::toString(control_mode_),
        nominal_reference.x(), nominal_reference.y(), nominal_reference.z(),
        safe_reference.x(), safe_reference.y(), safe_reference.z(),
        x_meas, y_meas, z_meas,
        ladrc_output.x(), ladrc_output.y(), ladrc_output.z(),
        (iapf.active ? " !IAPF!" : ""));
  }

  ladrc_controller::AvoidanceMode currentAvoidanceMode() const
  {
    return ladrc_controller::parseAvoidanceMode(
      this->get_parameter("avoidance_mode").as_string());
  }

  double currentEnterDistance() const
  {
    if (profile_soft_safety_active_) return profile_iapf_enter_distance_;
    return this->get_parameter("iapf_enter_distance").as_double();
  }

  void resetIAPFState()
  {
    filtered_iapf_position_offset_.setZero();
    filtered_iapf_acceleration_offset_.setZero();
    for (auto & item : neighbor_states_)
    {
      item.second.iapf_active = false;
    }
  }

  ladrc_controller::IAPFResult computeAvoidance(
      double x_meas, double y_meas, double z_meas)
  {
    ladrc_controller::IAPFParameters parameters;
    parameters.violation_distance =
      this->get_parameter("iapf_violation_distance").as_double();
    parameters.enter_distance = currentEnterDistance();
    parameters.exit_distance = profile_soft_safety_active_
      ? profile_iapf_exit_distance_
      : this->get_parameter("iapf_exit_distance").as_double();
    parameters.repulsion_gain =
      this->get_parameter("iapf_repulsion_gain").as_double() *
      (profile_soft_safety_active_ ? profile_iapf_repulsion_scale_ : 1.0);
    parameters.distance_epsilon =
      this->get_parameter("iapf_distance_epsilon").as_double();
    parameters.position_gain =
      this->get_parameter("iapf_position_gain").as_double();
    parameters.position_limit =
      this->get_parameter("iapf_position_limit").as_double();
    parameters.acceleration_gain =
      this->get_parameter("iapf_accel_gain").as_double();
    parameters.acceleration_limit =
      this->get_parameter("iapf_accel_limit").as_double();
    parameters.escape_gain =
      this->get_parameter("iapf_escape_gain").as_double();

    std::vector<ladrc_controller::NeighborSample> neighbors;
    neighbors.reserve(configured_neighbor_ids_.size());
    const ros::Time now = this->now();
    const double timeout = this->get_parameter("neighbor_timeout").as_double();
    if (timeout <= 0.0)
    {
      throw std::invalid_argument("neighbor_timeout must be positive");
    }
    for (const auto neighbor_id : configured_neighbor_ids_)
    {
      const auto state_iterator = neighbor_states_.find(neighbor_id);
      if (state_iterator == neighbor_states_.end())
      {
        neighbors.push_back({
          neighbor_id, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
          false, false});
        continue;
      }
      const auto & state = state_iterator->second;
      neighbors.push_back({
        neighbor_id,
        state.position,
        state.velocity,
        (now - state.receive_time).toSec() <= timeout,
        state.iapf_active});
    }
    double my_off_x = this->get_parameter("enu_offset_x").as_double();
    double my_off_y = this->get_parameter("enu_offset_y").as_double();
    double my_off_z = this->get_parameter("enu_offset_z").as_double();
    Eigen::Vector3d pos_own(x_meas + my_off_x, y_meas + my_off_y, z_meas + my_off_z);
    const Eigen::Vector3d velocity_own(
      current_odom_.velocity[1],
      current_odom_.velocity[0],
      -current_odom_.velocity[2]);

    auto result = ladrc_controller::computeIAPF(
      pos_own, velocity_own, self_uav_id_, neighbors, currentAvoidanceMode(),
      ladrc_controller::parseEscapeMode(
        this->get_parameter("iapf_escape_mode").as_string()),
      parameters);

    for (auto & item : neighbor_states_)
    {
      item.second.iapf_active =
        std::find(
          result.active_neighbor_ids.begin(),
          result.active_neighbor_ids.end(),
          item.first) != result.active_neighbor_ids.end();
    }
    const double alpha =
      this->get_parameter("iapf_filter_alpha").as_double();
    if (currentAvoidanceMode() == ladrc_controller::AvoidanceMode::OFF)
    {
      resetIAPFState();
      result.position_offset.setZero();
      result.acceleration_offset.setZero();
      result.active = false;
    }
    else
    {
      filtered_iapf_position_offset_ = ladrc_controller::smoothOffset(
        result.position_offset, filtered_iapf_position_offset_, alpha);
      filtered_iapf_acceleration_offset_ = ladrc_controller::smoothOffset(
        result.acceleration_offset, filtered_iapf_acceleration_offset_, alpha);
      if (filtered_iapf_position_offset_.norm() < 1e-5)
      {
        filtered_iapf_position_offset_.setZero();
      }
      if (filtered_iapf_acceleration_offset_.norm() < 1e-5)
      {
        filtered_iapf_acceleration_offset_.setZero();
      }
      result.position_offset = filtered_iapf_position_offset_;
      result.acceleration_offset = filtered_iapf_acceleration_offset_;
      result.active = result.active ||
        !result.position_offset.isZero(1e-9) ||
        !result.acceleration_offset.isZero(1e-9);
    }
    if (result.active)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
          "%s 避障: nearest=U%d d=%.2fm Frep[%.1f,%.1f,%.1f]",
          ladrc_controller::toString(currentAvoidanceMode()).c_str(),
          result.nearest_neighbor_id, result.nearest_neighbor_distance,
          result.raw_repulsion.x(), result.raw_repulsion.y(),
          result.raw_repulsion.z());
    }
    return result;
  }

  static void setVector3(
      geometry_msgs::Vector3 & output, const Eigen::Vector3d & value)
  {
    output.x = value.x();
    output.y = value.y();
    output.z = value.z();
  }

  static void setPoint(
      geometry_msgs::Point & output, const Eigen::Vector3d & value)
  {
    output.x = value.x();
    output.y = value.y();
    output.z = value.z();
  }

  void publishIAPFDebug(
      const ladrc_controller::IAPFResult & result,
      const Eigen::Vector3d & nominal_reference,
      const Eigen::Vector3d & modulated_reference,
      const Eigen::Vector3d & nominal_acceleration,
      const Eigen::Vector3d & modulated_acceleration)
  {
    uav_swarm_interfaces::IAPFDebug msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "world";
    msg.mission_id = mission_id_;
    msg.uav_id = self_uav_id_;
    msg.avoidance_mode =
      ladrc_controller::toString(currentAvoidanceMode());
    msg.has_nearest_neighbor = result.has_nearest_neighbor;
    msg.nearest_neighbor_id = result.nearest_neighbor_id;
    msg.nearest_neighbor_distance =
      static_cast<float>(result.nearest_neighbor_distance);
    msg.nearest_neighbor_closing_speed =
      static_cast<float>(result.nearest_neighbor_closing_speed);
    msg.iapf_active = result.active;
    msg.hysteresis_active = result.hysteresis_active;
    msg.active_neighbor_count =
      static_cast<uint16_t>(result.active_neighbor_ids.size());
    setVector3(msg.raw_repulsion, result.raw_repulsion);
    setVector3(msg.position_offset, result.position_offset);
    setVector3(msg.acceleration_offset, result.acceleration_offset);
    msg.position_saturated = result.position_saturated;
    msg.acceleration_saturated = result.acceleration_saturated;
    msg.valid_neighbor_count = result.valid_neighbor_count;
    msg.stale_neighbor_count = result.stale_neighbor_count;
    setPoint(msg.nominal_reference, nominal_reference);
    setPoint(msg.modulated_reference, modulated_reference);
    setVector3(msg.nominal_acceleration, nominal_acceleration);
    setVector3(msg.modulated_acceleration, modulated_acceleration);
    iapf_debug_pub_->publish(msg);
  }

  static void setArrayVector3(
      geometry_msgs::Vector3 & output,
      const std::array<float, 3> & value)
  {
    output.x = value[0];
    output.y = value[1];
    output.z = value[2];
  }

  void publishControlTrackingDebug(
      const Eigen::Vector3d & nominal_reference,
      const Eigen::Vector3d & safe_reference,
      const Eigen::Vector3d & nominal_velocity,
      const Eigen::Vector3d & nominal_acceleration,
      const Eigen::Vector3d & safe_acceleration,
      const Eigen::Vector3d & ladrc_output,
      const Eigen::Vector3d & actual_position,
      const Eigen::Vector3d & actual_velocity,
      const transport_samples::TrajectorySetpoint & px4_setpoint)
  {
    uav_swarm_interfaces::ControlTrackingDebug msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "local_enu";
    msg.mission_id = mission_id_;
    msg.uav_id = self_uav_id_;
    msg.control_mode = ladrc_controller::toString(control_mode_);
    msg.has_command = has_command_;
    setPoint(msg.nominal_position, nominal_reference);
    setPoint(msg.safe_position, safe_reference);
    setVector3(msg.nominal_velocity, nominal_velocity);
    setVector3(msg.nominal_acceleration, nominal_acceleration);
    setVector3(msg.safe_acceleration, safe_acceleration);
    setVector3(msg.ladrc_output, ladrc_output);

    const auto states_x = ladrc_x_->getEstimatedStates();
    const auto states_y = ladrc_y_->getEstimatedStates();
    const auto states_z = ladrc_z_->getEstimatedStates();
    setVector3(msg.leso_z1, Eigen::Vector3d(
      states_x[0], states_y[0], states_z[0]));
    setVector3(msg.leso_z2, Eigen::Vector3d(
      states_x[1], states_y[1], states_z[1]));
    setVector3(msg.leso_z3, Eigen::Vector3d(
      states_x[2], states_y[2], states_z[2]));

    setPoint(msg.actual_position, actual_position);
    setVector3(msg.actual_velocity, actual_velocity);
    const Eigen::Vector3d position_derived_velocity =
      position_velocity_filter_.valid()
      ? position_velocity_filter_.velocity()
      : Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    setVector3(msg.position_derived_velocity, position_derived_velocity);
    msg.raw_ekf_speed = actual_velocity.norm();
    msg.position_derived_speed = position_velocity_filter_.valid()
      ? position_velocity_filter_.speed()
      : std::numeric_limits<double>::quiet_NaN();
    msg.leso_z2_speed = Eigen::Vector3d(
      states_x[1], states_y[1], states_z[1]).norm();
    setVector3(msg.tracking_error, safe_reference - actual_position);
    setArrayVector3(msg.px4_position_setpoint, px4_setpoint.position);
    setArrayVector3(msg.px4_velocity_setpoint, px4_setpoint.velocity);
    setArrayVector3(msg.px4_acceleration_setpoint, px4_setpoint.acceleration);
    control_tracking_debug_pub_->publish(msg);
  }

  // --- [Phase 1] 新增 UAVStatus 发布 ---
  void publishUAVStatus()
  {
    uav_swarm_interfaces::UAVStatus msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "local_enu";
    msg.uav_id = self_uav_id_;
    msg.mission_id = mission_id_;
    msg.system_ready =
      startup_.state() == ladrc_controller::StartupState::READY;
    msg.is_hover_stable = is_hover_stable_;
    msg.stability_state = hover_stability_.state;
    msg.position_error = static_cast<float>(latest_position_error_);
    msg.speed = position_velocity_filter_.valid() ?
      static_cast<float>(position_velocity_filter_.speed()) :
      std::numeric_limits<float>::infinity();
    msg.raw_ekf_speed = has_odom_ ? static_cast<float>(std::sqrt(
      current_odom_.velocity[0] * current_odom_.velocity[0] +
      current_odom_.velocity[1] * current_odom_.velocity[1] +
      current_odom_.velocity[2] * current_odom_.velocity[2])) :
      std::numeric_limits<float>::infinity();
    msg.startup_state = static_cast<uint8_t>(startup_.state());
    msg.armed = px4_armed_;
    msg.offboard = px4_offboard_;
    msg.failsafe = has_vehicle_status_ && current_vehicle_status_.failsafe;
    msg.pre_flight_checks_pass = has_vehicle_status_ &&
      current_vehicle_status_.pre_flight_checks_pass;
    msg.altitude = has_odom_ ? static_cast<float>(-current_odom_.position[2]) :
      std::numeric_limits<float>::quiet_NaN();
    status_pub_->publish(msg);
  }

  void initializeTrajectoryMetrics(double p0_x, double p0_y, double p0_z,
                                   double target_global_x,
                                   double target_global_y,
                                   double target_global_z)
  {
    double off_x = this->get_parameter("enu_offset_x").as_double();
    double off_y = this->get_parameter("enu_offset_y").as_double();
    double off_z = this->get_parameter("enu_offset_z").as_double();

    metrics_msg_ = uav_swarm_interfaces::TrajectoryMetrics();
    metrics_msg_.header.frame_id = "world";
    metrics_msg_.uav_id = uav_id_;
    metrics_msg_.start_pos.x = p0_x + off_x;
    metrics_msg_.start_pos.y = p0_y + off_y;
    metrics_msg_.start_pos.z = p0_z + off_z;
    metrics_msg_.target_pos.x = target_global_x;
    metrics_msg_.target_pos.y = target_global_y;
    metrics_msg_.target_pos.z = target_global_z;
    metrics_msg_.requested_duration = static_cast<float>(target_duration_);
    metrics_msg_.trajectory_duration = static_cast<float>(traj_x_.getDuration());
    metrics_msg_.motion_style = motion_style_;

    double dx = target_pos_x_ - p0_x;
    double dy = target_pos_y_ - p0_y;
    double dz = target_pos_z_ - p0_z;
    double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    double duration = traj_x_.getDuration();
    double duration2 = duration * duration;
    double duration3 = duration2 * duration;
    double duration5 = duration3 * duration2;

    metrics_msg_.path_length = static_cast<float>(distance);
    metrics_msg_.max_velocity = static_cast<float>(1.875 * distance / duration);
    metrics_msg_.max_acceleration =
        static_cast<float>((10.0 * std::sqrt(3.0) / 3.0) * distance / duration2);
    metrics_msg_.max_jerk = static_cast<float>(60.0 * distance / duration3);
    metrics_msg_.integrated_squared_jerk =
        static_cast<float>(720.0 * distance * distance / duration5);
    metrics_msg_.elapsed_time = 0.0f;
    metrics_msg_.arrival_time_error =
        std::numeric_limits<float>::quiet_NaN();
    metrics_msg_.final_position_error =
        static_cast<float>(distance);
    metrics_msg_.is_finished = false;
    metrics_msg_.is_hover_stable = false;
    has_trajectory_metrics_ = true;

    RCLCPP_INFO(this->get_logger(),
        "轨迹指标: path=%.2fm vmax=%.2fm/s amax=%.2fm/s^2 jmax=%.2fm/s^3 ISJ=%.2f",
        metrics_msg_.path_length,
        metrics_msg_.max_velocity,
        metrics_msg_.max_acceleration,
        metrics_msg_.max_jerk,
        metrics_msg_.integrated_squared_jerk);
  }

  void publishTrajectoryMetrics(double elapsed,
                                double x_meas,
                                double y_meas,
                                double z_meas,
                                bool is_finished)
  {
    if (!has_trajectory_metrics_) return;

    double off_x = this->get_parameter("enu_offset_x").as_double();
    double off_y = this->get_parameter("enu_offset_y").as_double();
    double off_z = this->get_parameter("enu_offset_z").as_double();
    double x_global = x_meas + off_x;
    double y_global = y_meas + off_y;
    double z_global = z_meas + off_z;
    double dx = metrics_msg_.target_pos.x - x_global;
    double dy = metrics_msg_.target_pos.y - y_global;
    double dz = metrics_msg_.target_pos.z - z_global;

    metrics_msg_.header.stamp = this->now();
    metrics_msg_.elapsed_time = static_cast<float>(elapsed);
    metrics_msg_.arrival_time_error = static_cast<float>(arrival_time_error_);
    metrics_msg_.final_position_error =
        static_cast<float>(std::sqrt(dx * dx + dy * dy + dz * dz));
    metrics_msg_.is_finished = is_finished;
    metrics_msg_.is_hover_stable = is_hover_stable_;

    trajectory_metrics_pub_->publish(metrics_msg_);
  }

  std::string defaultControlAdaptationLogPath() const
  {
    std::filesystem::path source_path(__FILE__);
    if (source_path.is_relative())
    {
      source_path = std::filesystem::current_path() / source_path;
    }

    for (auto path = source_path.parent_path(); !path.empty(); path = path.parent_path())
    {
      if (std::filesystem::exists(path / ".git"))
      {
        return (path / "logs" / "control_adaptation_log.csv").string();
      }
      if (path == path.root_path())
      {
        break;
      }
    }

    return "logs/control_adaptation_log.csv";
  }

  void resetControlAdaptationRuntimeMetrics()
  {
    peak_velocity_ = 0.0;
    peak_acceleration_ = 0.0;
    tracking_error_squared_sum_ = 0.0;
    tracking_sample_count_ = 0;
    settling_time_ = std::numeric_limits<double>::quiet_NaN();
    previous_velocity_valid_ = false;
    control_adaptation_csv_written_ = false;
    has_control_adaptation_metrics_ = true;
  }

  void updateControlAdaptationRuntimeMetrics(double elapsed,
                                             double x_ref,
                                             double y_ref,
                                             double z_ref,
                                             double x_meas,
                                             double y_meas,
                                             double z_meas)
  {
    if (!has_control_adaptation_metrics_) return;
    latest_elapsed_time_ = elapsed;

    Eigen::Vector3d measured_velocity(
        current_odom_.velocity[0],
        current_odom_.velocity[1],
        current_odom_.velocity[2]);
    peak_velocity_ = std::max(peak_velocity_, measured_velocity.norm());

    if (previous_velocity_valid_ && dt_ > 1e-6)
    {
      double acceleration = (measured_velocity - previous_velocity_).norm() / dt_;
      peak_acceleration_ = std::max(peak_acceleration_, acceleration);
    }
    previous_velocity_ = measured_velocity;
    previous_velocity_valid_ = true;

    double error = std::sqrt(
        (x_ref - x_meas) * (x_ref - x_meas) +
        (y_ref - y_meas) * (y_ref - y_meas) +
        (z_ref - z_meas) * (z_ref - z_meas));
    tracking_error_squared_sum_ += error * error;
    ++tracking_sample_count_;
  }

  uav_swarm_interfaces::ControlAdaptationLog buildControlAdaptationLogMsg()
  {
    uav_swarm_interfaces::ControlAdaptationLog msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "world";
    msg.mission_id = mission_id_;
    msg.uav_id = uav_id_;
    msg.motion_style = motion_style_;
    msg.target_distance = static_cast<float>(target_distance_);
    msg.duration = static_cast<float>(target_duration_);
    msg.average_speed = static_cast<float>(average_speed_);
    msg.gain_multiplier = static_cast<float>(gain_multiplier_);
    msg.omega_o_x = static_cast<float>(omega_o_x_);
    msg.omega_o_y = static_cast<float>(omega_o_y_);
    msg.omega_o_z = static_cast<float>(omega_o_z_);
    msg.omega_c_x = static_cast<float>(omega_c_x_);
    msg.omega_c_y = static_cast<float>(omega_c_y_);
    msg.omega_c_z = static_cast<float>(omega_c_z_);
    msg.peak_velocity = static_cast<float>(peak_velocity_);
    msg.peak_acceleration = static_cast<float>(peak_acceleration_);
    msg.settling_time = static_cast<float>(settling_time_);
    msg.tracking_rmse =
        tracking_sample_count_ > 0
            ? static_cast<float>(std::sqrt(
                  tracking_error_squared_sum_ /
                  static_cast<double>(tracking_sample_count_)))
            : std::numeric_limits<float>::quiet_NaN();
    return msg;
  }

  void publishControlAdaptationLog()
  {
    if (!has_control_adaptation_metrics_) return;
    control_adaptation_pub_->publish(buildControlAdaptationLogMsg());
  }

  void writeControlAdaptationCsvRow()
  {
    if (!has_control_adaptation_metrics_ || control_adaptation_csv_written_)
    {
      return;
    }

    std::filesystem::path log_path(
        this->get_parameter("control_adaptation_log_path").as_string());
    if (!log_path.has_parent_path())
    {
      log_path = std::filesystem::current_path() / log_path;
    }

    std::error_code ec;
    auto parent = log_path.parent_path();
    if (!parent.empty())
    {
      std::filesystem::create_directories(parent, ec);
      if (ec)
      {
        RCLCPP_WARN(this->get_logger(),
            "无法创建控制适应日志目录 %s: %s",
            parent.string().c_str(), ec.message().c_str());
        return;
      }
    }

    bool write_header =
        !std::filesystem::exists(log_path) ||
        std::filesystem::file_size(log_path, ec) == 0;
    ec.clear();

    std::ofstream log_file(log_path, std::ios::app);
    if (!log_file.is_open())
    {
      RCLCPP_WARN(this->get_logger(),
          "无法打开控制适应日志文件: %s", log_path.string().c_str());
      return;
    }

    if (write_header)
    {
      log_file
          << "mission_id,uav_id,motion_style,target_distance,duration,"
          << "average_speed,gain_multiplier,omega_o_x,omega_o_y,omega_o_z,"
          << "omega_c_x,omega_c_y,omega_c_z,peak_velocity,peak_acceleration,"
          << "settling_time,tracking_rmse\n";
    }

    auto msg = buildControlAdaptationLogMsg();
    auto value = [](float number) {
      return std::isfinite(number) ? std::to_string(number) : std::string("nan");
    };

    log_file << std::fixed << std::setprecision(6)
             << msg.mission_id << ','
             << static_cast<int>(msg.uav_id) << ','
             << msg.motion_style << ','
             << value(msg.target_distance) << ','
             << value(msg.duration) << ','
             << value(msg.average_speed) << ','
             << value(msg.gain_multiplier) << ','
             << value(msg.omega_o_x) << ','
             << value(msg.omega_o_y) << ','
             << value(msg.omega_o_z) << ','
             << value(msg.omega_c_x) << ','
             << value(msg.omega_c_y) << ','
             << value(msg.omega_c_z) << ','
             << value(msg.peak_velocity) << ','
             << value(msg.peak_acceleration) << ','
             << value(msg.settling_time) << ','
             << value(msg.tracking_rmse) << '\n';

    control_adaptation_csv_written_ = true;
    RCLCPP_INFO(this->get_logger(),
        "控制适应日志已写入: %s (mission=%u, uav=%d)",
        log_path.string().c_str(), mission_id_, static_cast<int>(uav_id_));
  }

  void publishOffboardControlMode()
  {
    // MAVROS raw setpoints supply the Offboard heartbeat and mode mask.
  }

  transport_samples::TrajectorySetpoint publishTrajectorySetpoint(
                                  double px_enu, double py_enu, double pz_enu,
                                  double ax_enu, double ay_enu, double az_enu,
                                  double yaw_ref,
                                  bool publish_accel_feedforward = false)
  {
    const auto msg = ladrc_controller::makeTrajectorySetpoint(
      control_mode_, px_enu, py_enu, pz_enu,
      ax_enu, ay_enu, az_enu, yaw_ref,
      publish_accel_feedforward,
      this->get_clock()->now().toNSec() / 1000);
    mavros_msgs::PositionTarget output;
    output.header.stamp = now();
    output.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    // MAVROS setpoint_raw/local accepts ENU and performs ENU -> NED itself.
    output.type_mask = output.IGNORE_VX | output.IGNORE_VY | output.IGNORE_VZ | output.IGNORE_YAW_RATE;
    if (control_mode_ == ladrc_controller::ControlMode::LADRC_ACCELERATION)
      output.type_mask |= output.IGNORE_PX | output.IGNORE_PY | output.IGNORE_PZ;
    if (!std::isfinite(msg.acceleration[0]))
      output.type_mask |= output.IGNORE_AFX | output.IGNORE_AFY | output.IGNORE_AFZ;
    output.position.x = px_enu; output.position.y = py_enu; output.position.z = pz_enu;
    output.acceleration_or_force.x = ax_enu;
    output.acceleration_or_force.y = ay_enu;
    output.acceleration_or_force.z = az_enu;
    output.yaw = M_PI_2 - yaw_ref;
    trajectory_pub_->publish(output);
    return msg;
  }

  // Member variables
  std::unique_ptr<ladrc_controller::LADRCController> ladrc_x_;
  std::unique_ptr<ladrc_controller::LADRCController> ladrc_y_;
  std::unique_ptr<ladrc_controller::LADRCController> ladrc_z_;

  // [Phase 1] Swarm 命令订阅 & 状态发布
  std::shared_ptr<ros::Subscriber>
      execution_command_sub_;
  std::shared_ptr<ros::Subscriber> odom_sub_;
  std::shared_ptr<ros::Subscriber> vehicle_status_sub_;
  std::shared_ptr<ros::Publisher> status_pub_;
  std::shared_ptr<ros::Publisher>
      startup_event_pub_;
  std::shared_ptr<ros::Publisher> swarm_state_pub_;
  std::shared_ptr<ros::Publisher>
      trajectory_metrics_pub_;
  std::shared_ptr<ros::Publisher>
      control_adaptation_pub_;
  std::shared_ptr<ros::Publisher>
      iapf_debug_pub_;
  std::shared_ptr<ros::Publisher>
      control_tracking_debug_pub_;


  std::shared_ptr<ros::Publisher> trajectory_pub_;
  ros::ServiceClient arming_client_, mode_client_;

  std::shared_ptr<ros::Timer> control_timer_;
  std::shared_ptr<ros::Timer> command_timer_;

  // 自身 UAV ID（从命名空间自动提取）
  uint8_t self_uav_id_ = 0;

  // [Phase 1] Swarm command 数据
  uint32_t mission_id_ = 0;
  uint8_t uav_id_ = 0;
  double target_pos_x_ = 0.0;
  double target_pos_y_ = 0.0;
  double target_pos_z_ = 0.0;
  double target_duration_ = 0.0;
  std::string motion_style_ = "normal";
  bool has_command_ = false;
  ladrc_controller::ControlMode control_mode_{
    ladrc_controller::ControlMode::PX4_POSITION};
  bool profile_soft_safety_active_ = false;
  double profile_iapf_enter_distance_ = 0.0;
  double profile_iapf_exit_distance_ = 0.0;
  double profile_iapf_repulsion_scale_ = 1.0;
  std::string active_profile_configuration_id_;

  // 悬停保持：用首次位置作为固定 setpoint，避免漂移正反馈
  bool hover_hold_set_ = false;
  double hover_hold_x_ = 0.0;
  double hover_hold_y_ = 0.0;
  double hover_hold_z_ = 0.0;

  // [Phase 2] 轨迹生成器
  ladrc_controller::MinimumJerkTrajectory traj_x_;
  ladrc_controller::MinimumJerkTrajectory traj_y_;
  ladrc_controller::MinimumJerkTrajectory traj_z_;
  ros::Time command_start_time_;

  uav_swarm_interfaces::TrajectoryMetrics metrics_msg_;
  bool has_trajectory_metrics_ = false;
  bool arrival_time_recorded_ = false;
  double arrival_time_error_ = std::numeric_limits<double>::quiet_NaN();
  int trajectory_metrics_pub_counter_ = 0;

  // 控制适应日志数据
  bool has_control_adaptation_metrics_ = false;
  bool control_adaptation_csv_written_ = false;
  double target_distance_ = 0.0;
  double average_speed_ = 0.0;
  double gain_multiplier_ = 1.0;
  double omega_o_x_ = 0.0;
  double omega_o_y_ = 0.0;
  double omega_o_z_ = 0.0;
  double omega_c_x_ = 0.0;
  double omega_c_y_ = 0.0;
  double omega_c_z_ = 0.0;
  double peak_velocity_ = 0.0;
  double peak_acceleration_ = 0.0;
  double settling_time_ = std::numeric_limits<double>::quiet_NaN();
  double latest_elapsed_time_ = 0.0;
  double tracking_error_squared_sum_ = 0.0;
  uint64_t tracking_sample_count_ = 0;
  Eigen::Vector3d previous_velocity_{0.0, 0.0, 0.0};
  bool previous_velocity_valid_ = false;

  // Odom 数据
  transport_samples::Odometry current_odom_;
  bool has_odom_ = false;
  double last_odom_receive_time_s_ =
    -std::numeric_limits<double>::infinity();
  ladrc_controller::PositionVelocityFilter position_velocity_filter_{0.5};
  transport_samples::VehicleStatus current_vehicle_status_;
  bool has_vehicle_status_ = false;
  double last_vehicle_status_receive_time_s_ =
    -std::numeric_limits<double>::infinity();
  bool px4_armed_ = false;
  bool px4_offboard_ = false;
  bool takeoff_reference_set_ = false;
  uav_swarm_interfaces::UAVExecutionCommand::ConstPtr pending_execution_command_;

  // [Phase 3 预置] 悬停状态（Phase 1 默认为 false，Phase 3 完整实现）
  bool is_hover_stable_ = false;
  ladrc_controller::HoverStabilityState hover_stability_;
  double latest_position_error_ = std::numeric_limits<double>::infinity();
  double latest_raw_ekf_speed_ = std::numeric_limits<double>::infinity();
  double latest_position_derived_speed_ =
    std::numeric_limits<double>::infinity();

  // Odom 发布计数器 (~10Hz throttle)

  // [Phase 4] IAPF 邻居状态
  Eigen::Vector3d filtered_iapf_position_offset_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d filtered_iapf_acceleration_offset_{Eigen::Vector3d::Zero()};
  std::vector<uint8_t> configured_neighbor_ids_;
  std::unordered_map<uint8_t, NeighborState> neighbor_states_;
  std::vector<std::shared_ptr<ros::Subscriber>> neighbor_subs_;

  // PX4 feedback-driven startup state machine
  ladrc_controller::StartupStateMachine startup_;

  double dt_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ladrc_position_controller");
  LADRCPositionControllerNode node;
  ros::spin();
  return 0;
}
