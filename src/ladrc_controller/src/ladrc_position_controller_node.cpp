/**
 * @file ladrc_position_controller_node.cpp
 * @brief LADRC 位置控制器 ROS 1 + MAVROS 版本
 *
 * 从 ROS 2 (rclcpp + px4_msgs + MicroXRCEAgent) 移植到 ROS 1 (roscpp + mavros_msgs + MAVROS)。
 * 核心算法 (LADRC/MinimumJerk/IAPF) 保持完全不变。
 *
 * 主要变化:
 *  - px4_msgs (NED) → nav_msgs/geometry_msgs (ENU)，消除所有 NED↔ENU 转换
 *  - OffboardControlMode 发布 → 删除 (MAVROS 自动维持 offboard 心跳)
 *  - VehicleCommand → mavros_msgs ROS 服务 (arming/set_mode)
 *  - 状态机基于 /mavros/state 真实反馈，不再使用纯定时延迟
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <uav_swarm_interfaces/UAVSwarmCommand.h>
#include <uav_swarm_interfaces/UAVStatus.h>
#include <boost/bind.hpp>
#include "ladrc_controller/ladrc_core.hpp"
#include "ladrc_controller/minimum_jerk_trajectory.hpp"
#include <cmath>
#include <chrono>
#include <atomic>
#include <unordered_map>
#include <Eigen/Dense>

// ====================== 自动起飞状态机 ======================
enum class FlightState
{
  INIT,
  ARMING,
  SETTING_OFFBOARD,
  RUNNING_TRAJECTORY
};

class LADRCPositionControllerNode
{
public:
  LADRCPositionControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh)
  {
    // ====================== 加载参数 ======================
    pnh_.param("control_frequency", control_freq_, 50.0);
    pnh_.param("omega_o_x", omega_o_x_, 15.0);
    pnh_.param("omega_o_y", omega_o_y_, 15.0);
    pnh_.param("omega_o_z", omega_o_z_, 15.0);
    pnh_.param("omega_c_x", omega_c_x_, 8.0);
    pnh_.param("omega_c_y", omega_c_y_, 8.0);
    pnh_.param("omega_c_z", omega_c_z_, 8.0);
    pnh_.param("b0_x", b0_x_, 1.0);
    pnh_.param("b0_y", b0_y_, 1.0);
    pnh_.param("b0_z", b0_z_, 1.0);
    pnh_.param("max_velocity", max_vel_, 5.0);
    pnh_.param("max_acceleration_x", max_acc_x_, 3.0);
    pnh_.param("max_acceleration_y", max_acc_y_, 3.0);
    pnh_.param("max_acceleration_z", max_acc_z_, 3.0);
    pnh_.param("enu_offset_x", enu_offset_x_, 0.0);
    pnh_.param("enu_offset_y", enu_offset_y_, 0.0);
    pnh_.param("enu_offset_z", enu_offset_z_, 0.0);
    pnh_.param("iapf_safe_distance", iapf_safe_dist_, 1.0);
    pnh_.param("iapf_repulsion_gain", iapf_rep_gain_, 1.0);

    dt_ = 1.0 / control_freq_;

    // 从命名空间提取自身 UAV ID
    std::string ns = ros::this_node::getNamespace();
    size_t uav_pos = ns.find("/uav");
    if (uav_pos != std::string::npos)
    {
      std::string id_str = ns.substr(uav_pos + 4);
      while (!id_str.empty() && id_str.back() == '/') id_str.pop_back();
      try { self_uav_id_ = static_cast<uint8_t>(std::stoi(id_str)); }
      catch (...) { self_uav_id_ = 0; }
    }

    // ====================== 初始化 LADRC ======================
    initializeControllers();

    // ====================== MAVROS 订阅 ======================
    // 里程计 (MAVROS 输出 ENU 坐标，无需 NED 转换)
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "mavros/local_position/odom", 10,
        &LADRCPositionControllerNode::odomCallback, this);

    // PX4 状态 (arming/mode 反馈)
    state_sub_ = nh_.subscribe<mavros_msgs::State>(
        "mavros/state", 10,
        &LADRCPositionControllerNode::stateCallback, this);

    // ====================== 自定义话题订阅 ======================
    // swarm_command (调度层 → 本节点，相对话题在命名空间内解析)
    cmd_sub_ = nh_.subscribe<uav_swarm_interfaces::UAVSwarmCommand>(
        "swarm_command", 10,
        &LADRCPositionControllerNode::swarmCommandCallback, this);

    // ====================== 发布器 ======================
    // 位置设定点 → MAVROS (PoseStamped, ENU 坐标)
    setpoint_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);

    // UAV 状态反馈 → 调度层
    status_pub_ = nh_.advertise<uav_swarm_interfaces::UAVStatus>("status", 10);

    // ENU 位置发布 (低频，供调度层)
    odom_pub_ = nh_.advertise<geometry_msgs::Point>("odom", 10);

    // ====================== MAVROS 服务客户端 ======================
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // ====================== 定时器 ======================
    control_timer_ = nh_.createTimer(ros::Duration(dt_),
        &LADRCPositionControllerNode::controlLoop, this);

    // 状态机定时器 (10 Hz)
    sm_timer_ = nh_.createTimer(ros::Duration(0.1),
        &LADRCPositionControllerNode::stateMachine, this);

    // ====================== 初始化状态 ======================
    flight_state_ = FlightState::INIT;
    offboard_setpoint_counter_ = 0;

    ROS_INFO("LADRC 集群执行节点已初始化 (命名空间: %s), ENU偏移=[%.1f, %.1f, %.1f]",
        ns.c_str(), enu_offset_x_, enu_offset_y_, enu_offset_z_);
    ROS_INFO("等待 MAVROS 连接和 swarm_command 消息...");
  }

private:
  // ====================== 邻居 Odom 订阅（在收到 swarm_command 后动态创建） ======================
  void setupNeighborSubs(const std::vector<int>& neighbor_ids)
  {
    neighbor_subs_.clear();
    for (int id : neighbor_ids)
    {
      if (id == 0 || id == self_uav_id_) continue;
      std::string topic = "/uav" + std::to_string(id) + "/mavros/local_position/odom";
      auto sub = nh_.subscribe<nav_msgs::Odometry>(
          topic, 10,
          boost::bind(&LADRCPositionControllerNode::neighborOdomCallback, this, _1, id));
      neighbor_subs_.push_back(sub);
    }
    ROS_INFO("已创建 %zu 个邻居 Odom 订阅", neighbor_subs_.size());
  }

  void neighborOdomCallback(const nav_msgs::Odometry::ConstPtr& msg, int neighbor_id)
  {
    // MAVROS odom 已是 ENU 坐标，加上 spawn 偏移得到全局 ENU
    neighbor_positions_[neighbor_id] = Eigen::Vector3d(
        msg->pose.pose.position.x + 3.0 * neighbor_id,  // ENU.x + offset
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
  }

  // ====================== LADRC 初始化 ======================
  void initializeControllers()
  {
    auto make_params = [this](double omega_o, double omega_c, double b0,
                               double max_acc) -> ladrc_controller::LADRCParams {
      ladrc_controller::LADRCParams p;
      p.omega_o = omega_o;
      p.omega_c = omega_c;
      p.kp = omega_c * omega_c;
      p.kd = 2.0 * omega_c;
      p.b0 = b0;
      p.dt = dt_;
      p.max_output = max_acc;
      p.min_output = -max_acc;
      return p;
    };

    ladrc_x_ = std::make_unique<ladrc_controller::LADRCController>(
        make_params(omega_o_x_, omega_c_x_, b0_x_, max_acc_x_));
    ladrc_y_ = std::make_unique<ladrc_controller::LADRCController>(
        make_params(omega_o_y_, omega_c_y_, b0_y_, max_acc_y_));
    ladrc_z_ = std::make_unique<ladrc_controller::LADRCController>(
        make_params(omega_o_z_, omega_c_z_, b0_z_, max_acc_z_));
  }

  // ====================== MAVROS 回调 ======================
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    ROS_INFO_ONCE("已接收到 MAVROS local_position/odom 消息");
    current_odom_ = *msg;
    has_odom_ = true;
  }

  void stateCallback(const mavros_msgs::State::ConstPtr& msg)
  {
    current_state_ = *msg;
  }

  // ====================== swarm_command 回调 ======================
  void swarmCommandCallback(const uav_swarm_interfaces::UAVSwarmCommand::ConstPtr& msg)
  {
    ROS_INFO("UAV%d swarm_cmd 回调触发 (目标=[%.1f,%.1f,%.1f])",
        self_uav_id_, msg->target_pos.x, msg->target_pos.y, msg->target_pos.z);

    // 状态机未就绪或未收到里程计
    if (flight_state_.load() != FlightState::RUNNING_TRAJECTORY || !has_odom_)
    {
      ROS_WARN("UAV%d 尚未就绪（状态=%d, odom=%d），忽略命令",
          msg->uav_id, (int)flight_state_.load(), has_odom_);
      return;
    }

    // 防重复命令
    if (has_command_)
    {
      bool same_target = (std::abs(msg->target_pos.x - (target_pos_x_ + enu_offset_x_)) < 1e-6 &&
                          std::abs(msg->target_pos.y - (target_pos_y_ + enu_offset_y_)) < 1e-6 &&
                          std::abs(msg->target_pos.z - (target_pos_z_ + enu_offset_z_)) < 1e-6);
      bool same_params = (std::abs(msg->duration - target_duration_) < 1e-6 &&
                          msg->motion_style == motion_style_);
      if (same_target && same_params) return;
      ROS_INFO("收到新任务指令 (UAV%d)，目标/参数已变更，覆盖旧任务", msg->uav_id);
    }

    uav_id_ = msg->uav_id;
    target_duration_ = msg->duration;
    motion_style_ = msg->motion_style;
    safety_factor_ = msg->safety_factor;
    has_command_ = true;

    // 全局 ENU → 本地 ENU：减去 spawn 偏移量
    target_pos_x_ = msg->target_pos.x - enu_offset_x_;
    target_pos_y_ = msg->target_pos.y - enu_offset_y_;
    target_pos_z_ = msg->target_pos.z - enu_offset_z_;

    // MAVROS odom 已是 ENU：直接使用
    double p0_x = current_odom_.pose.pose.position.x;
    double p0_y = current_odom_.pose.pose.position.y;
    double p0_z = current_odom_.pose.pose.position.z;

    // 初始化 Minimum Jerk 轨迹
    traj_x_.initialize(p0_x, target_pos_x_, target_duration_);
    traj_y_.initialize(p0_y, target_pos_y_, target_duration_);
    traj_z_.initialize(p0_z, target_pos_z_, target_duration_);

    // Warm start LESO
    ladrc_x_->setObserverInitialState(p0_x, 0.0, 0.0);
    ladrc_y_->setObserverInitialState(p0_y, 0.0, 0.0);
    ladrc_z_->setObserverInitialState(p0_z, 0.0, 0.0);

    command_start_time_ = ros::Time::now();
    applyDynamicGains();
    is_hover_stable_ = false;

    ROS_INFO(">>> UAV%d 全局[%.1f,%.1f,%.1f]→本地[%.1f,%.1f,%.1f] T=%.1fs %s",
        uav_id_,
        msg->target_pos.x, msg->target_pos.y, msg->target_pos.z,
        target_pos_x_, target_pos_y_, target_pos_z_,
        target_duration_, motion_style_.c_str());
  }

  // ====================== 状态机 (基于 MAVROS /mavros/state 真实反馈) ======================
  void stateMachine(const ros::TimerEvent&)
  {
    switch (flight_state_.load())
    {
    case FlightState::INIT:
    {
      // 等待 MAVROS 连接并收到 state、odom
      if (offboard_setpoint_counter_++ > 100)  // 10s @ 10Hz
      {
        ROS_INFO("系统稳定，发送解锁命令...");
        mavros_msgs::CommandBool arm_srv;
        arm_srv.request.value = true;
        if (arming_client_.call(arm_srv) && arm_srv.response.success)
        {
          ROS_INFO("解锁命令已发送");
          flight_state_ = FlightState::ARMING;
        }
        else
        {
          ROS_WARN("解锁命令失败，重试中...");
        }
        offboard_setpoint_counter_ = 0;
      }
      break;
    }

    case FlightState::ARMING:
    {
      // 检查是否已解锁
      if (current_state_.armed)
      {
        ROS_INFO("解锁成功。切换到 Offboard 模式...");
        // 发送 offboard setpoint 至少 2 秒才能切换模式（MAVROS 要求）
        if (offboard_setpoint_counter_++ > 20)  // 2s
        {
          mavros_msgs::SetMode mode_srv;
          mode_srv.request.custom_mode = "OFFBOARD";
          if (set_mode_client_.call(mode_srv) && mode_srv.response.mode_sent)
          {
            ROS_INFO("Offboard 模式切换命令已发送");
            flight_state_ = FlightState::SETTING_OFFBOARD;
          }
          else
          {
            ROS_WARN("Offboard 模式切换失败，重试中...");
          }
          offboard_setpoint_counter_ = 0;
        }
      }
      break;
    }

    case FlightState::SETTING_OFFBOARD:
    {
      // 检查是否已进入 offboard 模式
      if (current_state_.mode == "OFFBOARD" && current_state_.armed)
      {
        ROS_INFO("Offboard 模式已激活。LADRC 控制器接管。");
        flight_state_ = FlightState::RUNNING_TRAJECTORY;
        sm_timer_.stop();  // 状态机任务完成
      }
      break;
    }

    case FlightState::RUNNING_TRAJECTORY:
      sm_timer_.stop();
      break;
    }
  }

  // ====================== 主控制循环 (50Hz) ======================
  void controlLoop(const ros::TimerEvent&)
  {
    // 持续发布 offboard setpoint（MAVROS 需要 >10Hz 流来维持 offboard 模式）
    if (!has_odom_)
    {
      ROS_WARN_THROTTLE(5.0, "等待 MAVROS odom 消息...");
      return;
    }

    // 1. 获取测量值 (MAVROS odom 已是 ENU，无需转换)
    double x_meas = current_odom_.pose.pose.position.x;
    double y_meas = current_odom_.pose.pose.position.y;
    double z_meas = current_odom_.pose.pose.position.z;

    // 低频发布 ENU 位置 (~10Hz)
    if (++odom_pub_counter_ >= 5)
    {
      odom_pub_counter_ = 0;
      geometry_msgs::Point odom_msg;
      odom_msg.x = x_meas + enu_offset_x_;
      odom_msg.y = y_meas + enu_offset_y_;
      odom_msg.z = z_meas + enu_offset_z_;
      odom_pub_.publish(odom_msg);
    }

    // 若无命令，悬停保持
    if (!has_command_)
    {
      if (!hover_hold_set_)
      {
        hover_hold_x_ = x_meas;
        hover_hold_y_ = y_meas;
        hover_hold_z_ = z_meas;
        hover_hold_set_ = true;
        ROS_INFO("UAV%d 悬停保持锁定: [%.2f, %.2f, %.2f]",
            self_uav_id_, x_meas, y_meas, z_meas);
      }
      publishSetpoint(hover_hold_x_, hover_hold_y_, hover_hold_z_);
      ROS_INFO_THROTTLE(10.0,
          "UAV%d 悬停保持 Pos[%.2f,%.2f,%.2f]", self_uav_id_, x_meas, y_meas, z_meas);
      return;
    }
    hover_hold_set_ = false;

    // 2. 计算轨迹参考值
    double elapsed = (ros::Time::now() - command_start_time_).toSec();
    bool x_finished = traj_x_.isFinished(elapsed);
    bool y_finished = traj_y_.isFinished(elapsed);
    bool z_finished = traj_z_.isFinished(elapsed);

    auto ref_x = traj_x_.evaluate(elapsed);
    auto ref_y = traj_y_.evaluate(elapsed);
    auto ref_z = traj_z_.evaluate(elapsed);

    double x_ref = ref_x.position;
    double y_ref = ref_y.position;
    double z_ref = ref_z.position;
    double vx_ref = ref_x.velocity;
    double vy_ref = ref_y.velocity;
    double vz_ref = ref_z.velocity;
    double ax_ref = ref_x.acceleration;
    double ay_ref = ref_y.acceleration;
    double az_ref = ref_z.acceleration;

    // 3. 悬停检测
    bool all_finished = x_finished && y_finished && z_finished;
    if (all_finished)
    {
      double pos_err = std::sqrt(
          (x_ref - x_meas) * (x_ref - x_meas) +
          (y_ref - y_meas) * (y_ref - y_meas) +
          (z_ref - z_meas) * (z_ref - z_meas));
      double vel_mag = std::sqrt(
          current_odom_.twist.twist.linear.x * current_odom_.twist.twist.linear.x +
          current_odom_.twist.twist.linear.y * current_odom_.twist.twist.linear.y +
          current_odom_.twist.twist.linear.z * current_odom_.twist.twist.linear.z);

      if (pos_err < 0.3 && vel_mag < 0.3)
      {
        if (!is_hover_stable_)
        {
          is_hover_stable_ = true;
          ROS_INFO("悬停稳定! pos_err=%.2fm, vel=%.2fm/s", pos_err, vel_mag);
        }
      }
    }

    // 4. LADRC 观测器静默运行 (状态估计)
    double ax_cmd = ladrc_x_->update(x_ref, vx_ref, ax_ref, x_meas);
    double ay_cmd = ladrc_y_->update(y_ref, vy_ref, ay_ref, y_meas);
    double az_cmd = ladrc_z_->update(z_ref, vz_ref, az_ref, z_meas);

    // 5. IAPF 避障
    Eigen::Vector3d iapf = computeIAPF(x_meas, y_meas, z_meas);
    const double IAPF_POS_GAIN = 0.05;

    // 6. 发布设定点 (MAVROS ENU 坐标，直接发布)
    publishSetpoint(
        x_ref + IAPF_POS_GAIN * iapf.x(),
        y_ref + IAPF_POS_GAIN * iapf.y(),
        z_ref + IAPF_POS_GAIN * iapf.z());

    // 7. 发布 UAVStatus
    publishUAVStatus();

    // 日志
    ROS_INFO_THROTTLE(1.0,
        "UAV%d Ref[%.1f,%.1f,%.1f] Pos[%.2f,%.2f,%.2f] Cmd[%.1f,%.1f,%.1f]%s",
        uav_id_, x_ref, y_ref, z_ref,
        x_meas, y_meas, z_meas,
        ax_cmd, ay_cmd, az_cmd,
        (iapf.norm() > 0.1 ? " !IAPF!" : ""));
  }

  // ====================== 动态增益调节 ======================
  void applyDynamicGains()
  {
    double gain_mult = 1.0;
    if (motion_style_ == "smooth")       gain_mult = 0.7;
    else if (motion_style_ == "aggressive") gain_mult = 1.5;

    ladrc_x_->setObserverBandwidth(omega_o_x_ * gain_mult);
    ladrc_x_->setControllerBandwidth(omega_c_x_ * gain_mult);
    ladrc_y_->setObserverBandwidth(omega_o_y_ * gain_mult);
    ladrc_y_->setControllerBandwidth(omega_c_y_ * gain_mult);
    ladrc_z_->setObserverBandwidth(omega_o_z_ * gain_mult);
    ladrc_z_->setControllerBandwidth(omega_c_z_ * gain_mult);

    ROS_INFO_THROTTLE(5.0, "动态增益: %s → multiplier=%.1f", motion_style_.c_str(), gain_mult);
  }

  // ====================== IAPF 斥力计算 (与原 ROS2 版本完全一致) ======================
  Eigen::Vector3d computeIAPF(double x_meas, double y_meas, double z_meas)
  {
    Eigen::Vector3d F_rep(0.0, 0.0, 0.0);
    if (safety_factor_ <= 0.0 || neighbor_positions_.empty()) return F_rep;

    Eigen::Vector3d pos_own(x_meas + enu_offset_x_, y_meas + enu_offset_y_, z_meas + enu_offset_z_);

    for (const auto& [nbr_id, nbr_pos] : neighbor_positions_)
    {
      double d = (pos_own - nbr_pos).norm();
      if (d <= 0.01 || d >= iapf_safe_dist_) continue;

      Eigen::Vector3d dir = (pos_own - nbr_pos).normalized();
      double mag = iapf_rep_gain_ * (1.0 / d - 1.0 / iapf_safe_dist_) / (d * d);
      Eigen::Vector3d force = dir * mag;
      force.z() += mag * 0.05;  // Z 轴侧向力防止死锁
      F_rep += force;

      ROS_WARN_THROTTLE(0.5, "IAPF 避障: U%d d=%.2fm Frep[%.1f,%.1f,%.1f]",
          nbr_id, d, force.x(), force.y(), force.z());
    }

    return F_rep * safety_factor_;
  }

  // ====================== 辅助发布函数 ======================
  void publishUAVStatus()
  {
    uav_swarm_interfaces::UAVStatus msg;
    msg.uav_id = uav_id_;
    msg.is_hover_stable = is_hover_stable_;
    status_pub_.publish(msg);
  }

  // 发布位置设定点到 MAVROS (ENU 坐标，MAVROS 内部转为 PX4 NED)
  void publishSetpoint(double px_enu, double py_enu, double pz_enu)
  {
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.position.x = px_enu;
    msg.pose.position.y = py_enu;
    msg.pose.position.z = pz_enu;
    msg.pose.orientation.w = 1.0;  // 无旋转要求
    setpoint_pub_.publish(msg);
  }

  // ====================== 成员变量 ======================
  ros::NodeHandle nh_, pnh_;

  // LADRC 控制器 (不变)
  std::unique_ptr<ladrc_controller::LADRCController> ladrc_x_;
  std::unique_ptr<ladrc_controller::LADRCController> ladrc_y_;
  std::unique_ptr<ladrc_controller::LADRCController> ladrc_z_;

  // 订阅器
  ros::Subscriber odom_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber cmd_sub_;
  std::vector<ros::Subscriber> neighbor_subs_;

  // 发布器
  ros::Publisher setpoint_pub_;
  ros::Publisher status_pub_;
  ros::Publisher odom_pub_;

  // 服务客户端
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;

  // 定时器
  ros::Timer control_timer_;
  ros::Timer sm_timer_;

  // 状态
  std::atomic<FlightState> flight_state_{FlightState::INIT};
  uint64_t offboard_setpoint_counter_ = 0;
  uint8_t self_uav_id_ = 0;
  double dt_ = 0.02;

  // 参数
  double control_freq_, omega_o_x_, omega_o_y_, omega_o_z_;
  double omega_c_x_, omega_c_y_, omega_c_z_;
  double b0_x_, b0_y_, b0_z_;
  double max_vel_, max_acc_x_, max_acc_y_, max_acc_z_;
  double enu_offset_x_ = 0.0, enu_offset_y_ = 0.0, enu_offset_z_ = 0.0;
  double iapf_safe_dist_ = 1.0, iapf_rep_gain_ = 1.0;

  // 命令数据
  uint8_t uav_id_ = 0;
  double target_pos_x_ = 0.0, target_pos_y_ = 0.0, target_pos_z_ = 0.0;
  double target_duration_ = 0.0;
  std::string motion_style_ = "normal";
  double safety_factor_ = 0.0;
  bool has_command_ = false;

  // 悬停保持
  bool hover_hold_set_ = false;
  double hover_hold_x_ = 0.0, hover_hold_y_ = 0.0, hover_hold_z_ = 0.0;

  // 轨迹
  ladrc_controller::MinimumJerkTrajectory traj_x_, traj_y_, traj_z_;
  ros::Time command_start_time_;

  // Odom / State
  nav_msgs::Odometry current_odom_;
  mavros_msgs::State current_state_;
  bool has_odom_ = false;
  bool is_hover_stable_ = false;
  int odom_pub_counter_ = 0;

  // IAPF 邻居
  std::unordered_map<int, Eigen::Vector3d> neighbor_positions_;
};

// ====================== 入口 ======================
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ladrc_position_controller");
  ros::NodeHandle nh;       // 公共句柄 (命名空间内的相对话题)
  ros::NodeHandle pnh("~"); // 私有句柄 (节点命名空间内的参数)

  LADRCPositionControllerNode node(nh, pnh);
  ros::spin();
  return 0;
}
