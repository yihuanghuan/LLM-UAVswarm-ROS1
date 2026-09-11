#ifndef LADRC_CONTROLLER__SWARM_STATE_BUILDER_HPP_
#define LADRC_CONTROLLER__SWARM_STATE_BUILDER_HPP_

#include <cstdint>
#include <string>

#include "ladrc_controller/transport_samples.hpp"
#include <nav_msgs/Odometry.h>
#include "ladrc_controller/transport_samples.hpp"

namespace ladrc_controller
{

inline nav_msgs::Odometry buildSwarmState(
  const transport_samples::Odometry & source,
  const ros::Time & stamp,
  uint8_t uav_id,
  double offset_x,
  double offset_y,
  double offset_z)
{
  nav_msgs::Odometry state;
  state.header.stamp = stamp;
  state.header.frame_id = "world";
  state.child_frame_id =
    "uav" + std::to_string(uav_id) + "/base_link_enu";
  state.pose.pose.position.x = source.position[1] + offset_x;
  state.pose.pose.position.y = source.position[0] + offset_y;
  state.pose.pose.position.z = -source.position[2] + offset_z;
  state.pose.pose.orientation.w = 1.0;
  state.twist.twist.linear.x = source.velocity[1];
  state.twist.twist.linear.y = source.velocity[0];
  state.twist.twist.linear.z = -source.velocity[2];
  state.pose.covariance[0] = -1.0;
  state.twist.covariance[0] = -1.0;
  return state;
}

}  // namespace ladrc_controller

#endif  // LADRC_CONTROLLER__SWARM_STATE_BUILDER_HPP_
