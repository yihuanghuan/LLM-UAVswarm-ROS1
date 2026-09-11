#include <gtest/gtest.h>

#include "ladrc_controller/swarm_state_builder.hpp"

TEST(SwarmStateBuilder, ConvertsPx4NedToGlobalEnu)
{
  transport_samples::Odometry source;
  source.position = {1.0F, 2.0F, -3.0F};
  source.velocity = {4.0F, 5.0F, -6.0F};
  ros::Time stamp;
  stamp.sec = 12;
  stamp.nsec = 34;

  const auto state = ladrc_controller::buildSwarmState(
    source, stamp, 7, 10.0, 20.0, 0.5);

  EXPECT_EQ(state.header.frame_id, "world");
  EXPECT_EQ(state.child_frame_id, "uav7/base_link_enu");
  EXPECT_EQ(state.header.stamp.sec, 12);
  EXPECT_EQ(state.header.stamp.nsec, 34U);
  EXPECT_DOUBLE_EQ(state.pose.pose.position.x, 12.0);
  EXPECT_DOUBLE_EQ(state.pose.pose.position.y, 21.0);
  EXPECT_DOUBLE_EQ(state.pose.pose.position.z, 3.5);
  EXPECT_DOUBLE_EQ(state.twist.twist.linear.x, 5.0);
  EXPECT_DOUBLE_EQ(state.twist.twist.linear.y, 4.0);
  EXPECT_DOUBLE_EQ(state.twist.twist.linear.z, 6.0);
  EXPECT_DOUBLE_EQ(state.pose.pose.orientation.w, 1.0);
}
