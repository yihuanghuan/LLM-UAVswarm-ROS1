#pragma once
#include <array>
#include <cstdint>
// Internal NED samples preserve the frozen algorithm's coordinate contract.
// These are plain data, not PX4 ROS2 messages or ROS topics.
namespace transport_samples {
struct Odometry { std::array<double,3> position{},velocity{}; uint64_t timestamp_sample=0; uint8_t reset_counter=0; };
struct VehicleStatus {bool failsafe=false,pre_flight_checks_pass=false;};
struct OffboardControlMode {uint64_t timestamp;bool position,velocity,acceleration,attitude,body_rate;};
struct TrajectorySetpoint {uint64_t timestamp;std::array<float,3> position,velocity,acceleration;float yaw;};
}
