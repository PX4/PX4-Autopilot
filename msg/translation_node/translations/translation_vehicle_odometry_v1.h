#pragma once
// Translate VehicleOdometry v0 <-> v1

#include <limits>
#include <px4_msgs_old/msg/vehicle_odometry_v0.hpp>  // archived v0
#include <px4_msgs/msg/vehicle_odometry.hpp>         // current v1

class VehicleOdometryV1Translation {
public:
  using MessageOlder = px4_msgs_old::msg::VehicleOdometryV0;
  static_assert(MessageOlder::MESSAGE_VERSION == 0);

  using MessageNewer = px4_msgs::msg::VehicleOdometry;  // Make sure this file has MESSAGE_VERSION = 1
  static_assert(MessageNewer::MESSAGE_VERSION == 1);

  // Base topic name (no version suffix here; the client adds _v<VER>)
  static constexpr const char* kTopic = "fmu/out/vehicle_odometry";

  static inline void setNaN3(float v[3]) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    v[0] = nan; v[1] = nan; v[2] = nan;
  }

  // v0 -> v1
  static void fromOlder(const MessageOlder &old, MessageNewer &neo) {
    // scalar/enums
    neo.timestamp        = old.timestamp;
    neo.timestamp_sample = old.timestamp_sample;
    neo.pose_frame       = old.pose_frame;
    neo.velocity_frame   = old.velocity_frame;
    neo.reset_counter    = old.reset_counter;
    neo.quality          = old.quality;

    // arrays (copy-through)
    for (int i=0;i<3;i++) neo.position[i]            = old.position[i];
    for (int i=0;i<4;i++) neo.q[i]                   = old.q[i];
    for (int i=0;i<3;i++) neo.velocity[i]            = old.velocity[i];
    for (int i=0;i<3;i++) neo.angular_velocity[i]    = old.angular_velocity[i];
    for (int i=0;i<3;i++) neo.position_variance[i]   = old.position_variance[i];
    for (int i=0;i<3;i++) neo.orientation_variance[i]= old.orientation_variance[i];
    for (int i=0;i<3;i++) neo.velocity_variance[i]   = old.velocity_variance[i];

    // NEW in v1: acceleration[3] wasn't in v0 -> set unknown
    neo.acceleration.fill(std::numeric_limits<float>::quiet_NaN());

  }

  // v1 -> v0
  static void toOlder(const MessageNewer &neo, MessageOlder &old) {
    // scalar/enums
    old.timestamp        = neo.timestamp;
    old.timestamp_sample = neo.timestamp_sample;
    old.pose_frame       = neo.pose_frame;
    old.velocity_frame   = neo.velocity_frame;
    old.reset_counter    = neo.reset_counter;
    old.quality          = neo.quality;

    // arrays (copy-through)
    for (int i=0;i<3;i++) old.position[i]            = neo.position[i];
    for (int i=0;i<4;i++) old.q[i]                   = neo.q[i];
    for (int i=0;i<3;i++) old.velocity[i]            = neo.velocity[i];
    for (int i=0;i<3;i++) old.angular_velocity[i]    = neo.angular_velocity[i];
    for (int i=0;i<3;i++) old.position_variance[i]   = neo.position_variance[i];
    for (int i=0;i<3;i++) old.orientation_variance[i]= neo.orientation_variance[i];
    for (int i=0;i<3;i++) old.velocity_variance[i]   = neo.velocity_variance[i];

    // acceleration doesn't exist in v0 -> intentionally dropped
  }
};

REGISTER_TOPIC_TRANSLATION_DIRECT(VehicleOdometryV1Translation);
