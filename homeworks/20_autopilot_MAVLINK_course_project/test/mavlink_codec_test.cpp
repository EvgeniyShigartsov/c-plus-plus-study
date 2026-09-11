#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <numbers>

#include "mavlink/Codec.hpp"
#include "mavlink/Frames.hpp"
#include "mavlink/RadioControl.hpp"

TEST(MavlinkFrames, NedYawIsInvolution)
{
  const std::array<float, 5> sample_directions{-3.0f, -1.5f, 0.0f, 0.7f, 2.9f};

  for (const float dir : sample_directions) {
    EXPECT_NEAR(mav::ned_yaw(mav::ned_yaw(dir)), dir, 1e-5f);
  }
}

TEST(MavlinkFrames, NedYawMapsCardinalDirections)
{
  const float half_pi = std::numbers::pi_v<float> * 0.5f;
  EXPECT_NEAR(mav::ned_yaw(0.0f), half_pi, 1e-5f);
  EXPECT_NEAR(mav::ned_yaw(half_pi), 0.0f, 1e-5f);
}

TEST(MavlinkCodec, HeartbeatRoundTrip)
{
  const mav::Heartbeat original{
    .type = MAV_TYPE_ONBOARD_CONTROLLER,
    .autopilot = MAV_AUTOPILOT_INVALID,
    .base_mode = 0,
    .custom_mode = 3,
    .system_status = MAV_STATE_ACTIVE,
  };

  const mavlink_message_t msg = mav::pack_heartbeat(mav::kAutopilot, original);

  EXPECT_EQ(msg.msgid, MAVLINK_MSG_ID_HEARTBEAT);
  EXPECT_EQ(mav::parse_heartbeat(msg), original);
}

TEST(MavlinkCodec, HeartbeatFrameCarriesSender)
{
  const mavlink_message_t msg = mav::pack_heartbeat(mav::kGcs, {.type = MAV_TYPE_GCS});
  EXPECT_EQ((mav::Identity{msg.sysid, msg.compid}), mav::kGcs);
}

TEST(MavlinkCodec, LocalPositionNedSwapsAxes)
{
  VehicleState state{};
  state.mission_time_ms = 1234;
  state.x = 10.0f;   // схід
  state.y = -5.0f;   // північ
  state.z = 100.0f;  // висота
  state.vx = 3.0f;   // швидкість на схід
  state.vy = 4.0f;   // швидкість на північ

  const mav::LocalPositionNed ned = mav::parse_local_position_ned(mav::pack_local_position_ned(mav::kVehicle, state));

  const unsigned int expected_time_boot = 1234u;
  const float expected_y = -5.0f;
  const float expected_x = 10.0f;
  const float expeсted_alt = -100.f;
  const float expected_velocity_y = 4.0f;
  const float expected_velocity_x = 3.0f;

  EXPECT_EQ(ned.time_boot_ms, expected_time_boot);
  EXPECT_FLOAT_EQ(ned.north, expected_y);
  EXPECT_FLOAT_EQ(ned.east, expected_x);
  EXPECT_FLOAT_EQ(ned.down, expeсted_alt);
  EXPECT_FLOAT_EQ(ned.velocity_north, expected_velocity_y);
  EXPECT_FLOAT_EQ(ned.velocity_east, expected_velocity_x);
}

TEST(MavlinkCodec, VehicleStateSurvivesNedPlusAttitude)
{
  VehicleState original{};
  original.mission_time_ms = 5000;
  original.x = 120.0f;
  original.y = 80.0f;
  original.z = 50.0f;
  original.vx = 6.0f;
  original.vy = 2.5f;
  original.speed = std::hypot(original.vx, original.vy);
  original.dir = 0.9f;

  const mav::LocalPositionNed pos = mav::parse_local_position_ned(mav::pack_local_position_ned(mav::kVehicle, original));
  const mav::Attitude att = mav::parse_attitude(mav::pack_attitude(mav::kVehicle, original));
  const VehicleState back = mav::to_vehicle_state(pos, att);

  EXPECT_EQ(back.mission_time_ms, original.mission_time_ms);
  EXPECT_NEAR(back.x, original.x, 1e-3f);
  EXPECT_NEAR(back.y, original.y, 1e-3f);
  EXPECT_NEAR(back.z, original.z, 1e-3f);
  EXPECT_NEAR(back.vx, original.vx, 1e-3f);
  EXPECT_NEAR(back.vy, original.vy, 1e-3f);
  EXPECT_NEAR(back.speed, original.speed, 1e-3f);
  EXPECT_NEAR(back.dir, original.dir, 1e-5f);
}

TEST(MavlinkCodec, GlobalPositionIntConventions)
{
  VehicleState state{};
  state.mission_time_ms = 100;
  state.x = 0.0f;  // на опорній точці
  state.y = 0.0f;
  state.vx = 5.0f;  // рух на схід 5 м/с
  state.vy = 0.0f;
  state.dir = 0.0f;  // курс на схід

  const mavlink_message_t msg = mav::pack_global_position_int(mav::kVehicle, state);
  mavlink_global_position_int_t g{};
  mavlink_msg_global_position_int_decode(&msg, &g);

  const int32_t expected_lat = static_cast<int32_t>(std::llround(mav::kRefLat * 1e7));
  const int32_t expected_lon = static_cast<int32_t>(std::llround(mav::kRefLon * 1e7));
  const int16_t expected_velocity_north = 0;
  const int16_t expected_velocity_east = 500;  // 5 м/с -> см/с
  const uint16_t expected_heading = 9000;      // компас 90.00° -> сотих градуса

  EXPECT_EQ(g.lat, expected_lat);
  EXPECT_EQ(g.lon, expected_lon);
  EXPECT_EQ(g.vx, expected_velocity_north);
  EXPECT_EQ(g.vy, expected_velocity_east);
  EXPECT_EQ(g.hdg, expected_heading);
}

TEST(MavlinkRadioControl, NormalizedToPwmBoundaries)
{
  const uint16_t expected_neutral = 1500;
  const uint16_t expected_max = 2000;
  const uint16_t expected_min = 1000;

  EXPECT_EQ(mav::normalized_to_pwm(0.0f), expected_neutral);
  EXPECT_EQ(mav::normalized_to_pwm(1.0f), expected_max);
  EXPECT_EQ(mav::normalized_to_pwm(-1.0f), expected_min);
  EXPECT_EQ(mav::normalized_to_pwm(2.0f), expected_max);
  EXPECT_EQ(mav::normalized_to_pwm(-2.0f), expected_min);
}

TEST(MavlinkRadioControl, PwmToNormalizedRoundTrip)
{
  const std::array<float, 5> sample_values{-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};

  for (const float value : sample_values) {
    EXPECT_NEAR(mav::pwm_to_normalized(mav::normalized_to_pwm(value)), value, 1e-3f);
  }
}

TEST(MavlinkRadioControl, ReleasedPwmIsNeutral)
{
  EXPECT_FLOAT_EQ(mav::pwm_to_normalized(mav::kPwmReleased), 0.0f);
}

TEST(MavlinkCodec, RadioControlChannelsRoundTrip)
{
  const mav::RadioControlChannels original{.time_boot_ms = 42, .roll = 1600, .throttle = 1400};

  const mav::RadioControlChannels parsed = mav::parse_radio_control_channels(mav::pack_radio_control_channels(mav::kVehicle, original));

  EXPECT_EQ(parsed, original);
}

TEST(MavlinkCodec, RadioControlOverrideCarriesControlSignal)
{
  const ControlSignal original{.accel = 0.4f, .turnRate = -0.6f};

  const mavlink_message_t msg = mav::pack_radio_control_channels_override(mav::kAutopilot, mav::kVehicle, original);
  const mav::RadioControlOverride radio_control_override = mav::parse_radio_control_channels_override(msg);
  const ControlSignal back = mav::to_control_signal(radio_control_override);

  EXPECT_NEAR(back.accel, original.accel, 1e-3f);
  EXPECT_NEAR(back.turnRate, original.turnRate, 1e-3f);
}

TEST(MavlinkCodec, RadioControlOverrideAddressesTargetVehicle)
{
  const mavlink_message_t msg = mav::pack_radio_control_channels_override(mav::kAutopilot, mav::kVehicle, {});
  mavlink_rc_channels_override_t raw{};
  mavlink_msg_rc_channels_override_decode(&msg, &raw);

  EXPECT_EQ(raw.target_system, mav::kVehicle.sysid);
  EXPECT_EQ(raw.target_component, mav::kVehicle.compid);
}

TEST(MavlinkCodec, RadioControlOverrideReleasesUnusedChannels)
{
  const ControlSignal control{.accel = 0.5f, .turnRate = 0.5f};

  const mavlink_message_t msg = mav::pack_radio_control_channels_override(mav::kAutopilot, mav::kVehicle, control);
  mavlink_rc_channels_override_t raw{};
  mavlink_msg_rc_channels_override_decode(&msg, &raw);

  EXPECT_EQ(raw.chan2_raw, mav::kPwmReleased);
  EXPECT_EQ(raw.chan4_raw, mav::kPwmReleased);
}
