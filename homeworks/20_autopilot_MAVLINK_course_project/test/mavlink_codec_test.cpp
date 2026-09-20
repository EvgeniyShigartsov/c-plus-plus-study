#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <numbers>
#include <optional>

#include "link/UdpLink.hpp"
#include "mavlink/Codec.hpp"
#include "mavlink/Endpoint.hpp"
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

TEST(MavlinkRadioControl, NeutralPwmIsInDeadband)
{
  EXPECT_TRUE(mav::is_in_deadband(mav::kPwmNeutral));
}

TEST(MavlinkRadioControl, PwmAtDeadbandEdgeIsInDeadband)
{
  EXPECT_TRUE(mav::is_in_deadband(mav::kPwmNeutral - mav::kPwmDeadband));
  EXPECT_TRUE(mav::is_in_deadband(mav::kPwmNeutral + mav::kPwmDeadband));
}

TEST(MavlinkRadioControl, PwmPastDeadbandEdgeIsNotInDeadband)
{
  EXPECT_FALSE(mav::is_in_deadband(mav::kPwmNeutral - mav::kPwmDeadband - 1));
  EXPECT_FALSE(mav::is_in_deadband(mav::kPwmNeutral + mav::kPwmDeadband + 1));
}

TEST(MavlinkCodec, RadioControlChannelsRoundTrip)
{
  const mav::RadioControlChannels original{.time_boot_ms = 42, .roll = 1600, .throttle = 1400};

  const mav::RadioControlChannels parsed = mav::parse_radio_control_channels(mav::pack_radio_control_channels(mav::kVehicle, original));

  EXPECT_EQ(parsed, original);
}

TEST(MavlinkCodec, DeadbandRequiresBothChannelsNeutral)
{
  const mav::RadioControlChannels bothNeutral{.roll = mav::kPwmNeutral, .throttle = mav::kPwmNeutral};
  const mav::RadioControlChannels rollOff{.roll = mav::kPwmMax, .throttle = mav::kPwmNeutral};
  const mav::RadioControlChannels throttleOff{.roll = mav::kPwmNeutral, .throttle = mav::kPwmMax};

  EXPECT_TRUE(mav::are_channels_in_deadband(bothNeutral));
  EXPECT_FALSE(mav::are_channels_in_deadband(rollOff));
  EXPECT_FALSE(mav::are_channels_in_deadband(throttleOff));
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

TEST(MavlinkCodec, EnableCommandRoundTrip)
{
  const mav::EnableCommand original = {.enabled = true};

  const mavlink_message_t msg = mav::pack_enable_command(mav::kGcs, mav::kAutopilot, original);

  EXPECT_EQ(msg.msgid, MAVLINK_MSG_ID_COMMAND_LONG);
  EXPECT_EQ(mav::parse_enable_command(msg), original);
}

TEST(MavlinkCodec, TargetDesignationRoundTrip)
{
  const mav::TargetDesignation original = {
    .target_id = 2,
    .target_count = 5,
    .latitude = 50.45123,
    .longitude = 30.52456,
  };

  const mav::TargetDesignation parsed = mav::parse_target_designation(mav::pack_target_designation(mav::kGcs, mav::kAutopilot, original));

  EXPECT_EQ(parsed.target_id, original.target_id);
  EXPECT_EQ(parsed.target_count, original.target_count);
  EXPECT_NEAR(parsed.latitude, original.latitude, 1e-4);
  EXPECT_NEAR(parsed.longitude, original.longitude, 1e-4);
}

TEST(MavlinkCodec, DropNotificationRoundTrip)
{
  const mav::DropNotification original = {
    .latitude = 50.4510, .longitude = 30.5240, .altitude = 42.5f, .target_id = 2, .bomb_flight_time_sec = 3.75f};

  const mav::DropNotification parsed = mav::parse_drop_notification(mav::pack_drop_notification(mav::kAutopilot, mav::kVehicle, original));

  EXPECT_NEAR(parsed.latitude, original.latitude, 1e-4);
  EXPECT_NEAR(parsed.longitude, original.longitude, 1e-4);
  EXPECT_FLOAT_EQ(parsed.altitude, original.altitude);
  EXPECT_EQ(parsed.target_id, original.target_id);
  EXPECT_FLOAT_EQ(parsed.bomb_flight_time_sec, original.bomb_flight_time_sec);
}

TEST(MavlinkCodec, MissionCompleteNotificationRoundTrip)
{
  const mav::MissionCompleteNotification original = {.completed = true};

  const mav::MissionCompleteNotification parsed =
    mav::parse_mission_complete_notification(mav::pack_mission_complete_notification(mav::kAutopilot, mav::kVehicle, original));

  EXPECT_EQ(parsed.completed, original.completed);
}

TEST(MavlinkCodec, ManualDropCommandRoundTrip)
{
  const mavlink_message_t msg = mav::pack_manual_drop_command(mav::kGcs, mav::kAutopilot, {});

  EXPECT_EQ(msg.msgid, MAVLINK_MSG_ID_COMMAND_LONG);
  EXPECT_EQ(mavlink_msg_command_long_get_command(&msg), mav::kManualDropCommandId);
  EXPECT_EQ((mav::Identity{.sysid = msg.sysid, .compid = msg.compid}), mav::kGcs);

  mav::parse_manual_drop_command(msg);
}

TEST(MavlinkCodec, CommandAckRoundTrip)
{
  const mav::CommandAcknowledgement original = {.command = mav::kEnableCommandId, .result = MAV_RESULT_ACCEPTED};

  const mavlink_message_t msg = mav::pack_command_acknowledgement(mav::kAutopilot, mav::kGcs, original);

  EXPECT_EQ(msg.msgid, MAVLINK_MSG_ID_COMMAND_ACK);
  EXPECT_EQ(mav::parse_command_acknowledgement(msg), original);
}

TEST(MavlinkCodec, StatusTextRoundTrip)
{
  const mav::StatusText original{.severity = MAV_SEVERITY_WARNING, .text = "YIELD: operator input"};

  const mav::StatusText parsed = mav::parse_status_text(mav::pack_status_text(mav::kAutopilot, original));

  EXPECT_EQ(parsed, original);
}

TEST(MavlinkCodec, StatusTextTruncatesAtWireLimit)
{
  const std::string too_long(mav::kStatusTextMaxLength + 10, 'x');

  const mav::StatusText parsed = mav::parse_status_text(mav::pack_status_text(mav::kAutopilot, {.text = too_long}));

  EXPECT_EQ(parsed.text.size(), mav::kStatusTextMaxLength);
}

TEST(MavlinkCodec, SimStepRoundTripForEveryState)
{
  const std::array<const char*, 6> states{"Stopped", "Turning", "Accelerating", "Moving", "Decelerating", "WaitingForConnection"};

  for (const char* state : states) {
    const SimStep original{
      .pos = {-12.5f, 340.25f},
      .dropPoint = {100.0f, -200.5f},
      .aimPoint = {101.5f, -199.0f},
      .predictedTarget = {-3.0f, 4.0f},
      .direction = -2.5f,
      .state = state,
      .targetIdx = 3,
      .step = 123456,
      .timeSecSinceStart = 1234.56f,
    };

    const std::optional<SimStep> parsed = mav::parse_sim_step(mav::pack_sim_step(mav::kAutopilot, original));

    ASSERT_TRUE(parsed.has_value()) << state;
    EXPECT_EQ(parsed->state, original.state);
    EXPECT_EQ(parsed->step, original.step);
    EXPECT_EQ(parsed->targetIdx, original.targetIdx);
    EXPECT_FLOAT_EQ(parsed->pos.x, original.pos.x);
    EXPECT_FLOAT_EQ(parsed->pos.y, original.pos.y);
    EXPECT_FLOAT_EQ(parsed->dropPoint.x, original.dropPoint.x);
    EXPECT_FLOAT_EQ(parsed->dropPoint.y, original.dropPoint.y);
    EXPECT_FLOAT_EQ(parsed->aimPoint.x, original.aimPoint.x);
    EXPECT_FLOAT_EQ(parsed->aimPoint.y, original.aimPoint.y);
    EXPECT_FLOAT_EQ(parsed->predictedTarget.x, original.predictedTarget.x);
    EXPECT_FLOAT_EQ(parsed->predictedTarget.y, original.predictedTarget.y);
    EXPECT_FLOAT_EQ(parsed->direction, original.direction);
    EXPECT_FLOAT_EQ(parsed->timeSecSinceStart, original.timeSecSinceStart);
  }
}

TEST(MavlinkCodec, SimStepUnknownStateBecomesEmpty)
{
  const SimStep original{.pos = {0.0f, 0.0f}, .state = "Flying", .targetIdx = 0};

  const std::optional<SimStep> parsed = mav::parse_sim_step(mav::pack_sim_step(mav::kAutopilot, original));

  ASSERT_TRUE(parsed.has_value());
  EXPECT_TRUE(parsed->state.empty());
}

TEST(MavlinkCodec, SimStepSurvivesWireEncoding)
{
  const SimStep original{.pos = {1.0f, 2.0f}, .state = "Moving", .targetIdx = 2, .step = 42, .timeSecSinceStart = 0.42f};
  const mavlink_message_t msg = mav::pack_sim_step(mav::kAutopilot, original);

  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
  const uint16_t length = mavlink_msg_to_send_buffer(buffer.data(), &msg);
  mavlink_message_t received{};
  mavlink_status_t status{};
  bool complete = false;
  for (uint16_t i = 0; i < length && !complete; ++i) {
    complete = mavlink_parse_char(MAVLINK_COMM_3, buffer[i], &received, &status) != 0;
  }

  ASSERT_TRUE(complete);
  const std::optional<SimStep> parsed = mav::parse_sim_step(received);
  ASSERT_TRUE(parsed.has_value());
  EXPECT_EQ(parsed->step, 42);
  EXPECT_EQ(parsed->state, "Moving");
}

TEST(MavlinkCodec, SimStepRejectsForeignDebugFloatArray)
{
  std::array<float, MAVLINK_MSG_DEBUG_FLOAT_ARRAY_FIELD_DATA_LEN> data{};
  mavlink_message_t msg{};
  mavlink_msg_debug_float_array_pack(1, 1, &msg, 0, "other", 0, data.data());

  EXPECT_FALSE(mav::parse_sim_step(msg).has_value());
}

TEST(MavlinkEndpoint, ConstructsAndPollsIdleSocketCleanly)
{
  const UdpLink udp("127.0.0.1", 15020);
  ASSERT_TRUE(udp.isOpen());

  const mav::MavlinkEndpoint endpoint(udp, MAVLINK_COMM_1);

  EXPECT_TRUE(endpoint.poll().empty());
}

TEST(MavlinkEndpoint, SendDoesNotThrowWithNoListener)
{
  const UdpLink udp("127.0.0.1", 15021);
  const mav::MavlinkEndpoint endpoint(udp, MAVLINK_COMM_2);

  const mavlink_message_t heartbeat = mav::pack_heartbeat(mav::kAutopilot, {.type = MAV_TYPE_ONBOARD_CONTROLLER});

  EXPECT_NO_THROW(endpoint.send(heartbeat));
}
