#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

#include <common/mavlink.h>

#include "mavlink/Identity.hpp"
#include "types.hpp"

namespace mav {

struct Heartbeat {
  uint8_t type = 0;       // MAV_TYPE_*
  uint8_t autopilot = 0;  // MAV_AUTOPILOT_*
  uint8_t base_mode = 0;  // MAV_MODE_FLAG_*
  uint32_t custom_mode = 0;
  uint8_t system_status = 0;  // MAV_STATE_*

  bool operator==(const Heartbeat&) const = default;
};

struct LocalPositionNed {
  uint32_t time_boot_ms = 0;  // час від старту, мс

  // де я
  float north = 0.0f;
  float east = 0.0f;
  float down = 0.0f;  // висота зі знаком мінус

  // швидкість руху
  float velocity_north = 0.0f;
  float velocity_east = 0.0f;
  float velocity_down = 0.0f;
};

struct RadioControlChannels {
  uint32_t time_boot_ms = 0;
  uint16_t roll = 0;
  uint16_t throttle = 0;

  bool operator==(const RadioControlChannels&) const = default;
};

struct RadioControlOverride {
  uint16_t roll = 0;
  uint16_t throttle = 0;
};

struct Attitude {
  uint32_t time_boot_ms = 0;  // час від старту, мс
  float yaw = 0.0f;           // курс, куди дивиться ніс, у системі NED (за годинниковою)
};

constexpr uint16_t kEnableCommandId = MAV_CMD_USER_1;
constexpr uint16_t kTargetDesignationCommandId = MAV_CMD_USER_2;
constexpr uint16_t kDropNotificationCommandId = MAV_CMD_USER_3;
constexpr uint16_t kMissionCompleteCommandId = MAV_CMD_USER_4;
constexpr uint16_t kManualDropCommandId = MAV_CMD_USER_5;

// GCS -> автопілот, увімкнути / вимкнути.
struct EnableCommand {
  bool enabled = false;

  bool operator==(const EnableCommand&) const = default;
};

// Компаньйон (Pi) -> autopilot, призначення цілі
struct TargetDesignation {
  uint8_t target_id = 0;
  uint8_t target_count = 0;
  double latitude = 0.0;
  double longitude = 0.0;
};

// автопілот -> борт/GCS
struct DropNotification {
  double latitude = 0.0;
  double longitude = 0.0;
  float altitude = 0.0f;
  uint8_t target_id = 0;
  float bomb_flight_time_sec = 0.0f;
};

struct MissionCompleteNotification {
  bool completed = true;
};

struct ManualDropCommand {
  bool makeDrop = true;
};

struct CommandAcknowledgement {
  uint16_t command = 0;
  uint8_t result = 0;

  bool operator==(const CommandAcknowledgement&) const = default;
};

constexpr std::size_t kStatusTextMaxLength = MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN;

struct StatusText {
  uint8_t severity = MAV_SEVERITY_INFO;
  std::string text;

  bool operator==(const StatusText&) const = default;
};

mavlink_message_t pack_heartbeat(const Identity& from, const Heartbeat& heartbeat);
Heartbeat parse_heartbeat(const mavlink_message_t& msg);

mavlink_message_t pack_local_position_ned(const Identity& from, const VehicleState& state);
LocalPositionNed parse_local_position_ned(const mavlink_message_t& msg);

mavlink_message_t pack_attitude(const Identity& from, const VehicleState& state);
Attitude parse_attitude(const mavlink_message_t& msg);

mavlink_message_t pack_global_position_int(const Identity& from, const VehicleState& state);

VehicleState to_vehicle_state(const LocalPositionNed& position, const Attitude& attitude);

DroneTelemetry to_drone_telemetry(const VehicleState& state);

mavlink_message_t pack_radio_control_channels(const Identity& from, const RadioControlChannels& channels);
RadioControlChannels parse_radio_control_channels(const mavlink_message_t& msg);

bool are_channels_in_deadband(const RadioControlChannels& channels);

mavlink_message_t pack_radio_control_channels_override(const Identity& from, const Identity& target, const ControlSignal& control);
RadioControlOverride parse_radio_control_channels_override(const mavlink_message_t& msg);

RadioControlOverride to_radio_control_override(const ControlSignal& control);
ControlSignal to_control_signal(const RadioControlOverride& radio_control_override);

mavlink_message_t pack_enable_command(const Identity& from, const Identity& target, const EnableCommand& command);
EnableCommand parse_enable_command(const mavlink_message_t& msg);

mavlink_message_t pack_target_designation(const Identity& from, const Identity& target, const TargetDesignation& designation);
TargetDesignation parse_target_designation(const mavlink_message_t& msg);

mavlink_message_t pack_drop_notification(const Identity& from, const Identity& target, const DropNotification& drop);
DropNotification parse_drop_notification(const mavlink_message_t& msg);

mavlink_message_t pack_mission_complete_notification(const Identity& from,
                                                     const Identity& target,
                                                     const MissionCompleteNotification& notification);
MissionCompleteNotification parse_mission_complete_notification(const mavlink_message_t& msg);

mavlink_message_t pack_manual_drop_command(const Identity& from, const Identity& target, const ManualDropCommand& command);
ManualDropCommand parse_manual_drop_command(const mavlink_message_t& msg);

mavlink_message_t pack_command_acknowledgement(const Identity& from, const Identity& target, const CommandAcknowledgement& acknowledgement);
CommandAcknowledgement parse_command_acknowledgement(const mavlink_message_t& msg);

mavlink_message_t pack_status_text(const Identity& from, const StatusText& status);
StatusText parse_status_text(const mavlink_message_t& msg);

}  // namespace mav
