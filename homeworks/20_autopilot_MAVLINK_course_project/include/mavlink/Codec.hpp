#pragma once
#include <cstdint>

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

  bool operator==(const LocalPositionNed&) const = default;
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

  bool operator==(const RadioControlOverride&) const = default;
};

struct Attitude {
  uint32_t time_boot_ms = 0;  // час від старту, мс
  float yaw = 0.0f;           // курс, куди дивиться ніс, у системі NED (за годинниковою)

  bool operator==(const Attitude&) const = default;
};

constexpr uint16_t kEnableCommandId = MAV_CMD_USER_1;
constexpr uint16_t kTargetDesignationCommandId = MAV_CMD_USER_2;
constexpr uint16_t kDropNotificationCommandId = MAV_CMD_USER_3;

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

  bool operator==(const TargetDesignation&) const = default;
};

// автопілот -> борт/GCS
struct DropNotification {
  double latitude = 0.0;
  double longitude = 0.0;
  float altitude = 0.0f;

  bool operator==(const DropNotification&) const = default;
};

struct CommandAcknowledgement {
  uint16_t command = 0;
  uint8_t result = 0;

  bool operator==(const CommandAcknowledgement&) const = default;
};

mavlink_message_t pack_heartbeat(const Identity& from, const Heartbeat& heartbeat);
Heartbeat parse_heartbeat(const mavlink_message_t& msg);

mavlink_message_t pack_local_position_ned(const Identity& from, const VehicleState& state);
LocalPositionNed parse_local_position_ned(const mavlink_message_t& msg);

mavlink_message_t pack_attitude(const Identity& from, const VehicleState& state);
Attitude parse_attitude(const mavlink_message_t& msg);

mavlink_message_t pack_global_position_int(const Identity& from, const VehicleState& state);

VehicleState to_vehicle_state(const LocalPositionNed& position, const Attitude& attitude);

mavlink_message_t pack_radio_control_channels(const Identity& from, const RadioControlChannels& channels);
RadioControlChannels parse_radio_control_channels(const mavlink_message_t& msg);

mavlink_message_t pack_radio_control_channels_override(const Identity& from, const Identity& target, const ControlSignal& control);
RadioControlOverride parse_radio_control_channels_override(const mavlink_message_t& msg);

RadioControlOverride to_radio_control_override(const ControlSignal& control);
ControlSignal to_control_signal(const RadioControlOverride& radio_control_override);

// confirmation - MAVLink конвенція, де 0 = перша передача, 1..255 = повтор
mavlink_message_t pack_enable_command(const Identity& from, const Identity& target, const EnableCommand& command, uint8_t confirmation = 0);
EnableCommand parse_enable_command(const mavlink_message_t& msg);

mavlink_message_t pack_target_designation(const Identity& from, const Identity& target, const TargetDesignation& designation);
TargetDesignation parse_target_designation(const mavlink_message_t& msg);

mavlink_message_t pack_drop_notification(const Identity& from,
                                         const Identity& target,
                                         const DropNotification& drop,
                                         uint8_t confirmation = 0);
DropNotification parse_drop_notification(const mavlink_message_t& msg);

mavlink_message_t pack_command_acknowledgement(const Identity& from, const Identity& target, const CommandAcknowledgement& acknowledgement);
CommandAcknowledgement parse_command_acknowledgement(const mavlink_message_t& msg);

}  // namespace mav
