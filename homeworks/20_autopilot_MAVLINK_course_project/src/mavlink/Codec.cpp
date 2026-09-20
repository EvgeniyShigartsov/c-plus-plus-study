#include "mavlink/Codec.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <numbers>
#include <string_view>

#include "mavlink/Frames.hpp"
#include "mavlink/RadioControl.hpp"

namespace mav {

// --- HEARTBEAT ---

mavlink_message_t pack_heartbeat(const Identity& from, const Heartbeat& heartbeat)
{
  mavlink_message_t msg{};
  mavlink_msg_heartbeat_pack(from.sysid,
                             from.compid,
                             &msg,
                             heartbeat.type,
                             heartbeat.autopilot,
                             heartbeat.base_mode,
                             heartbeat.custom_mode,
                             heartbeat.system_status);
  return msg;
}

Heartbeat parse_heartbeat(const mavlink_message_t& msg)
{
  mavlink_heartbeat_t raw{};
  mavlink_msg_heartbeat_decode(&msg, &raw);

  return Heartbeat{
    .type = raw.type,
    .autopilot = raw.autopilot,
    .base_mode = raw.base_mode,
    .custom_mode = raw.custom_mode,
    .system_status = raw.system_status,
  };
}

// --- LOCAL_POSITION_NED --- (звичайний x/y/z -> NED x/y/z)

mavlink_message_t pack_local_position_ned(const Identity& from, const VehicleState& state)
{
  mavlink_message_t msg{};
  mavlink_msg_local_position_ned_pack(from.sysid,
                                      from.compid,
                                      &msg,
                                      state.mission_time_ms,
                                      state.y,   // NED x = північ
                                      state.x,   // NED y = схід
                                      -state.z,  // NED z = вниз
                                      state.vy,  // NED vx = швидкість на північ
                                      state.vx,  // NED vy = швидкість на схід
                                      0.0f);     // NED vz = швидкість по вертикалі
  return msg;
}

LocalPositionNed parse_local_position_ned(const mavlink_message_t& msg)
{
  mavlink_local_position_ned_t raw{};
  mavlink_msg_local_position_ned_decode(&msg, &raw);

  // x/y/z у local_position_ned задані в системі координат NED
  return {
    .time_boot_ms = raw.time_boot_ms,
    .north = raw.x,
    .east = raw.y,
    .down = raw.z,
    .velocity_north = raw.vx,
    .velocity_east = raw.vy,
    .velocity_down = raw.vz,
  };
}

// --- ATTITUDE ---

mavlink_message_t pack_attitude(const Identity& from, const VehicleState& state)
{
  mavlink_message_t msg{};
  mavlink_msg_attitude_pack(from.sysid,
                            from.compid,
                            &msg,
                            state.mission_time_ms,
                            0.0f,                // roll / крен
                            0.0f,                // pitch / тангаж
                            ned_yaw(state.dir),  // yaw / напрям носу
                            0.0f,
                            0.0f,
                            0.0f);
  return msg;
}

Attitude parse_attitude(const mavlink_message_t& msg)
{
  mavlink_attitude_t raw{};
  mavlink_msg_attitude_decode(&msg, &raw);
  return {.time_boot_ms = raw.time_boot_ms, .yaw = raw.yaw};
}

// --- GLOBAL_POSITION_INT ---

mavlink_message_t pack_global_position_int(const Identity& from, const VehicleState& state)
{
  const Gps gps = to_gps(state.x, state.y);

  const float yaw = ned_yaw(state.dir);
  float heading_deg = std::fmod(yaw * 180.0f / std::numbers::pi_v<float>, 360.0f);
  if (heading_deg < 0.0f) {
    heading_deg += 360.0f;
  }

  // MAVLink-одиниці: градуси × 1e7, метри - міліметри, м/с - см/с, градуси - сотих градуса
  const int32_t lat_1e7 = static_cast<int32_t>(std::llround(gps.lat * 1e7));
  const int32_t lon_1e7 = static_cast<int32_t>(std::llround(gps.lon * 1e7));
  const int32_t alt_mm = static_cast<int32_t>(std::lround(state.z * 1000.0f));
  const int16_t velocity_north_cm_s = static_cast<int16_t>(std::lround(state.vy * 100.0f));
  const int16_t velocity_east_cm_s = static_cast<int16_t>(std::lround(state.vx * 100.0f));
  const uint16_t heading_cdeg = static_cast<uint16_t>(std::lround(heading_deg * 100.0f));

  mavlink_message_t msg{};
  mavlink_msg_global_position_int_pack(from.sysid,
                                       from.compid,
                                       &msg,
                                       state.mission_time_ms,
                                       lat_1e7,
                                       lon_1e7,
                                       alt_mm,
                                       alt_mm,
                                       velocity_north_cm_s,
                                       velocity_east_cm_s,
                                       0,
                                       heading_cdeg);
  return msg;
}

VehicleState to_vehicle_state(const LocalPositionNed& position, const Attitude& attitude)
{
  VehicleState state{};
  state.mission_time_ms = position.time_boot_ms;
  state.x = position.east;
  state.y = position.north;
  state.z = -position.down;
  state.vx = position.velocity_east;
  state.vy = position.velocity_north;
  state.speed = std::hypot(state.vx, state.vy);
  state.dir = ned_yaw(attitude.yaw);
  return state;
}

DroneTelemetry to_drone_telemetry(const VehicleState& state)
{
  return {
    .pos = {.x = state.x, .y = state.y},
    .speed = state.speed,
    .dir = state.dir,
    .timeSinceStart = static_cast<float>(state.mission_time_ms) / 1000.0f,
  };
}

// --- RADIO_CONTROL_CHANNELS ---

mavlink_message_t pack_radio_control_channels(const Identity& from, const RadioControlChannels& channels)
{
  constexpr uint16_t kNoData = UINT16_MAX;  // канал не читаємо -> «немає даних»
  constexpr uint8_t kChannelCount = 8;
  constexpr uint8_t kRssiUnknown = UINT8_MAX;

  mavlink_message_t msg{};
  mavlink_msg_rc_channels_pack(from.sysid,
                               from.compid,
                               &msg,
                               channels.time_boot_ms,
                               kChannelCount,
                               channels.roll,
                               kNoData,
                               channels.throttle,
                               kNoData,
                               kNoData,
                               kNoData,
                               kNoData,
                               kNoData,
                               kNoData,
                               kNoData,
                               kNoData,
                               kNoData,
                               kNoData,
                               kNoData,
                               kNoData,
                               kNoData,
                               kNoData,
                               kNoData,
                               kRssiUnknown);
  return msg;
}

RadioControlChannels parse_radio_control_channels(const mavlink_message_t& msg)
{
  mavlink_rc_channels_t raw{};
  mavlink_msg_rc_channels_decode(&msg, &raw);
  return {.time_boot_ms = raw.time_boot_ms, .roll = raw.chan1_raw, .throttle = raw.chan3_raw};
}

bool are_channels_in_deadband(const RadioControlChannels& channels)
{
  return is_in_deadband(channels.roll) && is_in_deadband(channels.throttle);
}

// --- RADIO_CONTROL_CHANNELS_OVERRIDE ---

mavlink_message_t pack_radio_control_channels_override(const Identity& from, const Identity& target, const ControlSignal& control)
{
  const RadioControlOverride radio_control_override = to_radio_control_override(control);

  mavlink_message_t msg{};
  mavlink_msg_rc_channels_override_pack(from.sysid,
                                        from.compid,
                                        &msg,
                                        target.sysid,
                                        target.compid,
                                        radio_control_override.roll,
                                        kPwmReleased,
                                        radio_control_override.throttle,
                                        kPwmReleased,
                                        kPwmReleased,
                                        kPwmReleased,
                                        kPwmReleased,
                                        kPwmReleased,
                                        kPwmReleased,
                                        kPwmReleased,
                                        kPwmReleased,
                                        kPwmReleased,
                                        kPwmReleased,
                                        kPwmReleased,
                                        kPwmReleased,
                                        kPwmReleased,
                                        kPwmReleased,
                                        kPwmReleased);
  return msg;
}

RadioControlOverride parse_radio_control_channels_override(const mavlink_message_t& msg)
{
  mavlink_rc_channels_override_t raw{};
  mavlink_msg_rc_channels_override_decode(&msg, &raw);
  return {.roll = raw.chan1_raw, .throttle = raw.chan3_raw};
}

RadioControlOverride to_radio_control_override(const ControlSignal& control)
{
  return {
    .roll = normalized_to_pwm(control.turnRate),
    .throttle = normalized_to_pwm(control.accel),
  };
}

ControlSignal to_control_signal(const RadioControlOverride& radio_control_override)
{
  return {
    .accel = pwm_to_normalized(radio_control_override.throttle),
    .turnRate = pwm_to_normalized(radio_control_override.roll),
  };
}

mavlink_message_t pack_enable_command(const Identity& from, const Identity& target, const EnableCommand& command)
{
  mavlink_message_t msg{};
  mavlink_msg_command_long_pack(from.sysid,
                                from.compid,
                                &msg,
                                target.sysid,
                                target.compid,
                                kEnableCommandId,
                                0,
                                command.enabled ? 1.0f : 0.0f,
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f);
  return msg;
}

EnableCommand parse_enable_command(const mavlink_message_t& msg)
{
  mavlink_command_long_t raw{};
  mavlink_msg_command_long_decode(&msg, &raw);
  return {.enabled = raw.param1 != 0.0f};
}

constexpr double kLocalPositionScale = 1e4;

mavlink_message_t pack_target_designation(const Identity& from, const Identity& target, const TargetDesignation& designation)
{
  constexpr uint8_t kUnused = 0;

  mavlink_message_t msg{};
  mavlink_msg_command_int_pack(from.sysid,
                               from.compid,
                               &msg,
                               target.sysid,
                               target.compid,
                               MAV_FRAME_LOCAL_NED,
                               kTargetDesignationCommandId,
                               kUnused,
                               kUnused,
                               static_cast<float>(designation.target_id),
                               static_cast<float>(designation.target_count),
                               0.0f,
                               0.0f,
                               static_cast<int32_t>(std::llround(designation.latitude * kLocalPositionScale)),
                               static_cast<int32_t>(std::llround(designation.longitude * kLocalPositionScale)),
                               0.0f);
  return msg;
}

TargetDesignation parse_target_designation(const mavlink_message_t& msg)
{
  mavlink_command_int_t raw{};
  mavlink_msg_command_int_decode(&msg, &raw);
  return {
    .target_id = static_cast<uint8_t>(raw.param1),
    .target_count = static_cast<uint8_t>(raw.param2),
    .latitude = static_cast<double>(raw.x) / kLocalPositionScale,
    .longitude = static_cast<double>(raw.y) / kLocalPositionScale,
  };
}

mavlink_message_t pack_drop_notification(const Identity& from, const Identity& target, const DropNotification& drop)
{
  mavlink_message_t msg{};
  mavlink_msg_command_long_pack(from.sysid,
                                from.compid,
                                &msg,
                                target.sysid,
                                target.compid,
                                kDropNotificationCommandId,
                                0,
                                static_cast<float>(drop.target_id),
                                drop.bomb_flight_time_sec,
                                0.0f,
                                0.0f,
                                static_cast<float>(drop.latitude),
                                static_cast<float>(drop.longitude),
                                drop.altitude);
  return msg;
}

DropNotification parse_drop_notification(const mavlink_message_t& msg)
{
  mavlink_command_long_t raw{};
  mavlink_msg_command_long_decode(&msg, &raw);

  return {
    .latitude = static_cast<double>(raw.param5),
    .longitude = static_cast<double>(raw.param6),
    .altitude = raw.param7,
    .target_id = static_cast<uint8_t>(raw.param1),
    .bomb_flight_time_sec = raw.param2,
  };
}

mavlink_message_t pack_mission_complete_notification(const Identity& from,
                                                     const Identity& target,
                                                     const MissionCompleteNotification& notification)
{
  mavlink_message_t msg{};
  mavlink_msg_command_long_pack(from.sysid,
                                from.compid,
                                &msg,
                                target.sysid,
                                target.compid,
                                kMissionCompleteCommandId,
                                0,
                                notification.completed ? 1.0f : 0.0f,
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f);
  return msg;
}

MissionCompleteNotification parse_mission_complete_notification(const mavlink_message_t& msg)
{
  mavlink_command_long_t raw{};
  mavlink_msg_command_long_decode(&msg, &raw);
  return {.completed = raw.param1 != 0.0f};
}

mavlink_message_t pack_manual_drop_command(const Identity& from, const Identity& target, const ManualDropCommand& /* command */)
{
  mavlink_message_t msg{};
  mavlink_msg_command_long_pack(
    from.sysid, from.compid, &msg, target.sysid, target.compid, kManualDropCommandId, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  return msg;
}

ManualDropCommand parse_manual_drop_command(const mavlink_message_t& /* msg */)
{
  return {.makeDrop = true};
}

mavlink_message_t pack_reboot_command(const Identity& from, const Identity& target)
{
  constexpr float kRebootDrone = 1.0f;

  mavlink_message_t msg{};
  mavlink_msg_command_long_pack(
    from.sysid, from.compid, &msg, target.sysid, target.compid, kRebootCommandId, 0, kRebootDrone, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  return msg;
}

mavlink_message_t pack_command_acknowledgement(const Identity& from, const Identity& target, const CommandAcknowledgement& acknowledgement)
{
  constexpr uint8_t kProgressUnused = 0;
  constexpr int32_t kResultParam2Unused = 0;

  mavlink_message_t msg{};
  mavlink_msg_command_ack_pack(from.sysid,
                               from.compid,
                               &msg,
                               acknowledgement.command,
                               acknowledgement.result,
                               kProgressUnused,
                               kResultParam2Unused,
                               target.sysid,
                               target.compid);
  return msg;
}

CommandAcknowledgement parse_command_acknowledgement(const mavlink_message_t& msg)
{
  mavlink_command_ack_t raw{};
  mavlink_msg_command_ack_decode(&msg, &raw);
  return {.command = raw.command, .result = raw.result};
}

mavlink_message_t pack_status_text(const Identity& from, const StatusText& status)
{
  std::array<char, kStatusTextMaxLength> buffer{};
  const std::size_t length = std::min(status.text.size(), buffer.size());
  std::copy_n(status.text.begin(), length, buffer.begin());

  constexpr uint16_t kSingleChunk = 0;
  constexpr uint8_t kFirstChunk = 0;

  mavlink_message_t msg{};
  mavlink_msg_statustext_pack(from.sysid, from.compid, &msg, status.severity, buffer.data(), kSingleChunk, kFirstChunk);
  return msg;
}

StatusText parse_status_text(const mavlink_message_t& msg)
{
  mavlink_statustext_t raw{};
  mavlink_msg_statustext_decode(&msg, &raw);

  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
  return {.severity = raw.severity, .text = std::string(raw.text, strnlen(raw.text, kStatusTextMaxLength))};
}

// --- DEBUG_FLOAT_ARRAY (для SimStep) ---
namespace {

constexpr std::string_view kSimStepName = "simstep";

// Порядок як в enum DroneState, по дроту прилітає індекс
constexpr std::array<std::string_view, 6> kStateNames{
  "Stopped",
  "Turning",
  "Accelerating",
  "Moving",
  "Decelerating",
  "WaitingForConnection",
};
constexpr float kUnknownState = -1.0f;

// Розкладка полів SimStep у data[] у DEBUG_FLOAT_ARRAY
enum SimStepField : std::size_t {
  kStepIdx,
  kTimeSec,
  kPosX,
  kPosY,
  kDirection,
  kState,
  kTargetIdx,
  kDropX,
  kDropY,
  kAimX,
  kAimY,
  kPredictedX,
  kPredictedY,
  kFieldCount,
};

float to_state_code(const std::string_view name)
{
  const auto* found = std::ranges::find(kStateNames, name);

  return found == kStateNames.end() ? kUnknownState : static_cast<float>(found - kStateNames.begin());
}

std::string to_state_name(const float code)
{
  const int index = static_cast<int>(code);
  if (index < 0 || index >= static_cast<int>(kStateNames.size())) {
    return {};
  }
  return std::string(kStateNames[static_cast<std::size_t>(index)]);
}

}  // namespace

mavlink_message_t pack_sim_step(const Identity& from, const SimStep& step)
{
  std::array<float, MAVLINK_MSG_DEBUG_FLOAT_ARRAY_FIELD_DATA_LEN> data{};

  data[SimStepField::kStepIdx] = static_cast<float>(step.step);
  data[SimStepField::kTimeSec] = step.timeSecSinceStart;
  data[SimStepField::kPosX] = step.pos.x;
  data[SimStepField::kPosY] = step.pos.y;
  data[SimStepField::kDirection] = step.direction;
  data[SimStepField::kState] = to_state_code(step.state);
  data[SimStepField::kTargetIdx] = static_cast<float>(step.targetIdx);
  data[SimStepField::kDropX] = step.dropPoint.x;
  data[SimStepField::kDropY] = step.dropPoint.y;
  data[SimStepField::kAimX] = step.aimPoint.x;
  data[SimStepField::kAimY] = step.aimPoint.y;
  data[SimStepField::kPredictedX] = step.predictedTarget.x;
  data[SimStepField::kPredictedY] = step.predictedTarget.y;

  std::array<char, MAVLINK_MSG_DEBUG_FLOAT_ARRAY_FIELD_NAME_LEN> name{};
  std::ranges::copy(kSimStepName, name.begin());

  constexpr uint16_t kArrayIdUnused = 0;

  mavlink_message_t msg{};
  mavlink_msg_debug_float_array_pack(from.sysid,
                                     from.compid,
                                     &msg,
                                     static_cast<uint64_t>(std::llround(step.timeSecSinceStart * 1e6f)),
                                     name.data(),
                                     kArrayIdUnused,
                                     data.data());
  return msg;
}

std::optional<SimStep> parse_sim_step(const mavlink_message_t& msg)
{
  mavlink_debug_float_array_t raw{};
  mavlink_msg_debug_float_array_decode(&msg, &raw);

  // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
  const bool isSimStepReceived = std::string_view(raw.name, strnlen(raw.name, sizeof(raw.name))) == kSimStepName;

  if (!isSimStepReceived) {
    return std::nullopt;
  }

  return SimStep{
    .pos = {.x = raw.data[SimStepField::kPosX], .y = raw.data[SimStepField::kPosY]},
    .dropPoint = {.x = raw.data[SimStepField::kDropX], .y = raw.data[SimStepField::kDropY]},
    .aimPoint = {.x = raw.data[SimStepField::kAimX], .y = raw.data[SimStepField::kAimY]},
    .predictedTarget = {.x = raw.data[SimStepField::kPredictedX], .y = raw.data[SimStepField::kPredictedY]},
    .direction = raw.data[SimStepField::kDirection],
    .state = to_state_name(raw.data[SimStepField::kState]),
    .targetIdx = static_cast<int>(raw.data[SimStepField::kTargetIdx]),
    .step = static_cast<int>(raw.data[SimStepField::kStepIdx]),
    .timeSecSinceStart = raw.data[SimStepField::kTimeSec],
  };
}

}  // namespace mav
