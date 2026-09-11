#include "mavlink/Codec.hpp"

#include <cmath>
#include <cstdint>
#include <numbers>

#include "mavlink/Frames.hpp"

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

}  // namespace mav
