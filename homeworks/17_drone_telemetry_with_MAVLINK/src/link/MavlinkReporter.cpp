#include "link/MavlinkReporter.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <common/mavlink.h>
#include "link/MavlinkUtil.hpp"
#include "link/UdpLink.hpp"

namespace {
constexpr double LAT0 = 50.4501;
constexpr double LON0 = 30.5234;
constexpr double METERS_PER_DEG = 111320.0;

constexpr std::chrono::seconds HEARTBEAT_PERIOD = std::chrono::seconds(1);
}  // namespace

MavlinkReporter::MavlinkReporter(const UdpLink& udp)
  : udp(udp)
{
  mavlink_set_proto_version(MAVLINK_COMM_0, 2);
}

void MavlinkReporter::toGps(float x, float y, double& lat, double& lon)
{
  lat = LAT0 + y / METERS_PER_DEG;
  lon = LON0 + x / (METERS_PER_DEG * std::cos(LAT0 * M_PI / 180.0));
}

void MavlinkReporter::sendHeartbeatIfDue()
{
  const std::chrono::time_point now = std::chrono::steady_clock::now();
  if (now - lastHeartbeat < HEARTBEAT_PERIOD) {
    return;
  }
  lastHeartbeat = now;

  mavlink_message_t msg;
  mavlink_msg_heartbeat_pack(mavlinkUtil::SYSID,
                             mavlinkUtil::COMPID,
                             &msg,
                             MAV_TYPE_QUADROTOR,
                             MAV_AUTOPILOT_GENERIC,
                             MAV_MODE_FLAG_SAFETY_ARMED,
                             0,
                             MAV_STATE_ACTIVE);
  mavlinkUtil::send(udp, msg);
}

void MavlinkReporter::sendTelemetry(const dlink::Telemetry& telemetry) const
{
  double lat = 0.0;
  double lon = 0.0;
  toGps(telemetry.x, telemetry.y, lat, lon);

  const float yaw = mavlinkUtil::toNedYaw(telemetry.dir);

  double headingDeg = std::fmod(yaw * 180.0 / M_PI, 360.0);
  if (headingDeg < 0.0) {
    headingDeg += 360.0;
  }
  const uint16_t hdg = static_cast<uint16_t>(std::lround(headingDeg * 100.0));
  const int32_t altMm = static_cast<int32_t>(std::lround(telemetry.z * 1000.0f));

  const int16_t vNorth = static_cast<int16_t>(std::lround(telemetry.vy * 100.0f));
  const int16_t vEast = static_cast<int16_t>(std::lround(telemetry.vx * 100.0f));

  mavlink_message_t position_msg;

  mavlink_msg_global_position_int_pack(mavlinkUtil::SYSID,
                                       mavlinkUtil::COMPID,
                                       &position_msg,
                                       telemetry.t_ms,
                                       static_cast<int32_t>(std::llround(lat * 1e7)),
                                       static_cast<int32_t>(std::llround(lon * 1e7)),
                                       altMm,
                                       altMm,
                                       vNorth,
                                       vEast,
                                       0,
                                       hdg);
  mavlinkUtil::send(udp, position_msg);

  mavlink_message_t attitude_msg;

  mavlink_msg_attitude_pack(mavlinkUtil::SYSID, mavlinkUtil::COMPID, &attitude_msg, telemetry.t_ms, 0.0f, 0.0f, yaw, 0.0f, 0.0f, 0.0f);
  mavlinkUtil::send(udp, attitude_msg);
}
