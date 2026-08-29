#include "link/MavlinkDrop.hpp"

#include <common/mavlink.h>
#include <cstdint>

#include "Logger.hpp"
#include "link/MavlinkUtil.hpp"
#include "link/UdpLink.hpp"

namespace {
constexpr uint8_t TARGET_SYSTEM = 1;
constexpr uint8_t TARGET_COMPONENT = MAV_COMP_ID_AUTOPILOT1;

constexpr int MAX_ATTEMPTS = 5;
constexpr std::chrono::seconds RESEND_TIMEOUT = std::chrono::seconds(1);
}  // namespace

MavlinkDrop::MavlinkDrop(const UdpLink& udp)
  : udp(udp)
{
}

void MavlinkDrop::begin(double lat, double lon, float altitude)
{
  dropLat = lat;
  dropLon = lon;
  dropAltitude = altitude;
  attempts = 1;
  sendCommand();
  LOG("MAVLink: drop command sent (lat=" << lat << " lon=" << lon << " alt=" << altitude << ")");
}

void MavlinkDrop::sendCommand()
{
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(mavlinkUtil::SYSID,
                                mavlinkUtil::COMPID,
                                &msg,
                                TARGET_SYSTEM,
                                TARGET_COMPONENT,
                                MAV_CMD_USER_1,
                                static_cast<uint8_t>(attempts - 1),  // confirmation: 0 перша, далі 1..4
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f,
                                static_cast<float>(dropLat),
                                static_cast<float>(dropLon),
                                dropAltitude);
  mavlinkUtil::send(udp, msg);
  lastSend = std::chrono::steady_clock::now();
}

void MavlinkDrop::poll()
{
  if (isFinished || attempts == 0 || (std::chrono::steady_clock::now() - lastSend < RESEND_TIMEOUT)) {
    return;
  }

  if (attempts >= MAX_ATTEMPTS) {
    isFinished = true;
    LOG("MAVLink: no drop ACK after " << MAX_ATTEMPTS << " attempts");
    return;
  }
  attempts++;
  sendCommand();
  LOG("MAVLink: drop command resent (attempt " << attempts << ")");
}

void MavlinkDrop::feed(const uint8_t* buf, const int len)
{
  if (isFinished) {
    return;
  }

  mavlink_message_t msg;
  mavlink_status_t status;

  for (int i = 0; i < len; i++) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    const uint8_t frameReady = mavlink_parse_char(MAVLINK_COMM_1, buf[i], &msg, &status);
    if (frameReady == 0) {
      continue;  // збірку кадру не завершено
    }
    if (msg.msgid != MAVLINK_MSG_ID_COMMAND_ACK) {
      continue;
    }

    mavlink_command_ack_t ack;
    mavlink_msg_command_ack_decode(&msg, &ack);

    if (ack.command == MAV_CMD_USER_1 && ack.result == MAV_RESULT_ACCEPTED) {
      isFinished = true;
      isAcked = true;
      LOG("MAVLink: drop ACK received (attempt " << attempts << ")");
    }
  }
}

bool MavlinkDrop::started() const
{
  return attempts > 0;
}

bool MavlinkDrop::finished() const
{
  return isFinished;
}

bool MavlinkDrop::acked() const
{
  return isAcked;
}
