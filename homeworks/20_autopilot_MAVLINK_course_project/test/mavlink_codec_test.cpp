#include <gtest/gtest.h>

#include "mavlink/Codec.hpp"

TEST(MavlinkCodec, HeartbeatRoundTrip)
{
  const mav::Heartbeat original{
    .from = mav::kAutopilot,
    .type = MAV_TYPE_ONBOARD_CONTROLLER,
    .autopilot = MAV_AUTOPILOT_INVALID,
    .base_mode = 0,
    .custom_mode = 3,
    .system_status = MAV_STATE_ACTIVE,
  };

  const mavlink_message_t msg = mav::pack_heartbeat(original);
  const mav::Heartbeat parsed = mav::parse_heartbeat(msg);

  EXPECT_EQ(msg.msgid, MAVLINK_MSG_ID_HEARTBEAT);
  EXPECT_EQ(parsed, original);
}

TEST(MavlinkCodec, HeartbeatCarriesSender)
{
  const mav::Heartbeat gcs{.from = mav::kGcs, .type = MAV_TYPE_GCS};

  const mav::Heartbeat parsed = mav::parse_heartbeat(mav::pack_heartbeat(gcs));

  EXPECT_EQ(parsed.from, mav::kGcs);
}
