#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <deque>
#include <vector>

#include "bridge/SerialBridge.hpp"
#include "link/ILink.hpp"
#include "mavlink/Codec.hpp"
#include "mavlink/Identity.hpp"

namespace {

class FakeLink : public ILink {
public:
  mutable std::deque<uint8_t> inbox;
  mutable std::vector<uint8_t> sent;

  [[nodiscard]] bool isOpen() const override { return true; }

  void sendFrame(const uint8_t* buf, const size_t len) const override
  {
    sent.insert(sent.end(), buf, buf + len);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
  }

  ssize_t receive(uint8_t* buf, const size_t capacity) const override
  {
    if (inbox.empty()) {
      return -1;
    }

    const size_t count = std::min(capacity, inbox.size());
    std::copy_n(inbox.begin(), count, buf);
    inbox.erase(inbox.begin(), inbox.begin() + static_cast<std::ptrdiff_t>(count));
    return static_cast<ssize_t>(count);
  }

  void feed(const std::vector<uint8_t>& bytes) const { inbox.insert(inbox.end(), bytes.begin(), bytes.end()); }
};

std::vector<uint8_t> toBytes(const mavlink_message_t& msg)
{
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer{};
  const uint16_t length = mavlink_msg_to_send_buffer(buffer.data(), &msg);
  return {buffer.begin(), buffer.begin() + length};
}

// Чотири лінки й міст над ними; кожен ендпоінт має власний канал парсера
struct BridgeFixture {
  FakeLink droneLink;
  FakeLink appsLink;
  FakeLink autopilotLink;
  FakeLink gcsLink;

  mav::MavlinkEndpoint drone{droneLink, MAVLINK_COMM_0};
  mav::MavlinkEndpoint fromApps{appsLink, MAVLINK_COMM_1};
  mav::MavlinkEndpoint toAutopilot{autopilotLink, MAVLINK_COMM_2};
  mav::MavlinkEndpoint toGcs{gcsLink, MAVLINK_COMM_3};

  SerialBridge bridge{drone, fromApps, toAutopilot, toGcs};
};

const mavlink_message_t kHeartbeat = mav::pack_heartbeat(mav::kVehicle, {.type = MAV_TYPE_QUADROTOR, .system_status = MAV_STATE_ACTIVE});
const mavlink_message_t kRadioOverride =
  mav::pack_radio_control_channels_override(mav::kAutopilot, mav::kVehicle, {.accel = 1.0f, .turnRate = 0.0f});

}  // namespace

TEST(SerialBridge, ForwardsAppMessagesToDrone)
{
  BridgeFixture f;
  f.appsLink.feed(toBytes(kRadioOverride));

  f.bridge.pump();

  EXPECT_EQ(f.droneLink.sent, toBytes(kRadioOverride));
  EXPECT_TRUE(f.autopilotLink.sent.empty());
  EXPECT_TRUE(f.gcsLink.sent.empty());
}

TEST(SerialBridge, ForwardsDroneMessagesToAutopilotAndGcs)
{
  BridgeFixture f;
  f.droneLink.feed(toBytes(kHeartbeat));

  f.bridge.pump();

  EXPECT_EQ(f.autopilotLink.sent, toBytes(kHeartbeat));
  EXPECT_EQ(f.gcsLink.sent, toBytes(kHeartbeat));
  EXPECT_TRUE(f.droneLink.sent.empty());  // назад до дрона нічого не повертається
}

TEST(SerialBridge, DropsGarbageOnSerialButKeepsTheFrame)
{
  BridgeFixture f;

  // "Сміття" при завантаженні пристрою перед справжнім кадром
  f.droneLink.feed({0x00, 0x13, 0x37, 0xAA, 0x55, 0x0D, 0x0A});
  f.droneLink.feed(toBytes(kHeartbeat));

  f.bridge.pump();

  EXPECT_EQ(f.autopilotLink.sent, toBytes(kHeartbeat));
  EXPECT_EQ(f.gcsLink.sent, toBytes(kHeartbeat));
}

TEST(SerialBridge, ForwardsSeveralMessagesInOrder)
{
  BridgeFixture f;
  f.droneLink.feed(toBytes(kHeartbeat));
  f.droneLink.feed(toBytes(kRadioOverride));

  f.bridge.pump();

  std::vector<uint8_t> expected = toBytes(kHeartbeat);
  const std::vector<uint8_t> second = toBytes(kRadioOverride);
  expected.insert(expected.end(), second.begin(), second.end());
  EXPECT_EQ(f.autopilotLink.sent, expected);
}

TEST(SerialBridge, DoesNothingWhenThereIsNothingToForward)
{
  BridgeFixture f;

  f.bridge.pump();

  EXPECT_TRUE(f.droneLink.sent.empty());
  EXPECT_TRUE(f.autopilotLink.sent.empty());
  EXPECT_TRUE(f.gcsLink.sent.empty());
  EXPECT_FALSE(f.bridge.isMissionComplete());
}

TEST(SerialBridge, MissionCompleteIsForwardedToDroneAndRemembered)
{
  BridgeFixture f;
  const mavlink_message_t complete = mav::pack_mission_complete_notification(mav::kAutopilot, mav::kVehicle, {.completed = true});
  f.appsLink.feed(toBytes(complete));

  f.bridge.pump();

  EXPECT_EQ(f.droneLink.sent, toBytes(complete));
  EXPECT_TRUE(f.bridge.isMissionComplete());
}
