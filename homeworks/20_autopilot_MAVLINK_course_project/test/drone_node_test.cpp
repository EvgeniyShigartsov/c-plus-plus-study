#include <gtest/gtest.h>

#include <ranges>
#include <vector>

#include "drone/DroneNode.hpp"
#include "interfaces/IDroneOutput.hpp"
#include "mavlink/Codec.hpp"
#include "mavlink/Identity.hpp"

namespace {

constexpr float kPhysicsTimeStep = 0.01f;
constexpr float kTimeScale = 1.0f;
constexpr float kUpdateSec = 0.1f;  // = simTimeStep, тож кожен update дає рівно одну телеметрію

DroneConfig makeConfig()
{
  return {.startPos = {.x = 100.0f, .y = 100.0f},
          .altitude = 120.0f,
          .initialDir = 0.0f,
          .v0 = 15.0f,
          .accelerationPath = 20.0f,
          .simTimeStep = 0.1f,
          .hitRadius = 3.0f,
          .angularSpeed = 0.8f,
          .turnThreshold = 0.1f};
}

class RecordingOutput : public IDroneOutput {
public:
  std::vector<mavlink_message_t> sent;
  std::vector<Coord> drops;
  int missionStartedCount = 0;
  int missionCompleteCount = 0;

  void send(const mavlink_message_t& msg) override { sent.push_back(msg); }

  void onDrop(const Coord& aimPoint, const float /* timeSinceStart */) override { drops.push_back(aimPoint); }

  void onMissionStarted() override { missionStartedCount++; }

  void onMissionComplete() override { missionCompleteCount++; }

  int countByMsgId(const uint32_t msgid) const
  {
    int n = 0;
    for (const mavlink_message_t& msg : sent) {
      n += msg.msgid == msgid ? 1 : 0;
    }
    return n;
  }

  [[nodiscard]] mav::LocalPositionNed lastPosition() const
  {
    for (const mavlink_message_t& msg : std::views::reverse(sent)) {
      if (msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
        return mav::parse_local_position_ned(msg);
      }
    }
    return {};
  }

  [[nodiscard]] mav::Attitude lastAttitude() const
  {
    for (const mavlink_message_t& msg : std::views::reverse(sent)) {
      if (msg.msgid == MAVLINK_MSG_ID_ATTITUDE) {
        return mav::parse_attitude(msg);
      }
    }
    return {};
  }
};

void runFor(DroneNode& node, const int updates)
{
  for (int i = 0; i < updates; i++) {
    node.update(kUpdateSec);
  }
}

}  // namespace

TEST(DroneNodeTest, FirstUpdateSendsHeartbeatAndAllTelemetry)
{
  RecordingOutput output;
  DroneNode node(makeConfig(), kPhysicsTimeStep, kTimeScale, output);

  node.update(0.0f);

  EXPECT_EQ(output.countByMsgId(MAVLINK_MSG_ID_HEARTBEAT), 1);
  EXPECT_EQ(output.countByMsgId(MAVLINK_MSG_ID_LOCAL_POSITION_NED), 1);
  EXPECT_EQ(output.countByMsgId(MAVLINK_MSG_ID_ATTITUDE), 1);
  EXPECT_EQ(output.countByMsgId(MAVLINK_MSG_ID_GLOBAL_POSITION_INT), 1);
  EXPECT_EQ(output.countByMsgId(MAVLINK_MSG_ID_RC_CHANNELS), 1);
}

TEST(DroneNodeTest, HeartbeatIsSentOncePerRealSecond)
{
  RecordingOutput output;
  DroneNode node(makeConfig(), kPhysicsTimeStep, kTimeScale, output);

  runFor(node, 25);  // 2.5 секунди реального часу, heartbeat на старті кожної секунди
  EXPECT_EQ(output.countByMsgId(MAVLINK_MSG_ID_HEARTBEAT), 3);
}

TEST(DroneNodeTest, AutopilotOverrideAcceleratesDrone)
{
  RecordingOutput output;
  DroneNode node(makeConfig(), kPhysicsTimeStep, kTimeScale, output);

  node.onMessage(mav::pack_radio_control_channels_override(mav::kAutopilot, mav::kVehicle, {.accel = 1.0f, .turnRate = 0.0f}));
  runFor(node, 20);

  const mav::LocalPositionNed position = output.lastPosition();
  EXPECT_GT(position.north + position.east, 100.0f);
  EXPECT_GT(position.velocity_north * position.velocity_north + position.velocity_east * position.velocity_east, 0.0f);
}

TEST(DroneNodeTest, OperatorSticksOverrideAutopilot)
{
  RecordingOutput output;
  DroneNode node(makeConfig(), kPhysicsTimeStep, kTimeScale, output);

  // Автопілот тисне на газ, але оператор повертає стіком - слухається оператор
  node.onMessage(mav::pack_radio_control_channels_override(mav::kAutopilot, mav::kVehicle, {.accel = 1.0f, .turnRate = 0.0f}));
  node.onMessage(mav::pack_radio_control_channels_override(mav::kGcs, mav::kVehicle, {.accel = 0.0f, .turnRate = 1.0f}));
  runFor(node, 20);

  const mav::LocalPositionNed position = output.lastPosition();
  EXPECT_FLOAT_EQ(position.velocity_north, 0.0f);  // газу від автопілота не було
  EXPECT_FLOAT_EQ(position.velocity_east, 0.0f);
  EXPECT_NE(output.lastAttitude().yaw, 0.0f);  // а курс змінився від стіка оператора
}

TEST(DroneNodeTest, OperatorInDeadbandDoesNotBlockAutopilot)
{
  RecordingOutput output;
  DroneNode node(makeConfig(), kPhysicsTimeStep, kTimeScale, output);

  node.onMessage(mav::pack_radio_control_channels_override(mav::kAutopilot, mav::kVehicle, {.accel = 1.0f, .turnRate = 0.0f}));
  node.onMessage(mav::pack_radio_control_channels_override(mav::kGcs, mav::kVehicle, {.accel = 0.0f, .turnRate = 0.0f}));
  runFor(node, 20);

  const mav::LocalPositionNed position = output.lastPosition();
  EXPECT_GT(position.velocity_north * position.velocity_north + position.velocity_east * position.velocity_east, 0.0f);
}

TEST(DroneNodeTest, DropIsReportedOnlyOnce)
{
  RecordingOutput output;
  DroneNode node(makeConfig(), kPhysicsTimeStep, kTimeScale, output);

  const mavlink_message_t drop =
    mav::pack_drop_notification(mav::kAutopilot, mav::kVehicle, {.latitude = 235.0, .longitude = 250.0, .target_id = 2});
  node.onMessage(drop);
  node.onMessage(drop);

  ASSERT_EQ(output.drops.size(), 1u);
  EXPECT_FLOAT_EQ(output.drops[0].x, 235.0f);
  EXPECT_FLOAT_EQ(output.drops[0].y, 250.0f);
}

TEST(DroneNodeTest, MissionClockStandsStillUntilFirstMessage)
{
  RecordingOutput output;
  DroneNode node(makeConfig(), kPhysicsTimeStep, kTimeScale, output);

  runFor(node, 20);  // 2 секунди реального часу без жодного повідомлення

  EXPECT_FALSE(node.isMissionStarted());
  EXPECT_EQ(output.missionStartedCount, 0);
  EXPECT_EQ(output.lastPosition().time_boot_ms, 0U);
  EXPECT_GE(output.countByMsgId(MAVLINK_MSG_ID_HEARTBEAT), 1);  // heartbeat іде й до початку місії

  node.onMessage(mav::pack_radio_control_channels_override(mav::kGcs, mav::kVehicle, {.accel = 0.0f, .turnRate = 0.0f}));
  runFor(node, 20);

  node.onMessage(mav::pack_radio_control_channels_override(mav::kGcs, mav::kVehicle, {.accel = 0.0f, .turnRate = 0.0f}));

  EXPECT_TRUE(node.isMissionStarted());
  EXPECT_EQ(output.missionStartedCount, 1);  // повідомляється один раз, попри кілька повідомлень
  EXPECT_GE(output.lastPosition().time_boot_ms, 1500U);  // час місії пішов з нуля саме від першого повідомлення
}

TEST(DroneNodeTest, RebootCommandDoesNotStartMission)
{
  RecordingOutput output;
  DroneNode node(makeConfig(), kPhysicsTimeStep, kTimeScale, output);

  node.onMessage(mav::pack_reboot_command(mav::kAutopilot, mav::kVehicle));
  runFor(node, 20);

  EXPECT_FALSE(node.isMissionStarted());
  EXPECT_EQ(output.missionStartedCount, 0);
  EXPECT_EQ(output.lastPosition().time_boot_ms, 0U);
}

TEST(DroneNodeTest, RebootCommandRequestsReboot)
{
  RecordingOutput output;
  DroneNode node(makeConfig(), kPhysicsTimeStep, kTimeScale, output);

  EXPECT_FALSE(node.isRebootRequested());

  node.onMessage(mav::pack_reboot_command(mav::kAutopilot, mav::kVehicle));

  EXPECT_TRUE(node.isRebootRequested());
  EXPECT_FALSE(node.isMissionComplete());
}

TEST(DroneNodeTest, MissionCompleteNotificationFinishesMission)
{
  RecordingOutput output;
  DroneNode node(makeConfig(), kPhysicsTimeStep, kTimeScale, output);

  EXPECT_FALSE(node.isMissionComplete());

  node.onMessage(mav::pack_mission_complete_notification(mav::kAutopilot, mav::kVehicle, {.completed = true}));

  EXPECT_TRUE(node.isMissionComplete());
  EXPECT_EQ(output.missionCompleteCount, 1);
}
