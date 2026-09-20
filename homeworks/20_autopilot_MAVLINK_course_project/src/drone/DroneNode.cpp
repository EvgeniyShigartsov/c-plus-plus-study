#include "drone/DroneNode.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "mavlink/Identity.hpp"

namespace {

constexpr float heartbeatPeriodSec = 1.0f;
constexpr float maxCatchUpSec = 1.0f;  // не наздоганяти більше за секунду фізики за один update

VehicleState toVehicleState(const DroneTelemetry& telemetry, const float altitude)
{
  return {
    .mission_time_ms = static_cast<uint32_t>(telemetry.timeSinceStart * 1000.0f),
    .x = telemetry.pos.x,
    .y = telemetry.pos.y,
    .z = altitude,
    .vx = telemetry.speed * std::cos(telemetry.dir),
    .vy = telemetry.speed * std::sin(telemetry.dir),
    .speed = telemetry.speed,
    .dir = telemetry.dir,
  };
}

}  // namespace

DroneNode::DroneNode(const DroneConfig& config, const float physicsTimeStep, const float timeScale, IDroneOutput& output)
  : physics(config)
  , output(output)
  , altitude(config.altitude)
  , simTimeStep(config.simTimeStep)
  , physicsTimeStep(physicsTimeStep)
  , timeScale(timeScale)
  , secondsSinceHeartbeat(heartbeatPeriodSec)
{
}

void DroneNode::onMessage(const mavlink_message_t& msg)
{
  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG && mavlink_msg_command_long_get_command(&msg) == mav::kRebootCommandId) {
    REBOOT_REQUESTED = true;
    return;
  }

  if (!MISSION_STARTED) {
    MISSION_STARTED = true;
    output.onMissionStarted();
  }

  if (msg.msgid == MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE) {
    const mav::RadioControlOverride rc_override = mav::parse_radio_control_channels_override(msg);
    const mav::Identity from{.sysid = msg.sysid, .compid = msg.compid};

    if (from == mav::kGcs) {
      lastOperatorOverride = rc_override;
    }
    else {
      lastAutopilotOverride = rc_override;
    }
  }
  else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG && mavlink_msg_command_long_get_command(&msg) == mav::kDropNotificationCommandId &&
           !dropped) {
    dropped = true;

    const mav::DropNotification drop = mav::parse_drop_notification(msg);
    const Coord aimPoint{.x = static_cast<float>(drop.latitude), .y = static_cast<float>(drop.longitude)};

    output.onDrop(aimPoint, physics.getTelemetry().timeSinceStart);
  }
  else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG && mavlink_msg_command_long_get_command(&msg) == mav::kMissionCompleteCommandId) {
    output.onMissionComplete();
    MISSION_COMPLETE = true;
  }
}

void DroneNode::update(const float realDeltaSec)
{
  const bool operatorActive =
    !mav::are_channels_in_deadband({.roll = lastOperatorOverride.roll, .throttle = lastOperatorOverride.throttle});

  physics.setControl(mav::to_control_signal(operatorActive ? lastOperatorOverride : lastAutopilotOverride));

  // Поки місія не почалась, час місії не йде
  if (MISSION_STARTED) {
    pendingPhysicsTimeSec = std::min(pendingPhysicsTimeSec + realDeltaSec * timeScale, maxCatchUpSec);

    while (pendingPhysicsTimeSec >= physicsTimeStep) {
      physics.stepPhysics(physicsTimeStep);
      pendingPhysicsTimeSec -= physicsTimeStep;
    }
  }

  secondsSinceHeartbeat += realDeltaSec;
  if (secondsSinceHeartbeat >= heartbeatPeriodSec) {
    sendHeartbeat();
    secondsSinceHeartbeat = 0.0f;
  }

  if (physics.getTelemetry().timeSinceStart >= nextTelemetryTime) {
    sendTelemetry();
    nextTelemetryTime += simTimeStep;
  }
}

bool DroneNode::isMissionStarted() const
{
  return MISSION_STARTED;
}

bool DroneNode::isMissionComplete() const
{
  return MISSION_COMPLETE;
}

bool DroneNode::isRebootRequested() const
{
  return REBOOT_REQUESTED;
}

void DroneNode::sendHeartbeat() const
{
  output.send(mav::pack_heartbeat(mav::kVehicle, {.type = MAV_TYPE_QUADROTOR, .system_status = MAV_STATE_ACTIVE}));
}

void DroneNode::sendTelemetry() const
{
  const VehicleState state = toVehicleState(physics.getTelemetry(), altitude);

  output.send(mav::pack_local_position_ned(mav::kVehicle, state));
  output.send(mav::pack_attitude(mav::kVehicle, state));
  output.send(mav::pack_global_position_int(mav::kVehicle, state));
  output.send(mav::pack_radio_control_channels(
    mav::kVehicle, {.time_boot_ms = state.mission_time_ms, .roll = lastOperatorOverride.roll, .throttle = lastOperatorOverride.throttle}));
}
