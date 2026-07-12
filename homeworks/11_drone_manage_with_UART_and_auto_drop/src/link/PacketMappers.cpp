#include "link/PacketMappers.hpp"

DroneTelemetry toDroneTelemetry(const dlink::Telemetry& telemetry)
{
  return {
    .pos = {telemetry.x, telemetry.y},
    .speed = telemetry.speed,
    .dir = telemetry.dir,
    .timeSinceStart = static_cast<float>(telemetry.t_ms) / 1000.0f,
  };
}

BombParams toBombParams(const dlink::AmmoCfg& ammo)
{
  return {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
    .name = ammo.name,
    .mass = ammo.mass,
    .drag = ammo.drag,
    .lift = ammo.lift,
  };
}

DroneConfig buildDroneConfig(const dlink::AmmoCfg& ammo, const dlink::DroneCfg& droneConfig, const dlink::Telemetry& telemetry)
{
  return {
    .startPos = {telemetry.x, telemetry.y},
    .altitude = telemetry.z,
    .initialDir = telemetry.dir,
    .v0 = droneConfig.attackSpeed,
    .accelerationPath = droneConfig.accelerationPath,
    .simTimeStep = droneConfig.timeStep,
    .hitRadius = ammo.hitRadius,
    .angularSpeed = droneConfig.angularSpeed,
    .turnThreshold = droneConfig.turnThreshold,
  };
}
