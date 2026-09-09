#pragma once
#include "third_party/drone_link.h"
#include "types.hpp"

DroneTelemetry toDroneTelemetry(const dlink::Telemetry& telemetry);

BombParams toBombParams(const dlink::AmmoCfg& ammo);

DroneConfig buildDroneConfig(const dlink::AmmoCfg& ammo, const dlink::DroneCfg& droneConfig, const dlink::Telemetry& telemetry);
