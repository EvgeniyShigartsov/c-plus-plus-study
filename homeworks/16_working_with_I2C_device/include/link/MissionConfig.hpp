#pragma once
#include <vector>
#include "link/UartLink.hpp"
#include "third_party/drone_link.h"
#include "types.hpp"

// Все, що чекер надсилає на старті місії, параметри боєприпасу і дрона,
// перша телеметрія та початкові позиції всіх цілей.
struct MissionConfigPackets {
  dlink::AmmoCfg ammo{};
  dlink::DroneCfg droneCfg{};
  dlink::Telemetry firstTelemetry{};
  std::vector<Coord> initialTargets;
};

MissionConfigPackets collectMissionConfig(UartLink& link, dlink::Parser& parser);
