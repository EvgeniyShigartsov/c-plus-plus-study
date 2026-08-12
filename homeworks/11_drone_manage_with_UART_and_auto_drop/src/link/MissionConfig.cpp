#include "link/MissionConfig.hpp"

#include <unistd.h>
#include <cstring>

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays)
MissionConfigPackets collectMissionConfig(UartLink& link, dlink::Parser& parser)
{
  MissionConfigPackets packets;

  uint8_t buf[256];
  uint8_t type = 0;
  uint8_t len = 0;
  uint8_t payload[260];

  bool haveAmmo = false;
  bool haveCfg = false;
  bool haveTelmetry = false;

  std::vector<bool> targetSeen;
  size_t targetsSeen = 0;

  while (true) {
    const int bytes = link.readBytes(buf, sizeof(buf));

    if (bytes <= 0) {
      usleep(1000);
      continue;
    }

    for (int i = 0; i < bytes; i++) {
      if (!parser.feed(buf[i], type, payload, len)) {
        continue;
      }

      if (type == dlink::PKT_AMMO) {
        std::memcpy(&packets.ammo, payload, sizeof packets.ammo);
        haveAmmo = true;
        packets.initialTargets.resize(packets.ammo.nTargets);
        targetSeen.resize(packets.ammo.nTargets, false);
      }
      else if (type == dlink::PKT_CONFIG) {
        std::memcpy(&packets.droneCfg, payload, sizeof packets.droneCfg);
        haveCfg = true;
      }
      else if (type == dlink::PKT_TELEMETRY) {
        std::memcpy(&packets.firstTelemetry, payload, sizeof packets.firstTelemetry);
        haveTelmetry = true;
      }
      else if (type == dlink::PKT_TARGET && haveAmmo) {
        dlink::TargetPos targetPosition{};
        std::memcpy(&targetPosition, payload, sizeof targetPosition);

        if (targetPosition.id < packets.initialTargets.size() && !targetSeen[targetPosition.id]) {
          packets.initialTargets[targetPosition.id] = {targetPosition.x, targetPosition.y};
          targetSeen[targetPosition.id] = true;
          targetsSeen++;
        }
      }
    }

    if (haveAmmo && haveCfg && haveTelmetry && targetsSeen == packets.initialTargets.size()) {
      return packets;
    }
  }
}
// NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay,cppcoreguidelines-avoid-c-arrays)
