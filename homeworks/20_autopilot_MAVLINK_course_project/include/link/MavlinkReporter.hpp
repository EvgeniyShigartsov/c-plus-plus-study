#pragma once
#include <chrono>
#include "types.hpp"

class UdpLink;

class MavlinkReporter {
public:
  explicit MavlinkReporter(const UdpLink& udp);

  void sendHeartbeatIfDue();

  void sendTelemetry(const VehicleState& state) const;

  // Локальні метри від старту -> градуси (опорна точка lat0/lon0 у .cpp).
  static void toGps(float x, float y, double& lat, double& lon);

private:
  // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
  const UdpLink& udp;
  std::chrono::steady_clock::time_point lastHeartbeat;
};
