#pragma once
#include <cstdint>

namespace mav {

// Хто говорить на дроті, sysid — апарат, compid — конкретний вузол усередині
struct Identity {
  uint8_t sysid = 0;
  uint8_t compid = 0;

  bool operator==(const Identity&) const = default;
};

// MAV_COMP_ID_AUTOPILOT1 — симульований борт
inline constexpr Identity kVehicle{.sysid = 1, .compid = 1};

// MAV_COMP_ID_ONBOARD_COMPUTER — модуль автопілота
inline constexpr Identity kAutopilot{.sysid = 1, .compid = 191};

// MAV_COMP_ID_MISSIONPLANNER — оператор / GCS (Ground Control Station)
inline constexpr Identity kGcs{.sysid = 255, .compid = 190};

}  // namespace mav
