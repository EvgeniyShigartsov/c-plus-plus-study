#pragma once
#include <common/mavlink.h>

#include "mavlink/Endpoint.hpp"

// Міст між дроном (UART) та рештою системи, "прокидання" даних які прийшли по UDP далі через UART, та навпаки.
class SerialBridge {
public:
  SerialBridge(const mav::MavlinkEndpoint& drone,  // UART порт до дрона
               const mav::MavlinkEndpoint& fromApps,  // UDP порт на який автопілот і оператор шлють повідомлення
               const mav::MavlinkEndpoint& toAutopilot,  // куди пересилати повідомлення від дрона: автопілоту
               const mav::MavlinkEndpoint& toGcs  // куди пересилати повідомлення від дрона: на GCS
  );

  // Один прохід у обидва боки від оператора/автопілота до дрона і навпаки
  void pump();

  // Автопілот повідомив дрону про завершення місії
  [[nodiscard]] bool isMissionComplete() const;

private:
  // NOLINTBEGIN(cppcoreguidelines-avoid-const-or-ref-data-members)
  const mav::MavlinkEndpoint& drone;
  const mav::MavlinkEndpoint& fromApps;
  const mav::MavlinkEndpoint& toAutopilot;
  const mav::MavlinkEndpoint& toGcs;
  // NOLINTEND(cppcoreguidelines-avoid-const-or-ref-data-members)

  bool MISSION_COMPLETE = false;
};
