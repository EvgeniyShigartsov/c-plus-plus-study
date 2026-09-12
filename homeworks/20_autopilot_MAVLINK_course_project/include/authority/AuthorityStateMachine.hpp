#pragma once

enum AuthorityState {
  Standby,   // вимкнений, оператор керує сам, або ніхто не керує
  Armed,     // увімкнений, але ще нема цілі або телеметрії, чекаємо
  Engaged,   // увімкнений, є ціль+телеметрія, рахує й шле RC_CHANNELS_OVERRIDE
  Yielding,  // оператор торкнувся джойстиків, автопілот віддає керування
  Failsafe,  // HEARTBEAT оператора пропав під час місії - автопілот сам доводить її до кінця
  Complete  // скид відбувся, місію завершено
};

struct AuthorityInput {
  bool enabled = false;            // EnableCommand (USER_1)
  bool hasMission = false;         // Є ціль + телеметрія
  bool operatorInDeadband = true;  // RC_CHANNELS у нейтралі, тобто оператор не втручається
  bool operatorLinkOk = true;      // HEARTBEAT оператора в порядку
  bool reachedFirePoint = false;   // mission->hasNext() == false
};

// NOLINTNEXTLINE(cppcoreguidelines-special-member-functions)
class AuthorityStateMachine {
public:
  [[nodiscard]] AuthorityState state() const;

  // Оновлює стан за поточними умовами, повертає true, якщо стан щойно змінився.
  bool update(const AuthorityInput& inputs);

  // Чи має зараз автопілот право слати RC_CHANNELS_OVERRIDE
  [[nodiscard]] bool hasControl() const;

private:
  AuthorityState current = AuthorityState::Standby;

  [[nodiscard]] AuthorityState computeNextState(const AuthorityInput& in) const;
};
