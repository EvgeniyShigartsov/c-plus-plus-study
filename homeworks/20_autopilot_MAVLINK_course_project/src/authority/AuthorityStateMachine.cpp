#include "authority/AuthorityStateMachine.hpp"
#include <string>

AuthorityState AuthorityStateMachine::state() const
{
  return current;
}

bool AuthorityStateMachine::update(const AuthorityInput& inputs)
{
  const AuthorityState next = computeNextState(inputs);
  const bool hasChanged = next != current;
  current = next;
  return hasChanged;
}

bool AuthorityStateMachine::hasControl() const
{
  return current == AuthorityState::Engaged || current == AuthorityState::Failsafe;
}

std::string AuthorityStateMachine::to_string() const
{
  switch (current) {
    case AuthorityState::Standby:
      return "Standby";
    case AuthorityState::Armed:
      return "Armed";
    case AuthorityState::Engaged:
      return "Engaged";
    case AuthorityState::Yielding:
      return "Yielding";
    case AuthorityState::Failsafe:
      return "Failsafe";
    case AuthorityState::Complete:
      return "Complete";
  }
  return "Unknown";
}

AuthorityState AuthorityStateMachine::computeNextState(const AuthorityInput& in) const
{
  switch (current) {
    case AuthorityState::Complete:
      return AuthorityState::Complete;

    case AuthorityState::Standby:
      return in.enabled ? AuthorityState::Armed : AuthorityState::Standby;

    case AuthorityState::Armed:
      if (!in.enabled) {
        return AuthorityState::Standby;
      }
      return in.hasMission ? AuthorityState::Engaged : AuthorityState::Armed;

    case AuthorityState::Engaged:
      if (in.reachedFirePoint) {
        return AuthorityState::Complete;
      }
      if (!in.operatorLinkOk) {
        return AuthorityState::Failsafe;
      }
      if (!in.enabled) {
        return AuthorityState::Standby;
      }
      if (!in.hasMission) {
        return AuthorityState::Armed;
      }
      if (!in.operatorInDeadband) {
        return AuthorityState::Yielding;
      }
      return AuthorityState::Engaged;

    case AuthorityState::Yielding:
      if (!in.operatorLinkOk) {
        return AuthorityState::Failsafe;
      }
      if (!in.enabled) {
        return AuthorityState::Standby;
      }
      if (!in.hasMission) {
        return AuthorityState::Armed;
      }
      if (in.operatorInDeadband) {
        return AuthorityState::Engaged;
      }
      return AuthorityState::Yielding;

    case AuthorityState::Failsafe:
      if (in.reachedFirePoint) {
        return AuthorityState::Complete;
      }
      if (in.operatorLinkOk) {
        return AuthorityState::Engaged;
      }
      return AuthorityState::Failsafe;
  }

  return current;
}
