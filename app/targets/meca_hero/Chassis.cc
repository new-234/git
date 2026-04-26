#include "Chassis.hpp"

bool ChassisHero::EnableJudgement() {
  // 主状态检测
  if (globals->state_machine.getMainState_() == StateMachine::MainState::kOffline) {
    Unabling();
    return false;
  } else if (globals->state_machine.getMainState_() == StateMachine::MainState::kEnableWaiting) {
    Enabling();
    return false;
  }
  // Chassis状态检测
  switch (globals->state_machine.getChassisState_()) {
    default:
      Unabling();
      return false;
    case StateMachine::ChassisState::kOffline:
      Unabling();
      return false;
    case StateMachine::ChassisState::kNoForce:
      Unabling();
      return false;
    case StateMachine::ChassisState::kEnableWaiting:
      Unabling();
      Enabling();
      return false;
    case StateMachine::ChassisState::kNormal:
      return true;
    case StateMachine::ChassisState::kAimbot:
      return true;
    case StateMachine::ChassisState::kRadarAimbot:
      return true;
  }
}

void ChassisHero::ChassisUpdate() {
  if (!EnableJudgement()) return;
}
