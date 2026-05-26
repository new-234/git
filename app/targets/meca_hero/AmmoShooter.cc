#include "AmmoShooter.hpp"

void AmmoHero::Unabling() {
  // globals->magazine_motor.SetMitCommand(0, 0, 0, 0, 0);
  globals->magazine_motor.SendInstruction(rm::device::DmMotorInstructions::kDisable);
  if (globals->shooter_motor_6.rpm() < limit) {
    globals->pid_shooter_1.Update(limit, globals->shooter_motor_1.rpm());
    globals->pid_shooter_2.Update(limit, globals->shooter_motor_2.rpm());
    globals->pid_shooter_3.Update(limit, globals->shooter_motor_3.rpm());
    globals->pid_shooter_4.Update(limit, globals->shooter_motor_4.rpm());
    globals->pid_shooter_5.Update(limit, globals->shooter_motor_5.rpm());
    globals->pid_shooter_6.Update(limit, globals->shooter_motor_6.rpm());
    globals->shooter_motor_1.SetCurrent(globals->pid_shooter_1.out());
    globals->shooter_motor_2.SetCurrent(globals->pid_shooter_2.out());
    globals->shooter_motor_3.SetCurrent(globals->pid_shooter_3.out());
    globals->shooter_motor_4.SetCurrent(globals->pid_shooter_4.out());
    globals->shooter_motor_5.SetCurrent(globals->pid_shooter_5.out());
    globals->shooter_motor_6.SetCurrent(globals->pid_shooter_6.out());
  } else {
    globals->shooter_motor_1.SetCurrent(0);
    globals->shooter_motor_2.SetCurrent(0);
    globals->shooter_motor_3.SetCurrent(0);
    globals->shooter_motor_4.SetCurrent(0);
    globals->shooter_motor_5.SetCurrent(0);
    globals->shooter_motor_6.SetCurrent(0);
  }
}

void AmmoHero::Enabling() {
  globals->magazine_motor.SendInstruction(rm::device::DmMotorInstructions::kClearError);
  globals->magazine_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);
}

bool AmmoHero::EnableJudgement() {
  // 主状态检测
  if (globals->state_machine.getMainState_() == StateMachine::MainState::kOffline) {
    Unabling();
    return false;
  } else if (globals->state_machine.getMainState_() == StateMachine::MainState::kEnableWaiting) {
    Enabling();
    return false;
  } else if (globals->state_machine.getMainState_() == StateMachine::MainState::kTest &&
             globals->state_machine.getSubState_() != StateMachine::SubState::kSnipe) {
    Unabling();
    return false;
  }
  // Ammo状态检测
  switch (globals->state_machine.getAmmoState_()) {
    default:
      Unabling();
      return false;
    case StateMachine::AmmoState::kOffline:
      Unabling();
      return false;
    case StateMachine::AmmoState::kNoForce:
      Unabling();
      return false;
    case StateMachine::AmmoState::kEnableWaiting:
      Unabling();
      Enabling();
      return false;
    case StateMachine::AmmoState::kNormal:
      return true;
    case StateMachine::AmmoState::kSnipe:
      return true;
  }
}

void AmmoHero::BulletSpeedControl() {
  if (key_once_tc(rm::device::VT03::KeyboardKey::kE) &&
      !(globals->tc.data().keyboard_key & rm::device::VT03::KeyboardKey::kShift)) {
    shooter_m -= 10;
  }
  if (key_once_tc(rm::device::VT03::KeyboardKey::kQ) &&
      !(globals->tc.data().keyboard_key & rm::device::VT03::KeyboardKey::kShift)) {
    shooter_m += 10;
  }
  if (globals->state_machine.getSubState_() == StateMachine::SubState::kSnipe &&
      globals->state_machine.getGimbalState_() == StateMachine::GimbalState::kDroneAim &&
      globals->refsub_manager.all_device_ok()) {
    shooter_m = globals->refSub->data().drone_2_hero.cmd_ammo_adjust;
  }
}

void AmmoHero::ShooterControl() {  // 摩擦轮逻辑

  // 目标速度PID
  switch (globals->state_machine.getSubState_()) {
    default:
      Unabling();
    case StateMachine::SubState::kNormal:
      if (globals->shooter_motor_6.rpm() > limit) {
        globals->pid_shooter_1.Update(limit, globals->shooter_motor_1.rpm());
        globals->pid_shooter_2.Update(limit, globals->shooter_motor_2.rpm());
        globals->pid_shooter_3.Update(limit, globals->shooter_motor_3.rpm());
        globals->pid_shooter_4.Update(limit, globals->shooter_motor_4.rpm());
        globals->pid_shooter_5.Update(limit, globals->shooter_motor_5.rpm());
        globals->pid_shooter_6.Update(limit, globals->shooter_motor_6.rpm());
      } else {
        globals->pid_shooter_1.Update(v12_shooter_1, globals->shooter_motor_1.rpm());
        globals->pid_shooter_3.Update(v12_shooter_1, globals->shooter_motor_3.rpm());
        globals->pid_shooter_5.Update(v12_shooter_1, globals->shooter_motor_5.rpm());
        globals->pid_shooter_2.Update(v12_shooter_2 + shooter_m, globals->shooter_motor_2.rpm());
        globals->pid_shooter_4.Update(v12_shooter_2 + shooter_m, globals->shooter_motor_4.rpm());
        globals->pid_shooter_6.Update(v12_shooter_2 + shooter_m, globals->shooter_motor_6.rpm());
      }
      break;

    case StateMachine::SubState::kSnipe:
      if (globals->shooter_motor_6.rpm() > limit /*||
          ((globals->ref.data().robot_status.power_management_chassis_output == 1 ||
            !globals->custom_data.snipe_state) &&
           globals->state_machine.getMainState_() != StateMachine::MainState::kTest)*/) {
        globals->pid_shooter_1.Update(v12_shooter_1, globals->shooter_motor_1.rpm());
        globals->pid_shooter_3.Update(v12_shooter_1, globals->shooter_motor_3.rpm());
        globals->pid_shooter_5.Update(v12_shooter_1, globals->shooter_motor_5.rpm());
        globals->pid_shooter_2.Update(v12_shooter_2 + shooter_m, globals->shooter_motor_2.rpm());
        globals->pid_shooter_4.Update(v12_shooter_2 + shooter_m, globals->shooter_motor_4.rpm());
        globals->pid_shooter_6.Update(v12_shooter_2 + shooter_m, globals->shooter_motor_6.rpm());
      } else {
        globals->pid_shooter_1.Update(v16_shooter_1, globals->shooter_motor_1.rpm());
        globals->pid_shooter_3.Update(v16_shooter_1, globals->shooter_motor_3.rpm());
        globals->pid_shooter_5.Update(v16_shooter_1, globals->shooter_motor_5.rpm());
        globals->pid_shooter_2.Update(v16_shooter_2 + shooter_m, globals->shooter_motor_2.rpm());
        globals->pid_shooter_4.Update(v16_shooter_2 + shooter_m, globals->shooter_motor_4.rpm());
        globals->pid_shooter_6.Update(v16_shooter_2 + shooter_m, globals->shooter_motor_6.rpm());
      }
      break;
  }

  // 给shooter电机发送指令
  globals->shooter_motor_1.SetCurrent(static_cast<int16_t>(globals->pid_shooter_1.out()));
  globals->shooter_motor_2.SetCurrent(static_cast<int16_t>(globals->pid_shooter_2.out()));
  globals->shooter_motor_3.SetCurrent(static_cast<int16_t>(globals->pid_shooter_3.out()));
  globals->shooter_motor_4.SetCurrent(static_cast<int16_t>(globals->pid_shooter_4.out()));
  globals->shooter_motor_5.SetCurrent(static_cast<int16_t>(globals->pid_shooter_5.out()));
  globals->shooter_motor_6.SetCurrent(static_cast<int16_t>(globals->pid_shooter_6.out()));
}

void AmmoHero::MagzineControl() {
  switch (globals->state_machine.getAmmoSubState_()) {
    default:
      globals->magazine_motor.SetMitCommand(0, 0, 0, 0, 0);
      break;
    case StateMachine::AmmoSubState::kInit:
      globals->magazine_motor.SetMitCommand(0, 0, 0, 0, 0);
      break;
    case StateMachine::AmmoSubState::kReady:
    case StateMachine::AmmoSubState::kFire:
    case StateMachine::AmmoSubState::kCooling:
      // 拨盘电机串级PID（开循环）
      globals->pid_magz_position.SetCircular(true).SetCircularCycle(3.141593 * 2);
      globals->pid_magz_position.Update(globals->state_machine.now_angle, globals->magazine_motor.pos(), 0.002);
      globals->pid_magz_velocity.Update(globals->pid_magz_position.out(), globals->magazine_motor.vel(), 0.002);
      globals->magazine_motor.SetMitCommand(0, 0, globals->pid_magz_velocity.out(), 0, 0);
      break;
  }
}

void AmmoHero::AmmoControl() {
  BulletSpeedControl();
  ShooterControl();
  MagzineControl();
}

void AmmoHero::AmmoUpdate() {
  if (!EnableJudgement()) return;
  AmmoControl();
}
