#include "AmmoShooter.hpp"

/**
 * @brief 发射机构子状态切换逻辑（拨盘打弹逻辑）
 */
void AmmoShooter::AmmoSubState() {
  switch (globals->state_machine.getlastAmmoSubState_()) {
    default:
      globals->state_machine.setAmmoSubState(StateMachine::AmmoSubState::kInit);
      break;

    case StateMachine::AmmoSubState::kInit:
      now_angle = globals->magazine_motor.pos();
      // 检测当前角度，防止大幅转动
      while (next_angle < globals->magazine_motor.pos()) {
        next_angle += 1.0472 /*（π/3）*/;
      }
      while (next_angle > globals->magazine_motor.pos()) {
        next_angle -= 1.0472 /*（π/3）*/;
      }
      if (next_angle < -3.141593) {
        next_angle += 2 * 3.141593;
      }
      globals->state_machine.setAmmoSubState(StateMachine::AmmoSubState::kReady);
      break;
      // 使能
      // globals->magazine_motor.SendInstruction(rm::device::DmMotorInstructions::kClearError);
      // globals->magazine_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);

    case StateMachine::AmmoSubState::kReady:
      if ((globals->rc.dial() >= 500 || globals->rc.dial() < -500 || globals->rc.mouse_button_left() ||
               globals->tc.data().mouse_button_left /*|| globals->custom_client->mouse_left()*/ /*||
               (globals->aimbot_can_communicator->aimbot_state() == 0x03 &&
                 globals->tc->data().keyboard_key & static_cast<int16_t>(VT03::KeyboardKey::kCtrl))*/) &&
              (globals->ref.data().robot_status.shooter_barrel_heat_limit >=
               globals->ref.data().power_heat_data.shooter_42mm_barrel_heat + 100) &&
              (globals->shooter_motor_1.rpm() < -3000 && globals->shooter_motor_4.rpm() < -3000)) {
        globals->state_machine.setAmmoSubState(StateMachine::AmmoSubState::kFire);
      }
      break;

    case StateMachine::AmmoSubState::kFire:
      // 堵转检测
      if (globals->state_machine.Waiting(waiting_count_, ammo_sub_tick_)) {
        if (rm::modules::Wrap(now_angle - globals->magazine_motor.pos(), -3.141593, 3.141593) < -3.141593 / 18) {
          now_angle = globals->magazine_motor.pos() + 3.141593 / 18;
        } else {
          next_angle -= 3.141593 / 3;
          if (next_angle < -3.141593) {
            next_angle += 2 * 3.141593;
          }
        }
        globals->state_machine.setAmmoSubState(StateMachine::AmmoSubState::kCooling);
      }
      break;

    case StateMachine::AmmoSubState::kCooling:
      if ((globals->ref.data().robot_status.shooter_barrel_heat_limit >=
           globals->ref.data().power_heat_data.shooter_42mm_barrel_heat + 120) ||
          globals->tc.data().mouse_button_middle) {  // 鼠标中键强制打弹
        globals->state_machine.setAmmoSubState(StateMachine::AmmoSubState::kReady);
      }
  }

  void AmmoShooter::AmmoUpdate() {}
