#include "Chassis.hpp"

void ChassisHero::Unabling() {
  for (int i = 0; i < 4; i++) {
    globals->chassis_motor[i].SetCurrent(0);
  }
}

void ChassisHero::IfFollow() {
  // 随动判断
  switch (globals->state_machine.getChassisState_()) {
    default:
      Unabling();
      break;

    case (StateMachine::ChassisState::kNormal):
      break;

    case StateMachine::ChassisState::kFollow:
      globals->pid_chassis_follow_pos.SetCircular(true).SetCircularCycle(3.141593 * 2);
      globals->pid_chassis_follow_pos.Update(0.61, globals->gimbal_motor_yaw.pos(),
                                             0.0011);  // 逆时针旋转为增大
      globals->pid_chassis_follow_vel.Update(globals->pid_chassis_follow_pos.out(), globals->gimbal_motor_yaw.vel(),
                                             0.0011);
      Vw = static_cast<rm::i16>(globals->pid_chassis_follow_vel.out()) *
           (1 + globals->ahrs.euler_angle().pitch / 0.6644 * 0.7);  ///< 理应pitch越大，输出越小，注意pitch的正负号
      break;
  }
}

void ChassisHero::IfOverPower() {
  // 超功率判断
  if (globals->state_machine.getSubState_() == StateMachine::SubState::kOverPower) {
    // 超功率
    power_limit = 180;
    if (abs(eulerangle_pitch) >= 0.6 && overpower_count > 0) {
      power_limit = 360;
      overpower_count--;
    }
  } else {
    power_limit = globals->ref.data().robot_status.chassis_power_limit == 0
                      ? 50
                      : static_cast<float>(globals->ref.data().robot_status.chassis_power_limit);
    overpower_count = 400;
  }

  globals->power_model.DistributePower<4>(globals->motor_states, globals->initial_currents, power_limit,
                                          globals->output_currents);

  if (globals->state_machine.getSubState_() == StateMachine::SubState::kOverPower) {
    if (globals->ref.data().power_heat_data.buffer_energy <= 15) {
      for (int i = 0; i < 4; i++)
        globals->chassis_motor[i].SetCurrent(static_cast<int16_t>(
            globals->output_currents[i] * (globals->ref.data().power_heat_data.buffer_energy) / (60 * 2)));
    } else {
      for (int i = 0; i < 4; i++)
        globals->chassis_motor[i].SetCurrent(static_cast<int16_t>(globals->output_currents[i]));
    }
  } else {
    if (globals->cms.cms_v >= 25.0) {
      for (int i = 0; i < 4; i++)
        globals->chassis_motor[i].SetCurrent(static_cast<int16_t>(globals->output_currents[i]));
    } else {
      for (int i = 0; i < 4; i++)
        globals->chassis_motor[i].SetCurrent(static_cast<int16_t>(
            globals->output_currents[i] * (globals->ref.data().power_heat_data.buffer_energy) / 60));
    }
  }
}

void ChassisHero::Remote() {
  if (globals->state_machine.getMainState_() == StateMachine::MainState::kGame) {
    if (globals->rc.key(rm::device::DR16::Key::kW) ||
        (globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kW) &&
         globals->tc_manager.all_device_ok()) /*||
        globals->custom_client->key(rm::device::DR16::Key::kW)*/) {
      Vy += 30;
      if (Vy >= 8400) {
        Vy = 8500;
      }
    } else if (globals->rc.key(rm::device::DR16::Key::kS) ||
                              (globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kS) &&
                               globals->tc_manager.all_device_ok()) /*||
                              globals->custom_client->key(rm::device::DR16::Key::kS)*/) {
      Vy -= 30;
      if (Vy <= -8400) {
        Vy = -8500;
      }
    } else {
      Vy = 0;
    }
    if (globals->rc.key(rm::device::DR16::Key::kD) ||
        (globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kD) &&
         globals->tc_manager.all_device_ok()) /*||
        globals->custom_client->key(rm::device::DR16::Key::kD)*/) {
      Vw += 30;
      if (Vw >= 8400) {
        Vw = 7000;
      }
    } else if (globals->rc.key(rm::device::DR16::Key::kA) ||
                              (globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kA) &&
                               globals->tc_manager.all_device_ok()) /*||
                              globals->custom_client->key(rm::device::DR16::Key::kA)*/) {
      Vw -= 30;
      if (Vw <= -8400) {
        Vw = -7000;
      }
    } else {
      Vw = 0;
    }
  } else {
    Vw = globals->rc.left_x() * 7000 / 660;
    Vy = globals->rc.left_y() * 8500 / 660;
  }
}

void ChassisHero::ChassisControl() {
  IfFollow();
  Remote();
  IfOverPower();
}

bool ChassisHero::EnableJudgement() {
  // 主状态检测
  if (globals->state_machine.getMainState_() == StateMachine::MainState::kOffline ||
      globals->state_machine.getMainState_() == StateMachine::MainState::kEnableWaiting ||
      globals->state_machine.getSubState_() == StateMachine::SubState::kSnipe) {
    Unabling();
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
    case StateMachine::ChassisState::kNormal:
      return true;
    case StateMachine::ChassisState::kFollow:
      return true;
  }
}

void ChassisHero::ChassisUpdate() {
  if (!EnableJudgement()) return;
  ChassisControl();
}
