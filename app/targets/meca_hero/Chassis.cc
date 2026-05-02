#include "Chassis.hpp"

#include "librm/device/remote/dr16.hpp"

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
      globals->pid_chassis_follow_pos.Update(-0.288, globals->gimbal_motor_yaw.pos(),
                                             0.002);  // 逆时针旋转为增大
      globals->pid_chassis_follow_vel.Update(globals->pid_chassis_follow_pos.out(), globals->gimbal_motor_yaw.vel(),
                                             0.002);
      Vw = static_cast<rm::i16>(globals->pid_chassis_follow_vel.out()) /**
           (1 + globals->ahrs.euler_angle().pitch / 0.6644 * 0.7)*/;  ///< 理应pitch越大，输出越小，注意pitch的正负号
      break;
  }
}

void ChassisHero::Remote() {
  switch (globals->state_machine.getChassisState_()) {
    default:
      Unabling();
      break;

    case StateMachine::ChassisState::kNormal:
      if (globals->tc_manager.all_device_ok()) {
        if (globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kW)) {
          V_w = 8500;
        } else {
          V_w = 0;
        }
        if (globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kS)) {
          V_s = 8500;
        } else {
          V_s = 0;
        }
        if (globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kD)) {
          V_d = 8500;
        } else {
          V_d = 0;
        }
        if (globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kA)) {
          V_a = 8500;
        } else {
          V_a = 0;
        }
        Vy = V_w - V_s + globals->rc.left_y() * 8500 / 660;
        Vw = V_d - V_a + globals->rc.left_x() * 7000 / 660;
      } else if (globals->rc_manager.all_device_ok()) {
        if (globals->rc.key(DR16::Key::kW)) {
          V_w = 8500;
        } else {
          V_w = 0;
        }
        if (globals->rc.key(DR16::Key::kS)) {
          V_s = 8500;
        } else {
          V_s = 0;
        }
        if (globals->rc.key(DR16::Key::kD)) {
          V_d = 8500;
        } else {
          V_d = 0;
        }
        if (globals->rc.key(DR16::Key::kA)) {
          V_a = 8500;
        } else {
          V_a = 0;
        }
        Vy = V_w - V_s + globals->rc.left_y() * 8500 / 660;
        Vw = V_d - V_a + globals->rc.left_x() * 7000 / 660;
      }
      break;

    case StateMachine::ChassisState::kFollow:
      if (globals->tc_manager.all_device_ok()) {
        if (globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kW)) {
          V_w = 8500;
        } else {
          V_w = 0;
        }
        if (globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kS)) {
          V_s = 8500;
        } else {
          V_s = 0;
        }
        Vy = V_w - V_s + globals->rc.left_y() * 8500 / 660;
      } else if (globals->rc_manager.all_device_ok()) {
        if (globals->rc.key(DR16::Key::kW)) {
          V_w = 8500;
        } else {
          V_w = 0;
        }
        if (globals->rc.key(DR16::Key::kS)) {
          V_s = 8500;
        } else {
          V_s = 0;
        }
        Vy = V_w - V_s + globals->rc.left_y() * 8500 / 660;
      }
      break;
  }

  // 获取各轴目标速度
  // if (globals->state_machine.getMainState_() == StateMachine::MainState::kGame) {
  //   if ((globals->rc.key(rm::device::DR16::Key::kW) && globals->rc_manager.all_device_ok())||
  //         (globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kW) &&
  //          globals->tc_manager.all_device_ok()) /*||
  //         globals->custom_client->key(rm::device::DR16::Key::kW)*/) {
  //     Vy += 30;
  //     if (Vy >= 8400) {
  //       Vy = 8500;
  //     }
  //   } else if ((globals->rc.key(rm::device::DR16::Key::kS) && globals->rc_manager.all_device_ok())||
  //                               (globals->tc.data().keyboard_key &
  //                               static_cast<int16_t>(rm::device::VT03::KeyboardKey::kS) &&
  //                                globals->tc_manager.all_device_ok()) /*||
  //                               globals->custom_client->key(rm::device::DR16::Key::kS)*/) {
  //     Vy -= 30;
  //     if (Vy <= -8400) {
  //       Vy = -8500;
  //     }
  //   } else {
  //     Vy = 0;
  //   }
  //   if ((globals->rc.key(rm::device::DR16::Key::kD) && globals->rc_manager.all_device_ok())||
  //         (globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kD) &&
  //          globals->tc_manager.all_device_ok()&& globals->state_machine.getChassisState_() !=
  //          StateMachine::ChassisState::kFollow) /*||
  //         globals->custom_client->key(rm::device::DR16::Key::kD)*/) {
  //     Vw += 30;
  //     if (Vw >= 8400) {
  //       Vw = 7000;
  //     }
  //   } else if (((globals->rc.key(rm::device::DR16::Key::kA) && globals->rc_manager.all_device_ok())||
  //                               (globals->tc.data().keyboard_key &
  //                               static_cast<int16_t>(rm::device::VT03::KeyboardKey::kA) &&
  //                                globals->tc_manager.all_device_ok()))&& globals->state_machine.getChassisState_() !=
  //                                StateMachine::ChassisState::kFollow /*||
  //                               globals->custom_client->key(rm::device::DR16::Key::kA)*/) {
  //     Vw -= 30;
  //     if (Vw <= -8400) {
  //       Vw = -7000;
  //     }
  //   } else {
  //     Vw = 0;
  //   }
  // } else {
  //   if (globals->state_machine.getChassisState_() != StateMachine::ChassisState::kFollow) {
  //     Vw = globals->rc.left_x() * 7000 / 660;
  //   }
  //   Vy = globals->rc.left_y() * 8500 / 660;
  // }

  // 解算每个轮子的速度
  V_wheel[0] = -Vy /*+ 1.5 * Vx */ + 1.5 * Vw;
  V_wheel[1] = Vy /*+ 1.5 * Vx*/ + 1.5 * Vw;
  V_wheel[2] = Vy /*- 1.5 * Vx */ + 1 * Vw;
  V_wheel[3] = -Vy /*- 1.5 * Vx*/ + 1 * Vw;
}

void ChassisHero::IfOverPower() {
  // 超功率判断
  if (globals->state_machine.getSubState_() == StateMachine::SubState::kOverPower) {
    // 超功率
    power_limit = 180;
    if (abs(globals->ahrs.euler_angle().pitch) >= 0.6 &&
        overpower_count > 0) {  // 超功率模式下，倾角大于一定值时，自动使输出电流变大
      power_limit = 360;
      overpower_count--;
    }
  } else {
    power_limit = globals->ref.data().robot_status.chassis_power_limit == 0
                      ? 50
                      : static_cast<float>(globals->ref.data().robot_status.chassis_power_limit);
    overpower_count = 400;
  }
}

/**
 * @brief 电机状态构建，决定电机电流值
 */
void ChassisHero::MotorUpdate() {
  // 构建电机状态
  for (int i = 0; i < 4; i++) {
    globals->velocity_pid[i].Update(V_wheel[i], globals->chassis_motor[i].rpm());
    globals->initial_currents[i] = globals->velocity_pid[i].out();

    (globals->motor_states)[i].speed_rpm = globals->chassis_motor[i].rpm();
    (globals->motor_states)[i].give_current = globals->initial_currents[i];
    (globals->motor_states)[i].measured_current = globals->chassis_motor[i].current();
  }

  // 分配功率
  globals->power_model.DistributePower<4>(globals->motor_states, globals->initial_currents, power_limit,
                                          globals->output_currents);

  // 直接的输出电流限制
  if (globals->state_machine.getSubState_() == StateMachine::SubState::kOverPower) {
    if (globals->ref.data().power_heat_data.buffer_energy <= 15 || !globals->cms_manager.all_device_ok()) {
      for (int i = 0; i < 4; i++)
        globals->chassis_motor[i].SetCurrent(static_cast<int16_t>(
            globals->output_currents[i] * (globals->ref.data().power_heat_data.buffer_energy) / (60 * 2)));
    } else {
      for (int i = 0; i < 4; i++)
        globals->chassis_motor[i].SetCurrent(static_cast<int16_t>(globals->output_currents[i]));
    }
  } else {
    if (globals->cms.rx_data_.cap_energy >= 250 && !globals->cms_manager.all_device_ok()) {
      for (int i = 0; i < 4; i++)
        globals->chassis_motor[i].SetCurrent(static_cast<int16_t>(globals->output_currents[i]));
    } else {
      for (int i = 0; i < 4; i++)
        globals->chassis_motor[i].SetCurrent(static_cast<int16_t>(
            globals->output_currents[i] * (globals->ref.data().power_heat_data.buffer_energy) / 60));
    }
  }
}

/**
 * @brief 底盘控制
 */
void ChassisHero::ChassisControl() {
  Remote();
  IfFollow();
  IfOverPower();
  MotorUpdate();
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
