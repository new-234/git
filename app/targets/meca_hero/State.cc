#include "State.hpp"
#include "librm.hpp"
#include "globals.hpp"

/*-------------------------------------------------- 状态 标志 ---------------------------------------------------*/
bool snipe{false};         ///< 部署标志位
bool overpower{false};     ///< 爬坡-自动触发
bool follow{true};         ///< 随动标志位
bool radar_aimbot{false};  ///< 部署模式下的雷达自瞄
bool fire = {false};       ///< 允许自瞄开火标志位
bool low_hp{false};
// bool pitchlock{false};     ///< pitch锁定标志位

using namespace rm;

/**
 * @brief 本地计算热量并同步裁判系统更新，防止热量更新频率低导致超热量。
 * @param fired 是否已开火
 * @param cooling_value 裁判系统每秒冷却值
 * @param heat_limit 裁判系统热量限制
 * @param barrel_heat 裁判系统当前热量
 * @param frequency 函数执行频率
 * @param heat_per_bullet 每发弹丸所需热量
 * @return bool:是否允许发弹
 */
bool HeatLocalCaculate(bool fired, const u16 cooling_value, const u16 heat_limit, const u16 barrel_heat, int frequency,
                       int heat_per_bullet) {
  static float local_barrel_heat;   ///< 实时计算值
  static u16 last_ref_barrel_heat;  ///< 检测值
  if (last_ref_barrel_heat == barrel_heat && barrel_heat != 0) {
    local_barrel_heat -=
        static_cast<float>(cooling_value) / static_cast<float>(frequency) - static_cast<float>(fired * heat_per_bullet);
  } else {
    last_ref_barrel_heat = barrel_heat;
    local_barrel_heat = barrel_heat;
  }
  if (static_cast<float>(heat_limit) - local_barrel_heat >= 100) {
    return true;
  } else {
    return false;
  }
}

/**
 * @brief 发射机构子状态切换逻辑（拨盘打弹逻辑）
 */
void StateMachine::AmmoSubStateUpdate() {
  static bool fired{false};
  switch (current_ammo_sub_state_) {
    default:
      current_ammo_sub_state_ = StateMachine::AmmoSubState::kInit;
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
      current_ammo_sub_state_ = StateMachine::AmmoSubState::kReady;
      break;
      // 使能
      // globals->magazine_motor.SendInstruction(rm::device::DmMotorInstructions::kClearError);
      // globals->magazine_motor.SendInstruction(rm::device::DmMotorInstructions::kEnable);

    case StateMachine::AmmoSubState::kReady:
      if ((fire && globals->shooter_motor_1.rpm() < -2800 && globals->shooter_motor_4.rpm() < -2800 && !low_hp_) ||
          globals->tc.data().mouse_button_middle) {  // 鼠标中键强制打弹
        now_angle = next_angle;
        current_ammo_sub_state_ = StateMachine::AmmoSubState::kFire;
      }
      break;

    case StateMachine::AmmoSubState::kFire:
      // 堵转检测
      if (globals->state_machine.Waiting(waiting_count_ / 4, ammo_sub_tick_)) {
        if (rm::modules::Wrap(now_angle - globals->magazine_motor.pos(), -3.141593, 3.141593) < -3.141593 / 36) {
// if (globals->magazine_motor.status())
          magz_stucked_ = true;
          now_angle = globals->magazine_motor.pos() + 3.141593 / 9;
        } else {
          magz_stucked_ = false;
          next_angle -= 3.141593 / 3;
          if (next_angle < -3.141593) {
            next_angle += 2 * 3.141593;
          }
        }
        fired = true;
        current_ammo_sub_state_ = StateMachine::AmmoSubState::kCooling;
      }
      break;

    case StateMachine::AmmoSubState::kCooling:
      if (current_main_state_ == MainState::kTest) {
        current_ammo_sub_state_ = StateMachine::AmmoSubState::kReady;
        break;
      }
      if (globals->state_machine.Waiting(waiting_count_, ammo_sub_tick_)) {
        if (globals->ref.data().robot_status.shooter_barrel_heat_limit -
                globals->ref.data().power_heat_data.shooter_42mm_barrel_heat >=
            100) {
          current_ammo_sub_state_ = StateMachine::AmmoSubState::kReady;
        }
        fired = false;
      }
  }
}

/**
 * @brief 比赛模式状态更新
 */
void StateMachine::GameState() {
  // 死亡判断
  if (!globals->ref.data().robot_status.power_management_chassis_output &&
      !globals->ref.data().robot_status.power_management_gimbal_output &&
      !globals->ref.data().robot_status.power_management_shooter_output) {
    gimbal_power_management_last_flag_ = 0;
    ammo_power_management_last_flag_ = 0;
    chassis_power_management_last_flag_ = 0;
    current_main_state_ = MainState::kOffline;
    return;
  }
  if (/*!globals->ref_manager.all_device_ok() ||*/ !globals->tc_manager.all_device_ok()) {
    // current_main_state_ = MainState::kOffline;
    // return;
  }
  if (!globals->gimbal_manager.all_device_ok()) {
    // current_gimbal_state_ = GimbalState::kOffline;
    // return;
  }
  // if (!globals->chassis_manager.all_device_ok()) {
  //   current_chassis_state_ = ChassisState::kOffline;
  //   return;
  // }
  // if (!globals->ammo_manager.all_device_ok()) {
  //   current_ammo_state_ = AmmoState::kOffline;
  //   return;
  // }

  /*---------------------------------------------------主状态----------------------------------------------------*/
  switch (current_main_state_) {
    default:
      // 默认进入使能
      current_main_state_ = MainState::kEnableWaiting;
      main_tick_.reset();
      ammo_tick_.reset();
      gimbal_tick_.reset();
      chassis_tick_.reset();
      break;

      // case MainState::kOffline:
      //   current_main_state_ = MainState::kEnableWaiting;
      //   main_tick_.reset();
      //   ammo_tick_.reset();
      //   gimbal_tick_.reset();
      //   chassis_tick_.reset();
      //   break;

    case MainState::kEnableWaiting:
      if (Waiting(waiting_count_ / 10, main_tick_)) current_main_state_ = MainState::kGame;
      break;

    case MainState::kGame:

      /*---------------------------------------------------子状态----------------------------------------------------*/
      if (snipe) {
        current_sub_state_ = SubState::kSnipe;
      } else if (overpower) {
        current_sub_state_ = SubState::kOverPower;
      } else {
        current_sub_state_ = SubState::kNormal;
      }

      /*--------------------------------------------------模块状态---------------------------------------------------*/
      /// 底盘状态
      switch (RefereePowerManagement(globals->ref.data().robot_status.power_management_chassis_output,
                                     chassis_power_management_last_flag_, chassis_tick_)) {  // 裁判系统断电检测
        default:
          current_chassis_state_ = ChassisState::kOffline;
          break;

        case (0):
          current_chassis_state_ = ChassisState::kOffline;
          break;

        case (1):
          if (follow) {
            current_chassis_state_ = ChassisState::kFollow;
          } else {
            current_chassis_state_ = ChassisState::kNormal;
          }
          break;

        case (2):
          current_chassis_state_ = ChassisState::kOffline;
          break;
      }

      /// 云台状态
      switch (RefereePowerManagement(globals->ref.data().robot_status.power_management_gimbal_output,
                                     gimbal_power_management_last_flag_, gimbal_tick_)) {  // 裁判系统断电检测
        default:
          current_gimbal_state_ = GimbalState::kOffline;
          break;

        case (0):
          current_gimbal_state_ = GimbalState::kOffline;
          break;

        case (1):
          if (globals->tc_manager.all_device_ok()) {
            if (globals->tc.data().keyboard_key & rm::device::VT03::KeyboardKey::kV) {
              current_gimbal_state_ = GimbalState::kDroneAim;
            } else if (globals->tc.data().keyboard_key & rm::device::VT03::KeyboardKey::kB) {
              current_gimbal_state_ = GimbalState::kRadarAimbot;
            } else if (globals->tc.data().mouse_button_right) {
              current_gimbal_state_ = GimbalState::kAimbot;
            } else {
              current_gimbal_state_ = GimbalState::kNormal;
            }
          } else if (globals->rc_manager.all_device_ok()) {
            if (globals->rc.key(device::DR16::Key::kV)) {
              current_gimbal_state_ = GimbalState::kDroneAim;
            } else if (globals->rc.key(device::DR16::Key::kB)) {
              current_gimbal_state_ = GimbalState::kRadarAimbot;
            } else if (globals->rc.mouse_button_right()) {
              current_gimbal_state_ = GimbalState::kAimbot;
            } else {
              current_gimbal_state_ = GimbalState::kNormal;
            }
          } else {
            current_gimbal_state_ = GimbalState::kNormal;
          }

          // if (radar_aimbot) {
          //   last_gimbal_state_ = current_gimbal_state_;
          //   current_gimbal_state_ = GimbalState::kRadarAimbot;
          // } else {
          //   current_gimbal_state_ = GimbalState::kNormal;
          // }
          // if (pitchlock) {
          //   last_gimbal_sub_state_ = current_gimbal_sub_state_;
          //   current_gimbal_sub_state_ = GimbalSubState::kPitchlock;
          // } else {
          //   current_gimbal_sub_state_ = GimbalSubState::kNormal;
          // }
          break;

        case (2):
          current_gimbal_state_ = GimbalState::kEnableWaiting;
          break;
      }

      /// 发射状态
      switch (RefereePowerManagement(globals->ref.data().robot_status.power_management_shooter_output,
                                     ammo_power_management_last_flag_, ammo_tick_)) {
        default:
          current_ammo_state_ = AmmoState::kOffline;
          break;

        case (0):
          current_ammo_state_ = AmmoState::kOffline;
          break;

        case (1):
          current_ammo_state_ = AmmoState::kNormal;
          globals->state_machine.AmmoSubStateUpdate();
          break;

        case (2):
          current_ammo_state_ = AmmoState::kEnableWaiting;
          break;
      }

      break;
  }
}

/**
 *@brief:DT7遥控器切换状态
 */
void StateMachine::DT7Switch() {
  // 右拨杆
  switch (StateMachine::rc_switch_position_r) {
    case rm::device::DR16::SwitchPosition::kUnknown:  /// 保险
      current_main_state_ = MainState::kOffline;
      current_ammo_state_ = AmmoState::kOffline;
      break;

    case rm::device::DR16::SwitchPosition::kDown:  /// 失能
      current_main_state_ = MainState::kOffline;
      current_ammo_state_ = AmmoState::kOffline;
      break;

    case rm::device::DR16::SwitchPosition::kMid:  /// 调试
      switch (current_main_state_) {
        case MainState::kOffline:
          current_main_state_ = MainState::kEnableWaiting;
          main_tick_.reset();
          gimbal_tick_.reset();
          ammo_tick_.reset();
          chassis_tick_.reset();
          test_switch_tick_.reset();
          break;

        case MainState::kEnableWaiting:
          // if (!waiting_first_flag) waiting_first_flag = true;
          if (Waiting(waiting_count_ / 10, main_tick_)) {
            current_main_state_ = MainState::kTest;
            current_sub_state_ = SubState::kNormal;
            current_chassis_state_ = ChassisState::kNormal;
            current_gimbal_state_ = GimbalState::kNormal;
            // current_ammo_state_ = AmmoState::kNormal;
          }
          break;

        case MainState::kTest:
          switch (rc_switch_position_l) {
            case rm::device::DR16::SwitchPosition::kDown:  /// 普通遥控
              current_sub_state_ = SubState::kNormal;
              // current_chassis_state_ = ChassisState::kNormal;
              current_gimbal_state_ = GimbalState::kNormal;
              current_ammo_state_ = AmmoState::kNormal;
              TestDialSwitch(globals->rc.dial(), current_chassis_state_, last_chassis_state_,
                             ChassisState::kFollow);  /// 随动切换
              break;

            case rm::device::DR16::SwitchPosition::kMid:  /// 超功率+上坡（应该直接加pitch无力
              TestDialSwitch(globals->rc.dial(), current_sub_state_, last_sub_state_,
                             SubState::kOverPower);  /// 上坡切换
              break;

            case rm::device::DR16::SwitchPosition::kUp: {
              current_sub_state_ = SubState::kSnipe;
              switch (current_ammo_state_) {
                default:
                  globals->state_machine.current_ammo_state_ = AmmoState::kEnableWaiting;
                  break;
                case AmmoState::kEnableWaiting:
                  if (Waiting(waiting_count_ / 10, ammo_tick_)) current_ammo_state_ = AmmoState::kNormal;
                  break;
                case AmmoState::kNormal:
                  if (abs(globals->rc.left_y()) > 650) {
                    radar_aimbot = true;
                  } else {
                    radar_aimbot = false;
                  }
                  globals->state_machine.AmmoSubStateUpdate();
                  break;
              }

              break;
            }

            default:
              current_sub_state_ = SubState::kNormal;
              break;
          };
          break;

        default:
          current_main_state_ = MainState::kTest;
          break;
      }
      break;

    case rm::device::DR16::SwitchPosition::kUp:
      switch (current_main_state_) {
        case MainState::kOffline:  // 保险
          current_main_state_ = MainState::kEnableWaiting;
          main_tick_.reset();
          gimbal_tick_.reset();
          ammo_tick_.reset();
          chassis_tick_.reset();
          test_switch_tick_.reset();
          break;

        case MainState::kTest:
          current_main_state_ = MainState::kEnableWaiting;
          main_tick_.reset();
          gimbal_tick_.reset();
          ammo_tick_.reset();
          chassis_tick_.reset();
          test_switch_tick_.reset();
          break;

        case MainState::kEnableWaiting:
          // if (!waiting_first_flag) waiting_first_flag = true;
          if (Waiting(waiting_count_ / 10, main_tick_)) {
            current_main_state_ = MainState::kGame;
            current_sub_state_ = SubState::kNormal;
            current_chassis_state_ = ChassisState::kNormal;
            current_gimbal_state_ = GimbalState::kNormal;
            current_ammo_state_ = AmmoState::kNormal;
          }
          break;

        case MainState::kGame:
          current_main_state_ = MainState::kGame;
          GameState();  /// 比赛模式
          break;
        default:
          current_main_state_ = MainState::kOffline;
          break;
      }
  }
}

/**
 * @brief:状态机更新函数
 * @note:
 **/
void StateMachine::StateUpdate() {
  /*--------------------------------------------------外部变量拷贝---------------------------------------------------*/
  rc_switch_position_l = globals->rc.switch_l();
  tc_switch_position = globals->tc.data().switch_position;
  rc_switch_position_r = globals->rc.switch_r();
  // globals->keyboard_handler().Update(DR16::Key::)
  // gimbal_power_management_last_flag_ = globals->ref.data().robot_status.power_management_gimbal_output;
  // ammo_power_management_last_flag_ = globals->ref.data().robot_status.power_management_shooter_output;
  // chassis_power_management_last_flag_ = globals->ref.data().robot_status.power_management_chassis_output;

  /*--------------------------------------------------子模式状态位---------------------------------------------------*/
  // static bool snipe{false};  ///< 部署标志位
  if (key_once_tc(rm::device::VT03::KeyboardKey::kG)) snipe = !snipe;
  overpower =
      globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kShift);  ///< 爬坡-自动触发

  /*---------------------------------------------------模块 状态位---------------------------------------------------*/
  // static bool follow{true};  ///< 随动标志位
  if (key_once_tc(rm::device::VT03::KeyboardKey::kC)) follow = !follow;

  radar_aimbot = globals->tc.data().mouse_button_right || globals->rc.mouse_button_right();  ///< 部署模式下的雷达自瞄
  // bool aimbot = globals->tc.data().right_button;        ///< 常态模式下的装甲板自瞄

  if (((abs(globals->rc.dial()) >= 500 || globals->rc.mouse_button_left() || globals->tc.data().mouse_button_left ||
        globals->rc.mouse_button_left() || key_once_tc(rm::device::VT03::kF) ||
        (current_sub_state_ == SubState::kSnipe && current_gimbal_state_ == GimbalState::kDroneAim &&
         globals->refSub->data().drone_2_hero.cmd_shooting == 1)))) {
    fire = true;
  } else {
    fire = false;
  }
  // fire = globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kCtrl) ||
  //        globals->rc.key(rm::device::DR16::Key::kCtrl);  ///< 允许自瞄开火标志位
  if (key_once_tc(rm::device::VT03::kZ)) {
    scope_state++;
    scope_state %= 4;  // 循环切换状态（0→1→2→3→0）
    scope_count = 0;
  }
  if (globals->ref.data().robot_status.current_HP < 100) {
    low_hp_ = true;
  } else {
    low_hp_ = false;
  }
  // if (key_once_tc(rm::device::VT03::KeyboardKey::kF)) pitchlock = !pitchlock;

  /*---------------------------------------------------状态机逻辑----------------------------------------------------*/
  // 图传最高优先级，图传在C强制失能，S强制进入比赛并进入子模式
  if (globals->tc_manager.all_device_ok()) {
    if (tc_switch_position == rm::device::VT03::SwitchPosition::C) {  // Offline
      // current_main_state_ = MainState::kOffline;
      // main_tick_.reset();
      // return;

    } else if (tc_switch_position == rm::device::VT03::SwitchPosition::S) {  // Game
      // current_main_state_ = MainState::kEnableWaiting;
      GameState();
      return;
    }
  }

  DT7Switch();
}