#include "State.hpp"
#include "librm.hpp"
#include "globals.hpp"

/*-------------------------------------------------- 状态 标志 ---------------------------------------------------*/
bool snipe{false};         ///< 部署标志位
bool overpower{false};     ///< 爬坡-自动触发
bool follow{true};         ///< 随动标志位
bool radar_aimbot{false};  ///< 部署模式下的雷达自瞄
bool fire = {false};       ///< 允许自瞄开火标志位

/**
 * @brief 发射机构子状态切换逻辑（拨盘打弹逻辑）
 */
void StateMachine::AmmoSubState() {}

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
    return;
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
    case MainState::kOffline:
      current_main_state_ = MainState::kEnableWaiting;
      main_tick_.reset();
      ammo_tick_.reset();
      gimbal_tick_.reset();
      chassis_tick_.reset();
      break;
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
          current_gimbal_state_ = GimbalState::kOffline;
          break;

        case (0):
          current_gimbal_state_ = GimbalState::kOffline;
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
          if (radar_aimbot) {
            last_gimbal_state_ = current_gimbal_state_;
            current_gimbal_state_ = GimbalState::kRadarAimbot;
          } else {
            current_gimbal_state_ = GimbalState::kNormal;
          }
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
          // wait_ AmmoSubState();
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
      break;

    case rm::device::DR16::SwitchPosition::kDown:  /// 失能
      current_main_state_ = MainState::kOffline;
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
            current_ammo_state_ = AmmoState::kNormal;
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

            case rm::device::DR16::SwitchPosition::kUp:
              TestDialSwitch(globals->rc.dial(), current_sub_state_, last_sub_state_,
                             SubState::kSnipe);  /// 上坡切换
              break;

            default:
              current_sub_state_ = SubState::kNormal;
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
  gimbal_power_management_last_flag_ = globals->ref.data().robot_status.power_management_gimbal_output;
  ammo_power_management_last_flag_ = globals->ref.data().robot_status.power_management_shooter_output;
  chassis_power_management_last_flag_ = globals->ref.data().robot_status.power_management_chassis_output;

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
  fire = globals->tc.data().keyboard_key &
         static_cast<int16_t>(rm::device::VT03::KeyboardKey::kCtrl)||globals->rc.key(DR16::Key::kCtrl);  ///< 允许自瞄开火标志位

  /*---------------------------------------------------状态机逻辑----------------------------------------------------*/
  // 图传最高优先级，图传在C强制失能，S强制进入比赛并进入子模式
  if (globals->tc_manager.all_device_ok()) {
    if (tc_switch_position == rm::device::VT03::SwitchPosition::C) {  // Offline
      current_main_state_ = MainState::kOffline;
      main_tick_.reset();
      return;

    } else if (tc_switch_position == rm::device::VT03::SwitchPosition::S) {  // Game
      current_main_state_ = MainState::kGame;
      GameState();
      return;
    }
  }

  DT7Switch();
}