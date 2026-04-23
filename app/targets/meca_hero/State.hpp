#pragma once

#include "librm.hpp"

/**
 * @brife 状态机类
 * @param waiting_count 1s主函数运行次数
 *
 */
class StateMachine {
 public:
  StateMachine() = delete;

  explicit StateMachine(int waiting_count) : waiting_count_(waiting_count), count_(waiting_count) {}

  void SetWaitingCount(int count) { waiting_count_ = count; }

  enum class MainState {
    kOffline,
    kGame,
    kTest,
    kEnableWaiting,
  };

  enum class SubState {
    kNormal,
    kOverPower,  // 加速和上坡2in1
    kSnipe,
  };

  enum class ChassisState {
    kOffline,
    kNoForce,
    kNormal,
    kFollow,
  };

  enum class GimbalState {
    kOffline,
    kNoForce,
    kEnableWaiting,
    kNormal,
    kAimbot,
    kRadarAimbot,
  };

  enum class AmmoState {
    kOffline,
    kNoForce,
    kEnableWaiting,
    kNormal,
    kSnipe,
  };

  void StateUpdate();
  void DT7Switch();

  /**
   * @brief 计时器，为每个需要独立使用的Waiting注册一个实例
   */
  struct TickTimer {
    int count_{0};
    bool first_flag_{true};

    /**
     * @brief 重置计时器，使其在下次调用时重新开始计时
     */
    void reset() { first_flag_ = true; }
    void set(int set) { count_ = set; }
  };

  /**
   * @brief 非阻塞延时函数
   * @param waiting_count
   * @param tick_timer
   * @return bool
   * @note
   * 状态机切换过程中需要进行使能操作时，应有一定的反应时间，一般利用该函数停留在Waiting状态，通过监测状态切换到Waiting来决定是否发送使能信息
   */
  bool Waiting(int waiting_count, TickTimer &tick_timer) {
    ////使能时间段
    // static bool first_flag{true};
    if (tick_timer.first_flag_) {
      tick_timer.count_ = waiting_count;
      tick_timer.first_flag_ = false;
    }
    if (tick_timer.count_ > 0) {
      tick_timer.count_--;
      return false;
    } else {
      tick_timer.count_ = waiting_count;
      tick_timer.first_flag_ = true;
      return true;
    }
  }

  /**
   * @brief Test模式下随动切换
   */
  void TestFollowSwitch(int dial) {
    if (abs(dial) > 400) {
      if (Waiting(waiting_count_ / 2, waiting_first_flag_)) {
        if (current_chassis_state_ != ChassisState::kFollow) {
          last_chassis_state_ = current_chassis_state_;
          current_chassis_state_ = ChassisState::kFollow;
        } else {
          current_chassis_state_ = last_chassis_state_;
          last_chassis_state_ = ChassisState::kFollow;
        }
      }
    }
  }

  /**
   * @brief 实现拨轮切换状态
   * @return
   */
  void DialSwitch();

  [[nodiscard]] MainState getMainState() const { return current_main_state_; }
  [[nodiscard]] MainState getLastMainState() const { return last_main_state_; }
  [[nodiscard]] SubState getSubState() const { return current_sub_state_; }
  [[nodiscard]] SubState getLastSubState() const { return last_sub_state_; }
  [[nodiscard]] ChassisState getChassisState() const { return current_chassis_state_; }
  [[nodiscard]] ChassisState getLastChassisState() const { return last_chassis_state_; }
  [[nodiscard]] GimbalState getGimbalState() const { return current_gimbal_state_; }
  [[nodiscard]] GimbalState getlastGimbalState() const { return last_gimbal_state_; }
  [[nodiscard]] AmmoState getAmmoState() const { return current_ammo_state_; }
  [[nodiscard]] AmmoState getlastAmmoState() const { return last_ammo_state_; }

 private:
  MainState current_main_state_{MainState::kOffline};
  MainState last_main_state_{MainState::kOffline};
  TickTimer main_tick_;

  SubState current_sub_state_{SubState::kNormal};
  SubState last_sub_state_{SubState::kNormal};

  ChassisState current_chassis_state_{ChassisState::kOffline};
  ChassisState last_chassis_state_{ChassisState::kNormal};

  AmmoState current_ammo_state_{AmmoState::kNormal};
  AmmoState last_ammo_state_{AmmoState::kNormal};
  TickTimer ammo_tick_;

  GimbalState current_gimbal_state_{GimbalState::kOffline};
  GimbalState last_gimbal_state_{GimbalState::kNormal};
  TickTimer gimbal_tick_;

  rm::device::VT03::SwitchPosition tc_switch_position{rm::device::VT03::SwitchPosition::N};
  rm::device::DR16::SwitchPosition rc_switch_position_r{rm::device::DR16::SwitchPosition::kUnknown};
  rm::device::DR16::SwitchPosition rc_switch_position_l{rm::device::DR16::SwitchPosition::kUnknown};

  int waiting_count_{0};  // 应为 1s 的主循环执行次数
  int count_{0};
  bool waiting_first_flag_{true};

  bool gimbal_power_management_last_flag_{false};
  bool shooter_power_management_last_flag_{false};
  bool chassis_power_management_last_flag_{false};
};