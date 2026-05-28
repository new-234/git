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
    kDroneAim,
  };

  // enum class GimbalSubState {
  //   kNormal,
  //   kPitchlock,
  // };

  enum class AmmoState {
    kOffline,
    kNoForce,
    kEnableWaiting,
    kNormal,
    kSnipe,
  };

  enum class AmmoSubState {
    kInit,
    kCooling,
    kReady,
    kFire,
  };

  void StateUpdate();

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

  [[nodiscard]] MainState getMainState_() const { return current_main_state_; }
  [[nodiscard]] MainState getLastMainState_() const { return last_main_state_; }
  [[nodiscard]] SubState getSubState_() const { return current_sub_state_; }
  [[nodiscard]] SubState getLastSubState_() const { return last_sub_state_; }
  void setLastSubState(SubState state) { last_sub_state_ = state; }
  [[nodiscard]] ChassisState getChassisState_() const { return current_chassis_state_; }
  [[nodiscard]] ChassisState getLastChassisState_() const { return last_chassis_state_; }
  void setChassisState(ChassisState state) { current_chassis_state_ = state; }
  [[nodiscard]] GimbalState getGimbalState_() const { return current_gimbal_state_; }
  [[nodiscard]] GimbalState getlastGimbalState_() const { return last_gimbal_state_; }
  // [[nodiscard]] GimbalSubState getGimbalSubState_() const { return current_gimbal_sub_state_; }
  // [[nodiscard]] GimbalSubState getlastGimbalSubState_() const { return last_gimbal_sub_state_; }
  void setGimbalState(GimbalState state) { current_gimbal_state_ = state; }
  [[nodiscard]] AmmoState getAmmoState_() const { return current_ammo_state_; }
  [[nodiscard]] AmmoState getlastAmmoState_() const { return last_ammo_state_; }
  void setAmmoState(AmmoState state) { current_ammo_state_ = state; }
  [[nodiscard]] AmmoSubState getAmmoSubState_() const { return current_ammo_sub_state_; }
  [[nodiscard]] AmmoSubState getlastAmmoSubState_() const { return last_ammo_sub_state_; }
  void setAmmoSubState(AmmoSubState state) { current_ammo_sub_state_ = state; }

  rm::f32 now_angle{0};        ///< 拨盘真实位置
  rm::f32 next_angle{-0.4892695};  ///< 拨盘初始位/应该在的位置  1.711}  0.4892695 .314736

  bool magz_stucked_{false};  ///< 卡弹标志位

 private:
  MainState current_main_state_{MainState::kOffline};
  MainState last_main_state_{MainState::kOffline};
  TickTimer main_tick_;

  SubState current_sub_state_{SubState::kNormal};
  SubState last_sub_state_{SubState::kNormal};

  ChassisState current_chassis_state_{ChassisState::kOffline};
  ChassisState last_chassis_state_{ChassisState::kNormal};
  TickTimer chassis_tick_;

  AmmoState current_ammo_state_{AmmoState::kOffline};
  AmmoState last_ammo_state_{AmmoState::kOffline};
  TickTimer ammo_tick_;

  AmmoSubState current_ammo_sub_state_{AmmoSubState::kInit};
  AmmoSubState last_ammo_sub_state_{AmmoSubState::kInit};
  TickTimer ammo_sub_tick_;

  GimbalState current_gimbal_state_{GimbalState::kOffline};
  GimbalState last_gimbal_state_{GimbalState::kNormal};
  TickTimer gimbal_tick_;

  // GimbalSubState current_gimbal_sub_state_{GimbalSubState::kNormal};
  // GimbalSubState last_gimbal_sub_state_{GimbalSubState::kNormal};
  TickTimer gimbal_sub_tick_;

  TickTimer test_switch_tick_;

  rm::device::VT03::SwitchPosition tc_switch_position{rm::device::VT03::SwitchPosition::N};
  rm::device::DR16::SwitchPosition rc_switch_position_r{rm::device::DR16::SwitchPosition::kUnknown};
  rm::device::DR16::SwitchPosition rc_switch_position_l{rm::device::DR16::SwitchPosition::kUnknown};

  int waiting_count_{0};  // 应为 1s 的主循环执行次数
  int count_{0};

  int gimbal_power_management_last_flag_{0};
  int ammo_power_management_last_flag_{0};
  int chassis_power_management_last_flag_{0};

  void DT7Switch();
  void GameState();

 public:
  void AmmoSubStateUpdate();

  rm::u16 scope_state{0};
  rm::u16 scope_count{0};

  /**
   *
   * @param now
   * @param last
   * @param tick
   * @return int 0:失能 1:使能 2:使能中
   * @brief 当电管重新上电时，产生延迟，确保在等待使能状态足够久，确保模块成功使能
   */
  int RefereePowerManagement(const int &now, int &last, TickTimer &tick) {
    if (now && !last) {
      if (!Waiting(waiting_count_ * 2, tick)) {  // 延时2s
        return 2;                                // 测试用
      }
      last = now;
      return now;
    } else {
      last = now;
      return now;
    }
  }

  /**
   * @brief Test模式下的拨轮状态切换
   * @tparam T 状态类型
   * @param dial 拨轮
   * @param current 当前状态
   * @param last 上个状态
   * @param target 切换的状态
   */
  template <typename T>
  void TestDialSwitch(const int &dial, T &current, T &last, T target) {
    if (abs(dial) > 400) {
      if (Waiting(waiting_count_ / 2, test_switch_tick_)) {
        if (current != target) {
          last = current;
          current = target;
        } else {
          current = last;
          last = target;
        }
      }
    }
  }
};