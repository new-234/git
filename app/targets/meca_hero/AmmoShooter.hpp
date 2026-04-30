#pragma once

#include "librm.hpp"
#include "globals.hpp"

class AmmoShooter {
 public:
  AmmoShooter() = default;

  void AmmoUpdate();
  void AmmoSubState();

 private:
  f32 now_angle{0};
  f32 next_angle{0};        ///< 初始位
  int waiting_count_{100};  ///< 打弹间隔

  StateMachine::TickTimer ammo_sub_tick_;
};
