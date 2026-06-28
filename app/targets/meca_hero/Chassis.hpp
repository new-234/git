#pragma once

#include "globals.hpp"

class ChassisHero {
 public:
  ChassisHero() = default;

  void ChassisUpdate();

 private:
  rm::i16 Vw{0};       ///< 旋转速度
  rm::i16 Vy{0};       ///< 直线速度
  rm::i16 Vx{0};       ///< 平移速度
  rm::i16 V_wheel[4];  ///< 各个轮子的速度
  rm::i16 V_w{0};
  rm::i16 V_s{0};
  rm::i16 V_a{0};
  rm::i16 V_d{0};

  float power_limit{0};
  int overpower_count{0};

  bool EnableJudgement();
  void Unabling();
  void ChassisControl();
  void IfFollow();
  void Remote();
  void IfOverPower();
  void MotorUpdate();
};
