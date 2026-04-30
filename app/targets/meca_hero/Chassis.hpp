#pragma once

#include "globals.hpp"

class ChassisHero {
 public:
  ChassisHero() = default;

  void ChassisUpdate();

 private:
  rm::i16 Vw;  ///< 旋转速度
  rm::i16 Vy;  ///< 直线速度

  float power_limit{0};
  int overpower_count{0};

  bool EnableJudgement();
  void Unabling();
  void ChassisControl();
  void IfFollow();
  void IfOverPower();
  void Remote();
};
