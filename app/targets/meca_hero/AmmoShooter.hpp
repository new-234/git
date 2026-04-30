#pragma once

#include "State.hpp"
#include "globals.hpp"

class AmmoHero {
 public:
  AmmoHero() = default;

  void AmmoUpdate();

 private:
  rm::i16 limit{-3000};
  int16_t shooter_m{0};
  rm::i16 V_shooter_1 = -4605;
  rm::i16 V_shooter_2 = -3770;  // 12m/s

  bool EnableJudgement();
  void Unabling();
  void Enabling();
  void AmmoControl();
  void ShooterControl();
  void MagzineControl();
};
