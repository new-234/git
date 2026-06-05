#pragma once

#include "State.hpp"
#include "globals.hpp"

class AmmoHero {
 public:
  AmmoHero() = default;

  void AmmoUpdate();

  rm::i16 limit{-1800};
  int16_t shooter_m{0};

  //
  rm::i16 v12_shooter_1 = -4200;
  rm::i16 v12_shooter_2 = -3550;

  rm::i16 v16_shooter_1 = -5800;  // -6170
  rm::i16 v16_shooter_2 = -4800; // -4885
  // rm::i16 V_shooter_1 = -0;
  // rm::i16 V_shooter_2 = -0;

 private:
  bool EnableJudgement();
  void Unabling();
  void Enabling();
  void AmmoControl();
  void ShooterControl();
  void MagzineControl();
  void BulletSpeedControl();
};
