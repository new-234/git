#pragma once

#include "globals.hpp"

class ChassisHero {
 public:
  ChassisHero() = default;

  void ChassisUpdate();

 private:
  bool EnableJudgement();
  void Unabling();
  void Enabling();
};
