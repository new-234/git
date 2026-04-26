#pragma once

#include "State.hpp"
#include "librm.hpp"
#include "globals.hpp"

class GimbalHero {
 public:
  GimbalHero() = default;

  void GimbalUpdate();

  // 监测变量
  struct {
    float euler_yaw_{0};
    float euler_pitch_{0};
    float euler_roll_{0};
  } monitor_;

 private:
  // 调参变量
  struct {
    rm::f32 gyro_rectification_{};  // 陀螺仪Z轴修正值
    int pitch_ff_{};                // pitch轴PID前馈
  } hyperparameters_;
  // 遥控值
  float target_yaw;
  float target_pitch;

  bool EnableJudgement();  //
  void Unabling();
  void Enabling();
  void AhrsUpdate();
  void AimbotControl();
};