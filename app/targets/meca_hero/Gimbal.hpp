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
    float gyro_x_{0};
    float gyro_y_{0};
    float gyro_z_{0};
  } monitor_;

  struct {
    float euler_yaw_{0};
    float euler_pitch_{0};
    float euler_roll_{0};
  } temp_euler_angle_;

 private:
  // 调参变量
  struct {
    rm::f32 gyro_rectification_{0.0};  // 陀螺仪Z轴修正值
    int pitch_ff_{0};                  // pitch轴PID前馈
  } hyperparameters_;
  // 遥控值
  float target_yaw_;
  float target_pitch_;
  float servo_angle_{1410};  // 1410--1730

  float last_pitch_{0};
  float last_target_pitch_{0};
  float negaive_ff_{0};
  float positive_ff_{0};

  bool EnableJudgement();
  void Unabling();
  void Enabling();
  void AhrsUpdate();
  void GimbalControl();
  void AimbotControl();
  void RadarAimbotControl();
  void DroneAimControl();
  void RemoterControl();
  void NormalRemoter();
  void SnipeRemoter();
  void MotorUpdate();
  void ServoMotorUpdate();
  void ScopeMotorUpdate();
};