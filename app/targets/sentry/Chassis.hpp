#ifndef APP_TARGETS_SENTRY_CHASSIS_HPP
#define APP_TARGETS_SENTRY_CHASSIS_HPP

#include <array>

#include <librm.hpp>

#include "main.hpp"

class Chassis {
 public:
  void Init();
  void Task1kHz();

 private:
  void UpdateMode();
  void UpdateSource();
  void UpdateRemoteSource();
  void UpdateNavigateSource();
  void UpdateKinematics();
  void UpdateControl();
  void ApplyPowerLimit();
  void SetMotorCurrent();
  void SetZeroOutput();

  ChassisMode mode_{ChassisMode::kNoForce};
  rm::modules::PID follow_pid_{};
  rm::modules::MotorPowerModel power_model_{rm::modules::MotorPowerModel::MotorType::kM3508};
  std::array<rm::modules::MotorPowerModel::MotorState, 4> motor_state_{};
  std::array<rm::modules::MotorPowerModel::PowerInfo, 4> power_info_{};

  float source_vx_{0.0f};
  float source_vy_{0.0f};
  float source_wz_{0.0f};
  float target_vx_{0.0f};
  float target_vy_{0.0f};
  float target_wz_{0.0f};
  float output_current_[4]{};
  float power_limit_{60.0f};
};

extern Chassis *chassis;

#endif  // APP_TARGETS_SENTRY_CHASSIS_HPP
