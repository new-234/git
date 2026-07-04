#ifndef APP_TARGETS_SENTRY_GIMBAL_HPP
#define APP_TARGETS_SENTRY_GIMBAL_HPP

#include <librm.hpp>

#include "main.hpp"

class Gimbal {
 public:
  void Init();
  void Task1kHz();

  [[nodiscard]] float SmallYawDegree() const;
  [[nodiscard]] float SmallPitchDegree() const;

 private:
  void UpdateMode();
  void UpdateTargets();
  void UpdateRemoteTarget();
  void UpdateScanTarget();
  void UpdateAimbotTarget();
  void UpdateControl();
  void UpdateShoot();
  void EnableDmMotors(bool enable);
  void SetZeroOutput();
  void SendMotorOutput();

  GimbalMode mode_{GimbalMode::kNoForce};
  GimbalMode last_mode_{GimbalMode::kNoForce};

  float down_yaw_target_{0.0f};
  float small_yaw_target_{0.0f};
  float pitch_target_{0.0f};

  float ammo_speed_{5500.0f};
  float last_shoot_speed_{22.5f};
  uint16_t single_shoot_lock_{0};
  uint16_t dm_instruction_tick_{0};
  uint32_t scan_tick_{0};
  bool single_shoot_started_{false};
};

extern Gimbal *gimbal;

#endif  // APP_TARGETS_SENTRY_GIMBAL_HPP
