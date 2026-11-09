#ifndef APP_TARGETS_SENTRY_SMALL_GIMBAL_HPP
#define APP_TARGETS_SENTRY_SMALL_GIMBAL_HPP

#include <cstdint>

#include <librm.hpp>

#include "main.hpp"

enum class SmallForceMode : uint8_t {
  kNoForce = 0,
  kInit,
  kYesForce,
};

enum class SmallRcMode : uint8_t {
  kNo = 0,
  kYes,
};

enum class SmallBasicMode : uint8_t {
  kMatch = 0,
  kVision,
  kEc,
};

enum class SmallAmmoMode : uint8_t {
  kNoForce = 0,
  kNoSpeed,
  kNormalSpeed,
};

enum class SmallRotorMode : uint8_t {
  kNoForce = 0,
  kStop,
  kForward,
  kSingle,
};

enum class SmallControlMode : uint8_t {
  kNoForce = 0,
  kInit,
  kRcControl,
  kScan,
  kAutoAmmo,
  kSmallRangeScan,
  kFullRangeScan,
  kTHighlandScan,
  kOutpostBaseScan,
  kOmnidirectionalPerception,
  kMachineZero,
  kKeepMachineAngle,
  kOutpostFinished,
};

struct SmallGimbalMotorMeasure {
  float yaw_angle{};
  float yaw_speed{};
  float pitch_angle{};
  float pitch_speed{};
  float ammo_left_speed{};
  float ammo_right_speed{};
  float rotor_speed{};
};

struct SmallGimbalImu {
  float yaw_angle{};
  float pitch_angle{};
  float roll_angle{};
  float yaw_speed{};
  float pitch_speed{};
  float roll_speed{};
};

struct SmallGimbalCommand {
  float yaw{};
  float pitch{};
  float left{};
  float right{};
  float rotor{};
};

struct SmallGimbalOutput {
  int16_t yaw{};
  float pitch{};
  int16_t left{};
  int16_t right{};
  int16_t rotor{};
};

struct SmallGimbalDebugSnapshot {
  uint8_t force_mode{};
  uint8_t basic_mode{};
  uint8_t control_mode{};
  uint8_t rc_mode{};
  uint8_t ammo_mode{};
  uint8_t rotor_mode{};

  float yaw_angle_kp{};
  float yaw_angle_ki{};
  float yaw_angle_kd{};
  float yaw_speed_kp{};
  float yaw_speed_ki{};
  float yaw_speed_kd{};
  float yaw_target_degree{};
  float yaw_current_degree{};
  float yaw_angle_feedback_degree{};
  float yaw_motor_degree{};
  float yaw_angle_error_degree{};
  float yaw_rate_target_dps{};
  float yaw_rate_current_dps{};
  float yaw_rate_feedback_dps{};
  float yaw_rate_error_dps{};
  float yaw_angle_p_out{};
  float yaw_angle_i_out{};
  float yaw_angle_d_out{};
  float yaw_speed_p_out{};
  float yaw_speed_i_out{};
  float yaw_speed_d_out{};
  float yaw_speed_output{};
  float yaw_motor_output{};
  float yaw_motor_rate_dps{};

  float pitch_angle_kp{};
  float pitch_angle_ki{};
  float pitch_angle_kd{};
  float pitch_speed_kp{};
  float pitch_speed_ki{};
  float pitch_speed_kd{};
  float pitch_target_degree{};
  float pitch_current_degree{};
  float pitch_angle_feedback_degree{};
  float pitch_motor_degree{};
  float pitch_angle_error_degree{};
  float pitch_rate_target_dps{};
  float pitch_rate_current_dps{};
  float pitch_rate_feedback_dps{};
  float pitch_rate_error_dps{};
  float pitch_angle_p_out{};
  float pitch_angle_i_out{};
  float pitch_angle_d_out{};
  float pitch_speed_p_out{};
  float pitch_speed_i_out{};
  float pitch_speed_d_out{};
  float pitch_speed_output{};
  float pitch_motor_output{};
  float pitch_motor_rate_dps{};

  float rc_left_x{};
  float rc_left_y{};
};

struct SmallGimbalScan {
  float yaw_left{};
  float yaw_right{};
  float yaw_start{};
  float yaw_a{};
  float yaw_bias{};
  float yaw_speed{};
  float pitch_up{};
  float pitch_down{};
  float pitch_start{};
  float pitch_a{};
  float pitch_bias{};
  uint32_t time{};
  bool flag{};
};

struct SmallGimbalLimit {
  float left_delta{};
  float right_delta{};
  float yaw_left{};
  float yaw_right{};
  float pitch_up{};
  float pitch_down{};
};

struct SmallGimbalSpecial {
  uint8_t match_mode_flag{};
  uint32_t match_mode_time{};
  uint32_t small_gimbal_init_time{};
  uint32_t auto_ammo_delay_time{};
  uint32_t rotor_mode_time{};
  uint32_t rotor_single_time{};
  uint8_t single_ammo_flag{};
  float real_ammo_speed{5500.0f};
  float last_initial_speed{};
};

struct SmallGimbalPid {
  rm::modules::PID yaw_angle;
  rm::modules::PID yaw_speed;
  rm::modules::PID pitch_angle;
  rm::modules::PID pitch_speed;
  rm::modules::PID left_speed;
  rm::modules::PID right_speed;
  rm::modules::PID rotor_speed;
};

class SmallGimbal {
 public:
  void Init();
  void Task1kHz();

  [[nodiscard]] float YawForNavigateDegree() const;
  [[nodiscard]] float PitchForNavigateDegree() const;
  [[nodiscard]] SmallGimbalDebugSnapshot DebugSnapshot() const;

 private:
  void UpdateSensorData();
  void ForceStateUpdate();
  void BasicStateUpdate();
  void RcStateUpdate();
  void ControlStateUpdate();
  void ControlStateYesForceUpdate();
  void ControlStateMatchUpdate();
  void ControlStateVisionUpdate();
  void ControlStateEcUpdate();
  void ControlPidUpdate();
  void AmmoPidUpdate();
  void RotorPidUpdate();
  void ResetFlag();
  void LimitUpdate();
  void CommandUpdate();
  void ShootCommandUpdate();
  void ScanInit(float yaw_left, float yaw_right, float pitch_up, float pitch_down);
  void ScanControl();
  void CommandLimit();
  void BulletSpeedLimit();
  [[nodiscard]] bool EnablePitchMotor(bool enable);
  void SendMotorOutput();

  void ConfigureGimbalPidFor(SmallControlMode mode);
  void ConfigureAmmoPidFor(SmallAmmoMode mode);
  void ConfigureRotorPidFor(SmallRotorMode mode);

  [[nodiscard]] bool TimeExceeded(uint32_t since, uint32_t duration_ms) const;
  [[nodiscard]] bool SwitchDown(int channel) const;
  [[nodiscard]] bool SwitchMid(int channel) const;
  [[nodiscard]] bool SwitchUp(int channel) const;

  SmallGimbalMotorMeasure motor_measure_{};
  SmallGimbalImu imu_{};
  SmallGimbalCommand command_{};
  SmallGimbalOutput output_{};
  SmallGimbalPid pid_{};
  SmallGimbalScan scan_{};
  SmallGimbalLimit limit_{};
  SmallGimbalSpecial special_{};

  SmallForceMode force_mode_{SmallForceMode::kNoForce};
  SmallRcMode rc_mode_{SmallRcMode::kNo};
  SmallBasicMode basic_mode_{SmallBasicMode::kMatch};
  SmallAmmoMode ammo_mode_{SmallAmmoMode::kNoForce};
  SmallAmmoMode last_ammo_mode_{SmallAmmoMode::kNoForce};
  SmallRotorMode rotor_mode_{SmallRotorMode::kNoForce};
  SmallRotorMode last_rotor_mode_{SmallRotorMode::kNoForce};
  SmallControlMode control_mode_{SmallControlMode::kNoForce};
  SmallControlMode last_control_mode_{SmallControlMode::kNoForce};
  uint16_t dm_instruction_tick_{};
};

extern SmallGimbal *small_gimbal;

#endif  // APP_TARGETS_SENTRY_SMALL_GIMBAL_HPP
