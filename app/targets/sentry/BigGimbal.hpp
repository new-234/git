#ifndef APP_TARGETS_SENTRY_BIG_GIMBAL_HPP
#define APP_TARGETS_SENTRY_BIG_GIMBAL_HPP

#include <array>
#include <cstdint>

#include <librm.hpp>

#include "main.hpp"

enum class BigGimbalForceMode : uint8_t {
  kNoForce = 0,
  kInit,
  kYesForce,
};

enum class BigGimbalRcMode : uint8_t {
  kNo = 0,
  kYes,
};

enum class BigGimbalBasicMode : uint8_t {
  kMatch = 0,
  kVision,
  kEc,
};

enum class BigGimbalControlMode : uint8_t {
  kNoForce = 0,
  kInit,
  kRcControl,
  kNavi,
  kKeep,
};

enum class BigChassisControlMode : uint8_t {
  kNoForce = 0,
  kInit,
  kFollow,
  kSlowRoting,
  kQuickRoting,
  kRandomRoting,
  kKeep,
};

struct BigGimbalMotorMeasure {
  float yaw_angle{};
  float yaw_speed{};
  std::array<float, 4> wheel_speed{};
};

struct BigGimbalImu {
  float yaw_angle{};
  float yaw_speed{};
};

struct BigGimbalCommand {
  float yaw{};
  std::array<float, 4> wheel_motor{};
};

struct BigGimbalOutput {
  float yaw{};
  std::array<int16_t, 4> wheel_motor{};
};

struct BigChassisVector {
  float vx{};
  float vy{};
  float wz{};
};

struct BigGimbalSpecial {
  uint32_t gimbal_init_time{};
  float follow_angle{};
  float differ_angle{};
  float delta_bias{};
  float real_buffer_limit{30.0f};
  float real_power_limit{};
  float total_power{};
  float total_power_kp{2.80099994e-06f};
  float sup_kp{1.0f};
};

struct BigGimbalPid {
  rm::modules::PID yaw_angle;
  rm::modules::PID yaw_speed;
  std::array<rm::modules::PID, 4> wheel_speed;
  rm::modules::PID follow_w;
};

class BigGimbal {
 public:
  void Init();
  void Task1kHz();

 private:
  void UpdateSensorData();
  void ForceStateUpdate();
  void BasicStateUpdate();
  void RcStateUpdate();
  void ControlStateUpdate();
  void ControlStateYesForceUpdate();
  void ControlStateVisionUpdate();
  void ControlStateEcUpdate();
  void ChassisControlModeUpdate();
  void ControlPidUpdate();
  void CommandUpdate();
  void ChassisSourceUpdate();
  void ChassisPowerControl();
  void EnableYawMotor(bool enable);
  void SendMotorOutput();

  void ConfigureGimbalPidFor(BigGimbalControlMode mode);
  void ConfigureChassisPidFor(BigChassisControlMode mode);

  [[nodiscard]] bool TimeExceeded(uint32_t since, uint32_t duration_ms) const;
  [[nodiscard]] bool SwitchDown(int channel) const;
  [[nodiscard]] bool SwitchMid(int channel) const;
  [[nodiscard]] bool SwitchUp(int channel) const;

  BigGimbalMotorMeasure motor_measure_{};
  BigGimbalImu imu_{};
  BigGimbalCommand command_{};
  BigGimbalOutput output_{};
  BigChassisVector source_{};
  BigChassisVector set_{};
  BigGimbalSpecial special_{};
  BigGimbalPid pid_{};

  BigGimbalForceMode force_mode_{BigGimbalForceMode::kNoForce};
  BigGimbalRcMode rc_mode_{BigGimbalRcMode::kNo};
  BigGimbalBasicMode basic_mode_{BigGimbalBasicMode::kEc};
  BigGimbalControlMode control_mode_{BigGimbalControlMode::kNoForce};
  BigGimbalControlMode last_control_mode_{BigGimbalControlMode::kNoForce};
  BigChassisControlMode chassis_mode_{BigChassisControlMode::kNoForce};
  BigChassisControlMode last_chassis_mode_{BigChassisControlMode::kNoForce};
  uint16_t dm_instruction_tick_{};
};

extern BigGimbal *big_gimbal;

#endif  // APP_TARGETS_SENTRY_BIG_GIMBAL_HPP
