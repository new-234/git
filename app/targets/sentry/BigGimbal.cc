#include "BigGimbal.hpp"

#include <cmath>

using namespace rm;

BigGimbal *big_gimbal{nullptr};

namespace {
constexpr float kRadToDeg = 180.0f / static_cast<float>(M_PI);
constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0f;

constexpr uint32_t kGimbalInitTime = 1000;
constexpr float kFollowAngle = -107.07164f;
constexpr float kRotateRate = 1.5f;
constexpr float kWheelRpmRate = 3000.0f;
constexpr uint8_t kAimbotTargetInside = 1U << 0U;

float WrapDegree(float value) {
  return rm::modules::Wrap(value, -180.0f, 180.0f);
}

float ConstrainAround(float value, float center) {
  return rm::modules::Wrap(value, center - 180.0f, center + 180.0f);
}

int16_t ClampCurrent(float value, float limit) {
  return static_cast<int16_t>(rm::modules::Clamp(value, -limit, limit));
}

float FirstOrderFilter(float previous, float input, float alpha) {
  return previous + alpha * (input - previous);
}
}  // namespace

void BigGimbal::Init() {
  special_.real_buffer_limit = 30.0f;
  special_.total_power_kp = 2.80099994e-06f;
  special_.sup_kp = 1.0f;

  pid_.yaw_angle.SetCircular(true).SetCircularCycle(360.0f);
  pid_.follow_w.SetCircular(true).SetCircularCycle(360.0f);

  ConfigureGimbalPidFor(BigGimbalControlMode::kNoForce);
  ConfigureChassisPidFor(BigChassisControlMode::kNoForce);

  UpdateSensorData();
  command_.yaw = imu_.yaw_angle;
}

void BigGimbal::Task1kHz() {
  UpdateSensorData();
  ForceStateUpdate();
  BasicStateUpdate();
  RcStateUpdate();
  ControlStateUpdate();
  ControlPidUpdate();
  CommandUpdate();
  ChassisSourceUpdate();
  ChassisPowerControl();
  SendMotorOutput();
}

void BigGimbal::UpdateSensorData() {
  motor_measure_.yaw_angle = WrapDegree(globals->down_yaw_motor->pos() * kRadToDeg);
  motor_measure_.yaw_speed = globals->down_yaw_motor->vel() * kRadToDeg;
  motor_measure_.wheel_speed[0] = static_cast<float>(globals->wheel_lf->rpm());
  motor_measure_.wheel_speed[1] = static_cast<float>(globals->wheel_rf->rpm());
  motor_measure_.wheel_speed[2] = static_cast<float>(globals->wheel_lb->rpm());
  motor_measure_.wheel_speed[3] = static_cast<float>(globals->wheel_rb->rpm());

  imu_.yaw_angle = WrapDegree(globals->ahrs.euler_angle().yaw * kRadToDeg);
  imu_.yaw_speed = globals->imu->gyro_z() * kRadToDeg;
}

void BigGimbal::ForceStateUpdate() {
  if (!globals->RcOnline()) {
    force_mode_ = BigGimbalForceMode::kNoForce;
    return;
  }

  if (!SwitchUp(rc_ch::SF)) {
    force_mode_ = BigGimbalForceMode::kNoForce;
    return;
  }

  if (!globals->device_big_gimbal.all_device_ok() && globals->robot_mode != RobotMode::kTest) {
    force_mode_ = BigGimbalForceMode::kNoForce;
  } else if (force_mode_ == BigGimbalForceMode::kNoForce) {
    force_mode_ = BigGimbalForceMode::kInit;
    special_.gimbal_init_time = globals->tick;
  } else if (force_mode_ == BigGimbalForceMode::kInit && TimeExceeded(special_.gimbal_init_time, kGimbalInitTime)) {
    force_mode_ = BigGimbalForceMode::kYesForce;
  } else if (force_mode_ != BigGimbalForceMode::kInit) {
    force_mode_ = BigGimbalForceMode::kYesForce;
  }
}

void BigGimbal::BasicStateUpdate() {
  if (SwitchUp(rc_ch::SE)) {
    basic_mode_ = BigGimbalBasicMode::kMatch;
  } else if (SwitchMid(rc_ch::SE)) {
    basic_mode_ = BigGimbalBasicMode::kVision;
  } else if (SwitchDown(rc_ch::SE)) {
    basic_mode_ = BigGimbalBasicMode::kEc;
  }
}

void BigGimbal::RcStateUpdate() {
  if (basic_mode_ != BigGimbalBasicMode::kMatch) {
    rc_mode_ = SwitchMid(rc_ch::SC) ? BigGimbalRcMode::kYes : BigGimbalRcMode::kNo;
  }
}

void BigGimbal::ControlStateUpdate() {
  switch (force_mode_) {
    case BigGimbalForceMode::kYesForce:
      ControlStateYesForceUpdate();
      break;
    case BigGimbalForceMode::kInit:
      control_mode_ = BigGimbalControlMode::kInit;
      break;
    case BigGimbalForceMode::kNoForce:
    default:
      control_mode_ = BigGimbalControlMode::kNoForce;
      chassis_mode_ = BigChassisControlMode::kNoForce;
      break;
  }
}

void BigGimbal::ControlStateYesForceUpdate() {
  switch (basic_mode_) {
    case BigGimbalBasicMode::kMatch:
      if (!globals->device_nuc.all_device_ok()) {
        control_mode_ = BigGimbalControlMode::kRcControl;
        chassis_mode_ = BigChassisControlMode::kSlowRoting;
      } else {
        control_mode_ = BigGimbalControlMode::kNavi;
        chassis_mode_ = BigChassisControlMode::kKeep;
      }
      break;
    case BigGimbalBasicMode::kVision:
      ControlStateVisionUpdate();
      chassis_mode_ = SwitchUp(rc_ch::SD) ? BigChassisControlMode::kNoForce : BigChassisControlMode::kKeep;
      break;
    case BigGimbalBasicMode::kEc:
      ControlStateEcUpdate();
      ChassisControlModeUpdate();
      break;
  }
}

void BigGimbal::ControlStateVisionUpdate() {
  if (SwitchUp(rc_ch::SA)) {
    control_mode_ = BigGimbalControlMode::kRcControl;
  } else if (SwitchMid(rc_ch::SA) || SwitchDown(rc_ch::SA)) {
    control_mode_ = globals->device_nuc.all_device_ok() ? BigGimbalControlMode::kNavi : BigGimbalControlMode::kRcControl;
  } else {
    control_mode_ = BigGimbalControlMode::kNoForce;
  }
}

void BigGimbal::ControlStateEcUpdate() {
  if (SwitchUp(rc_ch::SA) || SwitchMid(rc_ch::SA) || SwitchDown(rc_ch::SA)) {
    control_mode_ = BigGimbalControlMode::kRcControl;
  } else {
    control_mode_ = BigGimbalControlMode::kNoForce;
  }
}

void BigGimbal::ChassisControlModeUpdate() {
  if (SwitchUp(rc_ch::SG)) {
    chassis_mode_ = BigChassisControlMode::kSlowRoting;
  } else if (SwitchMid(rc_ch::SG)) {
    chassis_mode_ = BigChassisControlMode::kFollow;
  } else if (SwitchDown(rc_ch::SG)) {
    chassis_mode_ = BigChassisControlMode::kQuickRoting;
  }

  if (SwitchUp(rc_ch::SD)) {
    chassis_mode_ = BigChassisControlMode::kNoForce;
  }
}

void BigGimbal::ControlPidUpdate() {
  if (last_control_mode_ != control_mode_) {
    ConfigureGimbalPidFor(control_mode_);
    if (last_control_mode_ == BigGimbalControlMode::kNoForce) {
      command_.yaw = imu_.yaw_angle;
    }
    last_control_mode_ = control_mode_;
  }

  if (last_chassis_mode_ != chassis_mode_) {
    ConfigureChassisPidFor(chassis_mode_);
    last_chassis_mode_ = chassis_mode_;
  }
}

void BigGimbal::CommandUpdate() {
  switch (control_mode_) {
    case BigGimbalControlMode::kNoForce:
      command_.yaw = imu_.yaw_angle;
      output_.yaw = 0.0f;
      source_.vx = 0.0f;
      source_.vy = 0.0f;
      output_.wheel_motor = {};
      break;

    case BigGimbalControlMode::kInit:
      command_.yaw = imu_.yaw_angle;
      output_.yaw = 0.0f;
      break;

    case BigGimbalControlMode::kRcControl:
      if (rc_mode_ == BigGimbalRcMode::kYes) {
        command_.yaw += globals->wfly_et16s->RemoteChannalLeftX() * 0.25f;
      }
      command_.yaw = ConstrainAround(command_.yaw, imu_.yaw_angle);
      pid_.yaw_angle.Update(command_.yaw, imu_.yaw_angle, 0.001f);
      pid_.yaw_speed.Update(pid_.yaw_angle.out(), imu_.yaw_speed, 0.001f);
      output_.yaw = pid_.yaw_speed.out();
      source_.vx = globals->wfly_et16s->RemoteChannalRightX();
      source_.vy = globals->wfly_et16s->RemoteChannalRightY();
      break;

    case BigGimbalControlMode::kNavi:
      if (globals->navigate_communicator->scan_mode() == 1U ||
          (globals->aimbot_communicator->aimbot_state() & kAimbotTargetInside) != 0U) {
        command_.yaw = FirstOrderFilter(command_.yaw, globals->navigate_communicator->target_yaw_speed(), 0.15f);
      }
      command_.yaw = ConstrainAround(command_.yaw, imu_.yaw_angle);
      pid_.yaw_angle.Update(command_.yaw, imu_.yaw_angle, 0.001f);
      pid_.yaw_speed.Update(pid_.yaw_angle.out(), imu_.yaw_speed, 0.001f);
      output_.yaw = pid_.yaw_speed.out();
      source_.vx = globals->navigate_communicator->chassis_target_y();
      source_.vy = -globals->navigate_communicator->chassis_target_x();
      break;

    case BigGimbalControlMode::kKeep:
      command_.yaw = ConstrainAround(command_.yaw, imu_.yaw_angle);
      pid_.yaw_angle.Update(command_.yaw, imu_.yaw_angle, 0.001f);
      pid_.yaw_speed.Update(pid_.yaw_angle.out(), imu_.yaw_speed, 0.001f);
      output_.yaw = pid_.yaw_speed.out();
      break;
  }
}

void BigGimbal::ChassisSourceUpdate() {
  special_.differ_angle = WrapDegree(motor_measure_.yaw_angle - kFollowAngle) * kDegToRad;
  special_.follow_angle = ConstrainAround(kFollowAngle, motor_measure_.yaw_angle);

  switch (chassis_mode_) {
    case BigChassisControlMode::kNoForce:
      set_.wz = 0.0f;
      output_.wheel_motor = {};
      break;
    case BigChassisControlMode::kInit:
      set_.wz = 0.0f;
      break;
    case BigChassisControlMode::kFollow:
      pid_.follow_w.Update(special_.follow_angle, motor_measure_.yaw_angle, 0.001f);
      set_.wz = rm::modules::Clamp(pid_.follow_w.out(), -1.0f, 1.0f);
      break;
    case BigChassisControlMode::kSlowRoting:
      set_.wz = 0.5f;
      break;
    case BigChassisControlMode::kQuickRoting:
      set_.wz = -1.0f;
      break;
    case BigChassisControlMode::kRandomRoting:
      set_.wz = -0.8f;
      break;
    case BigChassisControlMode::kKeep:
      set_.wz = globals->navigate_communicator->chassis_target_w();
      break;
  }

  special_.delta_bias = 0.06f * motor_measure_.yaw_speed;
  const float angle = special_.differ_angle + special_.delta_bias;
  set_.vx = source_.vx * std::cos(angle) + source_.vy * std::sin(angle);
  set_.vy = -source_.vx * std::sin(angle) + source_.vy * std::cos(angle);

  if (chassis_mode_ == BigChassisControlMode::kNoForce || chassis_mode_ == BigChassisControlMode::kInit) {
    output_.wheel_motor = {};
    return;
  }

  command_.wheel_motor[0] = (-set_.vx + set_.vy + set_.wz * kRotateRate) * kWheelRpmRate;
  command_.wheel_motor[1] = (set_.vx - set_.vy + set_.wz * kRotateRate) * kWheelRpmRate;
  command_.wheel_motor[2] = (set_.vx + set_.vy + set_.wz * kRotateRate) * kWheelRpmRate;
  command_.wheel_motor[3] = (-set_.vx - set_.vy + set_.wz * kRotateRate) * kWheelRpmRate;

  for (size_t i = 0; i < output_.wheel_motor.size(); ++i) {
    pid_.wheel_speed[i].Update(command_.wheel_motor[i], motor_measure_.wheel_speed[i], 0.001f);
    output_.wheel_motor[i] = ClampCurrent(pid_.wheel_speed[i].out(), 16384.0f);
  }
}

void BigGimbal::ChassisPowerControl() {
  float power_limit = static_cast<float>(globals->referee_data->data().robot_status.chassis_power_limit);
  if (power_limit <= 0.0f) {
    power_limit = 100.0f;
  }

  float buffer_energy = globals->referee_data->data().power_heat_data.buffer_energy;
  if (globals->robot_mode == RobotMode::kTest && buffer_energy <= 0.0f) {
    buffer_energy = 60.0f;
  }

  if (buffer_energy < special_.real_buffer_limit) {
    special_.real_power_limit = buffer_energy;
  } else {
    special_.real_power_limit = 100.0f;
  }

  special_.total_power = 0.0f;
  for (size_t i = 0; i < output_.wheel_motor.size(); ++i) {
    special_.total_power +=
        std::abs(static_cast<float>(output_.wheel_motor[i]) * motor_measure_.wheel_speed[i]) * special_.total_power_kp;
  }

  if (special_.total_power <= special_.real_power_limit || special_.total_power <= 0.0f) {
    special_.sup_kp = 1.0f;
    return;
  }

  special_.sup_kp = special_.real_power_limit / special_.total_power;
  if (chassis_mode_ == BigChassisControlMode::kFollow) {
    if (power_limit < 90.0f && power_limit > 58.0f) {
      special_.sup_kp *= 1.2f;
    } else if (power_limit > 90.0f) {
      special_.sup_kp *= 1.5f;
    }
  } else if (chassis_mode_ == BigChassisControlMode::kQuickRoting ||
             chassis_mode_ == BigChassisControlMode::kRandomRoting ||
             chassis_mode_ == BigChassisControlMode::kSlowRoting) {
    if (power_limit < 90.0f && power_limit > 58.0f) {
      special_.sup_kp *= 1.3f;
    } else if (power_limit > 90.0f) {
      special_.sup_kp *= 1.7f;
    }
  }

  special_.sup_kp = rm::modules::Clamp(special_.sup_kp, 0.0f, 1.0f);
  for (auto &wheel_output : output_.wheel_motor) {
    wheel_output = ClampCurrent(static_cast<float>(wheel_output) * special_.sup_kp, 16384.0f);
  }
}

bool BigGimbal::EnableYawMotor(bool enable) {
  const auto status = globals->down_yaw_motor->status();
  if (++dm_instruction_tick_ < 100U) {
    return status == 0x01;
  }
  dm_instruction_tick_ = 0;

  if (enable && status == 0x00) {
    globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
  } else if (enable && status == 0x0d) {
    globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  } else if (enable && status != 0x01) {
    globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  } else if (!enable && status == 0x01) {
    globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
  }
  return status == 0x01;
}

void BigGimbal::SendMotorOutput() {
  const bool enable = control_mode_ != BigGimbalControlMode::kNoForce;
  const bool yaw_enabled = EnableYawMotor(enable);
  if (!enable || !yaw_enabled) {
    output_ = {};
  }

  if ((globals->tick & 1U) == 0U) {
    globals->down_yaw_motor->SetMitCommand(0.0f, 0.0f, rm::modules::Clamp(-output_.yaw / 10000.0f, -10.0f, 10.0f),
                                           0.0f, 0.0f);
  }
  globals->wheel_lf->SetCurrent(output_.wheel_motor[0]);
  globals->wheel_rf->SetCurrent(output_.wheel_motor[1]);
  globals->wheel_lb->SetCurrent(output_.wheel_motor[2]);
  globals->wheel_rb->SetCurrent(output_.wheel_motor[3]);
}

void BigGimbal::ConfigureGimbalPidFor(BigGimbalControlMode mode) {
  float yaw_angle_kp = 0.0f;
  float yaw_angle_kd = 0.0f;
  float yaw_speed_kp = 0.0f;
  float yaw_speed_kd = 0.0f;

  switch (mode) {
    case BigGimbalControlMode::kInit:
      yaw_angle_kp = 20.0f;
      yaw_angle_kd = 40.0f;
      yaw_speed_kp = 150.0f;
      yaw_speed_kd = 10.0f;
      break;
    case BigGimbalControlMode::kRcControl:
    case BigGimbalControlMode::kKeep:
      yaw_angle_kp = 16.0f;
      yaw_angle_kd = 1000.0f;
      yaw_speed_kp = 520.0f;
      break;
    case BigGimbalControlMode::kNavi:
      yaw_angle_kp = 22.0f;
      yaw_angle_kd = 1000.0f;
      yaw_speed_kp = 530.0f;
      break;
    case BigGimbalControlMode::kNoForce:
    default:
      break;
  }

  pid_.yaw_angle.SetKp(yaw_angle_kp).SetKi(0.0f).SetKd(yaw_angle_kd).SetMaxOut(250.0f).SetMaxIout(30.0f);
  pid_.yaw_speed.SetKp(yaw_speed_kp).SetKi(0.0f).SetKd(yaw_speed_kd).SetMaxOut(50000.0f).SetMaxIout(10000.0f);
  pid_.yaw_angle.SetDiffLpfAlpha(1.0f);
  pid_.yaw_speed.SetDiffLpfAlpha(1.0f);
}

void BigGimbal::ConfigureChassisPidFor(BigChassisControlMode mode) {
  const bool enable = mode != BigChassisControlMode::kNoForce && mode != BigChassisControlMode::kInit;
  for (auto &wheel_pid : pid_.wheel_speed) {
    wheel_pid.SetKp(enable ? 5.0f : 0.0f)
        .SetKi(0.0f)
        .SetKd(enable ? 0.3f : 0.0f)
        .SetMaxOut(16384.0f)
        .SetMaxIout(6000.0f)
        .SetDiffLpfAlpha(1.0f);
  }

  const bool follow_enable = mode != BigChassisControlMode::kNoForce && mode != BigChassisControlMode::kInit;
  pid_.follow_w.SetKp(follow_enable ? 0.1f : 0.0f)
      .SetKi(0.0f)
      .SetKd(0.0f)
      .SetMaxOut(300.0f)
      .SetMaxIout(25.0f)
      .SetDiffLpfAlpha(1.0f);
}

bool BigGimbal::TimeExceeded(uint32_t since, uint32_t duration_ms) const { return globals->tick - since >= duration_ms; }

bool BigGimbal::SwitchDown(int channel) const {
  return globals->wfly_et16s->SwitchIsDown(channel);
}

bool BigGimbal::SwitchMid(int channel) const {
  return globals->wfly_et16s->SwitchIsMid(channel);
}

bool BigGimbal::SwitchUp(int channel) const {
  return globals->wfly_et16s->SwitchIsUp(channel);
}
