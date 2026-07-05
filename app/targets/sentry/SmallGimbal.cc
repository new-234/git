#include "SmallGimbal.hpp"

#include <cmath>

using namespace rm;

SmallGimbal *small_gimbal{nullptr};

namespace {
constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0f;
constexpr float kRadToDeg = 180.0f / static_cast<float>(M_PI);
constexpr float kRpmToRadPerSec = static_cast<float>(M_PI) * 2.0f / 60.0f;

constexpr float kPitchUpAngle = 33.0f;
constexpr float kPitchDownAngle = -30.0f;
constexpr float kYawMachineLeft = -60.0f;
constexpr float kYawMachineRight = 170.0f;
constexpr float kSmallRangeLeftAngle = 20.0f;
constexpr float kSmallRangeRightAngle = 20.0f;
constexpr float kPitchScanUpAngle = 20.0f;
constexpr float kPitchScanDownAngle = -10.0f;
constexpr float kYawMachineZero = -121.0f;
constexpr float kDeltaPitch = 0.0030f;
constexpr float kDeltaYaw = 3.5f;

constexpr uint32_t kGimbalInitTime = 1000;
constexpr uint32_t kAutoAmmoToSmallRangeTime = 200;
constexpr uint32_t kSmallRangeToFullRangeTime = 1500;
constexpr uint32_t kAutoShootTime = 30;
constexpr uint32_t kSingleAmmoTime = 30;
constexpr uint32_t kSingleAmmoDelayTime = 1000;

constexpr uint8_t kAimbotTargetInside = 1U << 0U;
constexpr uint8_t kAimbotShootRequest = 1U << 1U;

constexpr float kAmmoSpeedSet = 5500.0f;
constexpr float kRotorSpeedSet = 2000.0f;

float WrapDegree(float value) {
  return rm::modules::Wrap(value, -180.0f, 180.0f);
}

float ConstrainAround(float value, float center) {
  return rm::modules::Wrap(value, center - 180.0f, center + 180.0f);
}

int16_t ClampCurrent(float value, float limit) {
  return static_cast<int16_t>(rm::modules::Clamp(value, -limit, limit));
}
}  // namespace

void SmallGimbal::Init() {
  limit_.pitch_up = kPitchUpAngle;
  limit_.pitch_down = kPitchDownAngle;
  special_.real_ammo_speed = kAmmoSpeedSet;

  pid_.yaw_angle.SetCircular(true).SetCircularCycle(360.0f);
  pid_.pitch_angle.SetCircular(false);

  ConfigureGimbalPidFor(SmallControlMode::kNoForce);
  ConfigureAmmoPidFor(SmallAmmoMode::kNoForce);
  ConfigureRotorPidFor(SmallRotorMode::kNoForce);

  UpdateSensorData();
  command_.yaw = imu_.yaw_angle;
  command_.pitch = imu_.pitch_angle;
}

void SmallGimbal::Task1kHz() {
  UpdateSensorData();
  ForceStateUpdate();
  BasicStateUpdate();
  RcStateUpdate();
  ControlStateUpdate();
  ControlPidUpdate();
  RotorPidUpdate();
  AmmoPidUpdate();
  ResetFlag();
  LimitUpdate();
  CommandUpdate();
  ShootCommandUpdate();
  SendMotorOutput();
}

float SmallGimbal::YawForNavigateDegree() const { return WrapDegree(motor_measure_.yaw_angle + 120.0f); }

float SmallGimbal::PitchForNavigateDegree() const { return imu_.pitch_angle; }

void SmallGimbal::UpdateSensorData() {
  motor_measure_.yaw_angle = WrapDegree(globals->up_yaw_motor->pos_degree());
  motor_measure_.yaw_speed = static_cast<float>(globals->up_yaw_motor->rpm()) * 6.0f;
  motor_measure_.pitch_angle = globals->pitch_motor->pos() * kRadToDeg;
  motor_measure_.pitch_speed = globals->pitch_motor->vel() * kRadToDeg;
  motor_measure_.ammo_left_speed = static_cast<float>(globals->friction_left->rpm());
  motor_measure_.ammo_right_speed = static_cast<float>(globals->friction_right->rpm());
  motor_measure_.rotor_speed = static_cast<float>(globals->dial_motor->rpm());

  imu_.yaw_angle = WrapDegree(globals->hipnuc_imu->yaw() * kRadToDeg);
  imu_.pitch_angle = globals->hipnuc_imu->pitch() * kRadToDeg;
  imu_.roll_angle = globals->hipnuc_imu->roll() * kRadToDeg;
  imu_.yaw_speed = globals->hipnuc_imu->gyro_z() * kRadToDeg;
  imu_.pitch_speed = globals->hipnuc_imu->gyro_x() * kRadToDeg;
  imu_.roll_speed = globals->hipnuc_imu->gyro_y() * kRadToDeg;
}

void SmallGimbal::ForceStateUpdate() {
  if (!globals->RcOnline()) {
    force_mode_ = SmallForceMode::kNoForce;
    return;
  }

  if (SwitchUp(rc_ch::SF)) {
    if (!globals->device_small_gimbal.all_device_ok() && globals->robot_mode != RobotMode::kTest) {
      force_mode_ = SmallForceMode::kNoForce;
    } else if (force_mode_ == SmallForceMode::kNoForce) {
      force_mode_ = SmallForceMode::kInit;
      special_.small_gimbal_init_time = globals->tick;
    } else if (force_mode_ == SmallForceMode::kInit && TimeExceeded(special_.small_gimbal_init_time, kGimbalInitTime)) {
      force_mode_ = SmallForceMode::kYesForce;
    } else if (force_mode_ != SmallForceMode::kInit) {
      force_mode_ = SmallForceMode::kYesForce;
    }
  } else {
    force_mode_ = SmallForceMode::kNoForce;
  }
}

void SmallGimbal::BasicStateUpdate() {
  if (SwitchUp(rc_ch::SE)) {
    basic_mode_ = SmallBasicMode::kMatch;
  } else if (SwitchMid(rc_ch::SE)) {
    basic_mode_ = SmallBasicMode::kVision;
  } else if (SwitchDown(rc_ch::SE)) {
    basic_mode_ = SmallBasicMode::kEc;
  }
}

void SmallGimbal::RcStateUpdate() {
  if (basic_mode_ == SmallBasicMode::kMatch) {
    return;
  }
  rc_mode_ = SwitchMid(rc_ch::SC) ? SmallRcMode::kYes : SmallRcMode::kNo;
}

void SmallGimbal::ControlStateUpdate() {
  switch (force_mode_) {
    case SmallForceMode::kYesForce:
      ControlStateYesForceUpdate();
      break;
    case SmallForceMode::kInit:
      control_mode_ = SmallControlMode::kInit;
      break;
    case SmallForceMode::kNoForce:
    default:
      control_mode_ = SmallControlMode::kNoForce;
      ammo_mode_ = SmallAmmoMode::kNoForce;
      rotor_mode_ = SmallRotorMode::kNoForce;
      break;
  }
}

void SmallGimbal::ControlStateYesForceUpdate() {
  ammo_mode_ = SwitchUp(rc_ch::SB) ? SmallAmmoMode::kNoSpeed : SmallAmmoMode::kNormalSpeed;

  switch (basic_mode_) {
    case SmallBasicMode::kMatch:
      ControlStateMatchUpdate();
      ammo_mode_ = SmallAmmoMode::kNormalSpeed;
      break;
    case SmallBasicMode::kVision:
      ControlStateVisionUpdate();
      break;
    case SmallBasicMode::kEc:
      ControlStateEcUpdate();
      break;
  }
}

void SmallGimbal::ControlStateMatchUpdate() {
  if ((globals->aimbot_communicator->aimbot_state() & kAimbotTargetInside) != 0U) {
    control_mode_ = SmallControlMode::kAutoAmmo;
    special_.match_mode_flag = 0;
  } else if (special_.match_mode_flag == 0) {
    special_.match_mode_flag = 1;
    special_.match_mode_time = globals->tick;
    control_mode_ = SmallControlMode::kAutoAmmo;
  } else if (special_.match_mode_flag == 1 && TimeExceeded(special_.match_mode_time, kAutoAmmoToSmallRangeTime)) {
    control_mode_ = SmallControlMode::kSmallRangeScan;
    special_.match_mode_flag = 2;
    special_.match_mode_time = globals->tick;
  } else if (special_.match_mode_flag == 2 && TimeExceeded(special_.match_mode_time, kSmallRangeToFullRangeTime)) {
    control_mode_ = SmallControlMode::kScan;
    special_.match_mode_flag = 3;
  } else {
    control_mode_ = (special_.match_mode_flag == 1) ? SmallControlMode::kAutoAmmo : SmallControlMode::kScan;
  }

  if ((globals->aimbot_communicator->aimbot_state() & kAimbotShootRequest) != 0U &&
      globals->HeatCurrent() < globals->HeatLimit() - 50U) {
    rotor_mode_ = SmallRotorMode::kForward;
    special_.rotor_mode_time = globals->tick;
  } else if (last_rotor_mode_ == SmallRotorMode::kForward && !TimeExceeded(special_.rotor_mode_time, kAutoShootTime)) {
    rotor_mode_ = SmallRotorMode::kForward;
  } else {
    rotor_mode_ = SmallRotorMode::kStop;
  }
}

void SmallGimbal::ControlStateVisionUpdate() {
  if ((SwitchUp(rc_ch::SA) || SwitchMid(rc_ch::SA)) &&
      (globals->aimbot_communicator->aimbot_state() & kAimbotTargetInside) != 0U) {
    control_mode_ = SmallControlMode::kAutoAmmo;
  } else {
    control_mode_ = SmallControlMode::kRcControl;
  }

  if (SwitchUp(rc_ch::SB)) {
    rotor_mode_ = SmallRotorMode::kStop;
  } else if (SwitchMid(rc_ch::SB)) {
    if (special_.single_ammo_flag == 0U &&
        (globals->aimbot_communicator->aimbot_state() & kAimbotShootRequest) != 0U && SwitchDown(rc_ch::SH) &&
        TimeExceeded(special_.auto_ammo_delay_time, kSingleAmmoDelayTime)) {
      special_.single_ammo_flag = 1;
      special_.rotor_single_time = globals->tick;
      special_.auto_ammo_delay_time = globals->tick;
      rotor_mode_ = SmallRotorMode::kForward;
    } else if (special_.single_ammo_flag != 0U && !TimeExceeded(special_.rotor_single_time, kSingleAmmoTime)) {
      rotor_mode_ = SmallRotorMode::kForward;
    } else {
      rotor_mode_ = SmallRotorMode::kStop;
      special_.single_ammo_flag = 0;
    }
  } else if ((globals->aimbot_communicator->aimbot_state() & kAimbotShootRequest) != 0U && SwitchDown(rc_ch::SH) &&
             globals->HeatCurrent() < globals->HeatLimit() - 45U) {
    rotor_mode_ = SmallRotorMode::kForward;
  } else {
    rotor_mode_ = SmallRotorMode::kStop;
  }
}

void SmallGimbal::ControlStateEcUpdate() {
  if (SwitchUp(rc_ch::SA)) {
    control_mode_ = SmallControlMode::kSmallRangeScan;
  } else if (SwitchMid(rc_ch::SA)) {
    control_mode_ = SmallControlMode::kScan;
  } else {
    control_mode_ = SmallControlMode::kRcControl;
  }

  if (SwitchUp(rc_ch::SB)) {
    rotor_mode_ = SmallRotorMode::kNoForce;
  } else if (SwitchMid(rc_ch::SB)) {
    if (special_.single_ammo_flag == 0U && SwitchDown(rc_ch::SH) &&
        TimeExceeded(special_.auto_ammo_delay_time, kSingleAmmoDelayTime)) {
      special_.single_ammo_flag = 1;
      special_.rotor_single_time = globals->tick;
      special_.auto_ammo_delay_time = globals->tick;
      rotor_mode_ = SmallRotorMode::kForward;
    } else if (special_.single_ammo_flag != 0U && !TimeExceeded(special_.rotor_single_time, kSingleAmmoTime)) {
      rotor_mode_ = SmallRotorMode::kForward;
    } else {
      rotor_mode_ = SmallRotorMode::kStop;
      special_.single_ammo_flag = 0;
    }
  } else {
    rotor_mode_ = SwitchDown(rc_ch::SH) ? SmallRotorMode::kForward : SmallRotorMode::kStop;
  }
}

void SmallGimbal::ControlPidUpdate() {
  if (last_control_mode_ != control_mode_) {
    ConfigureGimbalPidFor(control_mode_);
  }
}

void SmallGimbal::AmmoPidUpdate() {
  if (last_ammo_mode_ != ammo_mode_) {
    ConfigureAmmoPidFor(ammo_mode_);
    last_ammo_mode_ = ammo_mode_;
  }
}

void SmallGimbal::RotorPidUpdate() {
  if (last_rotor_mode_ != rotor_mode_) {
    ConfigureRotorPidFor(rotor_mode_);
    last_rotor_mode_ = rotor_mode_;
  }
}

void SmallGimbal::ResetFlag() {
  if (last_control_mode_ == control_mode_) {
    return;
  }
  command_.yaw = imu_.yaw_angle;
  command_.pitch = imu_.pitch_angle;
  if (control_mode_ == SmallControlMode::kKeepMachineAngle) {
    command_.yaw = motor_measure_.yaw_angle;
    command_.pitch = motor_measure_.pitch_angle;
  }
  scan_.flag = false;
  scan_.time = 0;
  last_control_mode_ = control_mode_;
}

void SmallGimbal::LimitUpdate() {
  if ((motor_measure_.yaw_angle >= -180.0f && motor_measure_.yaw_angle <= kYawMachineLeft + 5.0f) ||
      (motor_measure_.yaw_angle <= 180.0f && motor_measure_.yaw_angle >= kYawMachineRight - 5.0f)) {
    limit_.left_delta = kYawMachineLeft - motor_measure_.yaw_angle;
    limit_.right_delta = motor_measure_.yaw_angle - kYawMachineRight;
  }
  if (limit_.left_delta < -180.0f) limit_.left_delta += 360.0f;
  if (limit_.right_delta < -180.0f) limit_.right_delta += 360.0f;
  limit_.yaw_left = imu_.yaw_angle + limit_.left_delta;
  limit_.yaw_right = imu_.yaw_angle - limit_.right_delta;
  limit_.pitch_up = kPitchUpAngle;
  limit_.pitch_down = kPitchDownAngle;
}

void SmallGimbal::CommandUpdate() {
  switch (control_mode_) {
    case SmallControlMode::kNoForce:
    case SmallControlMode::kInit:
      command_.yaw = imu_.yaw_angle;
      command_.pitch = imu_.pitch_angle;
      output_.yaw = 0;
      output_.pitch = 0.0f;
      break;

    case SmallControlMode::kRcControl:
      if (rc_mode_ == SmallRcMode::kYes) {
        command_.yaw += globals->wfly_et16s->RemoteChannalLeftX() * 0.25f / 3.0f;
        command_.pitch += globals->wfly_et16s->RemoteChannalLeftY() * 0.25f / 3.0f;
      }
      CommandLimit();
      pid_.yaw_angle.Update(command_.yaw, imu_.yaw_angle, 0.001f);
      pid_.yaw_speed.Update(pid_.yaw_angle.out(), -imu_.yaw_speed, 0.001f);
      pid_.pitch_angle.Update(command_.pitch, imu_.pitch_angle, 0.001f);
      pid_.pitch_speed.Update(pid_.pitch_angle.out(), -imu_.pitch_speed, 0.001f);
      output_.yaw = ClampCurrent(pid_.yaw_speed.out(), 25000.0f);
      output_.pitch = pid_.pitch_speed.out() + 17900.0f * std::cos(imu_.pitch_angle * kDegToRad);
      break;

    case SmallControlMode::kAutoAmmo:
      if ((globals->aimbot_communicator->aimbot_state() & kAimbotTargetInside) != 0U) {
        command_.yaw = globals->aimbot_communicator->yaw();
        command_.pitch = globals->aimbot_communicator->pitch();
      }
      command_.pitch = rm::modules::Clamp(command_.pitch, kPitchDownAngle, kPitchUpAngle);
      CommandLimit();
      pid_.yaw_angle.Update(command_.yaw, imu_.yaw_angle, 0.001f);
      pid_.yaw_speed.Update(pid_.yaw_angle.out(), -imu_.yaw_speed, 0.001f);
      pid_.pitch_angle.Update(command_.pitch, imu_.pitch_angle, 0.001f);
      pid_.pitch_speed.Update(pid_.pitch_angle.out(), -imu_.pitch_speed, 0.001f);
      output_.yaw = ClampCurrent(pid_.yaw_speed.out(), 25000.0f);
      output_.pitch = pid_.pitch_speed.out() + 17900.0f * std::cos(imu_.pitch_angle * kDegToRad);
      break;

    case SmallControlMode::kScan:
      scan_.time += 2;
      if (scan_.pitch_up != kPitchScanUpAngle || scan_.pitch_down != kPitchScanDownAngle) {
        scan_.flag = false;
      }
      if (!scan_.flag) {
        ScanInit(kYawMachineLeft, kYawMachineRight, kPitchScanUpAngle, kPitchScanDownAngle);
      }
      scan_.yaw_speed = kDeltaYaw * kDegToRad;
      ScanControl();
      break;

    case SmallControlMode::kSmallRangeScan:
      ++scan_.time;
      if (!scan_.flag) {
        float yaw_left = motor_measure_.yaw_angle + kSmallRangeLeftAngle;
        float yaw_right = motor_measure_.yaw_angle - kSmallRangeRightAngle;
        if (motor_measure_.yaw_angle >= -80.0f && motor_measure_.yaw_angle <= -60.0f) {
          yaw_left = -80.0f + kSmallRangeLeftAngle;
          yaw_right = -80.0f - kSmallRangeRightAngle;
        } else if (motor_measure_.yaw_angle >= 170.0f && motor_measure_.yaw_angle <= 180.0f) {
          yaw_left = 180.0f + kSmallRangeLeftAngle;
          yaw_right = 180.0f - kSmallRangeRightAngle;
        }
        ScanInit(yaw_left, yaw_right, kPitchScanUpAngle, kPitchScanDownAngle);
        scan_.yaw_speed = 3.0f * kDeltaYaw * kDegToRad;
      }
      ScanControl();
      break;

    default:
      output_.yaw = 0;
      output_.pitch = 0.0f;
      break;
  }
}

void SmallGimbal::ShootCommandUpdate() {
  switch (ammo_mode_) {
    case SmallAmmoMode::kNormalSpeed:
      command_.left = special_.real_ammo_speed;
      command_.right = -special_.real_ammo_speed;
      BulletSpeedLimit();
      break;
    case SmallAmmoMode::kNoSpeed:
    case SmallAmmoMode::kNoForce:
    default:
      command_.left = 0.0f;
      command_.right = 0.0f;
      break;
  }

  switch (rotor_mode_) {
    case SmallRotorMode::kForward:
    case SmallRotorMode::kBackward:
      command_.rotor = kRotorSpeedSet;
      break;
    case SmallRotorMode::kStop:
    case SmallRotorMode::kNoForce:
    default:
      command_.rotor = 0.0f;
      break;
  }

  pid_.left_speed.Update(command_.left, motor_measure_.ammo_left_speed, 0.001f);
  pid_.right_speed.Update(command_.right, motor_measure_.ammo_right_speed, 0.001f);
  pid_.rotor_speed.Update(command_.rotor, motor_measure_.rotor_speed, 0.001f);
  output_.left = ClampCurrent(pid_.left_speed.out(), 16384.0f);
  output_.right = ClampCurrent(pid_.right_speed.out(), 16384.0f);
  output_.rotor = ClampCurrent(pid_.rotor_speed.out(), 16384.0f);
}

void SmallGimbal::ScanInit(float yaw_left, float yaw_right, float pitch_up, float pitch_down) {
  scan_.time = 0;
  scan_.flag = true;
  pitch_up = rm::modules::Clamp(pitch_up, kPitchDownAngle, kPitchUpAngle);
  pitch_down = rm::modules::Clamp(pitch_down, kPitchDownAngle, kPitchUpAngle);
  if (pitch_up < pitch_down) {
    const float tmp = pitch_up;
    pitch_up = pitch_down;
    pitch_down = tmp;
  }
  scan_.pitch_up = pitch_up;
  scan_.pitch_down = pitch_down;
  scan_.pitch_bias = (pitch_up + pitch_down) * 0.5f;
  scan_.pitch_a = (pitch_up - pitch_down) * 0.5f;
  scan_.pitch_start = imu_.pitch_angle * kDegToRad;

  scan_.yaw_left = yaw_left;
  scan_.yaw_right = yaw_right;
  scan_.yaw_bias = (yaw_left + yaw_right) * 0.5f;
  scan_.yaw_a = (yaw_left - yaw_right) * 0.5f;
  scan_.yaw_start = motor_measure_.yaw_angle - yaw_left;
  if (yaw_left < yaw_right) {
    scan_.yaw_bias = WrapDegree(scan_.yaw_bias + 180.0f);
    scan_.yaw_a = WrapDegree(scan_.yaw_a + 180.0f);
    if (motor_measure_.yaw_angle > yaw_left) {
      scan_.yaw_start -= 360.0f;
    }
  }
  scan_.yaw_start = (180.0f / (scan_.yaw_a * 2.0f)) * scan_.yaw_start + 90.0f;
  scan_.yaw_start *= kDegToRad;
}

void SmallGimbal::ScanControl() {
  command_.pitch = scan_.pitch_a * std::sin(scan_.pitch_start + kDeltaPitch * static_cast<float>(scan_.time)) +
                   scan_.pitch_bias;
  command_.pitch = rm::modules::Clamp(command_.pitch, scan_.pitch_down, scan_.pitch_up);
  command_.yaw =
      scan_.yaw_a * std::sin(scan_.yaw_start + scan_.yaw_speed * static_cast<float>(scan_.time) * kDegToRad) +
      scan_.yaw_bias;
  command_.yaw = WrapDegree(command_.yaw);
  command_.yaw = ConstrainAround(command_.yaw, motor_measure_.yaw_angle);

  pid_.pitch_angle.Update(command_.pitch, imu_.pitch_angle, 0.001f);
  pid_.pitch_speed.Update(pid_.pitch_angle.out(), imu_.pitch_speed, 0.001f);
  pid_.yaw_angle.Update(command_.yaw, motor_measure_.yaw_angle, 0.001f);
  pid_.yaw_speed.Update(pid_.yaw_angle.out(), motor_measure_.yaw_speed, 0.001f);
  output_.pitch = pid_.pitch_speed.out();
  output_.yaw = ClampCurrent(pid_.yaw_speed.out(), 25000.0f);
}

void SmallGimbal::CommandLimit() {
  command_.yaw = ConstrainAround(command_.yaw, imu_.yaw_angle);
  command_.yaw = rm::modules::Clamp(command_.yaw, limit_.yaw_right, limit_.yaw_left);
  command_.pitch = rm::modules::Clamp(command_.pitch, limit_.pitch_down, limit_.pitch_up);
}

void SmallGimbal::BulletSpeedLimit() {
  const float initial_speed = globals->referee_data->data().shoot_data.initial_speed;
  if (initial_speed < 1.0f) {
    return;
  }
  const float speed_error = 23.8f - initial_speed;
  if (std::abs(speed_error) <= 2.0f) {
    special_.real_ammo_speed += speed_error * 80.0f;
  } else {
    special_.real_ammo_speed += speed_error * 150.0f;
  }
  special_.real_ammo_speed = rm::modules::Clamp(special_.real_ammo_speed, kAmmoSpeedSet - 1500.0f,
                                                kAmmoSpeedSet + 1200.0f);
  command_.left = special_.real_ammo_speed;
  command_.right = -special_.real_ammo_speed;
}

bool SmallGimbal::EnablePitchMotor(bool enable) {
  const auto status = globals->pitch_motor->status();
  if (++dm_instruction_tick_ < 100U) {
    return status == 0x01;
  }
  dm_instruction_tick_ = 0;

  if (enable && status == 0x00) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
  } else if (enable && status == 0x0d) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  } else if (enable && status != 0x01) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  } else if (!enable && status == 0x01) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
  }
  return status == 0x01;
}

void SmallGimbal::SendMotorOutput() {
  const bool enable = control_mode_ != SmallControlMode::kNoForce;
  const bool pitch_enabled = EnablePitchMotor(enable);
  if (!enable || !pitch_enabled) {
    output_ = {};
  }

  globals->up_yaw_motor->SetCurrent(output_.yaw);
  if ((globals->tick & 1U) != 0U) {
    globals->pitch_motor->SetMitCommand(0.0f, 0.0f, rm::modules::Clamp(output_.pitch / 10000.0f, -10.0f, 10.0f), 0.0f,
                                        0.0f);
  }
  globals->friction_left->SetCurrent(output_.left);
  globals->friction_right->SetCurrent(output_.right);
  globals->dial_motor->SetCurrent(output_.rotor);
}

void SmallGimbal::ConfigureGimbalPidFor(SmallControlMode mode) {
  float yaw_angle_kp = 0.0f;
  float yaw_angle_kd = 0.0f;
  float yaw_speed_kp = 0.0f;
  float pitch_angle_kp = 0.0f;
  float pitch_angle_kd = 0.0f;
  float pitch_speed_kp = 0.0f;

  switch (mode) {
    case SmallControlMode::kRcControl:
      yaw_angle_kp = 12.5f;
      yaw_angle_kd = 700.0f;
      yaw_speed_kp = 180.0f;
      pitch_angle_kp = 19.0f;
      pitch_angle_kd = 1300.0f;
      pitch_speed_kp = 150.0f;
      break;
    case SmallControlMode::kAutoAmmo:
      yaw_angle_kp = 12.5f;
      yaw_angle_kd = 700.0f;
      yaw_speed_kp = 160.0f;
      pitch_angle_kp = 18.0f;
      pitch_angle_kd = 1000.0f;
      pitch_speed_kp = 135.0f;
      break;
    case SmallControlMode::kScan:
    case SmallControlMode::kSmallRangeScan:
      yaw_angle_kp = 15.0f;
      yaw_angle_kd = mode == SmallControlMode::kSmallRangeScan ? 500.0f : 240.0f;
      yaw_speed_kp = 150.0f;
      pitch_angle_kp = 15.0f;
      pitch_angle_kd = 1000.0f;
      pitch_speed_kp = 120.0f;
      break;
    case SmallControlMode::kInit:
      yaw_angle_kp = 20.0f;
      yaw_angle_kd = 40.0f;
      yaw_speed_kp = 150.0f;
      pitch_angle_kp = 24.0f;
      pitch_speed_kp = 300.0f;
      break;
    default:
      break;
  }

  pid_.yaw_angle.SetKp(yaw_angle_kp).SetKi(0.0f).SetKd(yaw_angle_kd).SetMaxOut(250.0f).SetMaxIout(0.0f);
  pid_.yaw_speed.SetKp(yaw_speed_kp).SetKi(0.0f).SetKd(0.0f).SetMaxOut(25000.0f).SetMaxIout(10000.0f);
  pid_.pitch_angle.SetKp(pitch_angle_kp).SetKi(0.0f).SetKd(pitch_angle_kd).SetMaxOut(600.0f).SetMaxIout(30.0f);
  pid_.pitch_speed.SetKp(pitch_speed_kp).SetKi(0.0f).SetKd(0.0f).SetMaxOut(50000.0f).SetMaxIout(10000.0f);
  pid_.yaw_angle.SetDiffLpfAlpha(mode == SmallControlMode::kNoForce ? 1.0f : 0.1f);
  pid_.yaw_speed.SetDiffLpfAlpha(1.0f);
  pid_.pitch_angle.SetDiffLpfAlpha(mode == SmallControlMode::kNoForce ? 1.0f : 0.1f);
  pid_.pitch_speed.SetDiffLpfAlpha(1.0f);
}

void SmallGimbal::ConfigureAmmoPidFor(SmallAmmoMode mode) {
  const bool enable = mode == SmallAmmoMode::kNormalSpeed;
  pid_.left_speed.SetKp(enable ? 12.0f : 0.0f).SetKi(0.0f).SetKd(enable ? 8.0f : 0.0f).SetMaxOut(16384.0f).SetMaxIout(6000.0f);
  pid_.right_speed.SetKp(enable ? 12.0f : 0.0f).SetKi(0.0f).SetKd(enable ? 8.0f : 0.0f).SetMaxOut(16384.0f).SetMaxIout(6000.0f);
}

void SmallGimbal::ConfigureRotorPidFor(SmallRotorMode mode) {
  const bool enable = mode != SmallRotorMode::kNoForce;
  pid_.rotor_speed.SetKp(enable ? 16.0f : 0.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(5000.0f);
}

bool SmallGimbal::TimeExceeded(uint32_t since, uint32_t duration_ms) const { return globals->tick - since >= duration_ms; }

bool SmallGimbal::SwitchDown(int channel) const { return globals->wfly_et16s->SwitchIsDown(channel); }

bool SmallGimbal::SwitchMid(int channel) const { return globals->wfly_et16s->SwitchIsMid(channel); }

bool SmallGimbal::SwitchUp(int channel) const { return globals->wfly_et16s->SwitchIsUp(channel); }
