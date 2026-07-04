#include "Gimbal.hpp"

#include <cmath>

using namespace rm;

Gimbal *gimbal{nullptr};

namespace {
constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0f;
constexpr float kRadToDeg = 180.0f / static_cast<float>(M_PI);
constexpr float kRpmToRadPerSec = static_cast<float>(M_PI) * 2.0f / 60.0f;

constexpr float kPitchUp = 33.0f * kDegToRad;
constexpr float kPitchDown = -30.0f * kDegToRad;
constexpr float kYawLeft = -60.0f * kDegToRad;
constexpr float kYawRight = 170.0f * kDegToRad;
constexpr float kYawMachineZero = -121.0f * kDegToRad;
constexpr uint8_t kAimbotTargetInside = 1U << 0U;
constexpr uint8_t kAimbotShootRequest = 1U << 1U;

float WrapRad(float value) {
  return rm::modules::Wrap(value, -static_cast<float>(M_PI), static_cast<float>(M_PI));
}

float WrapDegree(float value) {
  return rm::modules::Wrap(value, -180.0f, 180.0f);
}
}  // namespace

void Gimbal::Init() {
  down_yaw_target_ = globals->down_yaw_motor->pos();
  small_yaw_target_ = WrapRad(globals->up_yaw_motor->pos_rad());
  pitch_target_ = rm::modules::Clamp(globals->pitch_motor->pos(), kPitchDown, kPitchUp);
  globals->gimbal_controller.EnableSpeedPid(true);
}

void Gimbal::Task1kHz() {
  UpdateMode();
  UpdateTargets();
  UpdateControl();
  UpdateShoot();
  SendMotorOutput();
}

float Gimbal::SmallYawDegree() const { return WrapDegree(globals->up_yaw_motor->pos_degree() + 120.0f); }

float Gimbal::SmallPitchDegree() const { return globals->pitch_motor->pos() * kRadToDeg; }

void Gimbal::UpdateMode() {
  if (globals->robot_mode == RobotMode::kNoForce || !globals->GimbalPowerEnabled() || !globals->RcOnline()) {
    mode_ = GimbalMode::kNoForce;
  } else if (globals->device_gimbal.all_device_ok() || globals->robot_mode == RobotMode::kTest) {
    mode_ = globals->requested_gimbal_mode;
    if (mode_ == GimbalMode::kNoForce) {
      mode_ = GimbalMode::kRemote;
    }
  } else {
    mode_ = GimbalMode::kNoForce;
  }

  if (mode_ != last_mode_) {
    down_yaw_target_ = globals->down_yaw_motor->pos();
    small_yaw_target_ = WrapRad(globals->up_yaw_motor->pos_rad());
    pitch_target_ = rm::modules::Clamp(globals->pitch_motor->pos(), kPitchDown, kPitchUp);
    scan_tick_ = 0;
    last_mode_ = mode_;
  }
}

void Gimbal::UpdateTargets() {
  switch (mode_) {
    case GimbalMode::kRemote:
      UpdateRemoteTarget();
      break;
    case GimbalMode::kScan:
      UpdateScanTarget();
      break;
    case GimbalMode::kAimbot:
      UpdateAimbotTarget();
      break;
    case GimbalMode::kKeep:
      break;
    case GimbalMode::kInit:
      small_yaw_target_ = kYawMachineZero;
      pitch_target_ = 0.0f;
      break;
    case GimbalMode::kNoForce:
    default:
      small_yaw_target_ = WrapRad(globals->up_yaw_motor->pos_rad());
      pitch_target_ = globals->pitch_motor->pos();
      break;
  }

  small_yaw_target_ = rm::modules::Clamp(small_yaw_target_, kYawLeft, kYawRight);
  pitch_target_ = rm::modules::Clamp(pitch_target_, kPitchDown, kPitchUp);
}

void Gimbal::UpdateRemoteTarget() {
  small_yaw_target_ = WrapRad(small_yaw_target_ + globals->wfly_et16s->left_x() * 0.25f * kDegToRad);
  pitch_target_ = rm::modules::Clamp(pitch_target_ + globals->wfly_et16s->left_y() * 0.25f * kDegToRad,
                                     kPitchDown, kPitchUp);
}

void Gimbal::UpdateScanTarget() {
  ++scan_tick_;
  const float yaw_center = (kYawLeft + kYawRight) * 0.5f;
  const float yaw_amp = (kYawRight - kYawLeft) * 0.5f;
  const float pitch_center = (20.0f - 10.0f) * 0.5f * kDegToRad;
  const float pitch_amp = (20.0f + 10.0f) * 0.5f * kDegToRad;
  const float t = static_cast<float>(scan_tick_) * 0.001f;
  small_yaw_target_ = yaw_center + yaw_amp * std::sin(t * 0.45f);
  pitch_target_ = pitch_center + pitch_amp * std::sin(t * 0.9f);
}

void Gimbal::UpdateAimbotTarget() {
  if ((globals->aimbot_communicator->aimbot_state() & kAimbotTargetInside) != 0U) {
    small_yaw_target_ = WrapRad(globals->up_yaw_motor->pos_rad() + globals->aimbot_communicator->yaw() * kDegToRad);
    pitch_target_ = globals->pitch_motor->pos() + globals->aimbot_communicator->pitch() * kDegToRad;
  } else {
    UpdateScanTarget();
  }
}

void Gimbal::UpdateControl() {
  const bool enable = mode_ != GimbalMode::kNoForce;
  globals->gimbal_controller.Enable(enable);
  EnableDmMotors(enable);

  globals->gimbal_controller.SetTarget(small_yaw_target_, down_yaw_target_, pitch_target_);
  globals->gimbal_controller.Update(globals->up_yaw_motor->pos_rad(), globals->up_yaw_motor->rpm() * kRpmToRadPerSec,
                                    globals->down_yaw_motor->pos(), globals->down_yaw_motor->vel(),
                                    globals->pitch_motor->pos(), globals->pitch_motor->vel(), 0.001f);
}

void Gimbal::UpdateShoot() {
  const bool enabled = mode_ != GimbalMode::kNoForce && globals->ShooterPowerEnabled();
  globals->shoot_controller.Enable(enabled);
  globals->shoot_controller.Arm(enabled);

  if (!enabled) {
    globals->shoot_controller.SetShootFrequency(0.0f);
  } else {
    const float measured_speed = globals->referee_data->data().shoot_data.initial_speed;
    if (measured_speed >= 18.0f && measured_speed <= 27.0f && measured_speed > last_shoot_speed_) {
      ammo_speed_ *= std::cbrt(22.5f / measured_speed);
    }
    last_shoot_speed_ = measured_speed;
    globals->shoot_controller.SetArmSpeed(ammo_speed_);

    const uint16_t heat_left = globals->HeatLimit() > globals->HeatCurrent()
                                   ? globals->HeatLimit() - globals->HeatCurrent()
                                   : 0U;
    const bool manual_single = globals->wfly_et16s->wheel_position(rc_ch::LS) <= -650 &&
                               globals->wfly_et16s->switch_position(rc_ch::SH) == SwitchPosition::kDown;
    const bool manual_auto = globals->wfly_et16s->switch_position(rc_ch::SH) == SwitchPosition::kDown;
    const bool vision_fire = (globals->aimbot_communicator->aimbot_state() & kAimbotShootRequest) != 0U;

    if ((manual_single || vision_fire) && heat_left > 30U) {
      globals->shoot_controller.SetMode(Shoot3Fric::kSingleShot);
      if (!single_shoot_started_) {
        globals->shoot_controller.Fire();
        single_shoot_started_ = true;
        single_shoot_lock_ = 200;
      }
    } else if (manual_auto && heat_left > 40U) {
      globals->shoot_controller.SetMode(Shoot3Fric::kFullAuto);
      globals->shoot_controller.SetShootFrequency(heat_left > 120U ? 20.0f : static_cast<float>(heat_left) / 6.0f);
    } else {
      globals->shoot_controller.SetShootFrequency(0.0f);
    }
  }

  if (single_shoot_lock_ > 0U) {
    --single_shoot_lock_;
  } else {
    single_shoot_started_ = false;
  }

  globals->dial_encoder_counter.Update(globals->dial_motor->encoder());
  globals->shoot_controller.Update(globals->friction_left->rpm(), globals->friction_right->rpm(), 0.0f,
                                   static_cast<float>(globals->dial_encoder_counter.linear_ticks()),
                                   globals->dial_motor->rpm(), 0.001f);
}

void Gimbal::EnableDmMotors(bool enable) {
  if (++dm_instruction_tick_ < 100U) {
    return;
  }
  dm_instruction_tick_ = 0;

  auto instruction = enable ? rm::device::DmMotorInstructions::kEnable : rm::device::DmMotorInstructions::kDisable;
  if (globals->down_yaw_motor->status() != 0x01 && enable) {
    globals->down_yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  }
  if (globals->pitch_motor->status() != 0x01 && enable) {
    globals->pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  }
  globals->down_yaw_motor->SendInstruction(instruction);
  globals->pitch_motor->SendInstruction(instruction);
}

void Gimbal::SetZeroOutput() {
  globals->up_yaw_motor->SetCurrent(0);
  globals->friction_left->SetCurrent(0);
  globals->friction_right->SetCurrent(0);
  globals->dial_motor->SetCurrent(0);
  globals->down_yaw_motor->SetMitCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  globals->pitch_motor->SetMitCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void Gimbal::SendMotorOutput() {
  if (mode_ == GimbalMode::kNoForce) {
    SetZeroOutput();
    return;
  }

  globals->up_yaw_motor->SetCurrent(static_cast<int16_t>(
      rm::modules::Clamp(globals->gimbal_controller.output().up_yaw, -25000.0f, 25000.0f)));
  globals->down_yaw_motor->SetMitCommand(0.0f, 0.0f,
                                         rm::modules::Clamp(globals->gimbal_controller.output().down_yaw, -10.0f, 10.0f),
                                         0.0f, 0.0f);
  globals->pitch_motor->SetMitCommand(0.0f, 0.0f,
                                      rm::modules::Clamp(globals->gimbal_controller.output().pitch, -10.0f, 10.0f),
                                      0.0f, 3.0f);

  globals->friction_left->SetCurrent(static_cast<int16_t>(
      rm::modules::Clamp(globals->shoot_controller.output().fric_1, -16384.0f, 16384.0f)));
  globals->friction_right->SetCurrent(static_cast<int16_t>(
      rm::modules::Clamp(globals->shoot_controller.output().fric_2, -16384.0f, 16384.0f)));
  globals->dial_motor->SetCurrent(
      static_cast<int16_t>(rm::modules::Clamp(globals->shoot_controller.output().loader, -10000.0f, 10000.0f)));
}
