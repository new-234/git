#include "Chassis.hpp"

#include <cmath>

using namespace rm;

Chassis *chassis{nullptr};

namespace {
constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0f;
constexpr float kFollowAngle = -107.07164f * kDegToRad;
constexpr float kRotateRate = 1.5f;
constexpr float kWheelRpmRate = 3000.0f;
constexpr float kMoveDeltaAngle = -0.15f;
constexpr float kMaxInput = 1.0f;

float ClampInput(float value) {
  return rm::modules::Clamp(value, -kMaxInput, kMaxInput);
}
}  // namespace

void Chassis::Init() {
  follow_pid_.SetKp(0.1f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(1.0f).SetMaxIout(0.0f);
}

void Chassis::Task1kHz() {
  UpdateMode();
  UpdateSource();
  UpdateKinematics();
  UpdateControl();
  ApplyPowerLimit();
  SetMotorCurrent();
}

void Chassis::UpdateMode() {
  if (globals->robot_mode == RobotMode::kNoForce || !globals->ChassisPowerEnabled() || !globals->RcOnline()) {
    mode_ = ChassisMode::kNoForce;
    return;
  }
  if (!globals->device_chassis.all_device_ok() && globals->robot_mode != RobotMode::kTest) {
    mode_ = ChassisMode::kNoForce;
    return;
  }
  mode_ = globals->requested_chassis_mode;
}

void Chassis::UpdateSource() {
  source_vx_ = 0.0f;
  source_vy_ = 0.0f;
  source_wz_ = 0.0f;

  switch (mode_) {
    case ChassisMode::kFollow:
    case ChassisMode::kSlowRotate:
    case ChassisMode::kQuickRotate:
      UpdateRemoteSource();
      break;
    case ChassisMode::kNavigate:
      UpdateNavigateSource();
      break;
    case ChassisMode::kKeep:
    case ChassisMode::kNoForce:
    default:
      break;
  }
}

void Chassis::UpdateRemoteSource() {
  source_vx_ = ClampInput(globals->wfly_et16s->right_x());
  source_vy_ = ClampInput(-globals->wfly_et16s->right_y());

  if (mode_ == ChassisMode::kSlowRotate) {
    source_wz_ = 0.5f;
  } else if (mode_ == ChassisMode::kQuickRotate) {
    source_wz_ = -1.0f;
  } else {
    const float yaw_error = rm::modules::Wrap(globals->down_yaw_motor->pos() - kFollowAngle,
                                             -static_cast<float>(M_PI), static_cast<float>(M_PI));
    follow_pid_.Update(0.0f, yaw_error, 0.001f);
    source_wz_ = follow_pid_.out();
  }
}

void Chassis::UpdateNavigateSource() {
  source_vx_ = ClampInput(globals->navigate_communicator->chassis_target_y());
  source_vy_ = ClampInput(-globals->navigate_communicator->chassis_target_x());
  source_wz_ = ClampInput(globals->navigate_communicator->chassis_target_w());
}

void Chassis::UpdateKinematics() {
  if (mode_ == ChassisMode::kNoForce) {
    target_vx_ = 0.0f;
    target_vy_ = 0.0f;
    target_wz_ = 0.0f;
    return;
  }

  const float yaw_error =
      rm::modules::Wrap(globals->down_yaw_motor->pos() - kFollowAngle, -static_cast<float>(M_PI), static_cast<float>(M_PI));
  const float delta_bias = 0.06f * globals->down_yaw_motor->vel();
  const float rotate_bias = source_wz_ * kMoveDeltaAngle;
  const float theta = yaw_error + delta_bias + rotate_bias;

  target_vx_ = source_vx_ * std::cos(theta) + source_vy_ * std::sin(theta);
  target_vy_ = -source_vx_ * std::sin(theta) + source_vy_ * std::cos(theta);
  target_wz_ = source_wz_;

  target_vx_ = rm::modules::Clamp(target_vx_, -1.0f, 1.0f);
  target_vy_ = rm::modules::Clamp(target_vy_, -1.0f, 1.0f);
  target_wz_ = rm::modules::Clamp(target_wz_, -1.0f, 1.0f);
}

void Chassis::UpdateControl() {
  const bool enable = mode_ != ChassisMode::kNoForce;
  globals->chassis_controller.Enable(enable);
  globals->chassis_controller.SetTarget(target_vx_ * kWheelRpmRate, target_vy_ * kWheelRpmRate,
                                        target_wz_ * kRotateRate * kWheelRpmRate);
  globals->chassis_controller.Update(globals->wheel_lf->rpm(), globals->wheel_rf->rpm(), globals->wheel_lb->rpm(),
                                     globals->wheel_rb->rpm(), 0.001f);
}

void Chassis::ApplyPowerLimit() {
  if (mode_ == ChassisMode::kNoForce) {
    SetZeroOutput();
    return;
  }

  const auto &heat = globals->referee_data->data().power_heat_data;
  const auto &status = globals->referee_data->data().robot_status;
  power_limit_ = status.chassis_power_limit == 0 ? 60.0f : static_cast<float>(status.chassis_power_limit);
  if (heat.buffer_energy < 10U) {
    power_limit_ *= 0.6f;
  } else if (heat.buffer_energy < 60U) {
    power_limit_ *= 0.8f;
  }

  float initial[4] = {
      globals->chassis_controller.output().lf_wheel,
      globals->chassis_controller.output().rf_wheel,
      globals->chassis_controller.output().lb_wheel,
      globals->chassis_controller.output().rb_wheel,
  };

  rm::device::M3508 *motors[4] = {globals->wheel_lf, globals->wheel_rf, globals->wheel_lb, globals->wheel_rb};
  for (size_t i = 0; i < 4; ++i) {
    motor_state_[i].speed_rpm = motors[i]->rpm();
    motor_state_[i].give_current = initial[i];
    motor_state_[i].measured_current = motors[i]->current();
    power_info_[i] = power_model_.CalculatePower(motor_state_[i]);
  }

  power_model_.DistributePower<4>(motor_state_, initial, power_limit_, output_current_);
  for (float &current : output_current_) {
    current = rm::modules::Clamp(current, -15000.0f, 15000.0f);
  }
}

void Chassis::SetMotorCurrent() {
  globals->wheel_lf->SetCurrent(static_cast<int16_t>(output_current_[0]));
  globals->wheel_rf->SetCurrent(static_cast<int16_t>(output_current_[1]));
  globals->wheel_lb->SetCurrent(static_cast<int16_t>(output_current_[2]));
  globals->wheel_rb->SetCurrent(static_cast<int16_t>(output_current_[3]));
}

void Chassis::SetZeroOutput() {
  for (float &current : output_current_) {
    current = 0.0f;
  }
}
