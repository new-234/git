#include "Gimbal.hpp"
#include "globals.hpp"

void GimbalHero::Enabling() {
  globals->gimbal_motor_yaw.SendInstruction(rm::device::DmMotorInstructions::kClearError);
  globals->gimbal_motor_yaw.SendInstruction(rm::device::DmMotorInstructions::kEnable);
  globals->gimbal_motor_pitch.SetCurrent(0);
}

void GimbalHero::Unabling() {
  globals->gimbal_motor_yaw.SetMitCommand(0, 0, 0, 0, 0);
  globals->gimbal_motor_pitch.SetCurrent(0);
  globals->gimbal_motor_yaw.SendInstruction(rm::device::DmMotorInstructions::kDisable);
  target_yaw_ = -globals->ahrs.euler_angle().yaw;
  target_pitch_ = -globals->ahrs.euler_angle().pitch;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1600);
}

/**
 * @brief 姿态解算任务
 */
void GimbalHero::AhrsUpdate() {
  globals_no_dtcm.imu->Update();
  // 右手坐标系（前 x，左 y，上 z）
  hyperparameters_.gyro_rectification_ =
      -0.0033 - (globals->ahrs.euler_angle().pitch + 0.8) / 0.8 * -0.0035;  //  这个放在姿态结算后面会有问题，待查
  globals->ahrs.Update(rm::modules::ImuData6Dof{
      -globals_no_dtcm.imu->gyro_y(), globals_no_dtcm.imu->gyro_x(),
      globals->gyro_z =
          globals->gyro_z_filter.apply(globals_no_dtcm.imu->gyro_z()) + hyperparameters_.gyro_rectification_,
      -globals_no_dtcm.imu->accel_y(), globals_no_dtcm.imu->accel_x(), globals_no_dtcm.imu->accel_z()});

  monitor_.euler_yaw_ = -globals->ahrs.euler_angle().yaw;
  monitor_.euler_pitch_ = -globals->ahrs.euler_angle().pitch;
  monitor_.euler_roll_ = -globals->ahrs.euler_angle().roll;

  // 监测
  eulerangle_yaw = monitor_.euler_yaw_;
  eulerangle_pitch = monitor_.euler_pitch_;
  eulerangle_roll = monitor_.euler_roll_;
  gyro_x = globals_no_dtcm.imu->gyro_x();
  gyro_y = globals_no_dtcm.imu->gyro_y();
  gyro_z = globals_no_dtcm.imu->gyro_z();
  accel_x = globals_no_dtcm.imu->accel_x();
  accel_y = globals_no_dtcm.imu->accel_y();
  accel_z = globals_no_dtcm.imu->accel_z();
}

void GimbalHero::AimbotControl() {
  static int count{0};
  if (count < 1000 * 2) {
    count++;
  } else {
    if (globals->radar_can_communicator.vaild() && globals->radar_can_communicator.fresh() && globals->radar_manager.all_device_ok()) {
      target_yaw_ = -globals->ahrs.euler_angle().yaw - globals->radar_can_communicator.yaw_mard();
      target_pitch_ = -globals->ahrs.euler_angle().pitch - globals->radar_can_communicator.pitch_mard();
      count = 0;
    }
  }
}

void GimbalHero::NormalRemoter() {
  // tc->rc->cc
  last_target_pitch_ = target_pitch_;
  if ((globals->tc.data().mouse_x != 0 || globals->tc.data().mouse_y != 0) && globals->tc_manager.all_device_ok()) {
    target_yaw_ += static_cast<float>(globals->rc.right_x()) * 0.000005 +
                   static_cast<float>(globals->tc.data().mouse_x) / 32768 * 0.8;
    target_pitch_ += static_cast<float>(globals->rc.right_y()) * 0.0000005 +
                     static_cast<float>(globals->tc.data().mouse_y) / 32768 * 0.5;
  } else if ((globals->rc.mouse_x() != 0 || globals->rc.mouse_y() != 0) && globals->rc_manager.all_device_ok()) {
    target_yaw_ +=
        static_cast<float>(globals->rc.right_x()) * 0.000005 + static_cast<float>(globals->rc.mouse_x()) / 32768.0 * 3;
    target_pitch_ +=
        static_cast<float>(globals->rc.right_y()) * 0.0000005 + static_cast<float>(globals->rc.mouse_y() / 32768.0 * 3);
  } else {
    target_yaw_ += static_cast<float>(globals->rc.right_x()) * 0.000005;
    target_pitch_ += static_cast<float>(globals->rc.right_y()) * 0.0000005;
  }
}

void GimbalHero::SnipeRemoter() {
  // tc->rc->cc
  last_target_pitch_ = target_pitch_;
  if (globals->tc_manager.all_device_ok()) {
    target_yaw_ += static_cast<float>(globals->rc.right_x()) * 0.0000005;
    target_pitch_ += static_cast<float>(globals->rc.right_y()) * 0.00000025;
    if (globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kW)) {
      target_pitch_ += 0.000005;
    }
    if (globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kS)) {
      target_pitch_ -= 0.000005;
    }
    if (globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kD)) {
      target_yaw_ += 0.000005;
    }
    if (globals->tc.data().keyboard_key & static_cast<int16_t>(rm::device::VT03::KeyboardKey::kA)) {
      target_yaw_ -= 0.000005;
    }
  } else if (globals->rc_manager.all_device_ok()) {
    target_yaw_ += static_cast<float>(globals->rc.right_x()) * 0.0000005;
    target_pitch_ += static_cast<float>(globals->rc.right_y()) * 0.00000025;
    if (globals->rc.key(DR16::Key::kW)) {
      target_pitch_ += 0.000005;
    }
    if (globals->rc.key(DR16::Key::kS)) {
      target_pitch_ -= 0.000005;
    }
    if (globals->rc.key(DR16::Key::kD)) {
      target_yaw_ += 0.000005;
    }
    if (globals->rc.key(DR16::Key::kA)) {
      target_yaw_ -= 0.000005;
    }
  } else {
    target_yaw_ += static_cast<float>(globals->rc.right_x()) * 0.0000005;
    target_pitch_ += static_cast<float>(globals->rc.right_y()) * 0.00000025;
  }
}

void GimbalHero::RemoterControl() {
  switch (globals->state_machine.getSubState_()) {
    // 普通控制
    default:
      switch (globals->state_machine.getGimbalState_()) {
        default:
          Unabling();
          break;

        case StateMachine::GimbalState::kRadarAimbot:
          AimbotControl();
          break;

        case StateMachine::GimbalState::kNormal:
          NormalRemoter();
          break;
      }
      break;

    case StateMachine::SubState::kSnipe:
      // 吊射模式
      switch (globals->state_machine.getGimbalState_()) {
        default:
          Unabling();
          break;

        case StateMachine::GimbalState::kRadarAimbot:
          AimbotControl();
          break;

        case StateMachine::GimbalState::kNormal:
          SnipeRemoter();
          break;
      }

      break;
  }

  // pitch限位
  if (target_pitch_ < -0.02) {
    target_pitch_ = -0.02;
  } else if (target_pitch_ > 0.81) {
    target_pitch_ = 0.81;
  }
}

void GimbalHero::MotorUpdate() {
  // yawPID计算（双环）
  globals->pid_yaw_position.SetCircular(true).SetCircularCycle(3.141593 * 2);

  // yawPID计算
  globals->pid_yaw_position.Update(target_yaw_, -globals->ahrs.euler_angle().yaw, 0.001);
  globals->pid_yaw_velocity.Update(globals->pid_yaw_position.out(),
                                   -globals_no_dtcm.imu->gyro_z() - hyperparameters_.gyro_rectification_, 0.001);

  // 发送CAN
  globals->gimbal_motor_yaw.SetMitCommand(
      0, 0, /*(1 - 0.1 * globals->ahrs.euler_angle().pitch / 0.8) **/ globals->pid_yaw_velocity.out(), 0, 1.3);
  // globals->gimbal_motor_yaw.SetMitCommand(0, 0, 0, 0, 0);

  if (globals->state_machine.getSubState_() == StateMachine::SubState::kOverPower ||
      globals->state_machine.getGimbalSubState_() == StateMachine::GimbalSubState::kPitchlock ||
      globals->state_machine.getSubState_() == StateMachine::SubState::kSnipe) {
    globals->gimbal_motor_pitch.SetCurrent(0);
    target_pitch_ = globals->ahrs.euler_angle().pitch;
  } else if (target_pitch_ - last_target_pitch_ >= 0) {
    globals->pid_pitch_position.Update(target_pitch_, -globals->ahrs.euler_angle().pitch, 0.001);
    globals->pid_pitch_velocity.Update(globals->pid_pitch_position.out(), -globals_no_dtcm.imu->gyro_x(), 0.001);

    positive_ff_ = 3000;

    negaive_ff_ = 0;

  } else if (target_pitch_ - last_target_pitch_ < 0) {
    globals->pid_pitch_position.Update(target_pitch_, -globals->ahrs.euler_angle().pitch, 0.001);
    globals->pid_pitch_velocity.Update(globals->pid_pitch_position.out(), -globals_no_dtcm.imu->gyro_x(), 0.001);

    negaive_ff_ = 1000;

    positive_ff_ = 0;
  }
  globals->gimbal_motor_pitch.SetCurrent(
      static_cast<int16_t>(globals->pid_pitch_velocity.out() + positive_ff_ + negaive_ff_));
  last_pitch_ = globals->ahrs.euler_angle().pitch;
}

void GimbalHero::ServoMotorUpdate() {
  switch (globals->state_machine.getSubState_()) {
    default:
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1410);
      break;
    case (StateMachine::SubState::kSnipe):
      servo_angle_ = 1730 /*(1730 - 1440) * (-globals->ahrs.euler_angle().pitch / 0.8) + 1440*/;
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, servo_angle_);
      break;
  }
}

void GimbalHero::GimbalControl() {
  RemoterControl();
  MotorUpdate();
  ServoMotorUpdate();
}

bool GimbalHero::EnableJudgement() {
  // 监测
  yaw_pos = -globals->ahrs.euler_angle().yaw;
  yaw_vel = -globals_no_dtcm.imu->gyro_z() - hyperparameters_.gyro_rectification_;
  yaw_pos_pid_out = globals->pid_yaw_position.out();
  yaw_vel_pid_out = globals->pid_yaw_velocity.out();
  ::target_yaw = target_yaw_;

  pitch_pos = -globals->ahrs.euler_angle().pitch;
  pitch_vel = globals_no_dtcm.imu->gyro_x();
  pitch_pos_pid_out = globals->pid_pitch_position.out();
  pitch_vel_pid_out = globals->pid_pitch_velocity.out();
  ::target_pitch = target_pitch_;

  // 主状态检测
  if (globals->state_machine.getMainState_() == StateMachine::MainState::kOffline) {
    Unabling();
    return false;
  } else if (globals->state_machine.getMainState_() == StateMachine::MainState::kEnableWaiting) {
    Enabling();
    return false;
  }
  // Gimbal状态检测
  switch (globals->state_machine.getGimbalState_()) {
    default:
      Unabling();
      return false;
    case StateMachine::GimbalState::kOffline:
      Unabling();
      return false;
    case StateMachine::GimbalState::kNoForce:
      Unabling();
      return false;
    case StateMachine::GimbalState::kEnableWaiting:
      // Unabling();
      Enabling();
      return false;
    case StateMachine::GimbalState::kNormal:
      return true;
    case StateMachine::GimbalState::kAimbot:
      return true;
    case StateMachine::GimbalState::kRadarAimbot:
      return true;
  }
}

/**
 * @brief 云台主循环
 */
void GimbalHero::GimbalUpdate() {
  AhrsUpdate();
  if (!EnableJudgement()) return;
  GimbalControl();
}