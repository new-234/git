#include "Gimbal.hpp"
#include "globals.hpp"

void GimbalHero::Enabling() {
  globals->gimbal_motor_yaw.SendInstruction(rm::device::DmMotorInstructions::kClearError);
  globals->gimbal_motor_yaw.SendInstruction(rm::device::DmMotorInstructions::kEnable);
  globals->gimbal_motor_pitch.SetCurrent(0);
  // if (!(globals->rc.switch_r() == rm::device::DR16::SwitchPosition::kUp ||
  //       globals->tc.data().switch_position == rm::device::VT03::SwitchPosition::S))
  //   target_yaw_ = -(monitor_.euler_yaw_ - globals->gimbal_motor_yaw.pos() - 0.288);
}

void GimbalHero::Unabling() {
  globals->gimbal_motor_yaw.SetMitCommand(0, 0, 0, 0, 0);
  globals->gimbal_motor_pitch.SetCurrent(0);
  globals->gimbal_motor_yaw.SendInstruction(rm::device::DmMotorInstructions::kDisable);
  target_yaw_ = -monitor_.euler_yaw_;
  target_pitch_ = -monitor_.euler_pitch_;
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1690);
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

  if (globals->state_machine.getLastSubState_() != StateMachine::SubState::kSnipe &&
      globals->state_machine.getSubState_() == StateMachine::SubState::kSnipe) {
    temp_euler_angle_.euler_yaw_ = globals->ahrs.euler_angle().yaw - globals->HipnucImu.yaw();
    temp_euler_angle_.euler_pitch_ = globals->ahrs.euler_angle().pitch - globals->HipnucImu.pitch();
    temp_euler_angle_.euler_roll_ = globals->ahrs.euler_angle().roll - globals->HipnucImu.roll();
  }
  if (globals->state_machine.getLastSubState_() == StateMachine::SubState::kSnipe &&
      globals->state_machine.getSubState_() != StateMachine::SubState::kSnipe) {
    temp_euler_angle_.euler_yaw_ =
        globals->HipnucImu.yaw() + temp_euler_angle_.euler_yaw_ - globals->ahrs.euler_angle().yaw;
    temp_euler_angle_.euler_pitch_ =
        globals->HipnucImu.pitch() + temp_euler_angle_.euler_pitch_ - globals->ahrs.euler_angle().pitch;
    temp_euler_angle_.euler_roll_ =
        globals->HipnucImu.roll() + temp_euler_angle_.euler_roll_ - globals->ahrs.euler_angle().roll;
  }
  if (globals->state_machine.getSubState_() == StateMachine::SubState::kSnipe) {
    monitor_.euler_yaw_ = globals->HipnucImu.yaw() + temp_euler_angle_.euler_yaw_;
    monitor_.euler_pitch_ = globals->HipnucImu.pitch() + temp_euler_angle_.euler_pitch_;
    monitor_.euler_roll_ = globals->HipnucImu.roll() + temp_euler_angle_.euler_roll_;
    monitor_.gyro_x_ = globals->HipnucImu.gyro_x();
    monitor_.gyro_y_ = globals->HipnucImu.gyro_y();
    monitor_.gyro_z_ = globals->HipnucImu.gyro_z();
  } else {
    monitor_.euler_yaw_ = globals->ahrs.euler_angle().yaw + temp_euler_angle_.euler_yaw_;
    monitor_.euler_pitch_ = globals->ahrs.euler_angle().pitch + temp_euler_angle_.euler_pitch_;
    monitor_.euler_roll_ = globals->ahrs.euler_angle().roll + temp_euler_angle_.euler_roll_;
    monitor_.gyro_x_ = globals_no_dtcm.imu->gyro_x();
    monitor_.gyro_y_ = globals_no_dtcm.imu->gyro_y();
    monitor_.gyro_z_ = globals_no_dtcm.imu->gyro_z();
  }
  globals->state_machine.setLastSubState(globals->state_machine.getSubState_());

  // 监测
  eulerangle_yaw = -monitor_.euler_yaw_;
  eulerangle_pitch = -monitor_.euler_pitch_;
  eulerangle_roll = monitor_.euler_roll_;
  gyro_x = globals->HipnucImu.gyro_x();
  gyro_y = globals->HipnucImu.gyro_y();
  gyro_z = globals->HipnucImu.gyro_z();
  accel_x = globals->HipnucImu.acc_x();
  accel_y = globals->HipnucImu.acc_y();
  accel_z = globals->HipnucImu.acc_z();
}

void GimbalHero::DroneAimControl() {
  if (globals->refsub_manager.all_device_ok()) {
    target_yaw_ = -globals->refSub->data().drone_2_hero.cmd_yaw_angle;
  }
}

void GimbalHero::AimbotControl() {
  if (globals->aimbot_manager.all_device_ok() && globals->aimbot_can_communicator.aimbot_target() != 0) {
    target_yaw_ = -globals->aimbot_can_communicator.yaw();
    target_pitch_ = -globals->aimbot_can_communicator.pitch();
  }
}

void GimbalHero::RadarAimbotControl() {
  static int count{0};
  if (count < 1000) {
    count++;
  } else {
    if (globals->radar_can_communicator.vaild() && globals->radar_can_communicator.fresh() &&
        globals->radar_manager.all_device_ok()) {
      target_yaw_ = -monitor_.euler_yaw_ - globals->radar_can_communicator.yaw_mard();
      target_pitch_ = globals->radar_can_communicator.pitch_mard();
      count = 0;
    }
  }
}

void GimbalHero::NormalRemoter() {
  // tc->rc->cc
  last_target_pitch_ = target_pitch_;
  if (/*(globals->tc.data().mouse_x != 0 || globals->tc.data().mouse_y != 0) && */globals->tc_manager.all_device_ok()) {
    target_yaw_ += static_cast<float>(globals->tc.data().mouse_x) / 32768 * 0.8;
    target_pitch_ += static_cast<float>(globals->tc.data().mouse_y) / 32768 * 0.5;
  } else if (/*(globals->rc.mouse_x() != 0 || globals->rc.mouse_y() != 0) && */globals->rc_manager.all_device_ok()) {
    target_yaw_ +=
        static_cast<float>(globals->rc.right_x()) * 0.000005 + static_cast<float>(globals->rc.mouse_x()) / 32768.0 * 3;
    target_pitch_ +=
        static_cast<float>(globals->rc.right_y()) * 0.0000005 + static_cast<float>(globals->rc.mouse_y() / 32768.0 * 3);
  } else {
    target_yaw_ += 0;
    target_pitch_ += 0;
  }
}

void GimbalHero::SnipeRemoter() {
  // tc->rc->cc
  last_target_pitch_ = target_pitch_;
  if (globals->tc_manager.all_device_ok()) {
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
    if (globals->rc.key(rm::device::DR16::Key::kW)) {
      target_pitch_ += 0.000005;
    }
    if (globals->rc.key(rm::device::DR16::Key::kS)) {
      target_pitch_ -= 0.000005;
    }
    if (globals->rc.key(rm::device::DR16::Key::kD)) {
      target_yaw_ += 0.000005;
    }
    if (globals->rc.key(rm::device::DR16::Key::kA)) {
      target_yaw_ -= 0.000005;
    }
  } else {
    target_yaw_ += 0;
    target_pitch_ += 0;
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
          RadarAimbotControl();
          break;
        case StateMachine::GimbalState::kAimbot:
          AimbotControl();
          break;
        case StateMachine::GimbalState::kDroneAim:
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
          RadarAimbotControl();
          break;

        case StateMachine::GimbalState::kDroneAim:
          DroneAimControl();
          break;

        case StateMachine::GimbalState::kNormal:
          SnipeRemoter();
          break;
      }

      break;
  }

  // pitch限位
  if (target_pitch_ < 0.02) {
    target_pitch_ = 0.02;
  } else if (target_pitch_ > 0.8) {
    target_pitch_ = 0.8;
  }
}

void GimbalHero::MotorUpdate() {
  // yawPID计算（双环）
  globals->pid_yaw_position.SetCircular(true).SetCircularCycle(3.141593 * 2);
  // if (globals->state_machine.getSubState_() == StateMachine::SubState::kSnipe) {
  //   ChangePID(globals->pid_yaw_position, 28.5, 0, 0.01);
  //   ChangePID(globals->pid_yaw_velocity, 9, 0, 0.00001);
  // } else {
  //   ChangePID(globals->pid_yaw_position, 28, 0, 0.01);
  //   ChangePID(globals->pid_yaw_velocity, 9, 0, 0.00001);
  // }
  // yawPID计算
  globals->pid_yaw_position.Update(target_yaw_, -monitor_.euler_yaw_, 0.001);
  globals->pid_yaw_velocity.Update(globals->pid_yaw_position.out(),
                                   -monitor_.gyro_z_ /*- hyperparameters_.gyro_rectification_*/, 0.001);

  // 发送CAN
  globals->gimbal_motor_yaw.SetMitCommand(
      0, 0, /*(1 - 0.1 * globals->ahrs.euler_angle().pitch / 0.8) **/ globals->pid_yaw_velocity.out(), 0, 1.15);
  // globals->gimbal_motor_yaw.SetMitCommand(0, 0, 0, 0, 0);

  if (globals->state_machine.getSubState_() == StateMachine::SubState::kOverPower) {
    globals->gimbal_motor_pitch.SetCurrent(0);
    target_pitch_ = -globals->HipnucImu.pitch();

  } else if (target_pitch_ - last_target_pitch_ > 0) {
    ChangePID(globals->pid_pitch_position, 65, 0.01, 0.0);
    ChangePID(globals->pid_pitch_velocity, 4500, 20, 50);
    globals->pid_pitch_position.Update(target_pitch_, -monitor_.euler_pitch_, 0.001);
    globals->pid_pitch_velocity.Update(globals->pid_pitch_position.out(), -monitor_.gyro_x_, 0.001);

    positive_ff_ = 3000;

    negaive_ff_ = 0;

    globals->gimbal_motor_pitch.SetCurrent(
        static_cast<int16_t>(globals->pid_pitch_velocity.out() + positive_ff_ + negaive_ff_));

  } else if (target_pitch_ - last_target_pitch_ < 0) {
    ChangePID(globals->pid_pitch_position, 20, 0.01, 0.00001);
    ChangePID(globals->pid_pitch_velocity, 3000, 20, 30);
    globals->pid_pitch_position.Update(target_pitch_, -monitor_.euler_pitch_, 0.001);
    globals->pid_pitch_velocity.Update(globals->pid_pitch_position.out(), -monitor_.gyro_x_, 0.001);

    negaive_ff_ = -1600;

    positive_ff_ = 0;
    globals->gimbal_motor_pitch.SetCurrent(
        static_cast<int16_t>(globals->pid_pitch_velocity.out() + positive_ff_ + negaive_ff_));
  } else if (target_pitch_ == last_target_pitch_) {
    globals->gimbal_motor_pitch.SetCurrent(0);
    target_pitch_ = -globals->HipnucImu.pitch();
  }
  last_pitch_ = globals->HipnucImu.pitch();
}

void GimbalHero::ServoMotorUpdate() {
  switch (globals->state_machine.getSubState_()) {
    default:
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1500);
      break;
    case (StateMachine::SubState::kSnipe):
      // servo_angle_ = 1730 /*(1730 - 1440) * (-globals->ahrs.euler_angle().pitch / 0.8) + 1440*/;
      if (globals->tc.data().keyboard_key & rm::device::VT03::KeyboardKey::kShift &&
          globals->tc.data().keyboard_key & rm::device::VT03::KeyboardKey::kQ) {
        servo_angle_ += 0.1;
      }
      if (globals->tc.data().keyboard_key & rm::device::VT03::KeyboardKey::kShift &&
          globals->tc.data().keyboard_key & rm::device::VT03::KeyboardKey::kE) {
        servo_angle_ -= 0.1;
      }
      if (servo_angle_ > 1800) {
        servo_angle_ = 1800;
      } else if (servo_angle_ < 1500) {
        servo_angle_ = 1500;
      }
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, static_cast<int>(servo_angle_));
      break;
  }
}

void GimbalHero::ScopeMotorUpdate() {
  // 望远镜状态机
  if (globals->state_machine.scope_state == 1) {
    // 状态1：伸出（负方向旋转）
    globals->pid_scope_velocity.Update(-1000, globals->gimbal_telescope.rpm());  // 目标转速: -1000 rpm
    globals->gimbal_telescope.SetCurrent(globals->pid_scope_velocity.out());

    // 到位检测：转速 < 30 rpm 且持续计数 > 120
    if (fabs(globals->gimbal_telescope.rpm()) < 30 && globals->state_machine.scope_count <= 120) {
      globals->state_machine.scope_count++;
    }
    if (globals->state_machine.scope_count > 120) {
      globals->state_machine.scope_state++;  // 切换到状态2
      globals->state_machine.scope_count = 0;
      globals->gimbal_telescope.SetCurrent(0);
    }
  } else if (globals->state_machine.scope_state == 2) {
    // 状态2：保持（停止）
    globals->gimbal_telescope.SetCurrent(0);
  } else if (globals->state_machine.scope_state == 3) {
    // 状态3：收回（正方向旋转）
    globals->pid_scope_velocity.Update(1000, globals->gimbal_telescope.rpm());  // 目标转速: 1000 rpm
    globals->gimbal_telescope.SetCurrent(globals->pid_scope_velocity.out());

    // 到位检测
    if (fabs(globals->gimbal_telescope.rpm()) < 30 && globals->state_machine.scope_count <= 120) {
      globals->state_machine.scope_count++;
    }
    if (globals->state_machine.scope_count > 120) {
      globals->state_machine.scope_state++;  // 切换到状态0
      globals->state_machine.scope_count = 0;
      globals->gimbal_telescope.SetCurrent(0);
    }
  } else {
    // 状态0：停止
    globals->gimbal_telescope.SetCurrent(0);
  }
}

void GimbalHero::GimbalControl() {
  RemoterControl();
  MotorUpdate();
  ServoMotorUpdate();
  ScopeMotorUpdate();
}

bool GimbalHero::EnableJudgement() {
  // 监测
  yaw_pos = -globals->HipnucImu.yaw();
  yaw_vel = -globals->HipnucImu.gyro_z();
  yaw_pos_pid_out = globals->pid_yaw_position.out();
  yaw_vel_pid_out = globals->pid_yaw_velocity.out();
  ::target_yaw = target_yaw_;

  pitch_pos = -globals->HipnucImu.pitch();
  pitch_vel = globals->HipnucImu.gyro_x();
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
    case StateMachine::GimbalState::kDroneAim:
      return true;
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