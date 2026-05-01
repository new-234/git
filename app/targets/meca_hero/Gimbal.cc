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
  target_yaw = -globals->ahrs.euler_angle().yaw;
  target_pitch = -globals->ahrs.euler_angle().pitch;
}

/**
 * @brief 姿态解算任务
 */
void GimbalHero::AhrsUpdate() {
  globals_no_dtcm.imu->Update();
  // 右手坐标系（前 x，左 y，上 z）
  hyperparameters_.gyro_rectification_ =
      0.00040 - globals->ahrs.euler_angle().pitch / 0.6644 * 0.0042;  //  这个放在姿态结算后面会有问题，待查
  globals->ahrs.Update(rm::modules::ImuData6Dof{
      -globals_no_dtcm.imu->gyro_y(), globals_no_dtcm.imu->gyro_x(),
      globals->gyro_z =
          globals->gyro_z_filter.apply(globals_no_dtcm.imu->gyro_z()) + hyperparameters_.gyro_rectification_,
      -globals_no_dtcm.imu->accel_y(), globals_no_dtcm.imu->accel_x(), globals_no_dtcm.imu->accel_z()});

  // 监测
  eulerangle_yaw = globals->ahrs.euler_angle().yaw;
  eulerangle_pitch = globals->ahrs.euler_angle().pitch;
  eulerangle_roll = globals->ahrs.euler_angle().roll;
  gyro_x = globals_no_dtcm.imu->gyro_x();
  gyro_y = globals_no_dtcm.imu->gyro_y();
  gyro_z = globals_no_dtcm.imu->gyro_z();
  accel_x = globals_no_dtcm.imu->accel_x();
  accel_y = globals_no_dtcm.imu->accel_y();
  accel_z = globals_no_dtcm.imu->accel_z();
}

void GimbalHero::AimbotControl() {}

void GimbalHero::RemoterControl() {
  // tc->rc->cc
  if ((globals->tc.data().mouse_x != 0 || globals->tc.data().mouse_y != 0) && globals->tc_manager.all_device_ok()) {
    target_yaw += static_cast<float>(globals->rc.right_x()) * 0.000005 +
                  static_cast<float>(globals->tc.data().mouse_x) / 32768 * 0.8;
    // snipe_yaw -= static_cast<float>(globals->rc.right_x()) * 0.000005 * 0.5 +
    //                  static_cast<float>(globals->tc.data().mouse_x) / 32768 * 0.8 * 0.5;
    target_pitch += static_cast<float>(globals->rc.right_y()) * 0.0000005 +
                    static_cast<float>(globals->tc.data().mouse_y) / 32768 * 0.5;
  } else if ((globals->rc.mouse_x() != 0 || globals->rc.mouse_y() != 0) && globals->rc_manager.all_device_ok()) {
    target_yaw += static_cast<float>(globals->rc.right_x()) * 0.000005 +
                  static_cast<float>(globals->rc.mouse_x()) / 32768.0 * 3;  // ≈0.003/per
    // snipe_yaw -= static_cast<float>(globals->rc.right_x()) * 0.000005 * 0.5 +
    //                  static_cast<float>(globals->tc.data().mouse_x) / 32768 * 0.8 * 0.5;
    target_pitch +=
        static_cast<float>(globals->rc.right_y()) * 0.0000005 + static_cast<float>(globals->rc.mouse_y() / 32768.0 * 3);
  } else {
    target_yaw += static_cast<float>(globals->rc.right_x()) * 0.000005;
    // +static_cast<float>(globals->custom_client->mouse_x()) * 0.000015;  // ≈0.003/per
    // snipe_yaw -= static_cast<float>(globals->rc.right_x()) * 0.000005 * 0.5 +
    //                  static_cast<float>(globals->tc.data().mouse_x) / 32768 * 0.8 * 0.5;
    target_pitch += static_cast<float>(globals->rc.right_y()) * 0.0000005;
    // +static_cast<float>(globals->custom_client->mouse_y()) * 0.000015;  // 0.00033/per
  }

  // pitch限位
  if (target_pitch < 0.05) {
    target_pitch = 0.05;
  } else if (target_pitch > 0.8) {
    target_pitch = 0.8;
  }
}

void GimbalHero::MotorUpdate() {
  // yawPID计算（双环）
  globals->pid_yaw_position.SetCircular(true).SetCircularCycle(3.141593 * 2);

  // yawPID计算
  globals->pid_yaw_position.Update(target_yaw, -globals->ahrs.euler_angle().yaw, 0.001);
  globals->pid_yaw_velocity.Update(globals->pid_yaw_position.out(),
                                   -globals_no_dtcm.imu->gyro_z() - hyperparameters_.gyro_rectification_, 0.001);

  // pitchPID计算
  globals->pid_pitch_position.Update(target_pitch, -globals->ahrs.euler_angle().pitch, 0.001);
  globals->pid_pitch_velocity.Update(globals->pid_pitch_position.out(), -globals_no_dtcm.imu->gyro_x(), 0.001);

  // 发送CAN
  globals->gimbal_motor_yaw.SetMitCommand(
      0, 0, (0.98 + 0.5 * globals->ahrs.euler_angle().pitch / 0.8) * globals->pid_yaw_velocity.out(), 0, 1.4);
  // globals->gimbal_motor_yaw.SetMitCommand(0, 0, 0, 0, 0);

  if (globals->state_machine.getSubState_() == StateMachine::SubState::kOverPower) {
    globals->gimbal_motor_pitch.SetCurrent(0);
  } else {
    globals->gimbal_motor_pitch.SetCurrent(
        static_cast<int16_t>((globals->pid_pitch_velocity.out()) /*+out_feedforward*/));
  }
}

void GimbalHero::GimbalControl() {
  // AimbotControl();
  RemoterControl();
  MotorUpdate();
}

bool GimbalHero::EnableJudgement() {
  // 监测
  yaw_pos = -globals->ahrs.euler_angle().yaw;
  yaw_vel = -globals_no_dtcm.imu->gyro_z() - hyperparameters_.gyro_rectification_;
  yaw_pos_pid_out = globals->pid_yaw_position.out();
  yaw_vel_pid_out = globals->pid_yaw_velocity.out();
  ::target_yaw = target_yaw;

  pitch_pos = -globals->ahrs.euler_angle().pitch;
  pitch_vel = globals_no_dtcm.imu->gyro_x();
  pitch_pos_pid_out = globals->pid_pitch_position.out();
  pitch_vel_pid_out = globals->pid_pitch_velocity.out();
  ::target_pitch = target_pitch;

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