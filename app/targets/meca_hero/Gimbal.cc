#include "Gimbal.hpp"
#include "globals.hpp"

void GimbalHero::Enabling() {
  globals->gimbal_motor_yaw.SendInstruction(rm::device::DmMotorInstructions::kClearError);
  globals->gimbal_motor_yaw.SendInstruction(rm::device::DmMotorInstructions::kEnable);
}

void GimbalHero::Unabling() {
  globals->gimbal_motor_yaw.SetMitCommand(0, 0, 0, 0, 0);
  globals->gimbal_motor_pitch.SetCurrent(0);
  globals->gimbal_motor_yaw.SendInstruction(rm::device::DmMotorInstructions::kDisable);
  target_yaw = globals->ahrs.euler_angle().yaw;
  target_pitch = globals->ahrs.euler_angle().pitch;
}

/**
 * @brief 姿态解算任务
 */
void GimbalHero::AhrsUpdate() {
  globals_no_dtcm.imu->Update();
  // 右手坐标系（前 x，左 y，上 z）
  globals->ahrs.Update(rm::modules::ImuData6Dof{
      -globals_no_dtcm.imu->gyro_y(),globals_no_dtcm.imu->gyro_x(),
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

bool GimbalHero::EnableJudgement() {
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
      Unabling();
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
}