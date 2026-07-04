#include "aimbot_comm_can.hpp"

namespace rm::device {
constexpr f32 kDegToRad = static_cast<f32>(M_PI) / 180.0f;

AimbotCanCommunicator::AimbotCanCommunicator(rm::hal::CanInterface &can) : CanDevice(can, 0x170, 0x160) {}

u8 AimbotCanCommunicator::aimbot_state() const { return aimbot_state_; }

u8 AimbotCanCommunicator::aimbot_target() const { return aimbot_target_; }

f32 AimbotCanCommunicator::yaw() const { return yaw_; }

f32 AimbotCanCommunicator::pitch() const { return pitch_; }

f32 AimbotCanCommunicator::distance() const { return distance_; }

u8 AimbotCanCommunicator::nuc_start_flag() const { return nuc_start_flag_; }

f32 AimbotCanCommunicator::yaw_vel() const { return yaw_vel_; }

f32 AimbotCanCommunicator::pitch_vel() const { return pitch_vel_; }

f32 AimbotCanCommunicator::yaw_acc() const { return yaw_acc_; }

f32 AimbotCanCommunicator::pitch_acc() const { return pitch_acc_; }

void AimbotCanCommunicator::RxCallback(const hal::CanFrame *msg) {
  if (msg->rx_std_id == 0x170) {
    ReportStatus(kOk);
    aimbot_state_ = static_cast<u8>(msg->data[0]);
    aimbot_target_ = static_cast<u8>(msg->data[1]);
    yaw_ = modules::F16ToF32(static_cast<modules::f16>((static_cast<uint16_t>(msg->data[2]) << 8) | msg->data[3]));
    pitch_ = -modules::F16ToF32(static_cast<modules::f16>((static_cast<uint16_t>(msg->data[4]) << 8) | msg->data[5]));
    distance_ = modules::F16ToF32(static_cast<modules::f16>((static_cast<uint16_t>(msg->data[6]) << 8) | msg->data[7]));
    nuc_start_flag_ = (aimbot_state_ != 0U) ? 1U : 0U;
  } else if (msg->rx_std_id == 0x160) {
    ReportStatus(kOk);
    yaw_vel_ = modules::F16ToF32(static_cast<modules::f16>((static_cast<uint16_t>(msg->data[0]) << 8) | msg->data[1])) *
               kDegToRad;
    pitch_vel_ =
        modules::F16ToF32(static_cast<modules::f16>((static_cast<uint16_t>(msg->data[2]) << 8) | msg->data[3])) *
        kDegToRad;
    yaw_acc_ = modules::F16ToF32(static_cast<modules::f16>((static_cast<uint16_t>(msg->data[4]) << 8) | msg->data[5])) *
               kDegToRad;
    pitch_acc_ =
        modules::F16ToF32(static_cast<modules::f16>((static_cast<uint16_t>(msg->data[6]) << 8) | msg->data[7])) *
        kDegToRad;
  }
}

void AimbotCanCommunicator::UpdateControl(f32 yaw_degree, f32 pitch_degree, f32 roll_degree, u8 robot_id, u8 mode,
                                          u16 imu_count, f32 bullet_speed) {
  (void)imu_count;
  (void)bullet_speed;

  tx_buf_[0] = modules::F32ToF16(yaw_degree) >> 8;
  tx_buf_[1] = modules::F32ToF16(yaw_degree);
  tx_buf_[2] = modules::F32ToF16(pitch_degree) >> 8;
  tx_buf_[3] = modules::F32ToF16(pitch_degree);
  tx_buf_[4] = modules::F32ToF16(roll_degree) >> 8;
  tx_buf_[5] = modules::F32ToF16(roll_degree);
  const u8 id_bit = robot_id > 100 ? 1 : 0;
  const u8 mode_bits = mode & 0x7;  // 最低 2 位

  tx_buf_[6] = static_cast<u8>((id_bit << 7) | (mode_bits << 4));
  tx_buf_[7] = 0;

  this->can_->Write(0x150, tx_buf_, 8);
}

}  // namespace rm::device
