#include "aimbot_comm_can.hpp"

#include <cmath>

namespace rm::device {
namespace {
constexpr f32 kDegToRad = static_cast<f32>(M_PI) / 180.0f;

constexpr u16 kAimbotControlId = 0x130;
constexpr u16 kLegacyAimbotControlId = 0x170;
constexpr u16 kAimbotFeedForwardId = 0x160;
constexpr u16 kGimbalImuId = 0x148;

modules::f16 PackF16(f32 value) { return modules::F32ToF16(value); }

f32 UnpackF16(u8 high, u8 low) {
  return modules::F16ToF32(static_cast<modules::f16>((static_cast<u16>(high) << 8U) | low));
}
}  // namespace

AimbotCanCommunicator::AimbotCanCommunicator(rm::hal::CanInterface &can)
    : CanDevice(can, kAimbotControlId, kLegacyAimbotControlId, kAimbotFeedForwardId) {}

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
  if (msg->rx_std_id == kAimbotControlId || msg->rx_std_id == kLegacyAimbotControlId) {
    ReportStatus(kOk);
    const f32 yaw = UnpackF16(msg->data[2], msg->data[3]);
    const f32 pitch = -UnpackF16(msg->data[4], msg->data[5]);
    const f32 distance = UnpackF16(msg->data[6], msg->data[7]);

    if (!std::isfinite(yaw) || !std::isfinite(pitch)) {
      return;
    }

    aimbot_state_ = static_cast<u8>(msg->data[0]);
    aimbot_target_ = static_cast<u8>(msg->data[1]);
    yaw_ = yaw;
    pitch_ = pitch;
    distance_ = distance;
    nuc_start_flag_ = (aimbot_state_ != 0U) ? 1U : 0U;
  } else if (msg->rx_std_id == kAimbotFeedForwardId) {
    ReportStatus(kOk);
    yaw_vel_ = UnpackF16(msg->data[0], msg->data[1]) * kDegToRad;
    pitch_vel_ = -UnpackF16(msg->data[2], msg->data[3]) * kDegToRad;
    yaw_acc_ = UnpackF16(msg->data[4], msg->data[5]) * kDegToRad;
    pitch_acc_ = -UnpackF16(msg->data[6], msg->data[7]) * kDegToRad;
  }
}

void AimbotCanCommunicator::UpdateControl(f32 yaw_degree, f32 pitch_degree, f32 roll_degree, u8 robot_id, u8 mode) {
  const auto yaw = PackF16(yaw_degree);
  const auto pitch = PackF16(pitch_degree);
  const auto roll = PackF16(roll_degree);
  tx_buf_[0] = static_cast<u8>((yaw >> 8U) & 0xffU);
  tx_buf_[1] = static_cast<u8>(yaw & 0xffU);
  tx_buf_[2] = static_cast<u8>((pitch >> 8U) & 0xffU);
  tx_buf_[3] = static_cast<u8>(pitch & 0xffU);
  tx_buf_[4] = static_cast<u8>((roll >> 8U) & 0xffU);
  tx_buf_[5] = static_cast<u8>(roll & 0xffU);

  u8 flags = robot_id > 100 ? 0x80U : 0x00U;
  if (mode <= 6U) {
    flags |= static_cast<u8>((mode & 0x7U) << 4U);
  }
  if (flags == 0xf0U) {
    flags = 0x10U;
  }
  tx_buf_[6] = flags;
  tx_buf_[7] = 0;

  can_->Write(kGimbalImuId, tx_buf_, 8);
}

}  // namespace rm::device
