#include "navigate_comm_can.hpp"

namespace rm::device {
namespace {
constexpr u16 kNavigateControlId = 0x131;
constexpr u16 kLegacyNavigateControlId = 0x190;
constexpr u16 kGimbalSendNucId = 0x149;

i16 ToCentidegree(rm::f32 value) { return static_cast<i16>(value * 100.0f); }
}  // namespace

NavigateCanCommunicator::NavigateCanCommunicator(rm::hal::CanInterface &can)
    : CanDevice(can, kNavigateControlId, kLegacyNavigateControlId) {}

f32 NavigateCanCommunicator::chassis_target_x() const { return chassis_target_x_; }

f32 NavigateCanCommunicator::chassis_target_y() const { return chassis_target_y_; }

f32 NavigateCanCommunicator::chassis_target_w() const { return chassis_target_w_; }

f32 NavigateCanCommunicator::target_yaw_speed() const { return target_yaw_speed_; }

u8 NavigateCanCommunicator::scan_mode() const { return scan_mode_; }

u8 NavigateCanCommunicator::perception_flag() const { return perception_flag_; }

u8 NavigateCanCommunicator::aimbot_mode() const { return aimbot_mode_; }

u8 NavigateCanCommunicator::outpost_mode() const { return outpost_mode_; }

void NavigateCanCommunicator::RxCallback(const hal::CanFrame *msg) {
  if (msg->rx_std_id == kNavigateControlId || msg->rx_std_id == kLegacyNavigateControlId) {
    ReportStatus(kOk);
    chassis_target_x_ = static_cast<f32>(static_cast<int8_t>(msg->data[0])) / 127.0f;
    chassis_target_y_ = static_cast<f32>(static_cast<int8_t>(msg->data[1])) / 127.0f;
    target_yaw_speed_ = static_cast<f32>(static_cast<i16>((static_cast<u16>(msg->data[2]) << 8U) | msg->data[3])) / 100.0f;
    perception_flag_ = msg->data[4] & 0x0fU;
    scan_mode_ = (msg->data[4] & 0x10U) >> 4U;
    aimbot_mode_ = (msg->data[4] & 0x20U) >> 5U;
    outpost_mode_ = (msg->data[4] & 0x40U) >> 6U;
    chassis_target_w_ = static_cast<f32>(static_cast<int8_t>(msg->data[5])) / 127.0f;
  }
}

void NavigateCanCommunicator::UpdateControl(f32 small_gimbal_yaw_degree, f32 small_gimbal_pitch_degree,
                                            f32 aimbot_distance) {
  const i16 yaw = ToCentidegree(small_gimbal_yaw_degree);
  const i16 pitch = ToCentidegree(small_gimbal_pitch_degree);
  const i16 distance = ToCentidegree(aimbot_distance);

  tx_buf_[0] = static_cast<u8>((yaw >> 8U) & 0xffU);
  tx_buf_[1] = static_cast<u8>(yaw & 0xffU);
  tx_buf_[2] = static_cast<u8>((pitch >> 8U) & 0xffU);
  tx_buf_[3] = static_cast<u8>(pitch & 0xffU);
  tx_buf_[4] = static_cast<u8>((distance >> 8U) & 0xffU);
  tx_buf_[5] = static_cast<u8>(distance & 0xffU);
  tx_buf_[6] = 0;
  tx_buf_[7] = 0;

  can_->Write(kGimbalSendNucId, tx_buf_, 8);
}

}  // namespace rm::device
