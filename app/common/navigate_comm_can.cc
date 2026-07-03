#include "navigate_comm_can.hpp"

namespace rm::device {

NavigateCanCommunicator::NavigateCanCommunicator(rm::hal::CanInterface &can) : CanDevice(can, 0x180, 0x190) {}

f32 NavigateCanCommunicator::chassis_target_x() const { return chassis_target_x_; }

f32 NavigateCanCommunicator::chassis_target_y() const { return chassis_target_y_; }

f32 NavigateCanCommunicator::chassis_target_w() const { return chassis_target_w_; }

f32 NavigateCanCommunicator::target_yaw_speed() const { return target_yaw_speed_; }

u8 NavigateCanCommunicator::scan_mode() const { return scan_mode_; }

u8 NavigateCanCommunicator::perception_flag() const { return perception_flag_; }

u8 NavigateCanCommunicator::aimbot_mode() const { return aimbot_mode_; }

u8 NavigateCanCommunicator::outpost_mode() const { return outpost_mode_; }

namespace {
i16 ToCentidegree(rm::f32 value) { return static_cast<i16>(value * 100.0f); }
}  // namespace

void NavigateCanCommunicator::RxCallback(const hal::CanFrame *msg) {
  if (msg->rx_std_id == 0x190) {
    ReportStatus(kOk);
    chassis_target_x_ = static_cast<f32>(static_cast<int8_t>(msg->data[0])) / 127.0f;
    chassis_target_y_ = static_cast<f32>(static_cast<int8_t>(msg->data[1])) / 127.0f;
    target_yaw_speed_ = static_cast<f32>(static_cast<i16>((static_cast<u16>(msg->data[2]) << 8) | msg->data[3])) / 100.0f;
    perception_flag_ = msg->data[4] & 0x0f;
    scan_mode_ = (msg->data[4] & 0x10) >> 4;
    aimbot_mode_ = (msg->data[4] & 0x20) >> 5;
    outpost_mode_ = (msg->data[4] & 0x40) >> 6;
    chassis_target_w_ = static_cast<f32>(static_cast<int8_t>(msg->data[5])) / 127.0f;
  }
}

void NavigateCanCommunicator::UpdateControl(f32 small_gimbal_yaw_degree, f32 small_gimbal_pitch_degree,
                                            f32 aimbot_distance) {
  const i16 yaw = ToCentidegree(small_gimbal_yaw_degree);
  const i16 pitch = ToCentidegree(small_gimbal_pitch_degree);
  const i16 distance = ToCentidegree(aimbot_distance);

  tx_buf_[0] = static_cast<u8>((yaw >> 8) & 0xff);
  tx_buf_[1] = static_cast<u8>(yaw & 0xff);
  tx_buf_[2] = static_cast<u8>((pitch >> 8) & 0xff);
  tx_buf_[3] = static_cast<u8>(pitch & 0xff);
  tx_buf_[4] = static_cast<u8>((distance >> 8) & 0xff);
  tx_buf_[5] = static_cast<u8>(distance & 0xff);
  tx_buf_[6] = 0;
  tx_buf_[7] = 0;

  this->can_->Write(0x180, tx_buf_, 8);
}

}  // namespace rm::device
