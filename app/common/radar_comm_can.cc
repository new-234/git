#include "radar_comm_can.hpp"

namespace rm::device {

RadarCanCommunicator::RadarCanCommunicator(rm::hal::CanInterface &can) : CanDevice(can, 0x301) {}

void RadarCanCommunicator::RxCallback(const hal::CanFrame *msg) {
  if (msg->rx_std_id == 0x301) {
    ReportStatus(kOk);
    pitch_mard_ = static_cast<u16>((static_cast<uint16_t>(msg->data[1]) << 8) | msg->data[0]);
    planner_distance_mm_ = static_cast<u16>((static_cast<uint16_t>(msg->data[3]) << 8) | msg->data[2]);
    yaw_mard_ = static_cast<i16>((static_cast<uint16_t>(msg->data[5]) << 8) | msg->data[4]);
    status_ = static_cast<u8>(msg->data[6]);
    vaild_ = (status_ & 0x01) != 0;  // 假设最低位表示数据有效性
    fresh_ = (status_ & 0x02) != 0;  // 假设第二位表示数据新鲜度
    counter_ = static_cast<u8>(msg->data[7]);
  } else {
    vaild_ = false;
    fresh_ = false;
  }
}

void RadarCanCommunicator::UpdateInfo(const u8 robot_team) {
  if (robot_team > 100) {
    tx_buf_[0] = 0x01;
  } else {
    tx_buf_[0] = 0;
  }
  for (int i = 1; i < 8; i++) {
    tx_buf_[i] = 0;
  }
  this->can_->Write(0x302, tx_buf_, 8);
}

}  // namespace rm::device