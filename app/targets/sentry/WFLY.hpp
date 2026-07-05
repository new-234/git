#pragma once

#include <cstdint>

#include <librm.hpp>

namespace wfly {

constexpr int16_t kChannelMin = 321;
constexpr int16_t kChannelOffset = 992;
constexpr int16_t kChannelMax = 1663;
constexpr float kChannelRangeHalf = static_cast<float>(kChannelMax - kChannelOffset);

constexpr uint8_t kSwitchUnknown = 0;
constexpr uint8_t kSwitchUp = 1;
constexpr uint8_t kSwitchDown = 2;
constexpr uint8_t kSwitchMid = 3;

struct RemoteData {
  int16_t ch[4]{};
  uint8_t s[8]{};
  int16_t knob[4]{};
};

inline float NormalizedLimit(float input) {
  if (input > 1.0f) {
    return 1.0f;
  }
  if (input < -1.0f) {
    return -1.0f;
  }
  return input;
}

}  // namespace wfly

namespace rc_ch {

constexpr int LEFT_X = 3;
constexpr int LEFT_Y = 2;
constexpr int RIGHT_X = 0;
constexpr int RIGHT_Y = 1;

constexpr int SA = 4;
constexpr int SB = 5;
constexpr int SC = 6;
constexpr int SD = 7;
constexpr int SE = 8;
constexpr int SF = 9;
constexpr int SG = 10;
constexpr int SH = 11;

constexpr int LS = 12;
constexpr int RS = 13;
constexpr int LD = 14;
constexpr int RD = 15;

}  // namespace rc_ch

enum class SwitchPosition : uint8_t {
  kUnknown = wfly::kSwitchUnknown,
  kUp = wfly::kSwitchUp,
  kDown = wfly::kSwitchDown,
  kMid = wfly::kSwitchMid,
};

class WflyET16s : public rm::device::Sbus {
 public:
  explicit WflyET16s(rm::hal::SerialInterface &serial) : Sbus(serial) {}

  [[nodiscard]] wfly::RemoteData remote_data() const {
    wfly::RemoteData data{};
    data.ch[0] = static_cast<int16_t>(channel(rc_ch::RIGHT_X)) - wfly::kChannelOffset;
    data.ch[1] = -(static_cast<int16_t>(channel(rc_ch::RIGHT_Y)) - wfly::kChannelOffset);
    data.ch[2] = -(static_cast<int16_t>(channel(rc_ch::LEFT_Y)) - wfly::kChannelOffset);
    data.ch[3] = -(static_cast<int16_t>(channel(rc_ch::LEFT_X)) - wfly::kChannelOffset) - 3;

    for (int i = 0; i < 8; ++i) {
      data.s[i] = SwitchValue(rc_ch::SA + i);
    }
    for (int i = 0; i < 4; ++i) {
      data.knob[i] = -(static_cast<int16_t>(channel(rc_ch::LS + i)) - wfly::kChannelMax);
    }
    return data;
  }

  [[nodiscard]] float RemoteChannalRightX() const {
    return wfly::NormalizedLimit(static_cast<float>(remote_data().ch[0]) / wfly::kChannelRangeHalf);
  }
  [[nodiscard]] float RemoteChannalRightY() const {
    return wfly::NormalizedLimit(-static_cast<float>(remote_data().ch[1]) / wfly::kChannelRangeHalf);
  }
  [[nodiscard]] float RemoteChannalLeftX() const {
    return wfly::NormalizedLimit(static_cast<float>(remote_data().ch[3]) / wfly::kChannelRangeHalf);
  }
  [[nodiscard]] float RemoteChannalLeftY() const {
    return wfly::NormalizedLimit(-static_cast<float>(remote_data().ch[2]) / wfly::kChannelRangeHalf);
  }

  [[nodiscard]] float left_x() const { return RemoteChannalLeftX(); }
  [[nodiscard]] float left_y() const { return RemoteChannalLeftY(); }
  [[nodiscard]] float right_x() const { return RemoteChannalRightX(); }
  [[nodiscard]] float right_y() const { return RemoteChannalRightY(); }

  [[nodiscard]] SwitchPosition switch_position(i8 channel_num) const {
    switch (SwitchValue(channel_num)) {
      case wfly::kSwitchUp:
        return SwitchPosition::kUp;
      case wfly::kSwitchDown:
        return SwitchPosition::kDown;
      case wfly::kSwitchMid:
        return SwitchPosition::kMid;
      default:
        return SwitchPosition::kUnknown;
    }
  }

  [[nodiscard]] bool SwitchIsUp(i8 channel_num) const { return switch_position(channel_num) == SwitchPosition::kUp; }
  [[nodiscard]] bool SwitchIsMid(i8 channel_num) const { return switch_position(channel_num) == SwitchPosition::kMid; }
  [[nodiscard]] bool SwitchIsDown(i8 channel_num) const {
    return switch_position(channel_num) == SwitchPosition::kDown;
  }

  [[nodiscard]] i16 wheel_position(i8 channel_num) const {
    if (channel_num >= rc_ch::LS && channel_num <= rc_ch::RD) {
      return static_cast<i16>(channel(channel_num)) - wfly::kChannelOffset;
    }
    return 0;
  }

 private:
  [[nodiscard]] uint8_t SwitchValue(i8 channel_num) const {
    if (channel_num < rc_ch::SA || channel_num > rc_ch::SH) {
      return wfly::kSwitchUnknown;
    }

    const auto value = channel(channel_num);
    if (value > wfly::kChannelOffset && value <= wfly::kChannelMax) {
      return wfly::kSwitchDown;
    }
    if (value < wfly::kChannelOffset && value >= wfly::kChannelMin) {
      return wfly::kSwitchUp;
    }
    if (value == wfly::kChannelOffset) {
      return wfly::kSwitchMid;
    }
    return wfly::kSwitchUnknown;
  }
};
