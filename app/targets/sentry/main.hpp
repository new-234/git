#ifndef APP_TARGETS_SENTRY_MAIN_HPP
#define APP_TARGETS_SENTRY_MAIN_HPP

#include <cstdint>
#include <new>

#include <librm.hpp>

#include "common/aimbot_comm_can.hpp"
#include "common/bsp/buzzer.hpp"
#include "common/bsp/ws2812.hpp"
#include "common/navigate_comm_can.hpp"

#include "Referee.hpp"
#include "WFLY.hpp"

enum class RobotMode : uint8_t {
  kNoForce = 0,
  kTest,
  kMatch,
};

template <typename T>
struct DmaObjectStorage {
  alignas(32) alignas(T) uint8_t bytes[(sizeof(T) + 31u) & ~31u]{};

  T *get() { return reinterpret_cast<T *>(bytes); }
  const T *get() const { return reinterpret_cast<const T *>(bytes); }
};

struct alignas(32) DmaAdcSample {
  uint16_t value{0};
};

struct alignas(32) GlobalsNoDtcm {
  DmaAdcSample adc;
  DmaObjectStorage<rm::hal::Serial<128>> ident_uart;
  DmaObjectStorage<rm::hal::Serial<25>> dbus;
  DmaObjectStorage<rm::hal::Serial<128>> referee_uart;
  DmaObjectStorage<rm::hal::Serial<256>> hipnuc_uart;
};

struct GlobalWarehouse {
  Buzzer buzzer;
  rm::modules::BuzzerController<rm::modules::buzzer_melody::Silent, rm::modules::buzzer_melody::Startup,
                                rm::modules::buzzer_melody::Success, rm::modules::buzzer_melody::Error,
                                rm::modules::buzzer_melody::Beeps<1>>
      buzzer_controller;

  WS2812Led led;
  rm::modules::RgbLedController<rm::modules::led_pattern::Off, rm::modules::led_pattern::RedFlash,
                                rm::modules::led_pattern::GreenBreath, rm::modules::led_pattern::RgbFlow>
      led_controller;

  rm::hal::Can *can1{nullptr};
  rm::hal::Can *can2{nullptr};
  rm::hal::Can *can3{nullptr};

  rm::device::AimbotCanCommunicator *aimbot_communicator{nullptr};
  rm::device::NavigateCanCommunicator *navigate_communicator{nullptr};

  rm::hal::Serial<128> *ident_uart{nullptr};
  rm::hal::Serial<25> *dbus{nullptr};
  rm::hal::Serial<128> *referee_uart{nullptr};
  rm::hal::Serial<256> *hipnuc_uart{nullptr};

  WflyET16s *wfly_et16s{nullptr};
  rm::device::RxReferee *rx_referee{nullptr};
  rm::device::Referee<rm::device::RefereeRevision::kNewV120> *referee_data{nullptr};
  rm::device::BMI088 *imu{nullptr};
  rm::device::HipnucImu *hipnuc_imu{nullptr};

  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *down_yaw_motor{nullptr};
  rm::device::M3508 *wheel_lf{nullptr};
  rm::device::M3508 *wheel_rf{nullptr};
  rm::device::M3508 *wheel_lb{nullptr};
  rm::device::M3508 *wheel_rb{nullptr};

  rm::device::GM6020 *up_yaw_motor{nullptr};
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};
  rm::device::M3508 *friction_left{nullptr};
  rm::device::M3508 *friction_right{nullptr};
  rm::device::M3508 *dial_motor{nullptr};

  rm::device::DeviceManager<1> device_rc;
  rm::device::DeviceManager<2> device_nuc;
  rm::device::DeviceManager<1> device_big_gimbal;
  rm::device::DeviceManager<4> device_chassis;
  rm::device::DeviceManager<2> device_small_gimbal;
  rm::device::DeviceManager<3> device_shoot;

  rm::modules::MahonyAhrs ahrs{1000.0f};

  RobotMode robot_mode{RobotMode::kNoForce};

  uint32_t tick{0};
  uint8_t aim_mode{1};
  float vbus{0.0f};

  void Init();
  void Loop1kHz();

  bool RcOnline() const;
  bool ShooterPowerEnabled() const;
  uint16_t HeatLimit() const;
  uint16_t HeatCurrent() const;

 private:
  void UpdateBoardSensors();
  void UpdateDeviceManagers();
  void UpdateModeFromRemote();
  void SendVisionFrames();
  void UpdateIndicators();
};

extern GlobalWarehouse *globals;
extern GlobalsNoDtcm globals_no_dtcm;

#endif  // APP_TARGETS_SENTRY_MAIN_HPP
