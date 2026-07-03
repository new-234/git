#ifndef MAIN_HPP
#define MAIN_HPP

#include <cstdint>
#include <new>

#include <librm.hpp>

#include "common/bsp/buzzer.hpp"
#include "common/bsp/ws2812.hpp"
#include "common/encoder_counter.hpp"
#include "common/aimbot_comm_can.hpp"
#include "common/navigate_comm_can.hpp"
#include "common/controllers/gimbal_double_yaw.hpp"
#include "common/controllers/quad_omni_chassis.hpp"
#include "common/controllers/shoot_3fric.hpp"

#include "Referee.hpp"
#include "WFLY.hpp"

// 状态机
typedef enum {
  kUnable = 0u,  // 断电模式
  kNoForce,      // 无力模式
  kTest,         // 调试模式
  kMatch,        // 比赛模式

  kGbRemote,    // 云台遥控模式
  kGbScan,      // 扫描模式
  kGbNavigate,  // 云台导航模式
  kGbAimbot,    // 云台自瞄模式
  kGbPercept,   // 云台感知模式

  kCsRemote,    // 底盘遥控模式
  kCsNavigate,  // 底盘导航模式
  kGbIdentify,
  kGbFfVerify,
} StateMachineType;

struct GlobalWarehouse {
 public:
  Buzzer buzzer;  ///< 蜂鸣器
  rm::modules::BuzzerController<rm::modules::buzzer_melody::Silent, rm::modules::buzzer_melody::Startup,
                                rm::modules::buzzer_melody::Success, rm::modules::buzzer_melody::Error,
                                rm::modules::buzzer_melody::SuperMario, rm::modules::buzzer_melody::SeeUAgain,
                                rm::modules::buzzer_melody::TheLick, rm::modules::buzzer_melody::Beeps<1>>
      buzzer_controller;

  WS2812Led led;  ///< WS2812 LED灯
  rm::modules::RgbLedController<rm::modules::led_pattern::Off, rm::modules::led_pattern::RedFlash,
                                rm::modules::led_pattern::GreenBreath, rm::modules::led_pattern::RgbFlow>
      led_controller;

  // 硬件接口
  rm::hal::Can *can1{nullptr}, *can2{nullptr};
  rm::device::AimbotCanCommunicator *aimbot_communicator{nullptr};
  rm::device::NavigateCanCommunicator *navigate_communicator{nullptr};
  rm::hal::Serial<256> *hipnuc_uart{nullptr};
  rm::device::HipnucImu *hipnuc_imu{nullptr};
  rm::device::BMI088 *imu{nullptr};

  rm::hal::Serial<128> *ident_uart{nullptr};
  rm::hal::Serial<25> *dbus{nullptr};
  WflyET16s *wfly_et16s{nullptr};

  rm::hal::Serial<128> *referee_uart{nullptr};
  rm::device::RxReferee *rx_referee{nullptr};
  rm::device::Referee<rm::device::RefereeRevision::kNewV120> *referee_data{nullptr};

  // 云台
  rm::device::GM6020 *up_yaw_motor{nullptr};
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *down_yaw_motor{nullptr};
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> *pitch_motor{nullptr};
  rm::device::M3508 *friction_left{nullptr};
  rm::device::M3508 *friction_right{nullptr};
  rm::device::M3508 *dial_motor{nullptr};

  // 底盘
  rm::device::M3508 *wheel_lf{nullptr};
  rm::device::M3508 *wheel_rf{nullptr};
  rm::device::M3508 *wheel_lb{nullptr};
  rm::device::M3508 *wheel_rb{nullptr};

  // 设备
  rm::device::DeviceManager<1> device_rc;
  rm::device::DeviceManager<2> device_nuc;
  rm::device::DeviceManager<3> device_gimbal;
  rm::device::DeviceManager<3> device_shoot;
  rm::device::DeviceManager<4> device_chassis;

  // 控制器
  rm::modules::MahonyAhrs ahrs{500.0f};
  GimbalDoubleYaw gimbal_controller;
  QuadOmniChassis chassis_controller;
  Shoot3Fric shoot_controller{9, 17.0666f, true};
  EncoderCounter dail_encoder_counter;

  StateMachineType StateMachine_ = {kNoForce};

  SwitchPosition last_switch[7] = {
      SwitchPosition::kDown, SwitchPosition::kDown, SwitchPosition::kDown, SwitchPosition::kDown,
      SwitchPosition::kUp,   SwitchPosition::kUp,   SwitchPosition::kUp,
  };

  uint8_t time = 0;
  uint16_t gimbal_init_time = 0;
  uint16_t shooter_init_time = 0;
  uint16_t chassis_init_time = 0;
  uint16_t hurt_time = 0;
  uint8_t time_camera = 0;
  uint16_t imu_count = 0;
  uint8_t aim_mode = 0;
  uint8_t music_choice = 0;
  uint8_t music = 0;
  bool music_change_flag = false;
  bool last_gimbal_power = false;
  bool last_shooter_power = false;
  bool last_chassis_power = false;

  float vbus = 0.0f;

 public:
  void Init();

  void SubLoop500Hz();
  void SubLoop250Hz();
  void SubLoop100Hz();
  void SubLoop50Hz();
  void SubLoop10Hz();

 private:
  void GimbalPIDInit();
  void ChassisPIDInit();
  void ShootPIDInit();
  void RCStateUpdate();
  void Music();
};

/**
 * DMA对象存储区。
 * 注意：Serial 指针不用放 .sram4，真正需要放 .sram4 的是 Serial 对象本体。
 */
template <typename T>
struct DmaObjectStorage {
  alignas(32) alignas(T) uint8_t bytes[(sizeof(T) + 31u) & ~31u]{};

  T *get() {
    return reinterpret_cast<T *>(bytes);
  }

  const T *get() const {
    return reinterpret_cast<const T *>(bytes);
  }
};

struct alignas(32) DmaAdcSample {
  uint16_t value{0};
};

/**
 * STM32H7 的 DTCM 只能 CPU 访问，DMA 无法访问。
 * 所以 ADC DMA 采样值、串口 DMA 相关 Serial 对象本体放到 .sram4。
 */
struct alignas(32) GlobalsNoDtcm {
  DmaAdcSample adc;  ///< 输入电压 ADC DMA 采样值

  DmaObjectStorage<rm::hal::Serial<128>> ident_uart;
  DmaObjectStorage<rm::hal::Serial<25>> dbus;
  DmaObjectStorage<rm::hal::Serial<128>> referee_uart;
  DmaObjectStorage<rm::hal::Serial<256>> hipnuc_uart;
};

extern GlobalWarehouse *globals;
extern GlobalsNoDtcm globals_no_dtcm;

#endif  // MAIN_HPP
