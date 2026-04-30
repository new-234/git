#pragma once

#include <librm.hpp>
#include "adc.h"
#include "fdcan.h"
#include "usart.h"
#include "spi.h"
#include "main.h"

#include "common/bsp/buzzer.hpp"
#include "common/bsp/ws2812.hpp"
#include "State.hpp"
#include "CMS.H"

class ChassisHero;
class GimbalHero;

/**
 * STM32H7的DTCM区域只能被CPU访问，不能被DMA访问，所以需要开一块在DTCM之外的全局数据区域，专门放一些需要被DMA访问的对象
 * 注意linker script里要相应的开一块放在RAM_D1里的区域sram4：
 *   .sram4 (NOLOAD) :
 * {
 *   . = ALIGN(32);
 *   *(.sram4)
 *   . = ALIGN(32);
 * } >RAM_D1
 */
struct GlobalsNoDtcm {
  /*---------------------------------------------------全局对象---------------------------------------------------*/
  /*---------------------------------------------------全局对象---------------------------------------------------*/
  /*---------------------------------------------------全局对象---------------------------------------------------*/

  rm::hal::Serial<127> dbus{huart5, false, true}, usart1_ref{huart1, false, false},
      uart10_tc{huart10, true, true};  ///< 串口接口

  rm::device::BMI088 *imu{nullptr};  ///< IMU

  /*---------------------------------------------------全局变量---------------------------------------------------*/
  /*---------------------------------------------------全局变量---------------------------------------------------*/
  /*---------------------------------------------------全局变量---------------------------------------------------*/
  uint16_t adc_val;  ///< 输入电压ADC采样值

  void Init() {
    imu = new BMI088{hspi2, ACC_CS_GPIO_Port, ACC_CS_Pin, GYRO_CS_GPIO_Port, GYRO_CS_Pin};
    // 初始化输入电压测量
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t *>(&adc_val), 1);
  }
};

extern __attribute__((section(".sram4"))) GlobalsNoDtcm globals_no_dtcm;

struct Globals {
  /*---------------------------------------------------全局对象---------------------------------------------------*/
  /*---------------------------------------------------全局对象---------------------------------------------------*/
  /*---------------------------------------------------全局对象---------------------------------------------------*/
  StateMachine state_machine{2000};           ///< 状态机
  rm::hal::Can can1{hfdcan1}, can2{hfdcan2};  ///< CAN 总线接口
  CMS cms{can1};                              ///< 超电

  // 底盘电机
  // rm::device::M3508 chassis_motor_fr{globals_no_dtcm.can2, 1, false};
  // rm::device::M3508 chassis_motor_fl{globals_no_dtcm.can2, 2, false};
  // rm::device::M3508 chassis_motor_ll{globals_no_dtcm.can2, 3, false};
  // rm::device::M3508 chassis_motor_lr{globals_no_dtcm.can2, 4, false};

  ChassisHero *chassis_hero{nullptr};
  rm::device::M3508 chassis_motor[4] = {{can1, 1, false}, {can1, 2, false}, {can1, 3, false}, {can1, 4, false}};

  rm::modules::PID velocity_pids[4] = {
      {10, 0.5, 0, 16384, 5000}, {10, 0.5, 0, 16384, 5000}, {10, 0.5, 0, 16384, 5000}, {10, 0.5, 0, 16384, 5000}};

  std::array<rm::modules::MotorPowerModel::MotorState, 4> motor_states;
  rm::modules::MotorPowerModel power_model{rm::modules::MotorPowerModel::kM3508};
  // 初始电流
  float initial_currents[4];
  // 输出电流
  float output_currents[4];
  // 底盘随动
  rm::modules::PID pid_chassis_follow_pos{600, 0, 0, 1500, 0};
  rm::modules::PID pid_chassis_follow_vel{23, 0, 0.21, 16000, 0};

  // 云台电机
  GimbalHero *gimbal_hero{nullptr};
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> gimbal_motor_yaw{
      can1, {0x12, 0x02, 3.141593f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}, true};
  rm::device::M3508 gimbal_motor_pitch{can2, 7, true};

  // 发射机构
  // 拨盘电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> magazine_motor{
      can1, {0x13, 0x03, 3.141593f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}};
  // 摩擦轮电机
  rm::device::M3508 shooter_motor_1{can2, 1, false};
  rm::device::M3508 shooter_motor_2{can2, 2, false};
  rm::device::M3508 shooter_motor_3{can2, 3, false};
  rm::device::M3508 shooter_motor_4{can2, 4, false};
  rm::device::M3508 shooter_motor_5{can2, 5, false};
  rm::device::M3508 shooter_motor_6{can2, 6, false};

  rm::device::DR16 rc{globals_no_dtcm.dbus};                       ///< DT7
  rm::device::Referee<rm::device::RefereeRevision::kNewV120> ref;  ///< 裁判系统
  rm::device::VT03 tc;                                             ///< 图传

  rm::modules::MahonyAhrs ahrs{1000.0f};  ///< mahony 姿态解算器，频率 1000Hz
  rm::modules::LowPassFilterConstDt<float> gyro_z_filter;

  Buzzer buzzer;  ///< 蜂鸣器
  rm::modules::BuzzerController<rm::modules::buzzer_melody::Silent, rm::modules::buzzer_melody::Beeps<1>,
                                rm::modules::buzzer_melody::Beeps<2>, rm::modules::buzzer_melody::Startup,
                                rm::modules::buzzer_melody::TheLick>
      buzzer_controller;
  WS2812Led led;                                                        ///< WS2812 LED灯
  rm::modules::RgbLedController<rm::modules::led_pattern::Off,          //
                                rm::modules::led_pattern::GreenBreath,  //
                                rm::modules::led_pattern::RedFlash>
      led_controller;  ///< LED控制器

  rm::device::DeviceManager<2> gimbal_manager;
  rm::device::DeviceManager<4> chassis_manager;
  rm::device::DeviceManager<7> ammo_manager;
  rm::device::DeviceManager<1> ref_manager;
  rm::device::DeviceManager<1> tc_manager;
  rm::device::DeviceManager<1> rc_manager;
  /*---------------------------------------------------全局变量---------------------------------------------------*/
  /*---------------------------------------------------全局变量---------------------------------------------------*/
  /*---------------------------------------------------全局变量---------------------------------------------------*/
  float vbus;  ///< XT30输入电压
  float gyro_z;

  void Init() {
    // CAN初始化
    can1.SetFilter(0, 0);
    can1.Begin();
    can2.SetFilter(0, 0);
    can2.Begin();

    rc.Begin();

    // 裁判系统串口接收
    const rm::hal::SerialRxCallbackFunction ref_rx_callback = [&](const etl::span<const uint8_t> &data) {
      for (const auto byte : data) {
        ref << byte;
      }
    };
    globals_no_dtcm.usart1_ref.AttachRxCallback(ref_rx_callback);
    globals_no_dtcm.usart1_ref.Start();

    // 图传串口接收
    rm::hal::SerialRxCallbackFunction tc_rx_callback = [&](const etl::span<const uint8_t> &data) {
      for (const auto byte : data) {
        tc << byte;
      }
    };
    globals_no_dtcm.uart10_tc.AttachRxCallback(tc_rx_callback);
    globals_no_dtcm.uart10_tc.Start();

    // 设备管理器
    gimbal_manager << &gimbal_motor_yaw << &gimbal_motor_pitch;
    ref_manager << &ref;
    tc_manager << &tc;
    rc_manager << &rc;
    chassis_manager << &chassis_motor[0] << &chassis_motor[1] << &chassis_motor[2] << &chassis_motor[3]
                    << &chassis_motor[4];
    ammo_manager << &shooter_motor_1 << &shooter_motor_2 << &shooter_motor_3 << &shooter_motor_4 << &shooter_motor_5
                 << &shooter_motor_6;

    buzzer.Init();
    led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
    buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();
  }
};

/*---------------------------------------------------监测值---------------------------------------------------*/
inline float eulerangle_yaw{0};
inline float eulerangle_pitch{0};
inline float eulerangle_roll{0};
inline float gyro_x{0};
inline float gyro_y{0};
inline float gyro_z{0};
inline float accel_x{0};
inline float accel_y{0};
inline float accel_z{0};
inline StateMachine::MainState main_state;
inline StateMachine::SubState sub_state;
inline StateMachine::ChassisState chassis_state;
inline StateMachine::GimbalState gimbal_state;
inline StateMachine::AmmoState ammo_state;

extern Globals *globals;

inline bool key_once_tc(VT03::KeyboardKey key) {
  static int16_t key_once_flag{0};
  if (globals->tc.data().keyboard_key & static_cast<int16_t>(key)) {
    if (!(key_once_flag & static_cast<int16_t>(key))) {
      // 第一次按下
      key_once_flag |= static_cast<int16_t>(key);  // 标记已处理
      return true;
    }
    return false;  // 已经处理过，不再响应
  } else {
    key_once_flag &= ~static_cast<int16_t>(key);  // 按键松开，清除标记
    return false;
  }
}