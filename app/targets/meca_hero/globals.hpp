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
#include "referee_user.hpp"
#include "common/radar_comm_can.hpp"
#include "protocol_user.hpp"
#include "referee_user.hpp"
#include "TaskScheduler.hpp"
#include "common/aimbot_comm_can.hpp"

class ChassisHero;
class GimbalHero;
class AmmoHero;

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

  rm::hal::Serial<128> dbus{huart5, false, true}, usart1_ref{huart1, false, false}, uart10_tc{huart10, true, true};

  rm::hal::Serial<128> uart7_debug{huart7, true, true};  ///< 串口接口

  rm::device::BMI088 *imu{nullptr};  ///< IMU

  /*---------------------------------------------------全局变量---------------------------------------------------*/
  /*---------------------------------------------------全局变量---------------------------------------------------*/
  /*---------------------------------------------------全局变量---------------------------------------------------*/
  uint16_t adc_val;  ///< 输入电压ADC采样值

  void Init() {
    imu = new rm::device::BMI088{hspi2, ACC_CS_GPIO_Port, ACC_CS_Pin, GYRO_CS_GPIO_Port, GYRO_CS_Pin};
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
  StateMachine state_machine{2000};                          ///< 状态机
  rm::hal::Can can1{hfdcan1}, can2{hfdcan2}, can3{hfdcan3};  ///< CAN 总线接口
  rm::device::GkSupercap cms{can3};                          ///< 超电
  rm::device::GkSupercap::TxData cap_tx_data;
  rm::device::RadarCanCommunicator radar_can_communicator{can3};    ///< 雷达自瞄
  rm::device::AimbotCanCommunicator aimbot_can_communicator{can3};  ///< 视觉自瞄
  rm::modules::KeyboardHandler<> keyboard_handler(rm::device::DR16::Key);

  /* ------------------------------------------------底盘电机---------------------------------------------------------
   */
  // M3508 chassis_motor_fr{globals_no_dtcm.can2, 1, false};
  // M3508 chassis_motor_fl{globals_no_dtcm.can2, 2, false};
  // M3508 chassis_motor_ll{globals_no_dtcm.can2, 3, false};
  // M3508 chassis_motor_lr{globals_no_dtcm.can2, 4, false};

  ChassisHero *chassis_hero{nullptr};
  rm::device::M3508 chassis_motor[4] = {{can1, 1, false}, {can1, 2, false}, {can1, 3, false}, {can1, 4, false}};

  rm::modules::PID velocity_pid[4] = {{10, 0.5, 0.0, 16384, 5000},
                                      {10, 0.5, 0.0, 16384, 5000},
                                      {10, 0.5, 0.0, 16384, 5000},
                                      {10, 0.5, 0.0, 16384, 5000}};

  std::array<rm::modules::MotorPowerModel::MotorState, 4> motor_states;
  rm::modules::MotorPowerModel power_model{rm::modules::MotorPowerModel::kM3508};
  // 初始电流
  float initial_currents[4];
  // 输出电流
  float output_currents[4];
  // 底盘随动
  rm::modules::PID pid_chassis_follow_pos{450, 0, 0.5, 1500, 0};
  rm::modules::PID pid_chassis_follow_vel{26, 0, 0.0001, 16000, 0};

  /* ------------------------------------------------云台电机---------------------------------------------------------*/
  GimbalHero *gimbal_hero{nullptr};
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> gimbal_motor_yaw{
      can1, {0x12, 0x02, 3.141593f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}, true};
  rm::device::M3508 gimbal_motor_pitch{can2, 7, true};
  rm::device::M2006 gimbal_telescope{can3, 1, false};
  // PID
  rm::modules::PID pid_yaw_position{29, 0, 0.001, 15, 0};
  rm::modules::PID pid_yaw_velocity{9, 0, 0.001, 9, 0};
  rm::modules::PID pid_pitch_position{70, 0.01, 0.01, 1, 0.7};
  rm::modules::PID pid_pitch_velocity{5500, 20, 100, 16000, 4000};
  rm::modules::PID pid_scope_velocity{5.0, 0.0, 0.0, 3600, 0};
  // pid_snipe_yaw_position = new modules::PID{300, 0, 0.0, 15, 0};
  // pid_snipe_yaw_velocity = new modules::PID{0.5, 0, 0.0, 9, 0};

  /* ------------------------------------------------发射机构---------------------------------------------------------*/
  AmmoHero *ammo_hero{nullptr};
  // 拨盘电机
  rm::device::DmMotor<rm::device::DmMotorControlMode::kMit> magazine_motor{
      can1, {0x13, 0x03, 3.141593f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}}};
  // PID
  rm::modules::PID pid_magz_position{42, 0.001, 0.56, 24, 0};
  rm::modules::PID pid_magz_velocity{0.35, 0, 0.001, 7, 0};

  // 摩擦轮电机
  rm::device::M3508 shooter_motor_1{can2, 1, false};
  rm::device::M3508 shooter_motor_2{can2, 2, false};
  rm::device::M3508 shooter_motor_3{can2, 3, false};
  rm::device::M3508 shooter_motor_4{can2, 4, false};
  rm::device::M3508 shooter_motor_5{can2, 5, false};
  rm::device::M3508 shooter_motor_6{can2, 6, false};
  // PID
  rm::modules::PID pid_shooter_1{24, 0.0000007, 0, 8000, 800};  // 20
  rm::modules::PID pid_shooter_2{24, 0.0000007, 0, 8000, 800};  // 20
  rm::modules::PID pid_shooter_3{24, 0.0000007, 0, 8000, 800};
  rm::modules::PID pid_shooter_4{24, 0.0000007, 0, 8000, 800};
  rm::modules::PID pid_shooter_5{24, 0.0000007, 0, 8000, 800};
  rm::modules::PID pid_shooter_6{24, 0.0000007, 0, 8000, 800};

  rm::device::DR16 rc{globals_no_dtcm.dbus};                               ///< DT7
  rm::device::Referee<rm::device::RefereeRevision::kNewV120> ref;          ///< 裁判系统
  rm::device::RefereeUser<rm::device::RefereeRevision::kNewV120> *refSub;  /// 子协议
  rm::device::VT03 tc;                                                     ///< 图传

  rm::modules::MahonyAhrs ahrs{1000.0f};  ///< mahony 姿态解算器，频率 1000Hz
  rm::device::HipnucImu HipnucImu{globals_no_dtcm.uart7_debug};
  rm::modules::LowPassFilterConstDt<float> gyro_z_filter;

  Buzzer buzzer;  ///< 蜂鸣器
  rm::modules::BuzzerController<rm::modules::buzzer_melody::Silent, rm::modules::buzzer_melody::Beeps<1>,
                                rm::modules::buzzer_melody::Beeps<2>, rm::modules::buzzer_melody::Startup,
                                rm::modules::buzzer_melody::TheLick>
      buzzer_controller;
  WS2812Led led;                                                    ///< WS2812 LED灯
  rm::modules::RgbLedController<rm::modules::led_pattern::Off,      //
                                rm::modules::led_pattern::RgbFlow,  //
                                rm::modules::led_pattern::RedFlash>
      led_controller;  ///< LED控制器

  rm::device::DeviceManager<2> gimbal_manager;
  rm::device::DeviceManager<4> chassis_manager;
  rm::device::DeviceManager<7> ammo_manager;
  rm::device::DeviceManager<1> ref_manager;
  rm::device::DeviceManager<1> refsub_manager;
  rm::device::DeviceManager<1> tc_manager;
  rm::device::DeviceManager<1> rc_manager;
  rm::device::DeviceManager<1> cms_manager;
  rm::device::DeviceManager<1> radar_manager;
  rm::device::DeviceManager<1> imu_manager;
  rm::device::DeviceManager<1> aimbot_manager;
  /*---------------------------------------------------全局变量---------------------------------------------------*/
  /*---------------------------------------------------全局变量---------------------------------------------------*/
  /*---------------------------------------------------全局变量---------------------------------------------------*/
  float vbus;  ///< XT30输入电压
  float gyro_z;
  bool ui_send_choice{false};

  // 自定义客户端数据
#pragma pack(push, 1)
  struct {
    uint16_t enemy_hp_1{0};
    uint16_t enemy_hp_2{0};
    uint16_t enemy_hp_3{0};
    uint16_t enemy_hp_4{0};
    uint16_t enemy_hp_5{0};
    uint16_t enemy_hp_7{0};
    rm::u8 snipe_state{0};
  } custom_data;
#pragma pack(pop)

  void Init() {
    // CAN初始化
    can1.SetFilter(0, 0);
    can1.Begin();
    can2.SetFilter(0, 0);
    can2.Begin();
    can3.SetFilter(0, 0);
    can3.Begin();

    // 裁判系统串口接收
    const rm::hal::SerialRxCallbackFunction ref_rx_callback = [&](const etl::span<const uint8_t> &data) {
      for (const auto byte : data) {
        ref << byte;
      }
    };
    globals_no_dtcm.usart1_ref.AttachRxCallback(ref_rx_callback);
    globals_no_dtcm.uart10_tc.AttachRxCallback(ref_rx_callback);

    // 图传串口接收
    rm::hal::SerialRxCallbackFunction tc_rx_callback = [&](const etl::span<const uint8_t> &data) {
      for (const auto byte : data) {
        tc << byte;
      }
    };
    globals_no_dtcm.uart10_tc.AttachRxCallback(tc_rx_callback);

    refSub = new rm::device::RefereeUser<rm::device::RefereeRevision::kNewV120>(ref);

    ref.AttachCallback(std::bind(&rm::device::RefereeUser<rm::device::RefereeRevision::kNewV120>::AttachCallback,
                                 refSub, std::placeholders::_1, std::placeholders::_2));

    // 0x0311 客户端自定义数据 → custom_data
    ref.AttachCallback([&](rm::u16 cmd_id, rm::u8 seq) {
      if (cmd_id == 0x0311) {
        std::memcpy(&custom_data, ref.data().robot_custom_data_3.data, sizeof(custom_data));
      }
    });

    // 设备管理器
    gimbal_manager << &gimbal_motor_yaw << &gimbal_motor_pitch;
    ref_manager << &ref;
    refsub_manager << refSub;
    tc_manager << &tc;
    rc_manager << &rc;
    chassis_manager << &chassis_motor[0] << &chassis_motor[1] << &chassis_motor[2] << &chassis_motor[3];
    ammo_manager << &shooter_motor_1 << &shooter_motor_2 << &shooter_motor_3 << &shooter_motor_4 << &shooter_motor_5
                 << &shooter_motor_6;
    cms_manager << &cms;
    radar_manager << &radar_can_communicator;
    imu_manager << &HipnucImu;
    aimbot_manager << &aimbot_can_communicator;

    buzzer.Init();
    led_controller.SetPattern<rm::modules::led_pattern::RgbFlow>();
    buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();

    // UART7 PID 调参 RX
    // PidTuner::Init();
  }
};

/*---------------------------------------------------监测值---------------------------------------------------*/
// 陀螺仪
inline float eulerangle_yaw{0};
inline float eulerangle_pitch{0};
inline float eulerangle_roll{0};
inline float gyro_x{0};
inline float gyro_y{0};
inline float gyro_z{0};
inline float accel_x{0};
inline float accel_y{0};
inline float accel_z{0};

// yaw轴电机
inline float yaw_pos{0};
inline float yaw_vel{0};
inline float yaw_pos_pid_out{0};
inline float yaw_vel_pid_out{0};
inline float target_yaw{0};
inline float yaw_vel_error{0};

// pitch轴电机
inline float pitch_pos{0};
inline float pitch_vel{0};
inline float pitch_pos_pid_out{0};
inline float pitch_vel_pid_out{0};
inline float target_pitch{0};

// 随动
inline float follow_out{0};

// 底盘电机
inline float chassis_out_1{0};
inline float chassis_out_2{0};
inline float chassis_out_3{0};
inline float chassis_out_4{0};

// 摩擦轮电机
inline float shooter_1{0};
inline float shooter_2{0};
inline float shooter_3{0};
inline float shooter_4{0};
inline float shooter_5{0};
inline float shooter_6{0};

inline float limit{0};
inline float V1{0};
inline float V2{0};

// 弹速
inline float bullet_initial_speed{0};

// 42mm允许发单量
inline unsigned short projectile_allowance_42mm{0};
inline bool low_allowance_42mm{false};

inline short dial{0};

// 状态机
inline StateMachine::MainState main_state;
inline StateMachine::SubState sub_state;
inline StateMachine::ChassisState chassis_state;
inline StateMachine::GimbalState gimbal_state;
inline StateMachine::AmmoState ammo_state;
inline StateMachine::AmmoSubState ammo_sub_state;

inline bool OverOverPower{false};

// 电管输出状态
inline int chassis_power_state{0};
inline int gimbal_power_state{0};
inline int ammo_power_state{0};

// 超电
inline float cap_energy{0};
inline float cap_power{0};
inline uint8_t cap_error{0};
inline bool cap_online{false};
inline int cap_error_count{0};

// radar
inline bool radar_online{false};
inline float radar_yaw{0};
inline float radar_pitch{0};

// Imu
inline bool Imu_online{false};

// DroneAim
inline float drone_yaw{0};
inline uint8_t drone_fire{0};

// Custom_data
inline uint16_t enemy_hp_1{0};
inline uint16_t enemy_hp_2{0};
inline uint16_t enemy_hp_3{0};
inline uint16_t enemy_hp_4{0};
inline uint16_t enemy_hp_5{0};
inline uint16_t enemy_hp_7{0};
inline uint8_t snipe_state{0};

// magz
inline float magz_pos{0};
inline float magz_vel{0};

extern Globals *globals;

inline bool key_once_tc(rm::device::VT03::KeyboardKey key) {
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

inline void ChangePID(rm::modules::PID &pid, float kp, float ki, float kd) {
  pid.SetKp(kp);
  pid.SetKi(ki);
  pid.SetKd(kd);
}
