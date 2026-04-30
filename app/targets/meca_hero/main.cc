#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "common/bsp/timer_task.hpp"
#include "adc.h"
#include "globals.hpp"

__attribute__((section(".sram4"))) GlobalsNoDtcm globals_no_dtcm;
Globals *globals{nullptr};

volatile long long mainloop_time_us = 0;  ///< 主循环运行时间

void Init() {
  globals->rc.Begin();
  globals_no_dtcm.usart1_ref.Start();
  globals_no_dtcm.uart10_tc.Start();
}

/*---------------------------------------------------100HZ---------------------------------------------------*/
void SubLoop100hz() {
  // 更新LED和蜂鸣器
  const auto &[r, g, b] = globals->led_controller.Update();
  globals->led.SetColor(r, g, b);
  // globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);
}

/*---------------------------------------------------500HZ---------------------------------------------------*/
void SubLoop500hz() {
  // 设备状态更新
  globals->gimbal_manager.Update();
  globals->chassis_manager.Update();
  globals->ref_manager.Update();
  globals->tc_manager.Update();
  globals->rc_manager.Update();
  globals->ammo_manager.Update();

  // 底盘更新
  globals->chassis_hero->ChassisUpdate();

  // 发送Dji电机控制帧
  rm::device::DjiMotorBase::SendCommand();

  // 更新输入电压测量值
  globals->vbus = (globals_no_dtcm.adc_val * 3.3f / 65535) * 11.0f;
  globals->chassis_hero->ChassisUpdate();
}

/*---------------------------------------------------1000HZ---------------------------------------------------*/
void SubLoop1000hz() {
  // 状态机更新
  globals->state_machine.StateUpdate();
  globals->gimbal_hero->GimbalUpdate();
}

/*---------------------------------------------------2000HZ---------------------------------------------------*/
/**
 * @note %不同的数产生相位差，避免某一时刻任务量过大（任务片流转）
 */
void MainLoop2khz() {
  static size_t loop_divisor = 0;

  // 状态机监测
  main_state = globals->state_machine.getMainState_();
  sub_state = globals->state_machine.getSubState_();
  chassis_state = globals->state_machine.getChassisState_();
  gimbal_state = globals->state_machine.getGimbalState_();
  ammo_state = globals->state_machine.getAmmoState_();

  if (loop_divisor % 20 == 0) {  // 100hz
    SubLoop100hz();
  }
  if (loop_divisor % 4 == 1) {  // 500hz
    SubLoop500hz();
  }
  if (loop_divisor % 2 == 0) {  // 1000hz
    SubLoop1000hz();
  }

  loop_divisor = (loop_divisor + 1) % 2000;
}

extern "C" {

void AppMain() {
  // 创建主循环定时任务，定频2khz
  TimerTask mainloop{
      &htim13,                                       //
      etl::delegate<void()>::create<MainLoop2khz>()  //
  };

  // 全局变量初始化
  globals = new Globals;
  globals->Init();         ///< 软件
  globals_no_dtcm.Init();  ///< 硬件外设
  Init();

  globals->gyro_z_filter.set_cutoff_frequency(1000.0f, 50.0f);
  mainloop.Start();
  for (;;) {
  }
}
}
