#include "AmmoShooter.hpp"
#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "common/bsp/timer_task.hpp"
#include "adc.h"
#include "globals.hpp"

__attribute__((section(".sram4"))) GlobalsNoDtcm globals_no_dtcm;
Globals *globals{nullptr};

volatile long long mainloop_time_us = 0;  ///< 主循环运行时间

void Monitor() {
  // 状态机更新
  globals->state_machine.StateUpdate();

  // 状态机监测
  main_state = globals->state_machine.getMainState_();
  sub_state = globals->state_machine.getSubState_();
  chassis_state = globals->state_machine.getChassisState_();
  gimbal_state = globals->state_machine.getGimbalState_();
  ammo_state = globals->state_machine.getAmmoState_();
  ammo_sub_state = globals->state_machine.getAmmoSubState_();

  // 裁判电源监测
  chassis_power_state = globals->ref.data().robot_status.power_management_chassis_output;
  gimbal_power_state = globals->ref.data().robot_status.power_management_gimbal_output;
  ammo_power_state = globals->ref.data().robot_status.power_management_shooter_output;

  // 摩擦轮目标速度监测
  V1 = globals->ammo_hero->V_shooter_1;
  V2 = globals->ammo_hero->V_shooter_2;
  limit = globals->ammo_hero->limit;
  shooter_1 = globals->shooter_motor_1.rpm();
  shooter_2 = globals->shooter_motor_2.rpm();
  shooter_3 = globals->shooter_motor_3.rpm();
  shooter_4 = globals->shooter_motor_4.rpm();
  shooter_5 = globals->shooter_motor_5.rpm();
  shooter_6 = globals->shooter_motor_6.rpm();

  // 底盘输出监测
  chassis_out_1 = globals->chassis_motor[0].rpm();
  chassis_out_2 = globals->chassis_motor[1].rpm();
  chassis_out_3 = globals->chassis_motor[2].rpm();
  chassis_out_4 = globals->chassis_motor[3].rpm();

  dial = globals->rc.dial();

  follow_out = globals->pid_chassis_follow_vel.out();

  // 超电
  cap_energy = static_cast<f32>(globals->cms.rx_data_.cap_energy);
  cap_power = globals->cms.rx_data_.chassis_power;
}

void Init() {
  const auto prepare_uart_rx_to_idle_dma = [](UART_HandleTypeDef &huart, const IRQn_Type uart_irqn,
                                              const IRQn_Type dma_rx_irqn) {
    // Clear stale UART/DMA state before librm starts ReceiveToIdle mode.
    (void)HAL_UART_AbortReceive(&huart);
    (void)HAL_UART_Abort(&huart);
    __HAL_UART_CLEAR_FLAG(&huart, UART_CLEAR_PEF);
    __HAL_UART_CLEAR_FLAG(&huart, UART_CLEAR_FEF);
    __HAL_UART_CLEAR_FLAG(&huart, UART_CLEAR_NEF);
    __HAL_UART_CLEAR_FLAG(&huart, UART_CLEAR_OREF);
    __HAL_UART_SEND_REQ(&huart, UART_RXDATA_FLUSH_REQUEST);
    huart.ErrorCode = HAL_UART_ERROR_NONE;
    huart.RxState = HAL_UART_STATE_READY;
    huart.gState = HAL_UART_STATE_READY;
    HAL_NVIC_ClearPendingIRQ(uart_irqn);
    HAL_NVIC_ClearPendingIRQ(dma_rx_irqn);
  };

  prepare_uart_rx_to_idle_dma(huart5, UART5_IRQn, DMA1_Stream0_IRQn);
  prepare_uart_rx_to_idle_dma(huart10, USART10_IRQn, DMA1_Stream5_IRQn);

  globals->rc.Begin();
  globals_no_dtcm.usart1_ref.Start();
  globals_no_dtcm.uart10_tc.Start();

  globals->gyro_z_filter.set_cutoff_frequency(1000.0f, 50.0f);

  globals->chassis_hero = new ChassisHero();
  globals->gimbal_hero = new GimbalHero();
  globals->ammo_hero = new AmmoHero();
}

/*---------------------------------------------------100HZ---------------------------------------------------*/
void SubLoop100hz() {
  // 超电更新
  globals->cap_tx_data.feedback_referee_energy_buffer = globals->ref.data().power_heat_data.buffer_energy;
  globals->cap_tx_data.feedback_referee_power_limit = globals->ref.data().robot_status.chassis_power_limit;
  globals->cap_tx_data.enable_dcdc = 0x01;
  globals->cap_tx_data.system_restart = 0x00;

  globals->cms.Update(globals->cap_tx_data);

  // 更新LED和蜂鸣器
  const auto &[r, g, b] = globals->led_controller.Update();
  globals->led.SetColor(r, g, b);
  globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);
}

/*---------------------------------------------------250HZ---------------------------------------------------*/
void SubLoop250hz() {
  // 底盘更新
  globals->chassis_hero->ChassisUpdate();
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
  globals->cms_manager.Update();

  // 发射机构更新
  globals->ammo_hero->AmmoUpdate();

  // 发送Dji电机控制帧
  DjiMotorBase::SendCommand();

  // VOFA
  // VOFASend();

  // 更新输入电压测量值
  globals->vbus = (globals_no_dtcm.adc_val * 3.3f / 65535) * 11.0f;
}

/*---------------------------------------------------1000HZ---------------------------------------------------*/
void SubLoop1000hz() {
  // 云台状态更新
  globals->gimbal_hero->GimbalUpdate();
}

/*---------------------------------------------------2000HZ---------------------------------------------------*/
/**
 * @note %不同的数产生相位差，避免某一时刻任务量过大（任务片流转）
 */
void MainLoop2khz() {
  static size_t loop_divisor = 0;

  Monitor();

  if (loop_divisor % 20 == 0) {  // 100hz
    SubLoop100hz();
  }
  if (loop_divisor % 8 == 1) {
    SubLoop250hz();
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
  HAL_Delay(1000);
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

  mainloop.Start();
  for (;;) {
  }
}
}
