#include "AmmoShooter.hpp"
#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "common/bsp/timer_task.hpp"
#include "adc.h"
#include "globals.hpp"

__attribute__((section(".sram4"))) GlobalsNoDtcm globals_no_dtcm;
Globals *globals{nullptr};

volatile long long mainloop_time_us = 0;  ///< 主循环运行时间

Queue_t UI_send_buffer[2];
u32 irq;
u32 tmp_send[7];

extern u16 robot_id;
extern u8 len;
extern u8 Info_Arr[128];

// 瞄准参考线
Graph_Data image_y;
Graph_Data linex0, linex1, linex2, linex3, linex4;
// // 飞坡、前进参考线
// Graph_Data rfd1, rfd2, rfd3, rfd4;

// 瞄准角度
Float_Data linef1;
// Cap
Float_Data CapData;

// IMU
Float_Data Pitch;

// 弹速偏置
Float_Data AmmoSpeed;

// YAW
Graph_Data YawAngle;

// 自瞄状态框选
Graph_Data p1, p2;

// 鼠标操作显示
Graph_Data mouse_left, mouse_right;

// 自瞄提示字符
String_Data aimbotUI;  // 自瞄状态

void UiRefresh();
void UiSend();
void UI_send(rm::hal::Serial<128> *msg, u8 *data, u8 data_len);

void Monitor() {
  // 状态机更新
  globals->state_machine.StateUpdate();

  // PID 调参命令处理
  PidTuner::Process();

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
  cap_error = globals->cms.rx_data_.error_code;
  cap_online = globals->cms_manager.all_device_ok();
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
  prepare_uart_rx_to_idle_dma(huart7, UART7_IRQn, DMA1_Stream3_IRQn);

  globals->rc.Begin();
  globals_no_dtcm.usart1_ref.Start();
  globals_no_dtcm.uart10_tc.Start();
  globals_no_dtcm.uart7_debug.Start();

  globals->gyro_z_filter.set_cutoff_frequency(1000.0f, 50.0f);

  globals->chassis_hero = new ChassisHero();
  globals->gimbal_hero = new GimbalHero();
  globals->ammo_hero = new AmmoHero();

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

/*---------------------------------------------------100HZ---------------------------------------------------*/
void SubLoop100hz() {
  // 超电更新
  globals->cap_tx_data.feedback_referee_energy_buffer = globals->ref.data().power_heat_data.buffer_energy;
  globals->cap_tx_data.feedback_referee_power_limit = globals->ref.data().robot_status.chassis_power_limit;
  globals->cap_tx_data.enable_dcdc = 0x01;
  globals->cap_tx_data.system_restart = 0;
  globals->cap_tx_data.resv0 = 0;
  globals->cap_tx_data.resv1[0] = 0;
  globals->cap_tx_data.resv1[1] = 0;
  globals->cap_tx_data.resv1[2] = 0;

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
  // VOFA
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

  // 更新输入电压测量值
  globals->vbus = (globals_no_dtcm.adc_val * 3.3f / 65535) * 11.0f;
}

/*---------------------------------------------------1000HZ---------------------------------------------------*/
void SubLoop1000hz() {
  // 云台状态更新
  globals->gimbal_hero->GimbalUpdate();
  VOFASend();
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
  HAL_Delay(500);
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

// void UiRefresh() {
//   // 接收机器人ID
//   robot_id = globals->ref.data().robot_status.robot_id;
//   if (globals->tc->data().keyboard_key & static_cast<int16_t>(device::VT03::KeyboardKey::kR) ||
//       globals->rc->key(device::DR16::Key::kR)) {
//     // Line_Draw(&image_x, "xxx", UI_Graph_ADD, 0, UI_Color_Orange, 2, 918, 515, 978, 515);
//     Line_Draw(&image_y, "yyy", UI_Graph_ADD, 2, UI_Color_Orange, 2, 948, 465, 948, 565);
//     Line_Draw(&linex0, "xx0", UI_Graph_ADD, 2, UI_Color_Purplish_red, 2, 940, 525, 980, 525);
//     Line_Draw(&linex1, "xx1", UI_Graph_ADD, 2, UI_Color_Green, 2, 900, 500, 1020, 500);
//     Line_Draw(&linex2, "xx2", UI_Graph_ADD, 2, UI_Color_Pink, 2, 910, 470, 1010, 470);
//     Line_Draw(&linex3, "xx3", UI_Graph_ADD, 2, UI_Color_Black, 2, 920, 450, 1000, 450);
//     Line_Draw(&linex4, "xx4", UI_Graph_ADD, 2, UI_Color_Cyan, 2, 930, 420, 990, 420);
//
//     Float_Draw(&CapData, "cms", UI_Graph_ADD, 2, UI_Color_Main, 27, 2, 5, 7050, 150, cms_v * 1000);
//     Float_Draw(&Pitch, "gbp", UI_Graph_ADD, 2, UI_Color_Green, 27, 1, 3, 7300, 500, 0);
//     Float_Draw(&AmmoSpeed, "amm", UI_Graph_ADD, 2, UI_Color_White, 27, 1, 3, 7300, 560, 0);
//     Arc_Draw(&YawAngle, "yaw", UI_Graph_ADD, 2, UI_Color_Green, 0, 30, 2, 960, 540, 300, 300);
//
//     Rectangle_Draw(&p1, "p01", UI_Graph_ADD, 3, UI_Color_Purplish_red, 0, 360, 800, 420, 750);
//     Rectangle_Draw(&p2, "p02", UI_Graph_ADD, 3, UI_Color_Purplish_red, 0, 360, 750, 420, 700);
//
//     Char_Draw(&aimbotUI, "aim", UI_Graph_ADD, 0, UI_Color_Green, 25, 15, 2, 360, 800, "AIMBOT\nAUTOFIRE");
//
//     // Circle_Draw(&mouse_left, "msl", UI_Graph_ADD, 4, UI_Color_Black, 25, 1800, 100, 50);
//     // Circle_Draw(&mouse_right, "msr", UI_Graph_ADD, 4, UI_Color_Black, 25, 1850, 100, 50);
//
//     irq = (u32)&aimbotUI;
//     EnQueue(&UI_send_buffer[1], (u8 *)&irq, 4);
//     irq = (u32)&image_y;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//
//     // irq = (u32)&image_y;
//     // EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//     // irq = (u32)&mouse_left;
//     // EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//     // irq = (u32)&mouse_right;
//     // EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//
//     irq = (u32)&CapData;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//     irq = (u32)&AmmoSpeed;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//     irq = (u32)&Pitch;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//     irq = (u32)&YawAngle;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//
//     irq = (u32)&p1;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//     irq = (u32)&p2;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//
//     irq = (u32)&linex0;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//     irq = (u32)&linex1;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//     irq = (u32)&linex2;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//     irq = (u32)&linex3;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//     irq = (u32)&linex4;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//   } else {
//     // 电容电压
//     if (!overpower) {
//       Float_Draw(&CapData, "cms", UI_Graph_Change, 2, UI_Color_Green, 27, 1, 5, 900, 270, cms_v * 1000.0f);
//     } else {
//       Float_Draw(&CapData, "cms", UI_Graph_Change, 2, UI_Color_Main, 27, 1, 5, 900, 270, cms_v * 1000.0f);
//     }
//     // 弹速调节
//     if ((globals->rc->mouse_button_left() || globals->tc->data().mouse_button_left) &&
//         (globals->rc->mouse_button_right() || globals->tc->data().mouse_button_right))
//       Float_Draw(&AmmoSpeed, "amm", UI_Graph_Change, 2, UI_Color_Purplish_red, 25, 2, 2, 7300, 560,
//                  -shooter_m * 100.0f);
//     else if (globals->rc->mouse_button_right() || globals->tc->data().mouse_button_right) {
//       Float_Draw(&AmmoSpeed, "amm", UI_Graph_Change, 2, UI_Color_Yellow, 25, 2, 2, 7300, 560, -shooter_m * 100.0f);
//     } else if (globals->rc->mouse_button_left() || globals->tc->data().mouse_button_left) {
//       Float_Draw(&AmmoSpeed, "amm", UI_Graph_Change, 2, UI_Color_Orange, 25, 2, 2, 7300, 560, -shooter_m * 100.0f);
//     } else {
//       Float_Draw(&AmmoSpeed, "amm", UI_Graph_Change, 2, UI_Color_White, 25, 2, 2, 7300, 560, -shooter_m * 100.0f);
//     }
//
//     // Pitch
//     if (globals->rc->key(DR16::Key::kCtrl) ||
//         (globals->tc->data().keyboard_key & static_cast<int16_t>(VT03::KeyboardKey::kCtrl)))
//       Float_Draw(&Pitch, "gbp", UI_Graph_Change, 2, UI_Color_Purplish_red, 27, 1, 3, 7300, 500,
//                  (eulerangle_pitch / 3.14f * 180) * 1000);
//     else
//       Float_Draw(&Pitch, "gbp", UI_Graph_Change, 2, UI_Color_Green, 27, 1, 3, 7300, 500,
//                  (eulerangle_pitch / 3.14f * 180) * 1000);
//
//     // 底盘夹角
//     if (globals->state_machine.getChassisState_() == StateMachine::ChassisState::kFollow) {
//       Arc_Draw(&YawAngle, "yaw", UI_Graph_Change, 2, UI_Color_Green,
//                rm::modules::Wrap((globals->gimbal_motor_yaw->pos() - 0.49) / 3.14f * 180 - 15, 0, 360),
//                rm::modules::Wrap((globals->gimbal_motor_yaw->pos() - 0.49) / 3.14f * 180 + 15, 0, 360), 3, 960, 540,
//                300, 300);
//     } else {
//       Arc_Draw(&YawAngle, "yaw", UI_Graph_Change, 2, UI_Color_Purplish_red,
//                rm::modules::Wrap((globals->gimbal_motor_yaw->pos() - 0.49) / 3.14f * 180 - 15, 0, 360),
//                rm::modules::Wrap((globals->gimbal_motor_yaw->pos() - 0.49) / 3.14f * 180 + 15, 0, 360), 3, 960, 540,
//                300, 300);
//     }
//
//     // 自瞄模式
//     if (globals->radar_can_communicator.vaild()) {
//       Rectangle_Draw(&p1, "p01", UI_Graph_Change, 3, UI_Color_Purplish_red, 3, 350, 809, 513, 769);
//     } else {
//       Rectangle_Draw(&p1, "p01", UI_Graph_Change, 3, UI_Color_Purplish_red, 0, 350, 809, 513, 769);
//     }
//
//     if (globals->radar_can_communicator.fresh()) {
//       Rectangle_Draw(&p2, "p02", UI_Graph_Change, 3, UI_Color_Purplish_red, 3, 350, 765, 555, 730);
//     } else {
//       Rectangle_Draw(&p2, "p02", UI_Graph_Change, 3, UI_Color_Purplish_red, 0, 350, 765, 555, 730);
//     }
//
//     // 鼠标操作显示
//     // if (globals->tc->data().mouse_button_left || globals->rc->mouse_button_left())
//     //   Circle_Draw(&mouse_left, "msl", UI_Graph_Change, 4, UI_Color_Pink, 25, 1800, 200, 50);
//     // else
//     //   Circle_Draw(&mouse_left, "msl", UI_Graph_Change, 4, UI_Color_Pink, 0, 1800, 200, 50);
//
//     // if (globals->tc->data().mouse_button_right || globals->rc->mouse_button_right())
//     //   Circle_Draw(&mouse_right, "msr", UI_Graph_Change, 4, UI_Color_Pink, 25, 1850, 200, 50);
//     // else
//     //   Circle_Draw(&mouse_right, "msr", UI_Graph_Change, 4, UI_Color_Pink, 0, 1850, 200, 50);
//
//     irq = (u32)&CapData;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//     irq = (u32)&AmmoSpeed;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//     irq = (u32)&Pitch;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//     irq = (u32)&YawAngle;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//     irq = (u32)&p1;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//     irq = (u32)&p2;
//     EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//
//     // irq = (u32)&mouse_left;
//     // EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//     // irq = (u32)&mouse_right;
//     // EnQueue(&UI_send_buffer[0], (u8 *)&irq, 4);
//   }
// }
//
// void UiSend() {
//   if ((IsEmpty(&UI_send_buffer[1]) && !IsEmpty(&UI_send_buffer[0])) ||
//       (!IsEmpty(&UI_send_buffer[0]) && globals->ui_send_choice)) {
//     if (UI_send_buffer[0].counter / 4 >= 7) {
//       for (u8 i = 0; i < 7; i++) {
//         UI_Pop(&UI_send_buffer[0], (u8 *)&tmp_send[i]);
//       }
//       UI_ReFresh(7, *(Graph_Data *)tmp_send[0], *(Graph_Data *)tmp_send[1], *(Graph_Data *)tmp_send[2],
//                  *(Graph_Data *)tmp_send[3], *(Graph_Data *)tmp_send[4], *(Graph_Data *)tmp_send[5],
//                  *(Graph_Data *)tmp_send[6]);
//     } else if (UI_send_buffer[0].counter / 4 >= 5) {
//       for (u8 i = 0; i < 5; i++) {
//         UI_Pop(&UI_send_buffer[0], (u8 *)&tmp_send[i]);
//       }
//       UI_ReFresh(5, *(Graph_Data *)tmp_send[0], *(Graph_Data *)tmp_send[1], *(Graph_Data *)tmp_send[2],
//                  *(Graph_Data *)tmp_send[3], *(Graph_Data *)tmp_send[4]);
//     } else if (UI_send_buffer[0].counter / 4 >= 2) {
//       for (u8 i = 0; i < 2; i++) {
//         UI_Pop(&UI_send_buffer[0], (u8 *)&tmp_send[i]);
//       }
//       UI_ReFresh(2, *(Graph_Data *)tmp_send[0], *(Graph_Data *)tmp_send[1]);
//     } else {
//       UI_Pop(&UI_send_buffer[0], (u8 *)&tmp_send[0]);
//       UI_ReFresh(1, *(Graph_Data *)tmp_send[0]);
//     }
//     UI_send(&globals_no_dtcm.usart1_ref, Info_Arr, len);
//       } else if (!IsEmpty(&UI_send_buffer[1]) && !globals->ui_send_choice) {
//         UI_Pop(&UI_send_buffer[1], (u8 *)&tmp_send[0]);
//         Char_ReFresh(*(String_Data *)tmp_send[0]);
//         UI_send(&globals_no_dtcm.usart1_ref, Info_Arr, len);
//       }
//   globals->ui_send_choice ^= true;
// }
//
// void UI_send(rm::hal::Serial<128> *msg, u8 *data, u8 data_len) { msg->Write(data, data_len,50); }