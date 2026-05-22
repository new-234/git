#include "AmmoShooter.hpp"
#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "common/bsp/timer_task.hpp"
#include "adc.h"
#include "globals.hpp"

__attribute__((section(".sram4"))) GlobalsNoDtcm globals_no_dtcm;
Globals *globals{nullptr};

volatile long long mainloop_time_us = 0;  ///< 主循环运行时间

using namespace rm;
using namespace rm::device;

void subRefereeInteraction();

void h2dInteraction();
void UIGroup1_add();
void UIGroup1_edit();
void UIrefrenceline();

static auto UIh2d = device::UITask(h2dInteraction, 10);  // >>> 10hz
static auto UIA1 = device::UITask(UIGroup1_add, 1);      // >>> 10hz
static auto UILine = UITask(UIrefrenceline, 1);
static auto UIE1 = device::UITask(UIGroup1_edit, 10);

static auto schedule = device::UITaskScheduler(30);  // >>> 30hz主调度器

u8 len;
u8 Info_Arr[128];
void UI_send(rm::hal::Serial<128> *msg, u8 *data, u8 data_len) { msg->Write(data, data_len, 50); }

void Monitor() {
  // // PID 调参命令处理
  // PidTuner::Process();

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

  radar_online = globals->radar_manager.all_device_ok();
  radar_yaw = globals->radar_can_communicator.yaw_mard();
  radar_pitch = globals->radar_can_communicator.pitch_mard();

  Imu_online = globals->imu_manager.all_device_ok();

  drone_yaw = globals->refSub->data().drone_2_hero.cmd_yaw_angle;
  drone_fire = globals->refSub->data().drone_2_hero.cmd_shooting;

  enemy_hp_1 = globals->custom_data.enemy_hp_1;
  enemy_hp_2 = globals->custom_data.enemy_hp_2;
  enemy_hp_3 = globals->custom_data.enemy_hp_3;
  enemy_hp_4 = globals->custom_data.enemy_hp_4;
  enemy_hp_5 = globals->custom_data.enemy_hp_5;
  enemy_hp_7 = globals->custom_data.enemy_hp_7;
  snipe_state = globals->custom_data.snipe_state;
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

  schedule.addTask(&UIh2d);
  schedule.addTask(&UIE1);
}
/*---------------------------------------------------10HZ---------------------------------------------------*/
void SubLoop10hz() {}

/*---------------------------------------------------50HZ---------------------------------------------------*/
void SubLoop50hz() {}
/*---------------------------------------------------100HZ---------------------------------------------------*/
void SubLoop100hz() {
  // 超电更新
  globals->cap_tx_data.feedback_referee_energy_buffer = globals->ref.data().power_heat_data.buffer_energy;
  globals->cap_tx_data.feedback_referee_power_limit = globals->ref.data().robot_status.chassis_power_limit;
  globals->cap_tx_data.enable_dcdc = 0x01;
  static int cap_error_count = 0;
  ::cap_error_count = cap_error_count;
  if (globals->cms.rx_data_.error_code != 0 && cap_error_count < 5) {
    globals->cap_tx_data.system_restart = 1;
    cap_error_count++;
  } else {
    globals->cap_tx_data.system_restart = 0;
  }
  globals->cap_tx_data.resv0 = 0;
  globals->cap_tx_data.resv1[0] = 0;
  globals->cap_tx_data.resv1[1] = 0;
  globals->cap_tx_data.resv1[2] = 0;

  globals->cms.Update(globals->cap_tx_data);

  // 裁判系统数据
  bullet_initial_speed = globals->ref.data().shoot_data.initial_speed;
  projectile_allowance_42mm = globals->ref.data().projectile_allowance.projectile_allowance_42mm;
  if (projectile_allowance_42mm <= 2) {
    low_allowance_42mm = true;
  } else {
    low_allowance_42mm = false;
  }

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
  globals->refsub_manager.Update();
  globals->tc_manager.Update();
  globals->rc_manager.Update();
  globals->ammo_manager.Update();
  globals->cms_manager.Update();
  globals->radar_manager.Update();
  globals->imu_manager.Update();
  globals->aimbot_manager.Update();

  // 发射机构更新
  globals->ammo_hero->AmmoUpdate();

  // 发送Dji电机控制帧
  DjiMotorBase::SendCommand();

  // 更新输入电压测量值
  globals->vbus = (globals_no_dtcm.adc_val * 3.3f / 65535) * 11.0f;
}

/*---------------------------------------------------1000HZ---------------------------------------------------*/
void SubLoop1000hz() { globals->gimbal_hero->GimbalUpdate(); }

/*---------------------------------------------------2000HZ---------------------------------------------------*/
/**
 * @note %不同的数产生相位差，避免某一时刻任务量过大（任务片流转）
 */
void MainLoop2khz() {
  static size_t loop_divisor = 0;

  // 状态机更新
  globals->state_machine.StateUpdate();

  Monitor();

  if (loop_divisor % 200 == 0) {
    SubLoop10hz();
  }
  if (loop_divisor % 40 == 0) {
    SubLoop50hz();
  }
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
  if (loop_divisor % 67 == 0) {  // 30hz
    // 多机通信
    subRefereeInteraction();
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

void subRefereeInteraction() {
  static auto last_switch_l = rm::device::DR16::SwitchPosition::kUnknown;
  if (globals->rc.switch_l() != last_switch_l) {
    schedule.addTaskStatic(&UIA1);
  }
  last_switch_l = globals->rc.switch_l();
  schedule.schedule();
}

void h2dInteraction() {
  Hero2Drone h2d{globals->gimbal_hero->monitor_.euler_yaw_, globals->gimbal_hero->monitor_.euler_pitch_,
                 globals->ref.data().projectile_allowance.projectile_allowance_42mm,
                 globals->ref.data().shoot_data.initial_speed, globals->ammo_hero->shooter_m};
  len = device::Referee0x301Prepare(Info_Arr, 0, h2d, globals->ref.data().robot_status.robot_id,
                                    globals->ref.data().robot_status.robot_id + 5);
  UI_send(&globals_no_dtcm.usart1_ref, Info_Arr, len);
}

// void H2D_func() {
//   static struct rm::device::Hero2Drone h2d{0, 0, 0, 0, 0};
//   const u16 size = rm::device::Referee0x301Prepare(globals->dataBox, 0, h2d, 0x001, 0x006);
//   globals->refereeUart->Write(globals->dataBox, size);
// }
//
// void D2H_func() {
//   static struct rm::device::Drone2Hero d2h{0, 0, 0};
//   const auto size = rm::device::Referee0x301Prepare(globals->dataBox, 0, d2h, 0x006, 0x001);
//   globals->refereeUart->Write(globals->dataBox, size);
// }
// //

void UIGroup1_add() {
  device::UIFigure7 UIGroup7;
  UIGroup7.figure1.fillIntegrate(u8"amm", rm::device::UIFigure::Operation::Add, 0, rm::device::UIFigure::Color::Magenta,
                                 5, 1200, 590, 30, globals->ammo_hero->shooter_m);
  UIGroup7.figure2.fillFloat(u8"yaw", UIFigure::Operation::Add, 0, rm::device::UIFigure::Color::Pink, 5, 880, 150, 40,
                             globals->gimbal_hero->monitor_.euler_yaw_);
  UIGroup7.figure3.fillFloat(u8"pit", UIFigure::Operation::Add, 0, rm::device::UIFigure::Color::Pink, 5, 1200, 510, 40,
                             globals->gimbal_hero->monitor_.euler_yaw_);
  UIGroup7.figure4.fillArc(u8"flo", UIFigure::Operation::Add, 0, UIFigure::Color::Cyan, 10, 960, 542,
                           globals->gimbal_motor_yaw.pos() - (-0.288) - 15,
                           globals->gimbal_motor_yaw.pos() - (-0.288) + 15, 100, 100);
  UIGroup7.figure5.fillLine(u8"cap", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 50, 37, 260, 37,
                            static_cast<int16_t>((globals->cms.rx_data_.cap_energy / 252.0) * (750 - 260) + 260));
  UIGroup7.figure6.fillRec(u8"ca1", UIFigure::Operation::Add, 0, UIFigure::Color::RedBlue, 50, 37, 260, 87, 750);

  const auto dataLen = rm::device::Referee0x301Prepare(Info_Arr, 0, UIGroup7, globals->ref.data().robot_status.robot_id,
                                                       globals->ref.data().robot_status.robot_id + 256);
  globals_no_dtcm.usart1_ref.Write(Info_Arr, dataLen, 20);
}

void UIGroup1_edit() {
  device::UIFigure7 UIGroup7;
  UIGroup7.figure1.fillIntegrate(u8"amm", rm::device::UIFigure::Operation::Edit, 0,
                                 rm::device::UIFigure::Color::Magenta, 5, 1250, 570, 30, globals->ammo_hero->shooter_m);
  UIGroup7.figure2.fillFloat(u8"yaw", UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::Pink, 5, 880, 150, 40,
                             -globals->gimbal_hero->monitor_.euler_yaw_ * 1000);
  UIGroup7.figure3.fillFloat(u8"pit", UIFigure::Operation::Edit, 0, rm::device::UIFigure::Color::Pink, 5, 1200, 510, 40,
                             -globals->gimbal_hero->monitor_.euler_pitch_ * 1000);
  UIGroup7.figure4.fillArc(
      u8"flo", UIFigure::Operation::Edit, 0, UIFigure::Color::Cyan, 10, 960, 542,
      static_cast<uint16_t>(rm::modules::Wrap((globals->gimbal_motor_yaw.pos() - (-0.288)) * 57.3 - 20, 0, 360)),
      static_cast<uint16_t>(modules::Wrap((globals->gimbal_motor_yaw.pos() - (-0.288)) * 57.3 + 20, 0, 360)), 100, 100);
  UIGroup7.figure5.fillLine(u8"cap", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 50, 37, 260, 37,
                            static_cast<int16_t>((globals->cms.rx_data_.cap_energy / 252.0) * (750 - 260) + 260));
  UIGroup7.figure6.fillRec(u8"ca1", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 50, 37, 260, 87, 750);

  const auto dataLen = rm::device::Referee0x301Prepare(Info_Arr, 0, UIGroup7, globals->ref.data().robot_status.robot_id,
                                                       globals->ref.data().robot_status.robot_id + 256);
  globals_no_dtcm.usart1_ref.Write(Info_Arr, dataLen, 20);
}

void UIrefrenceline() {
  UIFigure7 UIGroup7;
  UIGroup7.figure1.fillLine(u8"cap", UIFigure::Operation::Edit, 0, UIFigure::Color::RedBlue, 50, 37, 260, 37,
                            static_cast<int16_t>((globals->cms.rx_data_.cap_energy / 252.0) * (750 - 260) + 260));
  const auto dataLen = rm::device::Referee0x301Prepare(Info_Arr, 0, UIGroup7, globals->ref.data().robot_status.robot_id,
                                                       globals->ref.data().robot_status.robot_id + 256);
  globals_no_dtcm.usart1_ref.Write(Info_Arr, dataLen, 20);
}

//
// // 主任务中只需要更新调度器即可
// void GlobalWarehouse::SubLoop10Hz() {
//   if (globals->time_ % 34 == 0) {
//     schedule.schedule();
//   }
// }