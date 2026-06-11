#include <cstddef>
#include <new>
#include <utility>

#include <librm.hpp>

#include "fdcan.h"
#include "usart.h"
#include "spi.h"
#include "adc.h"

#include "common/bsp/timer_task.hpp"

#include "main.hpp"
#include "Gimbal.hpp"
#include "Chassis.hpp"

using namespace rm;

GlobalWarehouse *globals{nullptr};

__attribute__((section(".sram4"), aligned(32))) GlobalsNoDtcm globals_no_dtcm;

volatile long long mainloop_time_us = 0;

namespace {

template <typename T, typename... Args>
T *ConstructInDma(DmaObjectStorage<T> &storage, Args &&...args) {
  return new (storage.get()) T{std::forward<Args>(args)...};
}

void InvalidateDCacheLineIfEnabled(const void *addr, size_t size) {
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U) {
    SCB_InvalidateDCache_by_Addr(const_cast<uint32_t *>(reinterpret_cast<const uint32_t *>(addr)), size);
  }
#else
  (void)addr;
  (void)size;
#endif
}

}  // namespace

void MainLoop2kHz() {
  static size_t loop_divisor = 0;

  if (loop_divisor % 4 == 0) globals->SubLoop500Hz();
  if (loop_divisor % 8 == 0) globals->SubLoop250Hz();
  if (loop_divisor % 20 == 0) globals->SubLoop100Hz();
  if (loop_divisor % 40 == 0) globals->SubLoop50Hz();
  if (loop_divisor % 200 == 0) globals->SubLoop10Hz();

  loop_divisor = (loop_divisor + 1) % 2000;
}

extern "C" [[noreturn]] void AppMain(void) {
  rm::Sleep(std::chrono::milliseconds(100));

  globals = new GlobalWarehouse;
  gimbal = new Gimbal;
  chassis = new Chassis;

  globals->Init();

  globals_no_dtcm.adc.value = 0;
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t *>(&globals_no_dtcm.adc.value), 1);

  // for (auto ch : {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4}) {
  //   HAL_TIM_PWM_Start(&htim1, ch);
  // }

  TimerTask mainloop{
      &htim13,
      etl::delegate<void()>::create<MainLoop2kHz>(),
  };

  mainloop.Start();

  for (;;) {
    // __WFI();
  }
}

void GlobalWarehouse::Init() {
  can1 = new rm::hal::Can{hfdcan1};
  can2 = new rm::hal::Can{hfdcan2};

  aimbot_communicator = new rm::device::AimbotCanCommunicator(*can1);
  navigate_communicator = new rm::device::NavigateCanCommunicator(*can2);

  // H7 串口使用 DMA，Serial 对象本体必须放到非 DTCM 区域。
  ident_uart = ConstructInDma(globals_no_dtcm.ident_uart, huart7, false, false);
  dbus = ConstructInDma(globals_no_dtcm.dbus, huart5, false, true);
  referee_uart = ConstructInDma(globals_no_dtcm.referee_uart, huart1, true, true);

  referee_data = new rm::device::Referee<rm::device::RefereeRevision::kNewV120>;
  rx_referee = new rm::device::RxReferee{*referee_uart, *referee_data};

  imu = new rm::device::BMI088{hspi1, ACC_CS_GPIO_Port, ACC_CS_Pin, GYRO_CS_GPIO_Port, GYRO_CS_Pin};
  hipnuc_imu = new rm::device::HipnucImuCan{*can2, 8};

  wfly_et16s = new WflyET16s{*dbus};

  up_yaw_motor = new rm::device::GM6020{*can2, 1};

  down_yaw_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
      *can1,
      {0x07, 0x06, 3.14159, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}},
  };

  pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
      *can2,
      {0x03, 0x02, 3.14159, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}},
  };

  friction_left = new rm::device::M3508{*can2, 3};
  friction_right = new rm::device::M3508{*can2, 2};
  dial_motor = new rm::device::M3508{*can2, 4};

  wheel_lf = new rm::device::M3508{*can1, 1};
  wheel_rf = new rm::device::M3508{*can1, 3};
  wheel_lb = new rm::device::M3508{*can1, 4};
  wheel_rb = new rm::device::M3508{*can1, 2};

  device_rc << wfly_et16s;
  device_nuc << aimbot_communicator << navigate_communicator;
  device_gimbal << up_yaw_motor << down_yaw_motor << pitch_motor;
  device_shoot << friction_left << friction_right << dial_motor;
  device_chassis << wheel_lf << wheel_rf << wheel_lb << wheel_rb;

  can1->SetFilter(0, 0);
  can1->Begin();

  can2->SetFilter(0, 0);
  can2->Begin();

  wfly_et16s->Begin();
  rx_referee->Begin();

  buzzer.Init();

  led_controller.SetPattern<modules::led_pattern::GreenBreath>();
  buzzer_controller.Play<modules::buzzer_melody::Startup>();

  GimbalPIDInit();
  ChassisPIDInit();
  ShootPIDInit();

  gimbal->GimbalInit();
  chassis->ChassisInit();
}

void GlobalWarehouse::GimbalPIDInit() {
  gimbal_controller.pid().up_yaw_position.SetKp(20.0f).SetKi(0.0f).SetKd(120.0f).SetMaxOut(20000.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().up_yaw_speed.SetKp(7000.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(25000.0f).SetMaxIout(0.0f);

  gimbal_controller.pid().down_yaw_position.SetKp(38.0f).SetKi(0.0f).SetKd(2000.0f).SetMaxOut(30.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().down_yaw_speed.SetKp(2.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10.0f).SetMaxIout(0.0f);

  gimbal_controller.pid().pitch_position.SetKp(120.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(30.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().pitch_speed.SetKp(0.5f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::ChassisPIDInit() {
  chassis_controller.pid().lf_wheel.SetKp(8.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(15000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rf_wheel.SetKp(8.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(15000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lb_wheel.SetKp(8.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(15000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rb_wheel.SetKp(8.0f).SetKi(0.0f).SetKd(1.0f).SetMaxOut(15000.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::ShootPIDInit() {
  shoot_controller.pid().fric_1_speed.SetKp(8.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(16384.0f).SetMaxIout(0.0f);
  shoot_controller.pid().fric_2_speed.SetKp(8.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(16384.0f).SetMaxIout(0.0f);
  shoot_controller.pid().loader_position.SetKp(0.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
  shoot_controller.pid().loader_speed.SetKp(8.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::RCStateUpdate() {
  if (referee_data->data().robot_status.power_management_gimbal_output && !last_gimbal_power) {
    gimbal_init_time = 500;
  }

  last_gimbal_power = referee_data->data().robot_status.power_management_gimbal_output;

  if (gimbal_init_time > 0) {
    gimbal_init_time--;
  }

  if (!device_rc.all_device_ok() || gimbal_init_time > 0 ||
      !referee_data->data().robot_status.power_management_gimbal_output) {
    StateMachine_ = kUnable;
  } else {
    switch (wfly_et16s->switch_position(rc_ch::SD)) {
      case SwitchPosition::kUp:
        switch (wfly_et16s->switch_position(rc_ch::SA)) {
          case SwitchPosition::kDown:
            StateMachine_ = kMatch;
            break;

          case SwitchPosition::kUp:
          case SwitchPosition::kMid:
          default:
            StateMachine_ = kNoForce;
            break;
        }
        break;

      case SwitchPosition::kMid:
        switch (wfly_et16s->switch_position(rc_ch::SA)) {
          case SwitchPosition::kDown:
            if (wfly_et16s->switch_position(rc_ch::SE) == SwitchPosition::kMid) {
              StateMachine_ = kTest;
              gimbal->GimbalMove_ = kGbIdentify;
              chassis->ChassisMove_ = kNoForce;
            } else if (wfly_et16s->switch_position(rc_ch::SE) == SwitchPosition::kDown) {
              StateMachine_ = kTest;
              gimbal->GimbalMove_ = kGbFfVerify;
              chassis->ChassisMove_ = kNoForce;
            } else {
              StateMachine_ = kTest;
              gimbal->GimbalMove_ = kGbRemote;
              chassis->ChassisMove_ = kCsRemote;
            }
            break;

          case SwitchPosition::kMid:
            StateMachine_ = kTest;
            gimbal->GimbalMove_ = kGbScan;
            chassis->ChassisMove_ = kCsNavigate;
            break;

          case SwitchPosition::kUp:
            StateMachine_ = kTest;
            gimbal->GimbalMove_ = kGbAimbot;
            chassis->ChassisMove_ = kNoForce;
            break;

          default:
            StateMachine_ = kNoForce;
            break;
        }
        break;

      case SwitchPosition::kDown:
        switch (wfly_et16s->switch_position(rc_ch::SA)) {
          case SwitchPosition::kUp:
            Music();
            StateMachine_ = kNoForce;
            break;

          case SwitchPosition::kMid:
          case SwitchPosition::kDown:
          default:
            StateMachine_ = kNoForce;
            break;
        }
        break;

      default:
        StateMachine_ = kNoForce;
        break;
    }
  }
}

void GlobalWarehouse::Music() {
  if (wfly_et16s->wheel_position(rc_ch::LS) >= 650 && music == 0 &&
      wfly_et16s->switch_position(rc_ch::SH) == SwitchPosition::kDown) {
    music = 1;
  }

  if (wfly_et16s->wheel_position(rc_ch::LS) <= -650 && !music_change_flag &&
      wfly_et16s->switch_position(rc_ch::SH) == SwitchPosition::kDown) {
    music_choice++;

    if (music_choice == 3) {
      music_choice = 0;
    }

    buzzer_controller.Play<modules::buzzer_melody::Beeps<1>>();
    music_change_flag = true;
  }

  if (wfly_et16s->switch_position(rc_ch::SH) == SwitchPosition::kUp && (music == 0 || music == 2)) {
    music_change_flag = false;
    music = 0;
  }

  if (music == 1) {
    if (music_choice == 1) {
      buzzer_controller.Play<modules::buzzer_melody::SeeUAgain>();
      music = 2;
    }

    if (music_choice == 2) {
      buzzer_controller.Play<modules::buzzer_melody::SuperMario>();
      music = 2;
    }
  }
}

void GlobalWarehouse::SubLoop500Hz() {
  InvalidateDCacheLineIfEnabled(&globals_no_dtcm.adc, sizeof(globals_no_dtcm.adc));

  vbus = (globals_no_dtcm.adc.value * 3.3f / 65535.0f) * 11.0f;

  imu->Update();

  ahrs.Update(rm::modules::ImuData6Dof{
      imu->gyro_x(),
      imu->gyro_y(),
      imu->gyro_z() - 0.0015f,
      imu->accel_x(),
      imu->accel_y(),
      imu->accel_z(),
  });

  rm::device::DjiMotorBase::SendCommand(*can1);
  rm::device::DjiMotorBase::SendCommand(*can2);

  RCStateUpdate();

  gimbal->GimbalTask();
  chassis->ChassisTask();

  f32 shoot_initial_speed = 0.0f;

  if (referee_data->data().shoot_data.initial_speed >= 18.f &&
      referee_data->data().shoot_data.initial_speed <= 26.f) {
    shoot_initial_speed = referee_data->data().shoot_data.initial_speed;
  } else {
    shoot_initial_speed = 22.5f;
  }

  aimbot_communicator->UpdateControl(
      hipnuc_imu->yaw(),
      hipnuc_imu->pitch(),
      -hipnuc_imu->roll(),
      referee_data->data().robot_status.robot_id,
      aim_mode,
      imu_count,
      shoot_initial_speed);

  down_yaw_motor->SetMitCommand(0, 0, -gimbal_controller.output().down_yaw, 0, 0);
  pitch_motor->SetMitCommand(0, 0, -gimbal->pitch_torque_, 0, 3.4f);
}

void GlobalWarehouse::SubLoop250Hz() {}

void GlobalWarehouse::SubLoop100Hz() {
  device_rc.Update();
  device_nuc.Update();
  device_gimbal.Update();
  device_shoot.Update();
  device_chassis.Update();

  for (i8 i = 0; i < 7; i++) {
    if (wfly_et16s->switch_position(i + 4) != SwitchPosition::kUnknown) {
      if (last_switch[i] != wfly_et16s->switch_position(i + 4)) {
        buzzer_controller.Play<modules::buzzer_melody::Beeps<1>>();
        last_switch[i] = wfly_et16s->switch_position(i + 4);
      }
    }
  }
}

void GlobalWarehouse::SubLoop50Hz() {
  gimbal->GimbalIdentifyDataSend();

  const auto &[led_r, led_g, led_b] = led_controller.Update();
  led.SetColor(led_r, led_g, led_b);

  buzzer.SetFrequency(buzzer_controller.Update().frequency);
}

void GlobalWarehouse::SubLoop10Hz() {
  time = 0;
}
