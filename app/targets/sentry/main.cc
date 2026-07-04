#include <cstddef>
#include <new>
#include <utility>

#include <librm.hpp>

#include "adc.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

#include "common/bsp/timer_task.hpp"

#include "Chassis.hpp"
#include "Gimbal.hpp"
#include "main.hpp"

using namespace rm;

GlobalWarehouse *globals{nullptr};
__attribute__((section(".sram4"), aligned(32))) GlobalsNoDtcm globals_no_dtcm;

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

void ClearUartRxErrorState(UART_HandleTypeDef &huart) {
  HAL_UART_AbortReceive(&huart);
  __HAL_UART_CLEAR_PEFLAG(&huart);
  __HAL_UART_CLEAR_FEFLAG(&huart);
  __HAL_UART_CLEAR_NEFLAG(&huart);
  __HAL_UART_CLEAR_OREFLAG(&huart);
  __HAL_UART_CLEAR_IDLEFLAG(&huart);
  huart.ErrorCode = HAL_UART_ERROR_NONE;
}

void SentryLoop1kHz() {
  if (globals != nullptr) {
    globals->Loop1kHz();
  }
}

float RadToDeg(float value) {
  return value * 180.0f / static_cast<float>(M_PI);
}

}  // namespace

extern "C" [[noreturn]] void AppMain(void) {
  rm::Sleep(std::chrono::milliseconds(100));

  globals = new GlobalWarehouse;
  gimbal = new Gimbal;
  chassis = new Chassis;

  globals->Init();

  TimerTask sentry_loop{
      &htim13,
      etl::delegate<void()>::create<SentryLoop1kHz>(),
  };
  sentry_loop.SetPrescalerAndPeriod(275U - 1U, 1000U - 1U);
  sentry_loop.Start();

  for (;;) {
    __WFI();
  }
}

void GlobalWarehouse::Init() {
  can1 = new rm::hal::Can{hfdcan1};
  can2 = new rm::hal::Can{hfdcan2};
  can3 = new rm::hal::Can{hfdcan3};

  aimbot_communicator = new rm::device::AimbotCanCommunicator(*can3);
  navigate_communicator = new rm::device::NavigateCanCommunicator(*can3);

  ident_uart = ConstructInDma(globals_no_dtcm.ident_uart, huart7, false, false);
  dbus = ConstructInDma(globals_no_dtcm.dbus, huart5, false, true);
  referee_uart = ConstructInDma(globals_no_dtcm.referee_uart, huart1, true, true);
  hipnuc_uart = ConstructInDma(globals_no_dtcm.hipnuc_uart, huart10, false, true);

  referee_data = new rm::device::Referee<rm::device::RefereeRevision::kNewV120>;
  rx_referee = new rm::device::RxReferee{*referee_uart, *referee_data};
  imu = new rm::device::BMI088{hspi2, ACC_CS_GPIO_Port, ACC_CS_Pin, GYRO_CS_GPIO_Port, GYRO_CS_Pin};
  hipnuc_imu = new rm::device::HipnucImu{*hipnuc_uart};
  wfly_et16s = new WflyET16s{*dbus};

  down_yaw_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
      *can1,
      {0x07, 0x06, 3.14159f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}},
  };
  up_yaw_motor = new rm::device::GM6020{*can2, 5};
  pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
      *can2,
      {0x03, 0x02, 3.14159f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}},
  };
  friction_left = new rm::device::M3508{*can2, 6};
  friction_right = new rm::device::M3508{*can2, 7};
  dial_motor = new rm::device::M3508{*can2, 8};

  wheel_lf = new rm::device::M3508{*can1, 1};
  wheel_rf = new rm::device::M3508{*can1, 2};
  wheel_lb = new rm::device::M3508{*can1, 3};
  wheel_rb = new rm::device::M3508{*can1, 4};

  device_rc << wfly_et16s;
  device_nuc << aimbot_communicator << navigate_communicator;
  device_gimbal << down_yaw_motor << up_yaw_motor << pitch_motor;
  device_shoot << friction_left << friction_right << dial_motor;
  device_chassis << wheel_lf << wheel_rf << wheel_lb << wheel_rb;

  can1->SetFilter(0, 0);
  can1->Begin();
  can2->SetFilter(0, 0);
  can2->Begin();
  can3->SetFilter(0, 0);
  can3->Begin();

  ClearUartRxErrorState(huart5);
  ClearUartRxErrorState(huart1);
  ClearUartRxErrorState(huart10);
  wfly_et16s->Begin();
  rx_referee->Begin();
  hipnuc_imu->Begin();

  globals_no_dtcm.adc.value = 0;
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t *>(&globals_no_dtcm.adc.value), 1);

  buzzer.Init();
  led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
  buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();

  InitPid();
  gimbal->Init();
  chassis->Init();
}

void GlobalWarehouse::InitPid() {
  gimbal_controller.pid().up_yaw_position.SetKp(12.5f).SetKi(0.0f).SetKd(700.0f).SetMaxOut(250.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().up_yaw_speed.SetKp(180.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(25000.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().down_yaw_position.SetKp(20.0f).SetKi(0.0f).SetKd(1000.0f).SetMaxOut(250.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().down_yaw_speed.SetKp(150.0f).SetKi(0.0f).SetKd(10.0f).SetMaxOut(10.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().pitch_position.SetKp(18.0f).SetKi(0.0f).SetKd(1000.0f).SetMaxOut(250.0f).SetMaxIout(0.0f);
  gimbal_controller.pid().pitch_speed.SetKp(135.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10.0f).SetMaxIout(0.0f);

  chassis_controller.pid().lf_wheel.SetKp(5.0f).SetKi(0.0f).SetKd(0.3f).SetMaxOut(15000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rf_wheel.SetKp(5.0f).SetKi(0.0f).SetKd(0.3f).SetMaxOut(15000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().lb_wheel.SetKp(5.0f).SetKi(0.0f).SetKd(0.3f).SetMaxOut(15000.0f).SetMaxIout(0.0f);
  chassis_controller.pid().rb_wheel.SetKp(5.0f).SetKi(0.0f).SetKd(0.3f).SetMaxOut(15000.0f).SetMaxIout(0.0f);

  shoot_controller.pid().fric_1_speed.SetKp(12.0f).SetKi(0.0f).SetKd(8.0f).SetMaxOut(16384.0f).SetMaxIout(0.0f);
  shoot_controller.pid().fric_2_speed.SetKp(12.0f).SetKi(0.0f).SetKd(8.0f).SetMaxOut(16384.0f).SetMaxIout(0.0f);
  shoot_controller.pid().fric_3_speed.SetKp(0.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(0.0f).SetMaxIout(0.0f);
  shoot_controller.pid().loader_speed.SetKp(16.0f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(10000.0f).SetMaxIout(0.0f);
}

void GlobalWarehouse::Loop1kHz() {
  ++tick;
  ++imu_count;

  UpdateBoardSensors();
  if ((tick % 10U) == 0U) {
    UpdateDeviceManagers();
    UpdateModeFromRemote();
  }

  gimbal->Task1kHz();
  chassis->Task1kHz();

  SendVisionFrames();

  rm::device::DjiMotorBase::SendCommand(*can1);
  rm::device::DjiMotorBase::SendCommand(*can2);

  if ((tick % 20U) == 0U) {
    UpdateIndicators();
  }
}

void GlobalWarehouse::UpdateBoardSensors() {
  InvalidateDCacheLineIfEnabled(&globals_no_dtcm.adc, sizeof(globals_no_dtcm.adc));
  vbus = (static_cast<float>(globals_no_dtcm.adc.value) * 3.3f / 65535.0f) * 11.0f;

  imu->Update();
  ahrs.Update(rm::modules::ImuData6Dof{
      imu->gyro_x(),
      imu->gyro_y(),
      imu->gyro_z() - 0.0015f,
      imu->accel_x(),
      imu->accel_y(),
      imu->accel_z(),
  });
}

void GlobalWarehouse::UpdateDeviceManagers() {
  device_rc.Update();
  device_nuc.Update();
  device_gimbal.Update();
  device_shoot.Update();
  device_chassis.Update();
}

void GlobalWarehouse::UpdateModeFromRemote() {
  robot_mode = RobotMode::kNoForce;
  requested_gimbal_mode = GimbalMode::kNoForce;
  requested_chassis_mode = ChassisMode::kNoForce;

  if (!RcOnline()) {
    return;
  }

  const auto sd = wfly_et16s->switch_position(rc_ch::SD);
  const auto sa = wfly_et16s->switch_position(rc_ch::SA);
  const auto sg = wfly_et16s->switch_position(rc_ch::SG);

  if (sd == SwitchPosition::kMid) {
    robot_mode = RobotMode::kTest;
    requested_gimbal_mode = (sa == SwitchPosition::kUp) ? GimbalMode::kAimbot : GimbalMode::kRemote;
    requested_chassis_mode = ChassisMode::kFollow;
  } else if (sd == SwitchPosition::kUp) {
    robot_mode = RobotMode::kMatch;
    requested_gimbal_mode = navigate_communicator->scan_mode() ? GimbalMode::kScan : GimbalMode::kAimbot;
    requested_chassis_mode = ChassisMode::kNavigate;
  }

  if (sa == SwitchPosition::kMid) {
    requested_gimbal_mode = GimbalMode::kScan;
  }
  if (sg == SwitchPosition::kDown) {
    requested_chassis_mode = ChassisMode::kSlowRotate;
  } else if (sg == SwitchPosition::kUp && robot_mode != RobotMode::kNoForce) {
    requested_chassis_mode = ChassisMode::kQuickRotate;
  }
}

void GlobalWarehouse::SendVisionFrames() {
  if ((tick & 1U) == 0U) {
    if (navigate_communicator->aimbot_mode() == 1U) {
      aim_mode = 2;
    } else if (navigate_communicator->outpost_mode() == 1U) {
      aim_mode = 4;
    } else {
      aim_mode = 1;
    }
    aimbot_communicator->UpdateControl(RadToDeg(hipnuc_imu->yaw()), RadToDeg(-hipnuc_imu->pitch()),
                                       RadToDeg(hipnuc_imu->roll()),
                                       referee_data->data().robot_status.robot_id, aim_mode, imu_count, 0.0f);
  } else {
    navigate_communicator->UpdateControl(gimbal->SmallYawDegree(), gimbal->SmallPitchDegree(),
                                         aimbot_communicator->distance());
  }
}

void GlobalWarehouse::UpdateIndicators() {
  const auto [r, g, b] = led_controller.Update();
  led.SetColor(r, g, b);
  buzzer.SetFrequency(buzzer_controller.Update().frequency);
}

bool GlobalWarehouse::RcOnline() const { return device_rc.all_device_ok(); }

bool GlobalWarehouse::GimbalPowerEnabled() const {
  return referee_data->data().robot_status.power_management_gimbal_output || robot_mode == RobotMode::kTest;
}

bool GlobalWarehouse::ChassisPowerEnabled() const {
  return referee_data->data().robot_status.power_management_chassis_output || robot_mode == RobotMode::kTest;
}

bool GlobalWarehouse::ShooterPowerEnabled() const {
  return referee_data->data().robot_status.power_management_shooter_output || robot_mode == RobotMode::kTest;
}

uint16_t GlobalWarehouse::HeatLimit() const {
  const uint16_t limit = referee_data->data().robot_status.shooter_barrel_heat_limit;
  return limit == 0 ? 350 : limit;
}

uint16_t GlobalWarehouse::HeatCurrent() const { return referee_data->data().power_heat_data.shooter_17mm_1_barrel_heat; }
