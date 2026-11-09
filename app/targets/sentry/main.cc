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

#include "BigGimbal.hpp"
#include "main.hpp"
#include "SmallGimbal.hpp"

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

float RadToDeg(float value) { return value * 180.0f / static_cast<float>(M_PI); }

}  // namespace

extern "C" [[noreturn]] void AppMain(void) {
  rm::Sleep(std::chrono::milliseconds(100));

  globals = new GlobalWarehouse;
  big_gimbal = new BigGimbal;
  small_gimbal = new SmallGimbal;

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
      {0x07, 0x06, 3.14159f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f},},
  };
  wheel_lf = new rm::device::M3508{*can1, 1, true};
  wheel_rf = new rm::device::M3508{*can1, 4};
  wheel_lb = new rm::device::M3508{*can1, 2, true};
  wheel_rb = new rm::device::M3508{*can1, 3};

  up_yaw_motor = new rm::device::GM6020{*can2, 1};
  pitch_motor = new rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>{
      *can2,
      {0x03, 0x02, 3.14159f, 30.0f, 10.0f, {0.0f, 500.0f}, {0.0f, 5.0f}},
  };
  friction_left = new rm::device::M3508{*can2, 6};
  friction_right = new rm::device::M3508{*can2, 7};
  dial_motor = new rm::device::M3508{*can2, 8};

  device_rc << wfly_et16s;
  device_nuc << aimbot_communicator << navigate_communicator;
  device_big_gimbal << down_yaw_motor;
  device_chassis << wheel_lf << wheel_rf << wheel_lb << wheel_rb;
  device_small_gimbal << up_yaw_motor << pitch_motor;
  device_shoot << friction_left << friction_right << dial_motor;

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

  big_gimbal->Init();
  small_gimbal->Init();
}

void GlobalWarehouse::Loop1kHz() {
  ++tick;

  UpdateBoardSensors();
  if ((tick % 10U) == 0U) {
    UpdateDeviceManagers();
    UpdateModeFromRemote();
  }

  big_gimbal->Task1kHz();
  small_gimbal->Task1kHz();

  SendVisionFrames();

  if ((tick & 1U) == 0U) {
    rm::device::DjiMotorBase::SendCommand(*can1);
  } else {
    rm::device::DjiMotorBase::SendCommand(*can2);
  }

  // if ((tick % 20U) == 0U) {
  //   UpdateIndicators();
  // }
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
  device_big_gimbal.Update();
  device_chassis.Update();
  device_small_gimbal.Update();
  device_shoot.Update();
}

void GlobalWarehouse::UpdateModeFromRemote() {
  robot_mode = RobotMode::kNoForce;

  if (!RcOnline()) {
    return;
  }

  if (wfly_et16s->SwitchIsUp(rc_ch::SF)) {
    robot_mode = RobotMode::kMatch;
  }
}

void GlobalWarehouse::SendVisionFrames() {
  if ((tick & 1U) == 0U) {
    if (wfly_et16s->SwitchIsUp(rc_ch::SE)) {
      if (navigate_communicator->aimbot_mode() == 1U) {
        aim_mode = referee_data->data().game_status.stage_remain_time >= 240U ? 2U : 3U;
      } else if (navigate_communicator->outpost_mode() == 1U) {
        aim_mode = 4U;
      } else {
        aim_mode = 1U;
      }
    } else {
      const auto ld = wfly_et16s->remote_data().knob[rc_ch::LD - rc_ch::LS];
      if (ld <= 150) {
        aim_mode = 2U;
      } else if (ld <= 700) {
        aim_mode = 3U;
      } else if (ld <= 1200) {
        aim_mode = 1U;
      } else {
        aim_mode = 4U;
      }
    }
    aimbot_communicator->UpdateControl(RadToDeg(hipnuc_imu->yaw()), RadToDeg(-hipnuc_imu->pitch()),
                                       RadToDeg(hipnuc_imu->roll()), referee_data->data().robot_status.robot_id,
                                       aim_mode);
  } else {
    navigate_communicator->UpdateControl(small_gimbal->YawForNavigateDegree(), small_gimbal->PitchForNavigateDegree(),
                                         aimbot_communicator->distance(), big_gimbal->YawForNavigateDegree());
  }
}

void GlobalWarehouse::UpdateIndicators() {
  const auto [r, g, b] = led_controller.Update();
  led.SetColor(r, g, b);
  buzzer.SetFrequency(buzzer_controller.Update().frequency);
}

bool GlobalWarehouse::RcOnline() const { return device_rc.all_device_ok(); }

bool GlobalWarehouse::ShooterPowerEnabled() const {
  return referee_data->data().robot_status.power_management_shooter_output || robot_mode == RobotMode::kTest;
}

uint16_t GlobalWarehouse::HeatLimit() const {
  const uint16_t limit = referee_data->data().robot_status.shooter_barrel_heat_limit;
  return limit == 0 ? 350 : limit;
}

uint16_t GlobalWarehouse::HeatCurrent() const {
  return referee_data->data().power_heat_data.shooter_17mm_1_barrel_heat;
}
