
#include "common/bsp/timer_task.hpp"

#include "adc.h"

#include "globals.hpp"

Globals *globals{nullptr};
__attribute__((section(".sram4"))) GlobalsNoDtcm globals_no_dtcm;

volatile long long mainloop_time_us = 0;  ///< 主循环运行时间

void SubLoop100hz() {
  // 更新LED和蜂鸣器
  const auto &[r, g, b] = globals->led_controller.Update();
  globals->led.SetColor(r, g, b);
  globals->buzzer.SetFrequency(globals->buzzer_controller.Update().frequency);
}
void SubLoop500hz() {
  // 更新输入电压测量值
  globals->vbus = (globals_no_dtcm.adc_val * 3.3f / 65535) * 11.0f;
}

void MainLoop2khz() {
  static size_t loop_divisor = 0;

  if (loop_divisor % 20 == 0) {  // 100hz
    SubLoop100hz();
  }
  if (loop_divisor % 4 == 1) {  // 500hz
    SubLoop500hz();
  }

  loop_divisor = (loop_divisor + 1) % 2000;
}

extern "C" {

void AppMain() {
  globals = new Globals;
  globals->buzzer.Init();

  // 初始化输入电压测量
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t *>(&globals_no_dtcm.adc_val), 1);

  // 创建主循环定时任务，定频2khz
  TimerTask mainloop{
      &htim13,                                       //
      etl::delegate<void()>::create<MainLoop2khz>()  //
  };
  mainloop.Start();

  globals->led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
  globals->buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();

  for (;;) {
  }
}
}