#pragma once

#include <librm.hpp>

#include "common/bsp/buzzer.hpp"
#include "common/bsp/ws2812.hpp"

struct Globals {
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

  float vbus;  ///< XT30输入电压
};

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
  uint16_t adc_val;  ///< 输入电压ADC采样值
};

extern Globals *globals;
extern __attribute__((section(".sram4"))) GlobalsNoDtcm globals_no_dtcm;