#pragma once

#include <librm.hpp>
#include "adc.h"
#include "fdcan.h"
#include "usart.h"
#include "spi.h"

#include "common/bsp/buzzer.hpp"
#include "common/bsp/ws2812.hpp"

struct Globals {
  /*---------------------------------------------------全局对象---------------------------------------------------*/
  /*---------------------------------------------------全局对象---------------------------------------------------*/
  /*---------------------------------------------------全局对象---------------------------------------------------*/

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

  /*---------------------------------------------------全局变量---------------------------------------------------*/
  /*---------------------------------------------------全局变量---------------------------------------------------*/
  /*---------------------------------------------------全局变量---------------------------------------------------*/
  float vbus;  ///< XT30输入电压

  void Init() {
    buzzer.Init();
    led_controller.SetPattern<rm::modules::led_pattern::GreenBreath>();
    buzzer_controller.Play<rm::modules::buzzer_melody::Startup>();
  }
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
  /*---------------------------------------------------全局对象---------------------------------------------------*/
  /*---------------------------------------------------全局对象---------------------------------------------------*/
  /*---------------------------------------------------全局对象---------------------------------------------------*/
  rm::hal::Can can1{hfdcan1}, can2{hfdcan2};  ///< CAN 总线接口
  rm::hal::Serial<50> dbus{huart5, true, true}, usart1_ref{huart1, true, true},
      uart10_tc{huart10, true, true};  ///< 串口接口

  rm::device::DR16 rc{dbus};                                       ///< DT7
  rm::device::Referee<rm::device::RefereeRevision::kNewV120> ref;  ///< 裁判系统
  rm::device::VT03 tc;                                             ///< 图传
  /*---------------------------------------------------全局变量---------------------------------------------------*/
  /*---------------------------------------------------全局变量---------------------------------------------------*/
  /*---------------------------------------------------全局变量---------------------------------------------------*/
  uint16_t adc_val;  ///< 输入电压ADC采样值

  void Init() {
    // 初始化输入电压测量
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t *>(&adc_val), 1);

    // CAN初始化
    can1.SetFilter(0, 0);
    can1.Begin();
    can2.SetFilter(0, 0);
    can2.Begin();
    rc.Begin();

    // 裁判系统串口接收
    const rm::hal::SerialRxCallbackFunction ref_rx_callback = [&](const etl::span<const uint8_t> &data) {
      for (const auto byte : data) {
        ref << byte;
      }
    };
    usart1_ref.AttachRxCallback(ref_rx_callback);
    usart1_ref.Start();

    // 图传串口接收
    rm::hal::SerialRxCallbackFunction tc_rx_callback = [&](const etl::span<const uint8_t> &data) {
      for (const auto byte : data) {
        tc << byte;
      }
    };
    uart10_tc.AttachRxCallback(tc_rx_callback);
    uart10_tc.Start();
  }
};

extern Globals *globals;
extern __attribute__((section(".sram4"))) GlobalsNoDtcm globals_no_dtcm;