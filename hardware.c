/*
 * Copyright (c) 2019-2020, Dmitry (DiSlord) dislordlive@gmail.com
 * Based on TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
 * All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * The software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */
#include "ch.h"
#include "hal.h"
#include "nanovna.h"

// Compact STM32 ADC library
#if HAL_USE_ADC == TRUE
#error "Error VNA use self ADC lib, define HAL_USE_ADC = FALSE in halconf.h"
#endif
#ifdef NANOVNA_F303
#include "NANOVNA_STM32_F303/adc_v3.c"
#else
#include "NANOVNA_STM32_F072/adc_v1.c"
#endif

// Compact STM32 I2C library
#if HAL_USE_I2C == TRUE
#error "Error VNA use self I2C lib, define HAL_USE_I2C = FALSE in halconf.h"
#endif
#ifdef NANOVNA_F303
#include "NANOVNA_STM32_F303/i2c_v2.c"
#else
#include "NANOVNA_STM32_F072/i2c_v2.c"
#endif

#ifdef __USE_RTC__
// Compact STM32 RTC time library
#if HAL_USE_RTC == TRUE
#error "Error VNA use self RTC lib, define HAL_USE_RTC = FALSE in halconf.h"
#endif
#ifdef NANOVNA_F303
#include "NANOVNA_STM32_F303/rtc_v2.c"
#else
#include "NANOVNA_STM32_F072/rtc_v2.c"
#endif
#endif

// Compact STM32 DAC library
#if HAL_USE_DAC == TRUE
#error "Need disable HAL_USE_DAC in halconf.h for use VNA_DAC"
#endif
#ifdef NANOVNA_F303
#include "NANOVNA_STM32_F303/dac_v1.c"
#else
#include "NANOVNA_STM32_F072/dac_v1.c"
#endif

// Compact STM32 I2S library
#if HAL_USE_I2S == TRUE
#error "Need disable HAL_USE_DAC in halconf.h for use VNA_DAC"
#endif
#ifdef NANOVNA_F303
#include "NANOVNA_STM32_F303/i2s.c"
#else
#include "NANOVNA_STM32_F072/i2s.c"
#endif

// Compact STM32 flash library
#ifdef NANOVNA_F303
#include "NANOVNA_STM32_F303/flash.c"
#else
#include "NANOVNA_STM32_F072/flash.c"
#endif

// Compact STM32 GPIO library
#if HAL_USE_PAL == FALSE
#ifdef NANOVNA_F303
#include "NANOVNA_STM32_F303/gpio_v2.c"
#else
#include "NANOVNA_STM32_F072/gpio_v2.c"
#endif
#endif

// Compact STM32 DMA library
#ifdef NANOVNA_F303
#include "NANOVNA_STM32_F303/dma_v1.c"
#else
#include "NANOVNA_STM32_F072/dma_v1.c"
#endif

// Compact STM32 EXT library
#if HAL_USE_EXT == FALSE
#ifdef NANOVNA_F303
#include "NANOVNA_STM32_F303/exti_v1.c"
#else
#include "NANOVNA_STM32_F072/exti_v1.c"
#endif
#endif

#if HAL_USE_GPT == FALSE
// Run TIM2 as us timer counter (used as STM32_ST_TIM timer in ChibiOS)
// Run TIM3 as ms timer counter
void initTimers(void) {
//  rccEnableTIM2(FALSE);
  rccEnableTIM3(FALSE);
  // TIM2 use AHB1 bus clock (32 bit timer), use STM32_TIMCLK1 clock source
//  TIM2->PSC = STM32_TIMCLK1 / (1000000U) - 1; // 1MHz tick
  // TIM3 use AHB1 bus clock (16 bit timer), used in touch period handler
  TIM3->PSC = STM32_TIMCLK1 / (1000U) - 1;    // 1kHz tick
  TIM3->CR2 = 0x20;                          // Generate TRIGO event for ADC watchdog
}

//
void startTimer(TIM_TypeDef *timer, uint32_t period) {
  timer->ARR = period - 1;
  timer->EGR = STM32_TIM_EGR_UG;
  timer->CNT = 0;
  timer->CR1 = STM32_TIM_CR1_URS | STM32_TIM_CR1_CEN;
}
#endif
