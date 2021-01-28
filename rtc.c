/*
 * Copyright (c) 2019-2020, Dmitry (DiSlord) dislordlive@gmail.com
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

#ifdef __USE_RTC__

// Compact STM32 RTC time library
#if HAL_USE_RTC == TRUE
#error "Error VNA use self RTC lib, define HAL_USE_RTC = FALSE in halconf.h"
#endif

// Get RTC time as binary structure in 0x00HHMMSS
uint32_t rtc_get_tr_bin(void){
  uint32_t tr = RTC->TR;
  uint32_t v = (tr&0x0F0F0F) + ((tr&0x707070)>>1) + ((tr&0x707070)>>3);
  return v;
}

// Get RTC time as binary structure in 0x00YYMMDD
uint32_t rtc_get_dr_bin(void){
  uint32_t dr = RTC->DR;
  uint32_t v = (dr&0x000F0F0F) + ((dr&0x00F01030)>>1) + ((dr&0x00F01030)>>3);
  return v;// | ((dr&0xE000)<<15); // day of week at end
}

uint32_t rtc_get_FAT(void) {
  uint32_t fattime;
  uint32_t tr = rtc_get_tr_bin();
  uint32_t dr = rtc_get_dr_bin();
  fattime  = ((tr>> 0)&0xFF) >>  1U; // Seconds / 2
  fattime |= ((tr>> 8)&0xFF) <<  5U; // Minutes
  fattime |= ((tr>>16)&0xFF) << 11U; // Hour
  fattime |= ((dr>> 0)&0xFF) << 16U; // Day
  fattime |= ((dr>> 8)&0xFF) << 21U; // Month
  fattime |= (((dr>>16)&0xFF) + RTC_START_YEAR - 1980) << 25U; // Local year begin from 2000, fat from 1980
  return fattime;
}

// Finish of configuration procedure.
static void rtc_exit_init(void) {
  RTC->ISR &= ~RTC_ISR_INIT;
}

// Beginning of configuration procedure.
static bool rtc_enter_init(void){
  RTC->ISR |= RTC_ISR_INIT;
  uint32_t count = 4*65536;
  while (--count)
    if (RTC->ISR & RTC_ISR_INITF)
      return true;
  return false;
}

void rtc_set_time(uint32_t dr, uint32_t tr) {
  if (rtc_enter_init()){
    RTC->TR = tr;     // Write TR register
    RTC->DR = dr;     // Write TD register
  }
  rtc_exit_init();
}

#ifdef VNA_AUTO_SELECT_RTC_SOURCE

// Enable LSE bypass if need
#if defined(STM32_LSE_BYPASS)
#define STM32_LSE_BYPASS     RCC_BDCR_LSEBYP
#else
#define STM32_LSE_BYPASS     0
#endif

// Startup LSE or if not work, LSI generator
static void rtc_start_source(void){
  // LSE already work (enabled and ready)
  if ((RCC->BDCR & (RCC_BDCR_LSEON|RCC_BDCR_LSERDY|STM32_LSE_BYPASS)) == (RCC_BDCR_LSEON|RCC_BDCR_LSERDY|STM32_LSE_BYPASS))
    return;

  // If LSE not enabled, try startup
  RCC->BDCR |= STM32_LSEDRV | STM32_LSE_BYPASS | RCC_BDCR_LSEON;
  // Waits until LSE is stable (need ~150ms for startup).
  chThdSleepMilliseconds(200);
  if (RCC->BDCR & RCC_BDCR_LSERDY) return;

  // Startup LSI if not allow start LSE
  RCC->CSR |= RCC_CSR_LSION;
  while ((RCC->CSR & RCC_CSR_LSIRDY) == 0);
}

static void resetBCDR(uint32_t rtc_drv){
  // Backup domain reset, for change source.
  RCC->BDCR = RCC_BDCR_BDRST;
  RCC->BDCR = 0;
  // Startup again source generator
  rtc_start_source();
  // Select new clock source. And enable
  RCC->BDCR|= rtc_drv;
}

void auto_backup_domain_init(void){
  // Init Backup domain, RTC clock source
  uint32_t rtc_drv;
  // Backup domain access enabled and left open.
  PWR->CR |= PWR_CR_DBP;
  // Start/check source
  rtc_start_source();
  // Check LSE ready, if ok, select as source
  rtc_drv = RCC->BDCR & RCC_BDCR_LSERDY ? STM32_RTCSEL_LSE|RCC_BDCR_RTCEN :  // Select LSE as source
                                          STM32_RTCSEL_LSI|RCC_BDCR_RTCEN;   // Select LSI as source
  // If the backup domain hasn't been initialized yet or work on different source, then proceed with initialization
  if ((RCC->BDCR & (STM32_RTCSEL_MASK|RCC_BDCR_RTCEN)) != rtc_drv)
    resetBCDR(rtc_drv);
/*
  // Check RTC clock, and reset backup domain to LSI if clock not start
  if (rtc_enter_init())
    rtc_exit_init();
  else
    resetBCDR(STM32_RTCSEL_LSI|RCC_BDCR_RTCEN);
*/
}
#endif

// Initiate RTC clock
void rtc_init(void){
#ifdef VNA_AUTO_SELECT_RTC_SOURCE
  // Auto start LSE or LSI source for RTC
  auto_backup_domain_init();
#else
  // ChibiOS init BDCR LSE or LSI source by self from user defined in mcuconf.h source
  // For add auto select RTC source need rewrite it
  // see hal_lld_backup_domain_init() in hal_lld.c for every CPU
  // Default RTC clock is LSE, but it possible not launch if no quartz installed
#endif
  uint32_t src = RCC->BDCR & STM32_RTCSEL_MASK;
  if (src == STM32_RTCSEL_NOCLOCK) return;
  // If calendar has not been initialized yet or different PRER settings then proceed with the initial setup.
  // Disable write protection.
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;
  uint32_t rtc_prer = (src == STM32_RTCSEL_LSE) ? STM32_RTC_LSE_PRER :
                                                  STM32_RTC_LSI_PRER;
  // If calendar has not been initialized yet then proceed with the initial setup.
  if ((RTC->ISR & RTC_ISR_INITS) == 0 || RTC->PRER != rtc_prer) {
    if (rtc_enter_init()){
      RTC->CR   = 0;
      RTC->ISR  = RTC_ISR_INIT;     // Clearing all but RTC_ISR_INIT.
      RTC->PRER = rtc_prer;         // Prescaler value loaded in registers 2 times
      RTC->PRER = rtc_prer;
    }
    // Finalizing of configuration procedure.
    rtc_exit_init();
  }
  else
    RTC->ISR &= ~RTC_ISR_RSF;
}
#endif // __USE_RTC__
