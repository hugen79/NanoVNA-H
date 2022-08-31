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

#include "halconf.h"
/*
 * adc.c
 * Used for:
 *  - battery voltage (adc_vbat_read)
 *  - software touch handler and measure touch position
 */
#if defined(NANOVNA_F303)
#define ADC_TOUCH_X  3
#define ADC_TOUCH_Y  4
#else
#define ADC_TOUCH_X  ADC_CHSELR_CHSEL6
#define ADC_TOUCH_Y  ADC_CHSELR_CHSEL7
#endif

// Measure vbat every 5 second
#define VBAT_MEASURE_INTERVAL   S2ST(5)

void adc_init(void);
uint16_t adc_single_read(uint32_t chsel);  // Read ADC channel (ADC_TOUCH_X for X, and ADC_TOUCH_X for Y)
void adc_start_analog_watchdog(void);      // Start touch interrupt handler
void adc_stop_analog_watchdog(void);       // Stop touch interrupt handler
int16_t adc_vbat_read(void);               // Read battery voltage

/*
 * I2C bus settings
 */
#define STM32_I2C_TIMINGS(presc, scldel, sdadel, sclh, scll) ( ((presc)  << I2C_TIMINGR_PRESC_Pos)  \
                                                             | ((scldel) << I2C_TIMINGR_SCLDEL_Pos) \
                                                             | ((sdadel) << I2C_TIMINGR_SDADEL_Pos) \
                                                             | ((sclh)   << I2C_TIMINGR_SCLH_Pos)   \
                                                             | ((scll)   << I2C_TIMINGR_SCLL_Pos) )
#if STM32_I2C1_CLOCK == 8    // STM32_I2C1SW == STM32_I2C1SW_HSI     (HSI=8MHz)
#if   STM32_I2C_SPEED == 400 // 400kHz @ HSI 8MHz (Use 26.4.10 I2C_TIMINGR register configuration examples from STM32 RM0091 Reference manual)
 #define STM32_I2C_INIT_T   STM32_I2C_TIMINGS(0U, 3U, 1U, 3U, 9U)
 #define STM32_I2C_TIMINGR  STM32_I2C_TIMINGS(0U, 3U, 1U, 3U, 9U)
#endif
#elif  STM32_I2C1_CLOCK == 48 // STM32_I2C1SW == STM32_I2C1SW_SYSCLK  (SYSCLK = 48MHz)
 #define STM32_I2C_INIT_T   STM32_I2C_TIMINGS(5U, 3U, 3U, 3U, 9U)
 #if   STM32_I2C_SPEED == 400 // 400kHz @ SYSCLK 48MHz (Use 26.4.10 I2C_TIMINGR register configuration examples from STM32 RM0091 Reference manual)
 #define STM32_I2C_TIMINGR  STM32_I2C_TIMINGS(5U, 3U, 3U, 3U, 9U)
 #elif STM32_I2C_SPEED == 600 // 600kHz @ SYSCLK 48MHz, manually get values, x1.5 I2C speed
 #define STM32_I2C_TIMINGR  STM32_I2C_TIMINGS(0U, 10U, 10U, 30U, 50U)
 #elif STM32_I2C_SPEED == 900 // 900kHz @ SYSCLK 48MHz, manually get values, x2 I2C speed
 #define STM32_I2C_TIMINGR  STM32_I2C_TIMINGS(0U, 10U, 10U, 23U, 30U)
 #endif
#elif  STM32_I2C1_CLOCK == 72 // STM32_I2C1SW == STM32_I2C1SW_SYSCLK  (SYSCLK = 72MHz)
 #define STM32_I2C_INIT_T   STM32_I2C_TIMINGS(0U, 20U, 20U, 80U, 100U)
 #if   STM32_I2C_SPEED == 400 // ~400kHz @ SYSCLK 72MHz (Use 26.4.10 I2C_TIMINGR register configuration examples from STM32 RM0091 Reference manual)
 #define STM32_I2C_TIMINGR  STM32_I2C_TIMINGS(0U, 10U, 10U, 80U, 100U)
 #elif STM32_I2C_SPEED == 600 // ~600kHz @ SYSCLK 72MHz, manually get values, x1.5 I2C speed
 #define STM32_I2C_TIMINGR  STM32_I2C_TIMINGS(0U, 10U, 10U, 40U, 80U)
 #elif STM32_I2C_SPEED == 900 // ~900kHz @ SYSCLK 72MHz, manually get values, x2 I2C speed
 #define STM32_I2C_TIMINGR  STM32_I2C_TIMINGS(0U, 10U, 10U, 40U, 65U)
 #endif
#endif

#ifndef STM32_I2C_TIMINGR
#error "Need define I2C bus TIMINGR settings"
#endif

void i2c_start(void);
void i2c_set_timings(uint32_t timings);
bool i2c_transfer(uint8_t addr, const uint8_t *w, size_t wn);
bool i2c_receive(uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn);

/*
 * rtc.c
 */
#ifdef __USE_RTC__
#define RTC_START_YEAR          2000

#define RTC_DR_YEAR(dr)         (((dr)>>16)&0xFF)
#define RTC_DR_MONTH(dr)        (((dr)>> 8)&0xFF)
#define RTC_DR_DAY(dr)          (((dr)>> 0)&0xFF)

#define RTC_TR_HOUR(dr)         (((tr)>>16)&0xFF)
#define RTC_TR_MIN(dr)          (((tr)>> 8)&0xFF)
#define RTC_TR_SEC(dr)          (((tr)>> 0)&0xFF)

// Init RTC
void rtc_init(void);
// Then read time and date TR should read first, after DR !!!
// Get RTC time as bcd structure in 0x00HHMMSS
#define rtc_get_tr_bcd() (RTC->TR & 0x007F7F7F)
// Get RTC date as bcd structure in 0x00YYMMDD (remove day of week information!!!!)
#define rtc_get_dr_bcd() (RTC->DR & 0x00FF1F3F)
// read TR as 0x00HHMMSS in bin (TR should be read first for sync)
uint32_t rtc_get_tr_bin(void);
// read DR as 0x00YYMMDD in bin (DR should be read second)
uint32_t rtc_get_dr_bin(void);
// Read time in FAT filesystem format
uint32_t rtc_get_FAT(void);
// Write date and time (need in bcd format!!!)
void rtc_set_time(uint32_t dr, uint32_t tr);
#endif

/*
 * Backup
 */
#ifdef __USE_BACKUP__
inline uint32_t get_backup_data32(uint16_t id) {
  switch (id) {
    case 0: return RTC->BKP0R;
    case 1: return RTC->BKP1R;
    case 2: return RTC->BKP2R;
    case 3: return RTC->BKP3R;
    case 4: return RTC->BKP4R;
  }
  return 0;
}
inline void set_backup_data32(uint16_t id, uint32_t data) {
  switch (id) {
    case 0: RTC->BKP0R = data; break;
    case 1: RTC->BKP1R = data; break;
    case 2: RTC->BKP2R = data; break;
    case 3: RTC->BKP3R = data; break;
    case 4: RTC->BKP4R = data; break;
  }
}
#endif

/*
 * dac.c
 * Used for LCD backlight control
 */
void dac_init(void);
void dac_setvalue_ch1(uint16_t v);
void dac_setvalue_ch2(uint16_t v);

/*
 * i2s.c
 * Used for read samples from audio codec
 */
void initI2S(void *buffer, uint16_t count);

/*
 * flash.c
 * Used for store config and calibration data on CPU flash
 */
#if defined(NANOVNA_F303)
// For STM32F303xC CPU setting
#define FLASH_START_ADDRESS   0x08000000
#define FLASH_TOTAL_SIZE     (256*1024)

#define FLASH_PAGESIZE 0x800

#define SAVEAREA_MAX 7

// Depend from config_t size, should be aligned by FLASH_PAGESIZE
#define SAVE_CONFIG_SIZE        0x00000800
// Depend from properties_t size, should be aligned by FLASH_PAGESIZE
#define SAVE_PROP_CONFIG_SIZE   0x00004000
#else
// For STM32F072xB CPU setting
#define FLASH_START_ADDRESS   0x08000000
#define FLASH_TOTAL_SIZE     (128*1024)

#define FLASH_PAGESIZE 0x800

#define SAVEAREA_MAX 5

// Depend from config_t size, should be aligned by FLASH_PAGESIZE
#define SAVE_CONFIG_SIZE        0x00000800
// Depend from properties_t size, should be aligned by FLASH_PAGESIZE
#define SAVE_PROP_CONFIG_SIZE   0x00001800
#endif

// Save config_t and properties_t flash area (see flash7 from *.ld settings)
#define SAVE_FULL_AREA_SIZE     (SAVE_CONFIG_SIZE + SAVEAREA_MAX * SAVE_PROP_CONFIG_SIZE)
// Save setting at end of CPU flash area
// Config at end minus full size
#define SAVE_CONFIG_ADDR        (FLASH_START_ADDRESS + FLASH_TOTAL_SIZE - SAVE_FULL_AREA_SIZE)
// Properties save area follow after config
#define SAVE_PROP_CONFIG_ADDR   (SAVE_CONFIG_ADDR + SAVE_CONFIG_SIZE)

// Erase settings on page
void flash_erase_pages(uint32_t page_address, uint32_t size);
// Write data
void flash_program_half_word_buffer(uint16_t* dst, uint16_t *data, uint16_t size);

/*
 * gpio.c
 */
#if HAL_USE_PAL == FALSE
#define PAL_STM32_MODE_MASK             (3U << 0U)
#define PAL_STM32_MODE_INPUT            (0U << 0U)
#define PAL_STM32_MODE_OUTPUT           (1U << 0U)
#define PAL_STM32_MODE_ALTERNATE        (2U << 0U)
#define PAL_STM32_MODE_ANALOG           (3U << 0U)

#define PAL_STM32_OTYPE_MASK            (1U << 2U)
#define PAL_STM32_OTYPE_PUSHPULL        (0U << 2U)
#define PAL_STM32_OTYPE_OPENDRAIN       (1U << 2U)

#define PAL_STM32_OSPEED_MASK           (3U << 3U)
#define PAL_STM32_OSPEED_LOW            (0U << 3U)
#define PAL_STM32_OSPEED_MEDIUM         (1U << 3U)
#define PAL_STM32_OSPEED_FAST           (2U << 3U)
#define PAL_STM32_OSPEED_HIGH           (3U << 3U)

#define PAL_STM32_PUPDR_MASK            (3U << 5U)
#define PAL_STM32_PUPDR_FLOATING        (0U << 5U)
#define PAL_STM32_PUPDR_PULLUP          (1U << 5U)
#define PAL_STM32_PUPDR_PULLDOWN        (2U << 5U)

#define PAL_STM32_ALTERNATE_MASK        (15U << 7U)
#define PAL_STM32_ALTERNATE(n)          ((n) << 7U)

#define PAL_STM32_ASCR_MASK             (1U << 11U)
#define PAL_STM32_ASCR_OFF              (0U << 11U)
#define PAL_STM32_ASCR_ON               (1U << 11U)

#define PAL_STM32_LOCKR_MASK            (1U << 12U)
#define PAL_STM32_LOCKR_OFF             (0U << 12U)
#define PAL_STM32_LOCKR_ON              (1U << 12U)

#define PAL_MODE_ALTERNATE(n)           (PAL_STM32_MODE_ALTERNATE |  PAL_STM32_ALTERNATE(n))
#define PAL_MODE_RESET                  PAL_STM32_MODE_INPUT
#define PAL_MODE_UNCONNECTED            (PAL_STM32_MODE_ANALOG |  PAL_STM32_ASCR_OFF | PAL_STM32_LOCKR_ON)
#define PAL_MODE_INPUT                  PAL_STM32_MODE_INPUT
#define PAL_MODE_INPUT_PULLUP           (PAL_STM32_MODE_INPUT | PAL_STM32_PUPDR_PULLUP)
#define PAL_MODE_INPUT_PULLDOWN         (PAL_STM32_MODE_INPUT | PAL_STM32_PUPDR_PULLDOWN)
#define PAL_MODE_INPUT_ANALOG           (PAL_STM32_MODE_ANALOG | PAL_STM32_ASCR_ON)
#define PAL_MODE_OUTPUT_PUSHPULL        (PAL_STM32_MODE_OUTPUT | PAL_STM32_OTYPE_PUSHPULL)

#define PAL_MODE_OUTPUT_OPENDRAIN       (PAL_STM32_MODE_OUTPUT | PAL_STM32_OTYPE_OPENDRAIN)

void initPal(void);

#define palSetPad(port, bit)   (port)->BSRR = 1<<((bit)+ 0)
#define palClearPad(port, bit) (port)->BSRR = 1<<((bit)+16)
#define palReadPort(port)      ((port)->IDR)

//#define palSetPadMode(port, bit, mask)  palSetPadGroupMode(port, 1U<<(bit), mask)
void palSetPadMode(GPIO_TypeDef *port, int bit, uint32_t mode);
void palSetPadGroupMode(GPIO_TypeDef *port, uint32_t mask, uint32_t mode);
#endif

/*
 * DMA channels support
 */
#define I2S_DMA_RX                             DMA1_Channel4              // DMA1 channel 4 use for I2S rx

// Interrupt handler for DMA
extern void i2s_lld_serve_rx_interrupt(uint32_t flags);
//#define DMA1_CH1_HANDLER_FUNC
//#define DMA1_CH2_HANDLER_FUNC
//#define DMA1_CH3_HANDLER_FUNC
#define DMA1_CH4_HANDLER_FUNC                  i2s_lld_serve_rx_interrupt
//#define DMA1_CH5_HANDLER_FUNC
//#define DMA1_CH6_HANDLER_FUNC
//#define DMA1_CH7_HANDLER_FUNC

#define dmaChannelSetMemory(ch, addr)          {(ch)->CMAR = (uint32_t)(addr);}
#define dmaChannelSetPeripheral(ch, addr)      {(ch)->CPAR = (uint32_t)(addr);}
#define dmaChannelSetTransactionSize(ch, size) {(ch)->CNDTR= (uint32_t)(size);}
#define dmaChannelGetTransactionSize(ch)       ((ch)->CNDTR)
#define dmaChannelSetMode(ch, mode)            {(ch)->CCR  = (uint32_t)(mode);}
#define dmaChannelEnable(ch)                   {(ch)->CCR |= STM32_DMA_CR_EN;}
#define dmaChannelDisable(ch)                  {(ch)->CCR &=~STM32_DMA_CR_EN;}
#define dmaChannelWaitCompletion(ch)           {while ((ch)->CNDTR > 0); (ch)->CCR = 0;}

#define STM32_DMA_CR_BYTE       (STM32_DMA_CR_PSIZE_BYTE | STM32_DMA_CR_MSIZE_BYTE)
#define STM32_DMA_CR_HWORD      (STM32_DMA_CR_PSIZE_HWORD | STM32_DMA_CR_MSIZE_HWORD)

/*
 * EXT interrupt support
 */
#if HAL_USE_EXT == FALSE
#define EXT_MODE_GPIO_OFF             4
#define EXT_MODE_GPIO_MASK         (0xF<<EXT_MODE_GPIO_OFF)
#define EXT_MODE_GPIOA             (  0<<EXT_MODE_GPIO_OFF)
#define EXT_MODE_GPIOB             (  1<<EXT_MODE_GPIO_OFF)
#define EXT_MODE_GPIOC             (  2<<EXT_MODE_GPIO_OFF)

#define EXT_CH_MODE_EDGES_MASK      3U
#define EXT_CH_MODE_DISABLED        0U
#define EXT_CH_MODE_RISING_EDGE     1U
#define EXT_CH_MODE_FALLING_EDGE    2U
#define EXT_CH_MODE_BOTH_EDGES      3U

void extStart(void);
void ext_channel_enable(uint16_t channel, uint16_t mode);
#endif

/*
 * Timers support (GPT system)
 */
#if HAL_USE_GPT == FALSE
// Run TIM2 as us timer counter
// Run TIM3 as ms timer counter
void initTimers(void);
void startTimer(TIM_TypeDef *timer, uint32_t period);
inline uint32_t getCounter(TIM_TypeDef *timer) {return timer->CNT;}
#endif
