/*
 * Copyright (c) 2019-2021, Dmitry (DiSlord) dislordlive@gmail.com
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

#define ADC_TR(low, high)               (((uint32_t)(high) << 16U) |        \
                                         (uint32_t)(low))
#define ADC_SMPR_SMP_1P5        0U  /**< @brief 14 cycles conversion time   */
#define ADC_SMPR_SMP_239P5      7U  /**< @brief 252 cycles conversion time. */ 
#define ADC_CFGR1_RES_12BIT             (0U << 3U)

// External Event Select for regular group
#define ADC_TIM1_TRGO  0                                       // 0b000
#define ADC_TIM1_CC4   (ADC_CFGR1_EXTSEL_0)                    // 0b001
#define ADC_TIM2_TRGO  (ADC_CFGR1_EXTSEL_1)                    // 0b010
#define ADC_TIM3_TRGO  (ADC_CFGR1_EXTSEL_1|ADC_CFGR1_EXTSEL_0) // 0b011
#define ADC_TIM15_TRGO (ADC_CFGR1_EXTSEL_2)                    // 0b100

#define VNA_ADC     ADC1

void adc_init(void)
{
  rccEnableADC1(FALSE);
  nvicEnableVector(ADC1_COMP_IRQn, STM32_EXT_EXTI21_22_IRQ_PRIORITY); // Shared vs GPIO handler
  VNA_ADC->CFGR1 = 0;
  /* Ensure flag states */
  VNA_ADC->ISR = VNA_ADC->CR;                  // clear ISR
  VNA_ADC->IER = 0;

  /* Calibration procedure.*/
  if (VNA_ADC->CR & ADC_CR_ADEN) {
    VNA_ADC->CR |= ~ADC_CR_ADDIS;      // Disable ADC
    while (VNA_ADC->CR & ADC_CR_ADEN); // Wait completion
  }

  VNA_ADC->CR |= ADC_CR_ADCAL;
  while (VNA_ADC->CR & ADC_CR_ADCAL);
  VNA_ADC->CR |= ADC_CR_ADEN;                  // Enable ADC
  while (!(VNA_ADC->ISR & ADC_ISR_ADRDY));
  // VBATEN enables resiter devider circuit. It consume vbat power.
  ADC->CCR = ADC_CCR_VREFEN | ADC_CCR_VBATEN;
}

#define ADC_AVERAGE_N 3
uint16_t adc_single_read(uint32_t chsel)
{
  // ADC setup
  VNA_ADC->ISR    = VNA_ADC->ISR;
  VNA_ADC->IER    = 0;
  VNA_ADC->TR     = ADC_TR(0, 0);
  VNA_ADC->SMPR   = ADC_SMPR_SMP_239P5;
  VNA_ADC->CFGR1  = ADC_CFGR1_RES_12BIT;
  VNA_ADC->CHSELR = chsel;
  uint16_t count = 1<<ADC_AVERAGE_N;
  uint16_t sample = 0;
  do {
    VNA_ADC->CR |= ADC_CR_ADSTART; // ADC conversion start
    while (VNA_ADC->CR & ADC_CR_ADSTART);
    sample+=VNA_ADC->DR;
  } while(--count);
  return sample>>ADC_AVERAGE_N;
}

int16_t adc_vbat_read(void)
{
  static int16_t   vbat_raw = 0;
#ifdef VBAT_MEASURE_INTERVAL
  static systime_t vbat_time = -VBAT_MEASURE_INTERVAL-1;
  systime_t _time = chVTGetSystemTimeX();
  if (_time - vbat_time < VBAT_MEASURE_INTERVAL)
    goto return_cached;
  vbat_time = _time;
#endif
// 13.9 Temperature sensor and internal reference voltage
// VREFINT_CAL calibrated on 3.3V, need get value in mV
  const uint16_t VREFINT = 3300;
  const uint16_t VREFINT_CAL = (*((uint16_t*)0x1FFFF7BA));
  uint8_t restart_touch = 0;
  if (VNA_ADC->CR & ADC_CR_ADSTART){
    adc_stop_analog_watchdog();
    restart_touch = 1;
  }
  uint32_t vrefint = adc_single_read(ADC_CHSELR_CHSEL17); // VREFINT == ADC_IN17
  uint32_t vbat    = adc_single_read(ADC_CHSELR_CHSEL18); // VBAT == ADC_IN18

  if (restart_touch)
    adc_start_analog_watchdog();

  // vbat_raw = (VREFINT * 2 * vbat / 4095) * (VREFINT_CAL / vrefint)
  // uint16_t vbat_raw = (VREFINT * VREFINT_CAL * (float)vbat * 2 / (vrefint * ((1<<12)-1)));
  // For speed divide not on 4095, divide on 4096, get little error, but no matter
  vbat_raw = ((VREFINT * 2 * vbat)>>12) * VREFINT_CAL / vrefint;
return_cached:
  if (vbat_raw < 100) {
    // maybe D2 is not installed
    return -1;
  }
  return vbat_raw + config._vbat_offset;
}

void adc_start_analog_watchdog(void)
{
  // ADC setup, if it is defined a callback for the analog watch dog then it is enabled.
  VNA_ADC->ISR    = VNA_ADC->ISR;
  VNA_ADC->IER    = ADC_IER_AWDIE;
  VNA_ADC->TR     = ADC_TR(0, TOUCH_THRESHOLD);
  VNA_ADC->SMPR   = ADC_SMPR_SMP_1P5;
  VNA_ADC->CHSELR = ADC_TOUCH_Y;

  /* ADC configuration and start.*/
  VNA_ADC->CFGR1  = ADC_CFGR1_RES_12BIT | ADC_CFGR1_AWDEN
                  | ADC_CFGR1_EXTEN_0 // rising edge of external trigger
                  | ADC_TIM3_TRGO; // External trigger is timer TIM3
  /* ADC conversion start.*/
  VNA_ADC->CR |= ADC_CR_ADSTART;
}

void adc_stop_analog_watchdog(void)
{
  if (VNA_ADC->CR & ADC_CR_ADSTART) {
    VNA_ADC->CR |= ADC_CR_ADSTP;
    while (VNA_ADC->CR & ADC_CR_ADSTP);
  }
}

static void adc_interrupt(void)
{
  uint32_t isr = VNA_ADC->ISR;
  VNA_ADC->ISR = isr;
//  if (isr & ADC_ISR_OVR) {  // ADC overflow condition
//  }
  if (isr & ADC_ISR_AWD) {    // Analog watchdog error.
    handle_touch_interrupt();
  }
}

OSAL_IRQ_HANDLER(STM32_ADC1_HANDLER)
{
  OSAL_IRQ_PROLOGUE();

  adc_interrupt();

  OSAL_IRQ_EPILOGUE();
}
