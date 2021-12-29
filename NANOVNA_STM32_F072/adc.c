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

  /* Ensure flag states */
  VNA_ADC->IER = 0;

  /* Calibration procedure.*/
  ADC->CCR = 0;
  if (VNA_ADC->CR & ADC_CR_ADEN) {
    VNA_ADC->CR |= ~ADC_CR_ADDIS; /* Disable ADC */
  }
  while (VNA_ADC->CR & ADC_CR_ADEN)
    ;
  VNA_ADC->CFGR1 &= ~ADC_CFGR1_DMAEN;
  VNA_ADC->CR |= ADC_CR_ADCAL;
  while (VNA_ADC->CR & ADC_CR_ADCAL)
    ;

  if (VNA_ADC->ISR & ADC_ISR_ADRDY) {
    VNA_ADC->ISR |= ADC_ISR_ADRDY; /* clear ADRDY */
  }
  /* Enable ADC */
  VNA_ADC->CR |= ADC_CR_ADEN;
  while (!(VNA_ADC->ISR & ADC_ISR_ADRDY))
    ;
}

uint16_t adc_single_read(uint32_t chsel)
{
  /* ADC setup */
  VNA_ADC->ISR    = VNA_ADC->ISR;
  VNA_ADC->IER    = 0;
  VNA_ADC->TR     = ADC_TR(0, 0);
  VNA_ADC->SMPR   = ADC_SMPR_SMP_239P5;
  VNA_ADC->CFGR1  = ADC_CFGR1_RES_12BIT;
  VNA_ADC->CHSELR = chsel;

  VNA_ADC->CR |= ADC_CR_ADSTART; // ADC conversion start
  while (VNA_ADC->CR & ADC_CR_ADSTART)
    ;

  return VNA_ADC->DR;
}

int16_t adc_vbat_read(void)
{
// Vbat measure averange count = 2^VBAT_AVERAGE
#define VBAT_AVERAGE 4
  static int16_t   vbat_raw = 0;
  static systime_t vbat_time = -VBAT_MEASURE_INTERVAL-1;
  systime_t _time = chVTGetSystemTimeX();
  if (_time - vbat_time < VBAT_MEASURE_INTERVAL)
    goto return_cached;
  vbat_time = _time;
// 13.9 Temperature sensor and internal reference voltage
// VREFINT_CAL calibrated on 3.3V, need get value in mV
#define ADC_FULL_SCALE 3300
#define VREFINT_CAL (*((uint16_t*)0x1FFFF7BA))
  uint32_t vrefint = 0;
  uint32_t vbat = 0;

  uint8_t restart_touch = 0;
  if (VNA_ADC->CR & ADC_CR_ADSTART){
    adc_stop_analog_watchdog();
    restart_touch = 1;
  }
  ADC->CCR |= ADC_CCR_VREFEN | ADC_CCR_VBATEN;
  for (uint16_t i = 0; i < 1<<VBAT_AVERAGE; i++){
    // VREFINT == ADC_IN17
    vrefint+= adc_single_read(ADC_CHSELR_CHSEL17);
    // VBAT == ADC_IN18
    // VBATEN enables resiter devider circuit. It consume vbat power.
    vbat+= adc_single_read(ADC_CHSELR_CHSEL18);
  }
  vbat>>=VBAT_AVERAGE;
  vrefint>>=VBAT_AVERAGE;
  ADC->CCR &= ~(ADC_CCR_VREFEN | ADC_CCR_VBATEN);

  if (restart_touch)
    adc_start_analog_watchdog();

  // vbat_raw = (3300 * 2 * vbat / 4095) * (VREFINT_CAL / vrefint)
  // uint16_t vbat_raw = (ADC_FULL_SCALE * VREFINT_CAL * (float)vbat * 2 / (vrefint * ((1<<12)-1)));
  // For speed divide not on 4095, divide on 4096, get little error, but no matter
  vbat_raw = ((ADC_FULL_SCALE * 2 * vbat)>>12) * VREFINT_CAL / vrefint;
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
  if (VNA_ADC->CR & ADC_CR_ADEN) {
    if (VNA_ADC->CR & ADC_CR_ADSTART) {
      VNA_ADC->CR |= ADC_CR_ADSTP;
      while (VNA_ADC->CR & ADC_CR_ADSTP)
        ;
    }

    /*    VNA_ADC->CR |= ADC_CR_ADDIS;
    while (VNA_ADC->CR & ADC_CR_ADDIS)
    ;*/
  }
}

static void adc_interrupt(void)
{
  uint32_t isr = VNA_ADC->ISR;
  VNA_ADC->ISR = isr;

  if (isr & ADC_ISR_OVR) {
    /* ADC overflow condition, this could happen only if the DMA is unable
       to read data fast enough.*/
    
  }
  if (isr & ADC_ISR_AWD) {
    /* Analog watchdog error.*/
    handle_touch_interrupt();
  }
}

OSAL_IRQ_HANDLER(STM32_ADC1_HANDLER)
{
  OSAL_IRQ_PROLOGUE();

  adc_interrupt();

  OSAL_IRQ_EPILOGUE();
}