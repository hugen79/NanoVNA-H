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

#define ADC_CFGR1_RES_12BIT     (0 << ADC_CFGR_RES_Pos)
#define ADC_CFGR1_RES_10BIT     (1 << ADC_CFGR_RES_Pos)
#define ADC_CFGR1_RES_8BIT      (2 << ADC_CFGR_RES_Pos)
#define ADC_CFGR1_RES_6BIT      (3 << ADC_CFGR_RES_Pos)

#define ADC_CFGR_DMACFG_SINGLE   (0 << ADC_CFGR_DMACFG_Pos)
#define ADC_CFGR_DMACFG_CIRCULAR (1 << ADC_CFGR_DMACFG_Pos)

#define ADC_SMPR_SMP_1P5        (0 << ADC_SMPR1_SMP0_Pos)
#define ADC_SMPR_SMP_2P5        (1 << ADC_SMPR1_SMP0_Pos)
#define ADC_SMPR_SMP_4P5        (2 << ADC_SMPR1_SMP0_Pos)
#define ADC_SMPR_SMP_7P5        (3 << ADC_SMPR1_SMP0_Pos)
#define ADC_SMPR_SMP_19P5       (4 << ADC_SMPR1_SMP0_Pos)
#define ADC_SMPR_SMP_61P5       (5 << ADC_SMPR1_SMP0_Pos)
#define ADC_SMPR_SMP_181P5      (6 << ADC_SMPR1_SMP0_Pos)
#define ADC_SMPR_SMP_601P5      (7 << ADC_SMPR1_SMP0_Pos)

#define ADC_TR(low, high)       (((uint32_t)(high) << ADC_TR1_HT1_Pos) | ((uint32_t)(low)<<ADC_TR1_LT1_Pos))

#define ADC_SMPR1_SMP_AN0(n)    ((n) << ADC_SMPR1_SMP0_Pos)
#define ADC_SMPR1_SMP_AN1(n)    ((n) << ADC_SMPR1_SMP1_Pos)
#define ADC_SMPR1_SMP_AN2(n)    ((n) << ADC_SMPR1_SMP2_Pos)
#define ADC_SMPR1_SMP_AN3(n)    ((n) << ADC_SMPR1_SMP3_Pos)
#define ADC_SMPR1_SMP_AN4(n)    ((n) << ADC_SMPR1_SMP4_Pos)
#define ADC_SMPR1_SMP_AN5(n)    ((n) << ADC_SMPR1_SMP5_Pos)
#define ADC_SMPR1_SMP_AN6(n)    ((n) << ADC_SMPR1_SMP6_Pos)
#define ADC_SMPR1_SMP_AN7(n)    ((n) << ADC_SMPR1_SMP7_Pos)
#define ADC_SMPR1_SMP_AN8(n)    ((n) << ADC_SMPR1_SMP8_Pos)
#define ADC_SMPR1_SMP_AN9(n)    ((n) << ADC_SMPR1_SMP9_Pos)

#define ADC_SMPR2_SMP_AN10(n)   ((n) << ADC_SMPR2_SMP10_Pos)
#define ADC_SMPR2_SMP_AN11(n)   ((n) << ADC_SMPR2_SMP11_Pos)
#define ADC_SMPR2_SMP_AN12(n)   ((n) << ADC_SMPR2_SMP12_Pos)
#define ADC_SMPR2_SMP_AN13(n)   ((n) << ADC_SMPR2_SMP13_Pos)
#define ADC_SMPR2_SMP_AN14(n)   ((n) << ADC_SMPR2_SMP14_Pos)
#define ADC_SMPR2_SMP_AN15(n)   ((n) << ADC_SMPR2_SMP15_Pos)
#define ADC_SMPR2_SMP_AN16(n)   ((n) << ADC_SMPR2_SMP16_Pos)
#define ADC_SMPR2_SMP_AN17(n)   ((n) << ADC_SMPR2_SMP17_Pos)
#define ADC_SMPR2_SMP_AN18(n)   ((n) << ADC_SMPR2_SMP18_Pos)

#define ADC_SQR1_NUM_CH(n)      (((n) - 1) << ADC_SQR1_L_Pos)

#define ADC_SQR1_SQ1_N(n)       ((n) << ADC_SQR1_SQ1_Pos)
#define ADC_SQR1_SQ2_N(n)       ((n) << ADC_SQR1_SQ2_Pos)
#define ADC_SQR1_SQ3_N(n)       ((n) << ADC_SQR1_SQ3_Pos)
#define ADC_SQR1_SQ4_N(n)       ((n) << ADC_SQR1_SQ4_Pos)

#define ADC_SQR2_SQ5_N(n)       ((n) << ADC_SQR2_SQ5_Pos)
#define ADC_SQR2_SQ6_N(n)       ((n) << ADC_SQR2_SQ6_Pos)
#define ADC_SQR2_SQ7_N(n)       ((n) << ADC_SQR2_SQ7_Pos)
#define ADC_SQR2_SQ8_N(n)       ((n) << ADC_SQR2_SQ8_Pos)
#define ADC_SQR2_SQ9_N(n)       ((n) << ADC_SQR2_SQ9_Pos)

#define ADC_SQR3_SQ10_N(n)      ((n) << ADC_SQR3_SQ10_Pos)
#define ADC_SQR3_SQ11_N(n)      ((n) << ADC_SQR3_SQ11_Pos)
#define ADC_SQR3_SQ12_N(n)      ((n) << ADC_SQR3_SQ12_Pos)
#define ADC_SQR3_SQ13_N(n)      ((n) << ADC_SQR3_SQ13_Pos)
#define ADC_SQR3_SQ14_N(n)      ((n) << ADC_SQR3_SQ14_Pos)

#define ADC_SQR4_SQ15_N(n)      ((n) << ADC_SQR4_SQ15_Pos)
#define ADC_SQR4_SQ16_N(n)      ((n) << ADC_SQR4_SQ16_Pos)

#define ADC_CCR_DUAL_FIELD(n)    ((n) << ADC_CCR_DUAL_Pos)
#define ADC_CCR_DELAY_FIELD(n)   ((n) << ADC12_CCR_DELAY_Pos)
#define ADC_CCR_DMACFG_ONESHOT   (0 << ADC12_CCR_DMACFG_Pos)
#define ADC_CCR_DMACFG_CIRCULAR  (1 << ADC12_CCR_DMACFG_Pos)
#define ADC_CCR_MDMA_DISABLED    (0 << ADC12_CCR_MDMA_Pos)
#define ADC_CCR_MDMA_WORD        (2 << ADC12_CCR_MDMA_Pos)
#define ADC_CCR_MDMA_HWORD       (3 << ADC12_CCR_MDMA_Pos)
#define ADC_CCR_CKMODE_ADCCK     (0 << ADC12_CCR_CKMODE_Pos)
#define ADC_CCR_CKMODE_AHB_DIV1  (1 << ADC12_CCR_CKMODE_Pos)
#define ADC_CCR_CKMODE_AHB_DIV2  (2 << ADC12_CCR_CKMODE_Pos)
#define ADC_CCR_CKMODE_AHB_DIV4  (3 << ADC12_CCR_CKMODE_Pos)

#define ADC_CCR_VREF_ENABLE      (1 << ADC_CCR_VREFEN_Pos)
#define ADC_CCR_TS_ENABLE        (1 << ADC_CCR_TSEN_Pos)
#define ADC_CCR_VBAT_ENABLE      (1 << ADC_CCR_VBATEN_Pos)

#define ADC_TOUCH_SMP_TIME       ADC_SMPR_SMP_601P5
#define ADC_VBAT_SMP_TIME        ADC_SMPR_SMP_601P5

#define BAT_ADC                  ADC1
#define TOUCH_ADC                ADC2

static void initADCDriver(ADC_TypeDef *adc) {
  adc->CR = 0;                            // Master ADC calibration.
  adc->CR = ADC_CR_ADVREGEN_0;
  adc->CR|= ADC_CR_ADCAL;
  while (adc->CR & ADC_CR_ADCAL);
  adc->CR|= ADC_CR_ADEN;                  // Master ADC enabled here in order to reduce conversions latencies.
  while ((adc->ISR & ADC_ISR_ADRD) == 0); // wait ready
  // Set measure timings for all channels
  adc->SMPR1 = ADC_SMPR1_SMP_AN3(ADC_TOUCH_SMP_TIME) | ADC_SMPR1_SMP_AN4(ADC_TOUCH_SMP_TIME);
  adc->SMPR2 = ADC_SMPR2_SMP_AN17(ADC_VBAT_SMP_TIME) | ADC_SMPR2_SMP_AN18(ADC_VBAT_SMP_TIME) | ADC_SMPR2_SMP_AN16(ADC_VBAT_SMP_TIME);
}

void adc_init(void)
{
  rccEnableADC12(FALSE);
  initADCDriver(ADC1);
  initADCDriver(ADC2);
  nvicEnableVector(STM32_ADC1_NUMBER, STM32_ADC_ADC12_IRQ_PRIORITY); // ADC 1 and 2 interrupt
  ADC1_2_COMMON->CCR = STM32_ADC_ADC12_CLOCK_MODE | ADC_CCR_VBAT_ENABLE | ADC_CCR_VREF_ENABLE;
//rccEnableADC34(FALSE);
//nvicEnableVector(STM32_ADC3_NUMBER, STM32_ADC_ADC3_IRQ_PRIORITY);
//ADC3_4_COMMON->CCR = STM32_ADC_ADC34_CLOCK_MODE;
}

#define ADC_AVERAGE_N 3
static void adcStartMeasure(ADC_TypeDef *adc, uint32_t sqr0, uint16_t *samples) {
  // ADC setup
  adc->ISR   = adc->ISR;            // reset interrupts
  adc->IER   = 0;                   // disable interrupts
  adc->TR1   = ADC_TR(0, 0);        // reset threshold
  adc->SQR1  = sqr0;                // set measure sequence
//adc->SQR2  = sqr1;
//adc->SQR3  = sqr2;
//adc->SQR4  = sqr3;
  adc->CFGR = ADC_CFGR1_RES_12BIT;  // ADC configuration.
  // Starting conversion.
  uint16_t count = sqr0 & 0xF;
  for (uint16_t i = 0; i <= count; i++) samples[i] = 0;
  int j = 1<<ADC_AVERAGE_N;
  do {
    adc->CR|= ADC_CR_ADSTART;
    for (uint16_t i = 0; i <= count; i++) {
      while((adc->ISR & ADC_ISR_EOC) == 0 && adc->CR & ADC_CR_ADSTART); // wait one sample ready
      samples[i]+= adc->DR;
    }
  } while (--j);
  for (uint16_t i = 0; i <= count; i++) samples[i]>>= ADC_AVERAGE_N;
}

uint16_t adc_single_read(uint32_t chsel)
{
  uint16_t samples[1];
  adcStartMeasure(TOUCH_ADC, ADC_SQR1_NUM_CH(1) | ADC_SQR1_SQ1_N(chsel), samples);
  return samples[0];
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
  const uint16_t VREFINT = 3300;
  const uint16_t VREFINT_CAL = (*((uint16_t*)0x1FFFF7BA));
//  const uint16_t V25 = 1750;// when V25=1.41V at ref 3.3V
//  const uint16_t Avg_Slope = 5; //when avg_slope=4.3mV/C at ref 3.3V
//  uint16_t temperature_cal1 = *((uint16_t*) ((uint32_t)0x1FFFF7B8U));
//                            /* Internal temperature sensor, address of parameter TS_CAL1: On STM32F3,
//                               temperature sensor ADC raw data acquired at temperature  25 DegC (tolerance: +-5 DegC),
//                               Vref+ = 3.3 V (tolerance: +-10 mV). */
//  uint16_t temperature_cal2 = *((uint16_t*) ((uint32_t)0x1FFFF7C2U));
//                            /* Internal temperature sensor, address of parameter TS_CAL2: On STM32F3,
//                               temperature sensor ADC raw data acquired at temperature 110 DegC (tolerance: +-5 DegC),
//                               Vref+ = 3.3 V (tolerance: +-10 mV). */
//  float avg_slope = ((float)(temperature_cal1 - temperature_cal2))/(110-25);
//  float ts;
  #define N_CH_VBAT    2  // 17 and 18 channels
  uint16_t samplesVBAT[N_CH_VBAT];
  adcStartMeasure(BAT_ADC, ADC_SQR1_NUM_CH(N_CH_VBAT) | ADC_SQR1_SQ1_N(17) | ADC_SQR1_SQ2_N(18)/*| ADC_SQR1_SQ3_N(16)*/, samplesVBAT);
  uint32_t vbat    = samplesVBAT[0];
  uint32_t vrefint = samplesVBAT[1];
//  ts = samplesVBAT[2];
//  uint16_t vts = (VREFINT * VREFINT_CAL * ts / (vrefint * ((1<<12)-1)));
//  uint16_t TemperatureC2 = (uint16_t)((V25-ts)/Avg_Slope+25);
//  uint16_t TemperatureC = (uint16_t)((V25-ts)/avg_slope+25);

  // vbat_raw = (3300 * 2 * vbat / 4095) * (VREFINT_CAL / vrefint)
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
  ADC_TypeDef *adc = TOUCH_ADC;                                 // ADC setup
  adc->ISR  = adc->ISR;                                         // reset interrupts
  adc->IER = /*ADC_ISR_OVR |*/ ADC_IER_AWD1;                    // enable watchdog interrupt (ignore overflow)
  adc->TR1  = ADC_TR(0, TOUCH_THRESHOLD);                       // Threshold
  adc->SQR1 = ADC_SQR1_NUM_CH(1) | ADC_SQR1_SQ1_N(ADC_TOUCH_Y); // Set channel, and count
  adc->CFGR = ADC_CFGR1_RES_12BIT                               // 12 bit mode
          | ADC_CFGR_EXTEN_0                                    // rising edge of external trigger
          | ADC_CFGR_EXTSEL_2                                   // EXT4 0x1000 event (TIM3_TRGO)
          | ADC_CFGR_AWD1EN                                     // Enable Analog watchdog check interrupt
          | ADC_CFGR_OVRMOD                                     // Overrun Mode
          ;
  adc->CR|= ADC_CR_ADSTART;                                     // Starting watchdog
}

void adc_stop_analog_watchdog(void)
{
  ADC_TypeDef *adc = TOUCH_ADC;  // ADC setup
  if (adc->CR & ADC_CR_ADSTART) {
    adc->CR|= ADC_CR_ADSTP;
    while (adc->CR & ADC_CR_ADSTP);
  }
}

static inline void adc_interrupt(void)
{
  ADC_TypeDef *adc = TOUCH_ADC;
  uint32_t isr = adc->ISR;
  adc->ISR = isr;
//  if (isr & ADC_ISR_OVR) { //   ADC overflow condition
//  }
  if (isr & ADC_ISR_AWD1) {  // Analog watchdog error.
    handle_touch_interrupt();
  }
}

OSAL_IRQ_HANDLER(STM32_ADC2_HANDLER)
{
  OSAL_IRQ_PROLOGUE();

  adc_interrupt();

  OSAL_IRQ_EPILOGUE();
}
