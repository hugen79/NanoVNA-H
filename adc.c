/*
 * Copyright (c) 2014-2015, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
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
#include <stdio.h> 
#include <string.h> 

#ifdef NANOVNA_F303
#define ADC_SMPR_SMP_239P5      7U
#define ADC_CFGR_RES_12BIT             (0 << 3)
ADCConversionGroup adcgrpcfg;
static adcsample_t samples[2];
msg_t adcConvert(ADCDriver *adcp,
                   const ADCConversionGroup *grpp,
                   adcsample_t *samples,
                   size_t depth);
#define  ADC_CR1_AWDEN                       ((uint32_t)0x00800000)  /*!< Analog watchdog enable on regular channels */
ADC_Common_TypeDef        *adcc;
#endif

#if USE_CHIBIOS_ADC_API
//static void adccallback(ADCDriver *adcp, adcsample_t *buffer, size_t n);
static void adcerrorcallback(ADCDriver *adcp, adcerror_t err);

// Read XY location
/*
 * ADC conversion group.
 * Mode:        Linear buffer, 4 samples of 2 channels, SW triggered.
 * Channels:    IN11   (48 cycles sample time)
 *              Sensor (192 cycles sample time)
 */
/* Total number of channels to be sampled by a single ADC operation.*/
/* Depth of the conversion buffer, channels are sampled four times each.*/
#define ADC_GRP_NUM_CHANNELS_XY   2
#define ADC_GRP_BUF_DEPTH_XY      4
static adcsample_t samplesXY[ADC_GRP_NUM_CHANNELS_XY * ADC_GRP_BUF_DEPTH_XY];
static const ADCConversionGroup adcgrpcfgXY = {
  FALSE,                 // Enable linear buffer mode
  ADC_GRP_NUM_CHANNELS_XY, // Number of analog channels belonging to the conversion group
  NULL,                 // ADC Complete Callback function
  NULL,      // ADC Error Callback function, NULL is none
  /* HW dependent part.*/
  // ADC CFGR register initialization data.
#ifdef NANOVNA_F303
  ADC_CFGR_CONT,
  // ADC TR1 register initialization data.
  0,       
  // ADC SMPR[2] registers initialization data.
  {ADC_SMPR_SMP_247P5,
  0},
  //  ADC SQR[4] register initialization data.
  {ADC_SQR1_SQ1_N(ADC_CHANNEL_IN6) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN7),
  0,
  0,
  0}
#else
  ADC_CFGR1_CONT | ADC_CFGR1_RES_12BIT,             /* CFGR1 */
  ADC_TR(0, 0),                                     /* TR */
  ADC_SMPR_SMP_239P5,                               /* SMPR */
  ADC_CHSELR_CHSEL6 |  ADC_CHSELR_CHSEL7,           /* CHSELR */
#endif
};

// Touch Standby
#define ADC_GRP_NUM_CHANNELS_TOUCH   2
#define ADC_GRP_BUF_DEPTH_TOUCH      4
static adcsample_t samplesTouch[ADC_GRP_NUM_CHANNELS_TOUCH * ADC_GRP_BUF_DEPTH_TOUCH];
static const ADCConversionGroup adcgrpcfgTouch = {
  FALSE,                 // Enable linear buffer mode
  ADC_GRP_NUM_CHANNELS_TOUCH, // Number of analog channels belonging to the conversion group
  NULL,                 // ADC Complete Callback function
  adcerrorcallback,      // ADC Error Callback function, NULL is none
  /* HW dependent part.*/
  // ADC CFGR register initialization data.
  #ifdef NANOVNA_F303
  ADC_CFGR_CONT  | ADC_CFGR_AWD1EN | ADC_CFGR_AWD1CH_0 
    | ADC_CFGR_EXTEN_0 // rising edge of external trigger
    | ADC_CFGR_EXTSEL_0 | ADC_CFGR_EXTSEL_1,
  // ADC TR1 register initialization data.
  ADC_TR(0, TOUCH_THRESHOLD),       
  // ADC SMPR[2] registers initialization data.
  {ADC_SMPR_SMP_247P5,
  0},
  //  ADC SQR[4] register initialization data.
  {ADC_SQR1_SQ1_N(ADC_CHANNEL_IN6) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN7),
  0,
  0,
  0}
  #else
  ADC_CFGR1_CONT | ADC_CFGR1_RES_12BIT,             /* CFGR1 */
  ADC_TR(0, TOUCH_THRESHOLD),                       /* TR */
  ADC_SMPR_SMP_239P5,                                 /* SMPR */
  ADC_CHSELR_CHSEL10                                /* CHSELR */
  #endif
};

#define ADC_GRP_NUM_CHANNELS_BAT   2
#define ADC_GRP_BUF_DEPTH_BAT      8
static adcsample_t samplesBat[ADC_GRP_NUM_CHANNELS_BAT * ADC_GRP_BUF_DEPTH_BAT];
static const ADCConversionGroup adcgrpcfgBat = {
  FALSE,
  ADC_GRP_NUM_CHANNELS_BAT,
  NULL,
  NULL,
  #ifdef NANOVNA_F303
  #else
  ADC_CFGR1_CONT | ADC_CFGR1_RES_12BIT,             /* CFGR1 */
  ADC_TR(0, 0),                                     /* TR */
  ADC_SMPR_SMP_28P5,                                /* SMPR */
  ADC_CHSELR_CHSEL17 | ADC_CHSELR_CHSEL18           /* CHSELR */
  #endif
};

#endif

void adc_init(void)
{
#if USE_CHIBIOS_ADC_API
  adcInit();
#else
 #ifdef NANOVNA_F303
  rccEnableADC12(FALSE);

  /* Ensure flag states */
  ADCx->IER = 0;

  /* Calibration procedure.*/
  if (ADCx->CR & ADC_CR_ADEN) {
      ADCx->CR |= ~ADC_CR_ADDIS; /* Disable ADC */
  }
  while (ADCx->CR & ADC_CR_ADEN)
    ;
  ADCx->CFGR &= ~ADC_CFGR_DMAEN;
  ADCx->CR |= ADC_CR_ADCAL;
  while (ADCx->CR & ADC_CR_ADCAL)
    ;

  if (ADCx->ISR & ADC_ISR_ADRD) {
      ADCx->ISR |= ADC_ISR_ADRD; /* clear ADRDY */
  }
  /* Enable ADC */
  ADCx->CR |= ADC_CR_ADEN;
  while (!(ADCx->ISR & ADC_ISR_ADRD))
    ;
 #else
  rccEnableADC1(FALSE);

  /* Ensure flag states */
  ADCx->IER = 0;

  /* Calibration procedure.*/
  ADC->CCR = 0;
  if (ADCx->CR & ADC_CR_ADEN) {
      ADCx->CR |= ~ADC_CR_ADDIS; /* Disable ADC */
  }
  while (ADCx->CR & ADC_CR_ADEN)
    ;
  ADCx->CFGR1 &= ~ADC_CFGR1_DMAEN;
  ADCx->CR |= ADC_CR_ADCAL;
  while (ADCx->CR & ADC_CR_ADCAL)
    ;

  if (ADCx->ISR & ADC_ISR_ADRDY) {
      ADCx->ISR |= ADC_ISR_ADRDY; /* clear ADRDY */
  }
  /* Enable ADC */
  ADCx->CR |= ADC_CR_ADEN;
  while (!(ADCx->ISR & ADC_ISR_ADRDY))
    ;
 #endif
#endif
    
}


uint16_t adc_single_read(ADC_TypeDef *adc, uint32_t chsel)
{
  /* ADC setup */
#ifdef NANOVNA_F303
  adcStart(&ADCD2, NULL);
  memset(&adcgrpcfg, 0, sizeof(adcgrpcfg));
  adcgrpcfg.circular = false;
  adcgrpcfg.num_channels = 1;
  adcgrpcfg.end_cb = (adccallback_t) adc->ISR;
  adcgrpcfg.sqr[0] = ADC_SQR1_SQ1_N(chsel);
  //adcgrpcfg.cr2 = ADC_CR2_SWSTART;
  adcgrpcfg.cfgr = ADC_CFGR_RES_12BIT;
  adcgrpcfg.tr1 = ADC_TR(0, 0);
  adcConvert(&ADCD2, &adcgrpcfg, samples, 1);
  chThdSleepMilliseconds(1000);
  return(samples[0]);
#else
  adc->ISR    = adc->ISR;
  adc->IER    = 0;
  adc->TR     = ADC_TR(0, 0);
  adc->SMPR   = ADC_SMPR_SMP_239P5;
  adc->CFGR1  = ADC_CFGR1_RES_12BIT;
  adc->CHSELR = chsel;
  /* ADC conversion start.*/
  adc->CR |= ADC_CR_ADSTART;
  while (adc->CR & ADC_CR_ADSTART)
    ;
  return adc->DR;
#endif  
}

#define ADC_FULL_SCALE 3300
#define VBAT_DIODE_VF 500
#define VREFINT_CAL (*((uint16_t*)0x1FFFF7BA))
int16_t adc_vbat_read(ADC_TypeDef *adc)
{
	float vbat = 0;
	float vrefint = 0;
  #ifdef NANOVNA_F303
	adcStart(&ADCD1, NULL);
	memset(&adcgrpcfg, 0, sizeof(adcgrpcfg));
	adcgrpcfg.circular = false;
	adcgrpcfg.num_channels = 1;
	adcgrpcfg.end_cb = (adccallback_t) adc->ISR;
	adcgrpcfg.sqr[0] =0;
	//adcgrpcfg.cr2 = ADC_CR2_SWSTART;
	adcgrpcfg.cfgr = ADC_CFGR_RES_12BIT;
	adcgrpcfg.tr1 = ADC_TR(0, 0);
	adcSTM32EnableTS(&ADCD1);
	adcSTM32EnableVBAT(&ADCD1);
        adcConvert(&ADCD1, &adcgrpcfg, samples, 1);
        chThdSleepMilliseconds(1000);
	vbat = samples[0];
  #else
	ADC->CCR |= ADC_CCR_VREFEN | ADC_CCR_VBATEN;
	// VREFINT == ADC_IN17
	vrefint = adc_single_read(adc, ADC_CHSELR_CHSEL17);
	// VBAT == ADC_IN18
	// VBATEN enables resiter devider circuit. It consume vbat power.
	vbat = adc_single_read(adc, ADC_CHSELR_CHSEL18);
	ADC->CCR &= ~(ADC_CCR_VREFEN | ADC_CCR_VBATEN);
  #endif	
	uint16_t vbat_raw = (ADC_FULL_SCALE * VREFINT_CAL * vbat * 2 / (vrefint * ((1<<12)-1)));
	if (vbat_raw < 100) {
		// maybe D2 is not installed
		return -1;
	}
	
	return vbat_raw + VBAT_DIODE_VF;

}


void adc_start_analog_watchdogd(ADC_TypeDef *adc, uint32_t chsel)
{
  uint32_t cfgr1;

#ifdef NANOVNA_F303
  cfgr1 = ADC_CFGR1_RES_12BIT | ADC_CR1_AWDEN
    | ADC_CFGR_EXTEN_0 // rising edge of external trigger
    | ADC_CFGR_EXTSEL_0 | ADC_CFGR_EXTSEL_1; // TRG3  , /* CFGR1 */
  adcStart(&ADCD2, NULL);
  memset(&adcgrpcfg, 0, sizeof(adcgrpcfg));
  adcgrpcfg.circular = false;
  adcgrpcfg.num_channels = 1;
  adcgrpcfg.end_cb = (adccallback_t) adc->ISR;
  adcgrpcfg.sqr[0] = ADC_SQR1_SQ1_N(chsel);
  //adcgrpcfg.cr2 = ADC_CR2_SWSTART;
  adcgrpcfg.cfgr = cfgr1;
  adcgrpcfg.tr1 = ADC_TR(0, 0);
  adcConvert(&ADCD2, &adcgrpcfg, samples, 1);
  
#else
  cfgr1 = ADC_CFGR1_RES_12BIT | ADC_CFGR1_AWDEN
    | ADC_CFGR1_EXTEN_0 // rising edge of external trigger
    | ADC_CFGR1_EXTSEL_0 | ADC_CFGR1_EXTSEL_1; // TRG3  , /* CFGR1 */

  /* ADC setup, if it is defined a callback for the analog watch dog then it
     is enabled.*/
  adc->ISR    = adc->ISR;
  adc->IER    = ADC_IER_AWDIE;
  adc->TR     = ADC_TR(0, TOUCH_THRESHOLD);
  adc->SMPR   = ADC_SMPR_SMP_1P5;
  adc->CHSELR = chsel;

  /* ADC configuration and start.*/
  adc->CFGR1  = cfgr1;

  /* ADC conversion start.*/
  adc->CR |= ADC_CR_ADSTART;
#endif
}


void adc_stop(ADC_TypeDef *adc)
{
#if USE_CHIBIOS_ADC_API
  adcStop(&ADCD2);
#else
  if (adc->CR & ADC_CR_ADEN) {
    if (adc->CR & ADC_CR_ADSTART) {
      adc->CR |= ADC_CR_ADSTP;
      while (adc->CR & ADC_CR_ADSTP)
        ;
    }

    /*    adc->CR |= ADC_CR_ADDIS;
    while (adc->CR & ADC_CR_ADDIS)
    ;*/
  }
#endif
}

#if 0
static void adcerrorcallback (ADCDriver *adcp, adcerror_t err)
{
  #ifdef NANOVNA_F303
  if (err == ADC_ERR_AWD1) {
  #else
  if (err == ADC_ERR_AWD) {
  #endif
    handle_touch_interrupt();
  }
}
#endif

void adc_interrupt(ADC_TypeDef *adc)
{
  uint32_t isr = adc->ISR;
 #ifdef NANOVNA_F303
 #else
  adc->ISR = isr;

  if (isr & ADC_ISR_OVR) {
    /* ADC overflow condition, this could happen only if the DMA is unable
       to read data fast enough.*/
    
  }
  if (isr & ADC_ISR_AWD) {
    /* Analog watchdog error.*/
    handle_touch_interrupt();
  }
 #endif
}

 #ifdef NANOVNA_F303
 #else
OSAL_IRQ_HANDLER(STM32_ADCx_HANDLER)
{
  OSAL_IRQ_PROLOGUE();

  adc_interrupt(ADCx);

  OSAL_IRQ_EPILOGUE();
}
 #endif
