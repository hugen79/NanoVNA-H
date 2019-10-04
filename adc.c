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
#include "adc_F303.h"
static adcsample_t samples[2];
static adcsample_t samplesVBAT[2];
void increment_touch_cnt();

// Read XY location
/*
 * ADC conversion group.
 * Mode:        Linear buffer, 1 samples of 1 channels, SW triggered.
 */
/* Total number of channels to be sampled by a single ADC operation.*/
/* Depth of the conversion buffer, channels are sampled four times each.*/
#define ADC_GRP_NUM_CHANNELS_XY   1
#define ADC_GRP_BUF_DEPTH_XY      1
static ADCConversionGroup adcgrpcfgXY = {
  FALSE,                 // Enable linear buffer mode
  ADC_GRP_NUM_CHANNELS_XY, // Number of analog channels belonging to the conversion group
  NULL,                 // ADC Complete Callback function
  NULL,      // ADC Error Callback function, NULL is none
  /* HW dependent part.*/
  // ADC CFGR register initialization data.
  ADC_CFGR_CONT,
  // ADC TR1 register initialization data.
  0,       
  // ADC SMPR[2] registers initialization data.
  {ADC_SMPR_SMP_247P5,
  0},
  //  ADC SQR[4] register initialization data.
  {ADC_SQR1_SQ1_N(ADC_CHANNEL_IN3) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN4),
  0,
  0,
  0}
};

#define ADC_GRP_NUM_CHANNELS_VBAT   2
#define ADC_GRP_BUF_DEPTH_VBAT      1
static const ADCConversionGroup adcgrpcfgVBAT = {
  FALSE,
  ADC_GRP_NUM_CHANNELS_VBAT,
  NULL,
  NULL,
  //  ADC_CFGR_CONT | ADC_CFGR1_RES_12BIT,             /* CFGR1 */
  ADC_CFGR_CONT ,             /* CFGR1 */
  ADC_TR(0, 4095),                                     /* TR */
  {0,0},                                /* SMPR */
  {ADC_SQR1_SQ1_N(ADC_CHANNEL_IN17) | ADC_SQR1_SQ2_N(ADC_CHANNEL_IN18),
   0,0,0}           /* CHSELR */
};

// Touch Standby
#define ADC_GRP_NUM_CHANNELS_TOUCH   1
#define ADC_GRP_BUF_DEPTH_TOUCH      1
static ADCConversionGroup adcgrpcfgTouch = {
  FALSE,                 // Enable linear buffer mode
  ADC_GRP_NUM_CHANNELS_TOUCH, // Number of analog channels belonging to the conversion group
  NULL,                 // ADC Complete Callback function
  NULL,
  //adcerrorcallback,      // ADC Error Callback function, NULL is none
  /* HW dependent part.*/
  // ADC CFGR register initialization data.
  ADC_CFGR_RES_12BIT
  //| ADC_CFGR_AWD1EN
  | ADC_CFGR_EXTEN_0     // rising edge of external trigger
  // | ADC_CFGR_AWD1SGL | ADC_CFGR_AWD1CH_2 // single channel, #4
  | ADC_CFGR_EXTSEL_2 , // TIM3_TRGO
  // ADC TR1 register initialization data.
  ADC_TR(0, TOUCH_THRESHOLD),       
  // ADC SMPR[2] registers initialization data.
  {7,
  0},
  //  ADC SQR[4] register initialization data.
  {ADC_CHSELR_CHSEL7,
  0,
  0,
  0}
};

#endif


void adc_init(void)
{
#ifdef NANOVNA_F303
  //adcInit(); /* This is part of Init sequence already */
  //rccEnableADC12(FALSE);
  adcStop(&ADCD1);  
  adcStop(&ADCD2);  
  //adcStart(&ADCD1, NULL);
  //adcStart(&ADCD2, NULL);
#else
  rccEnableADC1(FALSE);

  /* Ensure flag states */
  ADC1->IER = 0;

  /* Calibration procedure.*/
  ADC->CCR = 0;
  if (ADC1->CR & ADC_CR_ADEN) {
      ADC1->CR |= ~ADC_CR_ADDIS; /* Disable ADC */
  }
  while (ADC1->CR & ADC_CR_ADEN)
    ;
  ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;
  ADC1->CR |= ADC_CR_ADCAL;
  while (ADC1->CR & ADC_CR_ADCAL)
    ;

  if (ADC1->ISR & ADC_ISR_ADRDY) {
      ADC1->ISR |= ADC_ISR_ADRDY; /* clear ADRDY */
  }
  /* Enable ADC */
  ADC1->CR |= ADC_CR_ADEN;
  while (!(ADC1->ISR & ADC_ISR_ADRDY))
    ;
#endif
    
}


uint16_t adc_single_read(ADC_TypeDef *adc, uint32_t chsel)
{
  /* ADC setup */
#ifdef NANOVNA_F303
 #if 1
  adcStop(&ADCD2);  
  adcStart(&ADCD2,NULL);
  adcgrpcfgXY.sqr[0] = chsel;
  adcConvert(&ADCD2, &adcgrpcfgXY, samples, 1);
  adcStop(&ADCD2);  
  chThdSleepMilliseconds(20);
  //adcStopConversion(&ADCD2);  
  //chThdSleepMilliseconds(1000);
  return(samples[0]);
 #else
  ADCDriver *adcp;
  adcp = &ADCD2;
  
  adcp->adcm->ISR    = adcp->adcm->ISR;
  adcp->adcm->IER    = 0;
  adcp->adcm->TR1     = ADC_TR(0, 4095);
  adcp->adcm->SMPR1   = ADC_SMPR_SMP_239P5;
  adcp->adcm->SMPR2   = 0;
  adcp->adcm->CFGR  = ADC_CFGR1_RES_12BIT;
  adcp->adcm->SQR1 = ADC_SQR1_SQ1_N(chsel) | ADC_SQR1_NUM_CH(1);
  adcp->adcm->SQR2    = 0;
  adcp->adcm->SQR3    = 0;
  adcp->adcm->SQR3    = 0;
  /* ADC conversion start.*/
  adcp->adcm->CR |= ADC_CR_ADSTART;
  while (adcp->adcm->CR & ADC_CR_ADSTART)
    ;
  return adcp->adcm->DR;
  
 #endif
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

int16_t adc_vbat_read(ADC_TypeDef *adc)
{
#define ADC_FULL_SCALE 3300
#define VBAT_DIODE_VF 500
#define VREFINT_CAL (*((uint16_t*)0x1FFFF7BA))
	float vbat = 0;
	float vrefint = 0;
  #ifdef NANOVNA_F303
        adcStart(&ADCD1, NULL);
        adcSTM32EnableVBAT(&ADCD1);
        adcSTM32EnableVREF(&ADCD1);
        chThdSleepMilliseconds(3000);
        adcConvert(&ADCD1, &adcgrpcfgVBAT, samplesVBAT, 1);
        adcStop(&ADCD1);
        adcSTM32DisableVBAT(&ADCD1);
        adcSTM32DisableVREF(&ADCD1);
        vbat = samplesVBAT[0];
        vrefint = samplesVBAT[1];
	uint16_t vbat_raw;
        vbat_raw = 1230 * vbat * 2 / vrefint;  // banggap Vref = 1.23
	//vbat_raw = (ADC_FULL_SCALE * VREFINT_CAL * vbat * 2 / (vrefint * ((1<<12)-1)));
	//vbat_raw = vbat;
	return vbat_raw + VBAT_DIODE_VF;
#else
	ADC->CCR |= ADC_CCR_VREFEN | ADC_CCR_VBATEN;
	// VREFINT == ADC_IN17
	vrefint = adc_single_read(adc, ADC_CHSELR_VREFINT);
	// VBAT == ADC_IN18
	// VBATEN enables resiter devider circuit. It consume vbat power.
	vbat = adc_single_read(adc, ADC_CHSELR_VBAT);
	ADC->CCR &= ~(ADC_CCR_VREFEN | ADC_CCR_VBATEN);

	uint16_t vbat_raw = (ADC_FULL_SCALE * VREFINT_CAL * vbat * 2 / (vrefint * ((1<<12)-1)));
	if (vbat_raw < 100) {
		// maybe D2 is not installed
		return -1;
	}
	return vbat_raw + VBAT_DIODE_VF;
#endif
	

}

void adc_start_analog_watchdogd(ADC_TypeDef *adc, uint32_t chsel)
{
  uint32_t cfgr1;
  cfgr1=0;

#ifdef NANOVNA_F303
 #if DEBUG_ENABLE_AWDG
  #if 1
  adcStop(&ADCD2);  
  adcStart(&ADCD2, NULL);
  adcgrpcfgTouch.sqr[0] = chsel;
  ADCDriver *adcp;
  adcp = &ADCD2;
  adcp->adcm->ISR    = adcp->adcm->ISR;
  adcp->adcm->IER    = ADC_IER_AWD1;
  adcStartConversion(&ADCD2, &adcgrpcfgTouch, samples, 1);
  chThdSleepMilliseconds(20);
  //adcStop(&ADCD2);  
  adcStopConversion(&ADCD2);
  //if (samples[0] > TOUCH_THRESHOLD) {
  //  handle_touch_interrupt();
  //}
  #else
  ADCDriver *adcp;
  adcp = &ADCD2;
  cfgr1 =   ADC_CFGR_RES_12BIT | ADC_CFGR_AWD1EN
    | ADC_CFGR_EXTEN_0     // rising edge of external trigger
    | ADC_CFGR_EXTSEL_2 ; // TIM3_TRGO
  /* ADC setup, if it is defined a callback for the analog watch dog then it
     is enabled.*/
  adcp->adcm->ISR    = adcp->adcm->ISR;
  adcp->adcm->IER    = ADC_IER_AWD1;
  adcp->adcm->TR1     = ADC_TR(0, TOUCH_THRESHOLD);
  adcp->adcm->SMPR1   = ADC_SMPR_SMP_1P5;
  adcp->adcm->SMPR2   = 0;
  adcp->adcm->SQR1    = ADC_CHSELR_CHSEL7 | ADC_SQR1_NUM_CH(1);
  adcp->adcm->SQR2    = 0;
  adcp->adcm->SQR3    = 0;
  adcp->adcm->SQR3    = 0;
  adcp->adcm->CFGR    = cfgr1;
  adcp->adcm->CR     |= ADC_CR_ADSTART;
  #endif
#endif
  //adcStop(&ADCD2);  // not needed
  // adcp->CR |= ADC_CR_ADSTART;
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
#ifdef NANOVNA_F303
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

static void adcerrorcallback (ADCDriver *adcp, adcerror_t err)
{
#ifdef NANOVNA_F303
  //if (err == ADC_ERR_AWD1) {
  increment_touch_cnt();
  handle_touch_interrupt();
  //}
#endif
}

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
OSAL_IRQ_HANDLER(STM32_ADC1_HANDLER)
{
  OSAL_IRQ_PROLOGUE();

  adc_interrupt(ADC1);

  OSAL_IRQ_EPILOGUE();
}
 #endif


