
// F303 related ADC defines


#define ADC_SMPR_SMP_247P5      6   /**< @brief 260 cycles conversion time. */
#define ADC_SMPR_SMP_24P5       3   /**< @brief 37 cycles conversion time.  */


#define rccEnableWWDG(lp) rccEnableAPB1(RCC_APB1ENR_WWDGEN, lp)
#define ADC_CHSELR_CHSEL6  ADC_CHANNEL_IN3
#define ADC_CHSELR_CHSEL7  ADC_CHANNEL_IN4
#define ADC_SMPR_SMP_239P5      7U
#define ADC_SMPR_SMP_28P5       3U  /**< @brief 41 cycles conversion time.  */
#define ADC_CFGR_RES_12BIT             (0 << 3)
/*
msg_t adcConvert(ADCDriver *adcp,
                   const ADCConversionGroup *grpp,
                   adcsample_t *samples,
                   size_t depth);
*/
#define  ADC_CR1_AWDEN                       ((uint32_t)0x00800000)  /*!< Analog watchdog enable on regular channels */
//ADC_Common_TypeDef        *adcc;
#define ADC_CHSELR_VREFINT      ADC_CHANNEL_IN18
#define ADC_CHSELR_VBAT         ADC_CHANNEL_IN17


