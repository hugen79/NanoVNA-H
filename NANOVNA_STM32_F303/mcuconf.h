/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef MCUCONF_H
#define MCUCONF_H

/*
 * STM32F3xx drivers configuration.
 * The following settings override the default settings present in
 * the various device driver implementation headers.
 * Note that the settings for each driver only have effect if the whole
 * driver is enabled in halconf.h.
 *
 * IRQ priorities:
 * 15...0       Lowest...Highest.
 *
 * DMA priorities:
 * 0...3        Lowest...Highest.
 */

#define STM32F3xx_MCUCONF

/*
 * HAL driver system settings.
 */
#define STM32_NO_INIT                       FALSE
#define STM32_PVD_ENABLE                    FALSE
#define STM32_PLS                           STM32_PLS_LEV0
#define STM32_HSI_ENABLED                   FALSE
#define STM32_HSE_ENABLED                   TRUE
#define STM32_SW                            STM32_SW_PLL
#define STM32_PLLSRC                        STM32_PLLSRC_HSE
#define STM32_PREDIV_VALUE                  1
#define STM32_PLLMUL_VALUE                  9
#define STM32_HPRE                          STM32_HPRE_DIV1
#define STM32_PPRE1                         STM32_PPRE1_DIV2
// Set SPI1 more faster use PPRE2 max speed
#define STM32_PPRE2                         STM32_PPRE2_DIV1
#define STM32_MCOSEL                        STM32_MCOSEL_PLLDIV2
#define STM32_ADC12PRES                     STM32_ADC12PRES_DIV2
//#define STM32_ADC34PRES                     STM32_ADC34PRES_DIV1
#define STM32_USART1SW                      STM32_USART1SW_PCLK
//#define STM32_USART2SW                      STM32_USART2SW_PCLK
//#define STM32_USART3SW                      STM32_USART3SW_PCLK
#define STM32_I2C1SW                        STM32_I2C1SW_SYSCLK
//#define STM32_I2C2SW                        STM32_I2C2SW_SYSCLK
#define STM32_TIM1SW                        STM32_TIM1SW_PCLK2
#define STM32_TIM8SW                        STM32_TIM8SW_PCLK2
#define STM32_USB_CLOCK_REQUIRED            TRUE
#define STM32_USBPRE                        STM32_USBPRE_DIV1P5

// Define STM32_I2C1_CLOCK as 72MHz (STM32_I2C1SW is STM32_I2C1SW_SYSCLK)
#define STM32_I2C1_CLOCK                    72

/*
 * RTC driver system settings for stm32f303
 */
#define RTC_PRER(a, s)              ((((a) - 1) << 16) | ((s) - 1))

// LSE for 32768 quartz
#define STM32_RTC_LSE_PRER                  RTC_PRER(128,  256)
// LSI 40k
#define STM32_RTC_LSI_PRER                  RTC_PRER( 40, 1000)

// Use auto select source (LSE or LSI)
//!!!! Need disable hal_lld_backup_domain_init() in hal_lld.c for current CPU!!!!
//        And if need correct rtc_init part
#ifdef VNA_AUTO_SELECT_RTC_SOURCE
 #define STM32_LSEDRV                        (3 << 3)
 #define STM32_RTCSEL                        STM32_RTCSEL_NOCLOCK
 #define STM32_LSI_ENABLED                   FALSE
 #define STM32_LSE_ENABLED                   FALSE
// Disable this function call in ChibiOS, backup domain init on auto select RTC source
 #define STM32_NO_BACKUP_DOMAIN_INIT
 #define hal_lld_backup_domain_init
#else
 #ifndef VNA_USE_LSE
  // Use 40kHz LSI
  #define STM32_LSE_ENABLED                   FALSE
  #define STM32_LSI_ENABLED                   TRUE
  #define STM32_RTCSEL                        STM32_RTCSEL_LSI
 #else
// Use 32768Hz LSE
  #define STM32_LSE_ENABLED                   TRUE
  #define STM32_LSI_ENABLED                   FALSE
  #define STM32_RTCSEL                        STM32_RTCSEL_LSE
  #define STM32_LSEDRV                        (3 << 3)
 #endif
#endif

/*
 * ADC driver system settings.
 */
#define STM32_ADC_USE_ADC1                  TRUE
#define STM32_ADC_USE_ADC2                  TRUE
#define STM32_ADC_USE_ADC3                  FALSE
#define STM32_ADC_ADC1_DMA_STREAM           STM32_DMA_STREAM_ID(1, 1)
#define STM32_ADC_ADC2_DMA_STREAM           STM32_DMA_STREAM_ID(2, 1)
//#define STM32_ADC_ADC3_DMA_STREAM           STM32_DMA_STREAM_ID(2, 5)
//#define STM32_ADC_ADC4_DMA_STREAM           STM32_DMA_STREAM_ID(2, 2)
#define STM32_ADC_ADC12_DMA_PRIORITY        2
//#define STM32_ADC_ADC34_DMA_PRIORITY        2
#define STM32_ADC_ADC12_IRQ_PRIORITY        2
//#define STM32_ADC_ADC34_IRQ_PRIORITY        5
#define STM32_ADC_ADC12_DMA_IRQ_PRIORITY    2
//#define STM32_ADC_ADC34_DMA_IRQ_PRIORITY    5
//#define STM32_ADC_ADC12_CLOCK_MODE          ADC_CCR_CKMODE_ADCCK
//#define STM32_ADC_ADC12_CLOCK_MODE          ADC_CCR_CKMODE_AHB_DIV2
#define STM32_ADC_ADC12_CLOCK_MODE          ADC_CCR_CKMODE_AHB_DIV1
//#define STM32_ADC_ADC34_CLOCK_MODE          ADC_CCR_CKMODE_AHB_DIV1
#define STM32_ADC_DUAL_MODE                 FALSE

/*
 * CAN driver system settings.
 */
#define STM32_CAN_USE_CAN1                  FALSE
#define STM32_CAN_CAN1_IRQ_PRIORITY         11

/*
 * DAC driver system settings.
 */
#define STM32_DAC_DUAL_MODE                 FALSE
#define STM32_DAC_USE_DAC1_CH1              TRUE
#define STM32_DAC_USE_DAC1_CH2              TRUE
#define STM32_DAC_DAC1_CH1_IRQ_PRIORITY     10
#define STM32_DAC_DAC1_CH2_IRQ_PRIORITY     10
#define STM32_DAC_DAC1_CH1_DMA_PRIORITY     2
#define STM32_DAC_DAC1_CH2_DMA_PRIORITY     2

/*
 * EXT driver system settings.
 */
#define STM32_EXT_EXTI0_1_IRQ_PRIORITY      3
#define STM32_EXT_EXTI2_3_IRQ_PRIORITY      3
#define STM32_EXT_EXTI4_15_IRQ_PRIORITY     3
#define STM32_EXT_EXTI16_IRQ_PRIORITY       3
#define STM32_EXT_EXTI17_IRQ_PRIORITY       3
#define STM32_EXT_EXTI21_22_IRQ_PRIORITY    3

#define STM32_DISABLE_EXTI2122_HANDLER 		TRUE

/*
 * GPT driver system settings.
 */
#define STM32_GPT_USE_TIM1                  FALSE
#define STM32_GPT_USE_TIM2                  FALSE
#define STM32_GPT_USE_TIM3                  TRUE
#define STM32_GPT_USE_TIM4                  FALSE
#define STM32_GPT_TIM1_IRQ_PRIORITY         2
#define STM32_GPT_TIM2_IRQ_PRIORITY         2
#define STM32_GPT_TIM3_IRQ_PRIORITY         2
#define STM32_GPT_TIM4_IRQ_PRIORITY         2
// disable interrupt handlers
#define STM32_TIM1_SUPPRESS_ISR
#define STM32_TIM2_SUPPRESS_ISR
#define STM32_TIM3_SUPPRESS_ISR
#define STM32_TIM4_SUPPRESS_ISR

/*
 * I2C driver system settings.
 */
#define STM32_I2C_USE_I2C1                  TRUE
#define STM32_I2C_USE_I2C2                  FALSE
#define STM32_I2C_BUSY_TIMEOUT              50
#define STM32_I2C_I2C1_IRQ_PRIORITY         3
#define STM32_I2C_I2C2_IRQ_PRIORITY         3
#define STM32_I2C_USE_DMA                   FALSE
#define STM32_I2C_I2C1_DMA_PRIORITY         1
#define STM32_I2C_I2C2_DMA_PRIORITY         1
#define STM32_I2C_DMA_ERROR_HOOK(i2cp)      osalSysHalt("DMA failure")

/*
 * I2S driver system settings.
 */
#define STM32_I2S_USE_SPI1                  FALSE
#define STM32_I2S_USE_SPI2                  TRUE
#define STM32_I2S_SPI1_MODE                 (STM32_I2S_MODE_MASTER |        \
                                             STM32_I2S_MODE_RX)
#define STM32_I2S_SPI2_MODE                 (STM32_I2S_MODE_SLAVE |        \
                                             STM32_I2S_MODE_RX )
#define STM32_I2S_SPI1_IRQ_PRIORITY         3
#define STM32_I2S_SPI2_IRQ_PRIORITY         3
#define STM32_I2S_SPI1_DMA_PRIORITY         1
#define STM32_I2S_SPI2_DMA_PRIORITY         1
#define STM32_I2S_SPI1_RX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 2)
#define STM32_I2S_SPI1_TX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 3)
#define STM32_I2S_SPI2_RX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 4)
#define STM32_I2S_SPI2_TX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 5)
#define STM32_I2S_DMA_ERROR_HOOK(i2sp)      osalSysHalt("DMA failure")

/*
 * ICU driver system settings.
 */
#define STM32_ICU_USE_TIM1                  FALSE
#define STM32_ICU_USE_TIM2                  FALSE
#define STM32_ICU_USE_TIM3                  FALSE
#define STM32_ICU_TIM1_IRQ_PRIORITY         3
#define STM32_ICU_TIM2_IRQ_PRIORITY         3
#define STM32_ICU_TIM3_IRQ_PRIORITY         3

/*
 * PWM driver system settings.
 */
#define STM32_PWM_USE_ADVANCED              FALSE
#define STM32_PWM_USE_TIM1                  FALSE
#define STM32_PWM_USE_TIM2                  FALSE
#define STM32_PWM_USE_TIM3                  FALSE
#define STM32_PWM_TIM1_IRQ_PRIORITY         3
#define STM32_PWM_TIM2_IRQ_PRIORITY         3
#define STM32_PWM_TIM3_IRQ_PRIORITY         3

/*
 * SERIAL driver system settings.
 */
#define STM32_SERIAL_USE_USART1             TRUE
#define STM32_SERIAL_USE_USART2             FALSE
#define STM32_SERIAL_USART1_PRIORITY        2
#define STM32_SERIAL_USART2_PRIORITY        3

/*
 * SPI driver system settings.
 */
#define STM32_SPI_USE_SPI1                  FALSE
#define STM32_SPI_USE_SPI2                  FALSE
#define STM32_SPI_USE_SPI3                  FALSE
#define STM32_SPI_SPI1_RX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 2)
#define STM32_SPI_SPI1_TX_DMA_STREAM        STM32_DMA_STREAM_ID(1, 3)
#define STM32_SPI_SPI1_DMA_PRIORITY         1
#define STM32_SPI_SPI2_DMA_PRIORITY         1
#define STM32_SPI_SPI1_IRQ_PRIORITY         3
#define STM32_SPI_SPI2_IRQ_PRIORITY         3
#define STM32_SPI_DMA_ERROR_HOOK(spip)      osalSysHalt("DMA failure")

/*
 * ST driver system settings.
 */
#define STM32_ST_IRQ_PRIORITY               2
#define STM32_ST_USE_TIMER                  2

/*
 * UART driver system settings.
 */
#define STM32_UART_USE_USART1               FALSE
#define STM32_UART_USE_USART2               FALSE
#define STM32_UART_USART1_IRQ_PRIORITY      3
#define STM32_UART_USART2_IRQ_PRIORITY      3
#define STM32_UART_USART1_DMA_PRIORITY      0
#define STM32_UART_USART2_DMA_PRIORITY      0
#define STM32_UART_USART3_DMA_PRIORITY      0
#define STM32_UART_DMA_ERROR_HOOK(uartp)    osalSysHalt("DMA failure")

/*
 * USB driver system settings.
 */
#define STM32_USB_USE_USB1                  TRUE
#define STM32_USB_LOW_POWER_ON_SUSPEND      FALSE
#define STM32_USB_USB1_LP_IRQ_PRIORITY      3

/*
 * WDG driver system settings.
 */
#define STM32_WDG_USE_IWDG                  FALSE

#endif /* MCUCONF_H */
