/*
    ChibiOS - Copyright (C) 2006..2017 Giovanni Di Sirio

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

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

#ifndef BOARD_H
#define BOARD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Setup for NanoVNA-H 4 board.
 */

/*
 * Board identifier.
 */
#define BOARD_NANOVNA_STM32_F303
#define BOARD_NAME                  "NanoVNA-H 4"

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#define STM32_LSEDRV                (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

/*
 * MCU type as defined in the ST header.
 */
#define STM32F303xC

/*
 * IO pins assignments.
 */
#define GPIOA_BUTTON                0U
#define GPIOA_DOWN                  1U
#define GPIOA_PUSH                  2U
#define GPIOA_UP                    3U
#define GPIOA_USB_V_IN              4U
#define GPIOA_LCD_BL                5U
#define GPIOA_XP                    6U
#define GPIOA_YP                    7U
#define GPIOA_USB_DISC              8U
#define GPIOA_UART_TX               9U
#define GPIOA_UART_RX               10U
#define GPIOA_USB_DM                11U
#define GPIOA_USB_DP                12U
#define GPIOA_TMS                   13U
#define GPIOA_TCK                   14U
#define GPIOA_LCD_RESET             15U

#define GPIOB_XN                    0U
#define GPIOB_YN                    1U
#define GPIOB_SD_GP1                2U
#define GPIOB_SPI_SCLK              3U
#define GPIOB_SPI_MISO              4U
#define GPIOB_SPI_MOSI              5U
#define GPIOB_LCD_CS                6U
#define GPIOB_LCD_CD                7U
#define GPIOB_SCL                   8U
#define GPIOB_SDA                   9U
#define GPIOB_SD_GP2                10U
#define GPIOB_SD_CS                 11U
#define GPIOB_I2S_WCLK              12U
#define GPIOB_I2S_BCLK              13U
#define GPIOB_Codec_Reset_N         14U
#define GPIOB_I2S_DOUT              15U

#define GPIOC_PIN0                  0U
#define GPIOC_PIN1                  1U
#define GPIOC_PIN2                  2U
#define GPIOC_PIN3                  3U
#define GPIOC_PIN4                  4U
#define GPIOC_PIN5                  5U
#define GPIOC_PIN6                  6U
#define GPIOC_PIN7                  7U
#define GPIOC_PIN8                  8U
#define GPIOC_PIN9                  9U
#define GPIOC_PIN10                 10U
#define GPIOC_PIN11                 11U
#define GPIOC_PIN12                 12U
#define GPIOC_LED                   13U
#define GPIOC_PC13                  13U
#define GPIOC_PIN14                 14U
#define GPIOC_PIN15                 15U

#define GPIOF_PIN0                  0U
#define GPIOF_PIN1                  1U
#define GPIOF_PIN2                  2U
#define GPIOF_PIN3                  3U
#define GPIOF_PIN4                  4U
#define GPIOF_PIN5                  5U
#define GPIOF_PIN6                  6U
#define GPIOF_PIN7                  7U
#define GPIOF_PIN8                  8U
#define GPIOF_PIN9                  9U
#define GPIOF_PIN10                 10U
#define GPIOF_PIN11                 11U
#define GPIOF_PIN12                 12U
#define GPIOF_PIN13                 13U
#define GPIOF_PIN14                 14U
#define GPIOF_PIN15                 15U

/*
 * IO lines assignments.
 */
#define LINE_BUTTON                 PAL_LINE(GPIOA, 0U)
#define LINE_DOWN                   PAL_LINE(GPIOA, 1U)
#define LINE_PUSH                   PAL_LINE(GPIOA, 2U)
#define LINE_UP                     PAL_LINE(GPIOA, 3U)
#define LINE_USB_V_IN               PAL_LINE(GPIOA, 4U)
#define LINE_LCD_BL                 PAL_LINE(GPIOA, 5U)
#define LINE_XP                     PAL_LINE(GPIOA, 6U)
#define LINE_YP                     PAL_LINE(GPIOA, 7U)
#define LINE_USB_DISC               PAL_LINE(GPIOA, 8U)
#define LINE_UART_TX                PAL_LINE(GPIOA, 9U)
#define LINE_UART_RX                PAL_LINE(GPIOA, 10U)
#define LINE_USB_DM                 PAL_LINE(GPIOA, 11U)
#define LINE_USB_DP                 PAL_LINE(GPIOA, 12U)
#define LINE_TMS                    PAL_LINE(GPIOA, 13U)
#define LINE_TCK                    PAL_LINE(GPIOA, 14U)
#define LINE_LCD_RESET              PAL_LINE(GPIOA, 15U)
#define LINE_XN                     PAL_LINE(GPIOB, 0U)
#define LINE_YN                     PAL_LINE(GPIOB, 1U)
#define LINE_SD_GP1                 PAL_LINE(GPIOB, 2U)
#define LINE_SPI_SCLK               PAL_LINE(GPIOB, 3U)
#define LINE_SPI_MISO               PAL_LINE(GPIOB, 4U)
#define LINE_SPI_MOSI               PAL_LINE(GPIOB, 5U)
#define LINE_LCD_CS                 PAL_LINE(GPIOB, 6U)
#define LINE_LCD_CD                 PAL_LINE(GPIOB, 7U)
#define LINE_SCL                    PAL_LINE(GPIOB, 8U)
#define LINE_SDA                    PAL_LINE(GPIOB, 9U)
#define LINE_SD_GP2                 PAL_LINE(GPIOB, 10U)
#define LINE_SD_CS                  PAL_LINE(GPIOB, 11U)
#define LINE_I2S_WCLK               PAL_LINE(GPIOB, 12U)
#define LINE_I2S_BCLK               PAL_LINE(GPIOB, 13U)
#define LINE_Codec_Reset_N          PAL_LINE(GPIOB, 14U)
#define LINE_I2S_DOUT               PAL_LINE(GPIOB, 15U)
#define LINE_PC13                   PAL_LINE(GPIOC, 13U)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
 *
 * PA0  - BUTTON                    (input floating).
 * PA1  - DOWN                      (input pulldown).
 * PA2  - PUSH                      (input pulldown).
 * PA3  - UP                        (input pulldown).
 * PA4  - USB_V_IN                  (input floating).
 * PA5  - LCD_BL                    (input floating).
 * PA6  - XP                        (output pushpull high).
 * PA7  - YP                        (output pushpull high).
 * PA8  - USB_DISC                  (output pushpull high).
 * PA9  - UART_TX                   (alternate 7).
 * PA10 - UART_RX                   (alternate 7).
 * PA11 - USB_DM                    (alternate 14).
 * PA12 - USB_DP                    (alternate 14).
 * PA13 - TMS                       (alternate 0).
 * PA14 - TCK                       (alternate 0).
 * PA15 - LCD_RESET                 (output pushpull high).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(GPIOA_BUTTON) |         \
                                     PIN_MODE_INPUT(GPIOA_DOWN) |           \
                                     PIN_MODE_INPUT(GPIOA_PUSH) |           \
                                     PIN_MODE_INPUT(GPIOA_UP) |             \
                                     PIN_MODE_INPUT(GPIOA_USB_V_IN) |       \
                                     PIN_MODE_INPUT(GPIOA_LCD_BL) |         \
                                     PIN_MODE_OUTPUT(GPIOA_XP) |            \
                                     PIN_MODE_OUTPUT(GPIOA_YP) |            \
                                     PIN_MODE_OUTPUT(GPIOA_USB_DISC) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_UART_TX) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_UART_RX) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_DM) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_DP) |     \
                                     PIN_MODE_ALTERNATE(GPIOA_TMS) |        \
                                     PIN_MODE_ALTERNATE(GPIOA_TCK) |        \
                                     PIN_MODE_OUTPUT(GPIOA_LCD_RESET))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_BUTTON) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_DOWN) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_PUSH) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UP) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_V_IN) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LCD_BL) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_XP) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_YP) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DISC) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART_TX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_UART_RX) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DM) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DP) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOA_TMS) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOA_TCK) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LCD_RESET))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOA_BUTTON) |     \
                                     PIN_OSPEED_VERYLOW(GPIOA_DOWN) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_PUSH) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_UP) |         \
                                     PIN_OSPEED_VERYLOW(GPIOA_USB_V_IN) |   \
                                     PIN_OSPEED_VERYLOW(GPIOA_LCD_BL) |     \
                                     PIN_OSPEED_LOW(GPIOA_XP) |             \
                                     PIN_OSPEED_LOW(GPIOA_YP) |             \
                                     PIN_OSPEED_LOW(GPIOA_USB_DISC) |       \
                                     PIN_OSPEED_VERYLOW(GPIOA_UART_TX) |    \
                                     PIN_OSPEED_VERYLOW(GPIOA_UART_RX) |    \
                                     PIN_OSPEED_VERYLOW(GPIOA_USB_DM) |     \
                                     PIN_OSPEED_VERYLOW(GPIOA_USB_DP) |     \
                                     PIN_OSPEED_VERYLOW(GPIOA_TMS) |        \
                                     PIN_OSPEED_VERYLOW(GPIOA_TCK) |        \
                                     PIN_OSPEED_LOW(GPIOA_LCD_RESET))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_BUTTON) |     \
                                     PIN_PUPDR_PULLDOWN(GPIOA_DOWN) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOA_PUSH) |       \
                                     PIN_PUPDR_PULLDOWN(GPIOA_UP) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_V_IN) |   \
                                     PIN_PUPDR_FLOATING(GPIOA_LCD_BL) |     \
                                     PIN_PUPDR_PULLUP(GPIOA_XP) |           \
                                     PIN_PUPDR_PULLUP(GPIOA_YP) |           \
                                     PIN_PUPDR_PULLUP(GPIOA_USB_DISC) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_UART_TX) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_UART_RX) |    \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_DM) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_DP) |     \
                                     PIN_PUPDR_FLOATING(GPIOA_TMS) |        \
                                     PIN_PUPDR_FLOATING(GPIOA_TCK) |        \
                                     PIN_PUPDR_PULLUP(GPIOA_LCD_RESET))
#define VAL_GPIOA_ODR               (PIN_ODR_LOW(GPIOA_BUTTON) |            \
                                     PIN_ODR_LOW(GPIOA_DOWN) |              \
                                     PIN_ODR_LOW(GPIOA_PUSH) |              \
                                     PIN_ODR_LOW(GPIOA_UP) |                \
                                     PIN_ODR_LOW(GPIOA_USB_V_IN) |          \
                                     PIN_ODR_LOW(GPIOA_LCD_BL) |            \
                                     PIN_ODR_HIGH(GPIOA_XP) |               \
                                     PIN_ODR_HIGH(GPIOA_YP) |               \
                                     PIN_ODR_LOW(GPIOA_USB_DISC) |          \
                                     PIN_ODR_LOW(GPIOA_UART_TX) |           \
                                     PIN_ODR_LOW(GPIOA_UART_RX) |           \
                                     PIN_ODR_LOW(GPIOA_USB_DM) |            \
                                     PIN_ODR_LOW(GPIOA_USB_DP) |            \
                                     PIN_ODR_LOW(GPIOA_TMS) |               \
                                     PIN_ODR_LOW(GPIOA_TCK) |               \
                                     PIN_ODR_HIGH(GPIOA_LCD_RESET))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_BUTTON, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_DOWN, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_PUSH, 0U) |          \
                                     PIN_AFIO_AF(GPIOA_UP, 0U) |            \
                                     PIN_AFIO_AF(GPIOA_USB_V_IN, 0U) |      \
                                     PIN_AFIO_AF(GPIOA_LCD_BL, 0U) |        \
                                     PIN_AFIO_AF(GPIOA_XP, 0U) |            \
                                     PIN_AFIO_AF(GPIOA_YP, 0U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_USB_DISC, 0U) |      \
                                     PIN_AFIO_AF(GPIOA_UART_TX, 7U) |       \
                                     PIN_AFIO_AF(GPIOA_UART_RX, 7U) |       \
                                     PIN_AFIO_AF(GPIOA_USB_DM, 14U) |       \
                                     PIN_AFIO_AF(GPIOA_USB_DP, 14U) |       \
                                     PIN_AFIO_AF(GPIOA_TMS, 0U) |           \
                                     PIN_AFIO_AF(GPIOA_TCK, 0U) |           \
                                     PIN_AFIO_AF(GPIOA_LCD_RESET, 0U))

/*
 * GPIOB setup:
 *
 * PB0  - XN                        (output pushpull high).
 * PB1  - YN                        (output pushpull high).
 * PB2  - SD_GP1                    (output pushpull high).
 * PB3  - SPI_SCLK                  (alternate 5).
 * PB4  - SPI_MISO                  (alternate 5).
 * PB5  - SPI_MOSI                  (alternate 5).
 * PB6  - LCD_CS                    (output pushpull high).
 * PB7  - LCD_CD                    (output pushpull high).
 * PB8  - SCL                       (alternate 4).
 * PB9  - SDA                       (alternate 4).
 * PB10 - SD_GP2                    (output pushpull high).
 * PB11 - SD_CS                     (output pushpull high).
 * PB12 - I2S_WCLK                  (alternate 5).
 * PB13 - I2S_BCLK                  (alternate 5).
 * PB14 - Codec_Reset_N             (output pushpull high).
 * PB15 - I2S_DOUT                  (alternate 5).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_OUTPUT(GPIOB_XN) |            \
                                     PIN_MODE_OUTPUT(GPIOB_YN) |            \
                                     PIN_MODE_OUTPUT(GPIOB_SD_GP1) |        \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI_SCLK) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI_MISO) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI_MOSI) |   \
                                     PIN_MODE_OUTPUT(GPIOB_LCD_CS) |        \
                                     PIN_MODE_OUTPUT(GPIOB_LCD_CD) |        \
                                     PIN_MODE_ALTERNATE(GPIOB_SCL) |        \
                                     PIN_MODE_ALTERNATE(GPIOB_SDA) |        \
                                     PIN_MODE_OUTPUT(GPIOB_SD_GP2) |        \
                                     PIN_MODE_OUTPUT(GPIOB_SD_CS) |         \
                                     PIN_MODE_ALTERNATE(GPIOB_I2S_WCLK) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_I2S_BCLK) |   \
                                     PIN_MODE_OUTPUT(GPIOB_Codec_Reset_N) | \
                                     PIN_MODE_ALTERNATE(GPIOB_I2S_DOUT))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_XN) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_YN) |         \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SD_GP1) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI_SCLK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI_MISO) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI_MOSI) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LCD_CS) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LCD_CD) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SCL) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SDA) |        \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SD_GP2) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SD_CS) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOB_I2S_WCLK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_I2S_BCLK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOB_Codec_Reset_N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_I2S_DOUT))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_LOW(GPIOB_XN) |             \
                                     PIN_OSPEED_LOW(GPIOB_YN) |             \
                                     PIN_OSPEED_LOW(GPIOB_SD_GP1) |         \
                                     PIN_OSPEED_VERYLOW(GPIOB_SPI_SCLK) |   \
                                     PIN_OSPEED_VERYLOW(GPIOB_SPI_MISO) |   \
                                     PIN_OSPEED_VERYLOW(GPIOB_SPI_MOSI) |   \
                                     PIN_OSPEED_LOW(GPIOB_LCD_CS) |         \
                                     PIN_OSPEED_LOW(GPIOB_LCD_CD) |         \
                                     PIN_OSPEED_VERYLOW(GPIOB_SCL) |        \
                                     PIN_OSPEED_VERYLOW(GPIOB_SDA) |        \
                                     PIN_OSPEED_LOW(GPIOB_SD_GP2) |         \
                                     PIN_OSPEED_LOW(GPIOB_SD_CS) |          \
                                     PIN_OSPEED_VERYLOW(GPIOB_I2S_WCLK) |   \
                                     PIN_OSPEED_VERYLOW(GPIOB_I2S_BCLK) |   \
                                     PIN_OSPEED_LOW(GPIOB_Codec_Reset_N) |  \
                                     PIN_OSPEED_VERYLOW(GPIOB_I2S_DOUT))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_PULLUP(GPIOB_XN) |           \
                                     PIN_PUPDR_PULLUP(GPIOB_YN) |           \
                                     PIN_PUPDR_FLOATING(GPIOB_SD_GP1) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI_SCLK) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI_MISO) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_SPI_MOSI) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_LCD_CS) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_LCD_CD) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_SCL) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_SDA) |        \
                                     PIN_PUPDR_FLOATING(GPIOB_SD_GP2) |     \
                                     PIN_PUPDR_FLOATING(GPIOB_SD_CS) |      \
                                     PIN_PUPDR_FLOATING(GPIOB_I2S_WCLK) |   \
                                     PIN_PUPDR_FLOATING(GPIOB_I2S_BCLK) |   \
                                     PIN_PUPDR_PULLUP(GPIOB_Codec_Reset_N) |\
                                     PIN_PUPDR_FLOATING(GPIOB_I2S_DOUT))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_XN) |               \
                                     PIN_ODR_HIGH(GPIOB_YN) |               \
                                     PIN_ODR_LOW(GPIOB_SD_GP1) |            \
                                     PIN_ODR_LOW(GPIOB_SPI_SCLK) |          \
                                     PIN_ODR_LOW(GPIOB_SPI_MISO) |          \
                                     PIN_ODR_LOW(GPIOB_SPI_MOSI) |          \
                                     PIN_ODR_LOW(GPIOB_LCD_CS) |            \
                                     PIN_ODR_LOW(GPIOB_LCD_CD) |            \
                                     PIN_ODR_LOW(GPIOB_SCL) |               \
                                     PIN_ODR_LOW(GPIOB_SDA) |               \
                                     PIN_ODR_LOW(GPIOB_SD_GP2) |            \
                                     PIN_ODR_LOW(GPIOB_SD_CS) |             \
                                     PIN_ODR_LOW(GPIOB_I2S_WCLK) |          \
                                     PIN_ODR_LOW(GPIOB_I2S_BCLK) |          \
                                     PIN_ODR_HIGH(GPIOB_Codec_Reset_N) |    \
                                     PIN_ODR_LOW(GPIOB_I2S_DOUT))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_XN, 0U) |            \
                                     PIN_AFIO_AF(GPIOB_YN, 0U) |            \
                                     PIN_AFIO_AF(GPIOB_SD_GP1, 0U) |        \
                                     PIN_AFIO_AF(GPIOB_SPI_SCLK, 5U) |      \
                                     PIN_AFIO_AF(GPIOB_SPI_MISO, 5U) |      \
                                     PIN_AFIO_AF(GPIOB_SPI_MOSI, 5U) |      \
                                     PIN_AFIO_AF(GPIOB_LCD_CS, 0U) |        \
                                     PIN_AFIO_AF(GPIOB_LCD_CD, 0U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_SCL, 4U) |           \
                                     PIN_AFIO_AF(GPIOB_SDA, 4U) |           \
                                     PIN_AFIO_AF(GPIOB_SD_GP2, 0U) |        \
                                     PIN_AFIO_AF(GPIOB_SD_CS, 0U) |         \
                                     PIN_AFIO_AF(GPIOB_I2S_WCLK, 5U) |      \
                                     PIN_AFIO_AF(GPIOB_I2S_BCLK, 5U) |      \
                                     PIN_AFIO_AF(GPIOB_Codec_Reset_N, 0U) | \
                                     PIN_AFIO_AF(GPIOB_I2S_DOUT, 5U))

/*
 * GPIOC setup:
 *
 * PC0  - PIN0                      (alternate 5).
 * PC1  - PIN1                      (alternate 5).
 * PC2  - PIN2                      (alternate 5).
 * PC3  - PIN3                      (alternate 5).
 * PC4  - PIN4                      (alternate 5).
 * PC5  - PIN5                      (alternate 5).
 * PC6  - PIN6                      (alternate 5).
 * PC7  - PIN7                      (alternate 5).
 * PC8  - PIN8                      (alternate 5).
 * PC9  - PIN9                      (alternate 5).
 * PC10 - PIN10                     (alternate 5).
 * PC11 - PIN11                     (alternate 5).
 * PC12 - PIN12                     (alternate 5).
 * PC13 - PC13                      (output pushpull minimum).
 * PC14 - PIN14                     (input floating).
 * PC15 - PIN15                     (input floating).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ALTERNATE(GPIOC_PIN0) |       \
                                     PIN_MODE_ALTERNATE(GPIOC_PIN1) |       \
                                     PIN_MODE_ALTERNATE(GPIOC_PIN2) |       \
                                     PIN_MODE_ALTERNATE(GPIOC_PIN3) |       \
                                     PIN_MODE_ALTERNATE(GPIOC_PIN4) |       \
                                     PIN_MODE_ALTERNATE(GPIOC_PIN5) |       \
                                     PIN_MODE_ALTERNATE(GPIOC_PIN6) |       \
                                     PIN_MODE_ALTERNATE(GPIOC_PIN7) |       \
                                     PIN_MODE_ALTERNATE(GPIOC_PIN8) |       \
                                     PIN_MODE_ALTERNATE(GPIOC_PIN9) |       \
                                     PIN_MODE_ALTERNATE(GPIOC_PIN10) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_PIN11) |      \
                                     PIN_MODE_ALTERNATE(GPIOC_PIN12) |      \
                                     PIN_MODE_OUTPUT(GPIOC_PC13) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PC13) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOC_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_PC13) |       \
                                     PIN_OSPEED_HIGH(GPIOC_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOC_PIN15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_PC13) |       \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOC_PIN15))
#define VAL_GPIOC_ODR               (PIN_ODR_LOW(GPIOC_PIN0) |              \
                                     PIN_ODR_LOW(GPIOC_PIN1) |              \
                                     PIN_ODR_LOW(GPIOC_PIN2) |              \
                                     PIN_ODR_LOW(GPIOC_PIN3) |              \
                                     PIN_ODR_LOW(GPIOC_PIN4) |              \
                                     PIN_ODR_LOW(GPIOC_PIN5) |              \
                                     PIN_ODR_LOW(GPIOC_PIN6) |              \
                                     PIN_ODR_LOW(GPIOC_PIN7) |              \
                                     PIN_ODR_LOW(GPIOC_PIN8) |              \
                                     PIN_ODR_LOW(GPIOC_PIN9) |              \
                                     PIN_ODR_LOW(GPIOC_PIN10) |             \
                                     PIN_ODR_LOW(GPIOC_PIN11) |             \
                                     PIN_ODR_LOW(GPIOC_PIN12) |             \
                                     PIN_ODR_LOW(GPIOC_PC13) |              \
                                     PIN_ODR_HIGH(GPIOC_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN15))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_PIN0, 5U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN1, 5U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN2, 5U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN3, 5U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN4, 5U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN5, 5U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN6, 5U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN7, 5U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_PIN8, 5U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN9, 5U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN10, 5U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN11, 5U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN12, 5U) |         \
                                     PIN_AFIO_AF(GPIOC_PC13, 0U) |          \
                                     PIN_AFIO_AF(GPIOC_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN15, 0U))

/*
 * GPIOD setup:
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(0) |           \
                                     PIN_MODE_INPUT(1) |           \
                                     PIN_MODE_INPUT(2) |           \
                                     PIN_MODE_INPUT(3) |           \
                                     PIN_MODE_INPUT(4) |           \
                                     PIN_MODE_INPUT(5) |           \
                                     PIN_MODE_INPUT(6) |           \
                                     PIN_MODE_INPUT(7) |           \
                                     PIN_MODE_INPUT(8) |           \
                                     PIN_MODE_INPUT(9) |           \
                                     PIN_MODE_INPUT(10) |          \
                                     PIN_MODE_INPUT(11) |          \
                                     PIN_MODE_INPUT(12) |          \
                                     PIN_MODE_INPUT(13) |          \
                                     PIN_MODE_INPUT(14) |          \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(0) |       \
                                     PIN_OTYPE_PUSHPULL(1) |       \
                                     PIN_OTYPE_PUSHPULL(2) |       \
                                     PIN_OTYPE_PUSHPULL(3) |       \
                                     PIN_OTYPE_PUSHPULL(4) |       \
                                     PIN_OTYPE_PUSHPULL(5) |       \
                                     PIN_OTYPE_PUSHPULL(6) |       \
                                     PIN_OTYPE_PUSHPULL(7) |       \
                                     PIN_OTYPE_PUSHPULL(8) |       \
                                     PIN_OTYPE_PUSHPULL(9) |       \
                                     PIN_OTYPE_PUSHPULL(10) |      \
                                     PIN_OTYPE_PUSHPULL(11) |      \
                                     PIN_OTYPE_PUSHPULL(12) |      \
                                     PIN_OTYPE_PUSHPULL(13) |      \
                                     PIN_OTYPE_PUSHPULL(14) |      \
                                     PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_VERYLOW(0) |          \
                                     PIN_OSPEED_VERYLOW(1) |          \
                                     PIN_OSPEED_VERYLOW(2) |          \
                                     PIN_OSPEED_VERYLOW(3) |          \
                                     PIN_OSPEED_VERYLOW(4) |          \
                                     PIN_OSPEED_VERYLOW(5) |          \
                                     PIN_OSPEED_VERYLOW(6) |          \
                                     PIN_OSPEED_VERYLOW(7) |          \
                                     PIN_OSPEED_VERYLOW(8) |          \
                                     PIN_OSPEED_VERYLOW(9) |          \
                                     PIN_OSPEED_VERYLOW(10) |         \
                                     PIN_OSPEED_VERYLOW(11) |         \
                                     PIN_OSPEED_VERYLOW(12) |         \
                                     PIN_OSPEED_VERYLOW(13) |         \
                                     PIN_OSPEED_VERYLOW(14) |         \
                                     PIN_OSPEED_VERYLOW(15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLUP(0) |         \
                                     PIN_PUPDR_PULLUP(1) |         \
                                     PIN_PUPDR_PULLUP(2) |         \
                                     PIN_PUPDR_PULLUP(3) |         \
                                     PIN_PUPDR_PULLUP(4) |         \
                                     PIN_PUPDR_PULLUP(5) |         \
                                     PIN_PUPDR_PULLUP(6) |         \
                                     PIN_PUPDR_PULLUP(7) |         \
                                     PIN_PUPDR_PULLUP(8) |         \
                                     PIN_PUPDR_PULLUP(9) |         \
                                     PIN_PUPDR_PULLUP(10) |        \
                                     PIN_PUPDR_PULLUP(11) |        \
                                     PIN_PUPDR_PULLUP(12) |        \
                                     PIN_PUPDR_PULLUP(13) |        \
                                     PIN_PUPDR_PULLUP(14) |        \
                                     PIN_PUPDR_PULLUP(15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(0) |             \
                                     PIN_ODR_HIGH(1) |             \
                                     PIN_ODR_HIGH(2) |             \
                                     PIN_ODR_HIGH(3) |             \
                                     PIN_ODR_HIGH(4) |             \
                                     PIN_ODR_HIGH(5) |             \
                                     PIN_ODR_HIGH(6) |             \
                                     PIN_ODR_HIGH(7) |             \
                                     PIN_ODR_HIGH(8) |             \
                                     PIN_ODR_HIGH(9) |             \
                                     PIN_ODR_HIGH(10) |            \
                                     PIN_ODR_HIGH(11) |            \
                                     PIN_ODR_HIGH(12) |            \
                                     PIN_ODR_HIGH(13) |            \
                                     PIN_ODR_HIGH(14) |            \
                                     PIN_ODR_HIGH(15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(0, 0) |           \
                                     PIN_AFIO_AF(1, 0) |           \
                                     PIN_AFIO_AF(2, 0) |           \
                                     PIN_AFIO_AF(3, 0) |           \
                                     PIN_AFIO_AF(4, 0) |           \
                                     PIN_AFIO_AF(5, 0) |           \
                                     PIN_AFIO_AF(6, 0) |           \
                                     PIN_AFIO_AF(7, 0))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(8, 0) |           \
                                     PIN_AFIO_AF(9, 0) |           \
                                     PIN_AFIO_AF(10, 0) |          \
                                     PIN_AFIO_AF(11, 0) |          \
                                     PIN_AFIO_AF(12, 0) |          \
                                     PIN_AFIO_AF(13, 0) |          \
                                     PIN_AFIO_AF(14, 0) |          \
                                     PIN_AFIO_AF(15, 0))

/*
 * GPIOE setup:
 *
 * PE0  - PIN0                      (input pullup).
 * PE1  - PIN1                      (input pullup).
 * PE2  - PIN2                      (input floating).
 * PE3  - PIN3                      (input pullup).
 * PE4  - PIN4                      (input floating).
 * PE5  - PIN5                      (input floating).
 * PE6  - PIN6                      (input floating).
 * PE7  - PIN7                      (input floating).
 * PE8  - PIN8                      (input floating).
 * PE9  - PIN9                      (input floating).
 * PE10 - PIN10                     (input floating).
 * PE11 - PIN11                     (input floating).
 * PE12 - PIN12                     (input floating).
 * PE13 - PIN13                     (input floating).
 * PE14 - PIN14                     (input floating).
 * PE15 - PIN15                     (input floating).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(0) |           \
                                     PIN_MODE_INPUT(1) |           \
                                     PIN_MODE_INPUT(2) |           \
                                     PIN_MODE_INPUT(3) |           \
                                     PIN_MODE_INPUT(4) |           \
                                     PIN_MODE_INPUT(5) |           \
                                     PIN_MODE_INPUT(6) |           \
                                     PIN_MODE_INPUT(7) |           \
                                     PIN_MODE_INPUT(8) |           \
                                     PIN_MODE_INPUT(9) |           \
                                     PIN_MODE_INPUT(10) |          \
                                     PIN_MODE_INPUT(11) |          \
                                     PIN_MODE_INPUT(12) |          \
                                     PIN_MODE_INPUT(13) |          \
                                     PIN_MODE_INPUT(14) |          \
                                     PIN_MODE_INPUT(15))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(0) |       \
                                     PIN_OTYPE_PUSHPULL(1) |       \
                                     PIN_OTYPE_PUSHPULL(2) |       \
                                     PIN_OTYPE_PUSHPULL(3) |       \
                                     PIN_OTYPE_PUSHPULL(4) |       \
                                     PIN_OTYPE_PUSHPULL(5) |       \
                                     PIN_OTYPE_PUSHPULL(6) |       \
                                     PIN_OTYPE_PUSHPULL(7) |       \
                                     PIN_OTYPE_PUSHPULL(8) |       \
                                     PIN_OTYPE_PUSHPULL(9) |       \
                                     PIN_OTYPE_PUSHPULL(10) |      \
                                     PIN_OTYPE_PUSHPULL(11) |      \
                                     PIN_OTYPE_PUSHPULL(12) |      \
                                     PIN_OTYPE_PUSHPULL(13) |      \
                                     PIN_OTYPE_PUSHPULL(14) |      \
                                     PIN_OTYPE_PUSHPULL(15))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_VERYLOW(0) |          \
                                     PIN_OSPEED_VERYLOW(1) |          \
                                     PIN_OSPEED_VERYLOW(2) |          \
                                     PIN_OSPEED_VERYLOW(3) |          \
                                     PIN_OSPEED_VERYLOW(4) |          \
                                     PIN_OSPEED_VERYLOW(5) |          \
                                     PIN_OSPEED_VERYLOW(6) |          \
                                     PIN_OSPEED_VERYLOW(7) |          \
                                     PIN_OSPEED_VERYLOW(8) |          \
                                     PIN_OSPEED_VERYLOW(9) |          \
                                     PIN_OSPEED_VERYLOW(10) |         \
                                     PIN_OSPEED_VERYLOW(11) |         \
                                     PIN_OSPEED_VERYLOW(12) |         \
                                     PIN_OSPEED_VERYLOW(13) |         \
                                     PIN_OSPEED_VERYLOW(14) |         \
                                     PIN_OSPEED_VERYLOW(15))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLUP(0) |         \
                                     PIN_PUPDR_PULLUP(1) |         \
                                     PIN_PUPDR_FLOATING(2) |       \
                                     PIN_PUPDR_PULLUP(3) |         \
                                     PIN_PUPDR_FLOATING(4) |       \
                                     PIN_PUPDR_FLOATING(5) |       \
                                     PIN_PUPDR_FLOATING(6) |       \
                                     PIN_PUPDR_FLOATING(7) |       \
                                     PIN_PUPDR_FLOATING(8) |       \
                                     PIN_PUPDR_FLOATING(9) |       \
                                     PIN_PUPDR_FLOATING(10) |      \
                                     PIN_PUPDR_FLOATING(11) |      \
                                     PIN_PUPDR_FLOATING(12) |      \
                                     PIN_PUPDR_FLOATING(13) |      \
                                     PIN_PUPDR_FLOATING(14) |      \
                                     PIN_PUPDR_FLOATING(15))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(0) |             \
                                     PIN_ODR_HIGH(1) |             \
                                     PIN_ODR_HIGH(2) |             \
                                     PIN_ODR_HIGH(3) |             \
                                     PIN_ODR_HIGH(4) |             \
                                     PIN_ODR_HIGH(5) |             \
                                     PIN_ODR_HIGH(6) |             \
                                     PIN_ODR_HIGH(7) |             \
                                     PIN_ODR_HIGH(8) |             \
                                     PIN_ODR_HIGH(9) |             \
                                     PIN_ODR_HIGH(10) |            \
                                     PIN_ODR_HIGH(11) |            \
                                     PIN_ODR_HIGH(12) |            \
                                     PIN_ODR_HIGH(13) |            \
                                     PIN_ODR_HIGH(14) |            \
                                     PIN_ODR_HIGH(15))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(0, 0) |           \
                                     PIN_AFIO_AF(1, 0) |           \
                                     PIN_AFIO_AF(2, 0) |           \
                                     PIN_AFIO_AF(3, 0) |           \
                                     PIN_AFIO_AF(4, 0) |           \
                                     PIN_AFIO_AF(5, 0) |           \
                                     PIN_AFIO_AF(6, 0) |           \
                                     PIN_AFIO_AF(7, 0))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(8, 0) |           \
                                     PIN_AFIO_AF(9, 0) |           \
                                     PIN_AFIO_AF(10, 0) |          \
                                     PIN_AFIO_AF(11, 0) |          \
                                     PIN_AFIO_AF(12, 0) |          \
                                     PIN_AFIO_AF(13, 0) |          \
                                     PIN_AFIO_AF(14, 0) |          \
                                     PIN_AFIO_AF(15, 0))


/*
 * GPIOF setup:
 *
 * PF0  - PIN0                      (input floating).
 * PF1  - PIN1                      (input floating).
 * PF2  - PIN2                      (input floating).
 * PF3  - PIN3                      (input floating).
 * PF4  - PIN4                      (input floating).
 * PF5  - PIN5                      (input floating).
 * PF6  - PIN6                      (input floating).
 * PF7  - PIN7                      (input floating).
 * PF8  - PIN8                      (input floating).
 * PF9  - PIN9                      (input floating).
 * PF10 - PIN10                     (input floating).
 * PF11 - PIN11                     (input floating).
 * PF12 - PIN12                     (input floating).
 * PF13 - PIN13                     (input floating).
 * PF14 - PIN14                     (input floating).
 * PF15 - PIN15                     (input floating).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(GPIOF_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOF_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOF_PIN15))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOF_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN15))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_PIN0) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN1) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN2) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN3) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN4) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN5) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN6) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN7) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN8) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN9) |          \
                                     PIN_OSPEED_HIGH(GPIOF_PIN10) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN11) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN12) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN13) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN14) |         \
                                     PIN_OSPEED_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_PIN0) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN1) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN2) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN3) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN4) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN5) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN6) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN7) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN8) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN9) |       \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN10) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN11) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN12) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN13) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN14) |      \
                                     PIN_PUPDR_FLOATING(GPIOF_PIN15))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOF_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOF_PIN15))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN7, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_PIN15, 0U))

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

/*
 * USB bus activation/de-activation macro, required by the USB driver.
 */
#ifdef USB_DP_R_PA10			     
#define usb_lld_connect_bus(usbp) palSetPad(GPIOA, GPIOA_USB_DISC)
#define usb_lld_disconnect_bus(usbp) palClearPad(GPIO, GPIOA_USB_DISC)
#elifdef USB_DP_R_VDD
#define usb_lld_connect_bus(usbp) 
#define usb_lld_disconnect_bus(usbp)
#else // USB_DP connect to VDD by 1.5K R, and USB_DP short with PA10
#define usb_lld_connect_bus(usbp) palSetPadMode(GPIOA, GPIOA_USB_DISC, PAL_MODE_INPUT)
#define usb_lld_disconnect_bus(usbp) palClearPad(GPIOA, GPIOA_USB_DISC)
#endif

#endif /* BOARD_H */
