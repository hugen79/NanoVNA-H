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

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for the Strawberry Linux STbee
 */

/*
 * Board identifier.
 */
#define BOARD_NANOVNA_STM32_F303
#define BOARD_NAME              "NanoVNA-H 4"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            8000000

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F303xC

#define STM32F303xC_SYSTEM_MEMORY 0x1FFFD800
#define BOOT_FROM_SYTEM_MEMORY_MAGIC_ADDRESS 0x20009FF0
#define BOOT_FROM_SYTEM_MEMORY_MAGIC 0xDEADBEEF
//#define SYSTEM_BOOT_MSP *(uint32_t *)0  // 0x20001258
#define SYSTEM_BOOT_MSP 0x20001258

/*
 * IO pins assignments
 */

/* on-board */

#define GPIOA_BUTTON			0
#define GPIOA_LEVER1			1
#define GPIOA_LEVER2			2
#define GPIOA_PUSH				3
#define GPIOA_VBUS			    4
#define GPIOA_DAC2				5
#define GPIOA_XP				6
#define GPIOA_YP				7
#define GPIOA_MCO				8
#define GPIOA_USART1_TX         9
#define GPIOA_USART1_RX	        10
#define GPIOA_USB_DM            11
#define GPIOA_USB_DP            12
#define GPIOA_JTMS              13
#define GPIOA_JTCK              14
#define GPIOA_LCD_RESET			15

#define GPIOB_XN                0
#define GPIOB_YN                1
#define GPIOB_SD_GP1            2
#define GPIOB_SPI_SCLK          3
#define GPIOB_SPI_MISO          4
#define GPIOB_SPI_MOSI          5
#define GPIOB_LCD_CS	        6
#define GPIOB_LCD_CD    	    7
#define GPIOB_I2C1_SCL          8
#define GPIOB_I2C1_SDA          9
#define GPIOB_SD_GP2			10
#define GPIOB_SD_CS				11
#define GPIOB_I2S2_WCLK         12
#define GPIOB_I2S2_BCLK         13
#define GPIOB_I2S2_MISO         14
#define GPIOB_I2S2_MOSI         15

#define GPIOC_LED               13
#define GPIOC_OSC32_IN          14
#define GPIOC_OSC32_OUT         15

#define GPIOF_OSC_IN            0
#define GPIOF_OSC_OUT           1


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
#define PIN_OSPEED_2M(n)            (0U << ((n) * 2U))
#define PIN_OSPEED_25M(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_50M(n)           (2U << ((n) * 2U))
#define PIN_OSPEED_100M(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v##U) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
 * PA0 - free
 * PA1 - button left                (input)
 * PA2 - button right               (input)
 * PA3 - button middle              (input)
 * PA4 - usb vbus (free)
 * PA5 - lcd backlight              (analog out)
 * PA6 - touch XP                   (analog in)
 * PA7 - touch YP                   (analog in)
 * PA8  - MCO                       (analog).
 * PA9  - USART1_TX                 (alternate 7).
 * PA10 - USART1_RX	                (alternate 7).
 * PA11 - USB_DM                    (alternate 14).
 * PA12 - USB_DP                    (alternate 14).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - LCD RESET                 (output)
 */
#define VAL_GPIOA_MODER             (PIN_MODE_INPUT(0U) |           \
                                     PIN_MODE_INPUT(1U) |           \
                                     PIN_MODE_INPUT(2U) |  			\
                                     PIN_MODE_INPUT(3U) |   		\
                                     PIN_MODE_INPUT(4U) |           \
                                     PIN_MODE_ANALOG(GPIOA_DAC2) |  \
                                     PIN_MODE_ANALOG(GPIOA_XP) |    \
                                     PIN_MODE_ANALOG(GPIOA_YP) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_MCO) | \
                                     PIN_MODE_ALTERNATE(GPIOA_USART1_TX) | \
                                     PIN_MODE_ALTERNATE(GPIOA_USART1_RX) | \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_DM) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_USB_DP) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_JTMS) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_JTCK) |    \
                                     PIN_MODE_OUTPUT(GPIOA_LCD_RESET))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(0U) |       \
                                     PIN_OTYPE_PUSHPULL(1U) |       \
                                     PIN_OTYPE_PUSHPULL(2U) |   \
                                     PIN_OTYPE_PUSHPULL(3U) |   \
                                     PIN_OTYPE_PUSHPULL(4U) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_DAC2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_XP) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_YP) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_MCO) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART1_TX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USART1_RX) | \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DM) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_USB_DP) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTMS) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_JTCK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_LCD_RESET))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_2M(0) |          \
                                     PIN_OSPEED_2M(1) |          \
                                     PIN_OSPEED_2M(2) |       \
                                     PIN_OSPEED_2M(3) |       \
                                     PIN_OSPEED_2M(4) |          \
                                     PIN_OSPEED_2M(GPIOA_DAC2) |           \
                                     PIN_OSPEED_2M(GPIOA_XP) |          \
                                     PIN_OSPEED_2M(GPIOA_YP) |          \
                                     PIN_OSPEED_100M(GPIOA_MCO) | \
                                     PIN_OSPEED_100M(GPIOA_USART1_TX) |  \
                                     PIN_OSPEED_100M(GPIOA_USART1_RX) |  \
                                     PIN_OSPEED_100M(GPIOA_USB_DM) |     \
                                     PIN_OSPEED_100M(GPIOA_USB_DP) |     \
                                     PIN_OSPEED_100M(GPIOA_JTMS) |         \
                                     PIN_OSPEED_100M(GPIOA_JTCK) |         \
                                     PIN_OSPEED_100M(GPIOA_LCD_RESET))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_PULLDOWN(0) | \
                                     PIN_PUPDR_PULLDOWN(1) | \
                                     PIN_PUPDR_PULLDOWN(2) | \
                                     PIN_PUPDR_PULLDOWN(3) | \
                                     PIN_PUPDR_PULLUP(4) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_DAC2) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_XP) |         \
                                     PIN_PUPDR_FLOATING(GPIOA_YP) |         \
                                     PIN_PUPDR_PULLUP(GPIOA_MCO) | \
                                     PIN_PUPDR_FLOATING(GPIOA_USART1_TX) | \
                                     PIN_PUPDR_FLOATING(GPIOA_USART1_RX) | \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_DM) | \
                                     PIN_PUPDR_FLOATING(GPIOA_USB_DP) | \
                                     PIN_PUPDR_PULLDOWN(GPIOA_JTMS) |   \
                                     PIN_PUPDR_PULLDOWN(GPIOA_JTCK) |   \
                                     PIN_PUPDR_PULLDOWN(GPIOA_LCD_RESET))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(0) |             \
                                     PIN_ODR_HIGH(1) |             \
                                     PIN_ODR_HIGH(2) |         \
                                     PIN_ODR_HIGH(3) |         \
                                     PIN_ODR_HIGH(4) |             \
                                     PIN_ODR_LOW(GPIOA_DAC2) |              \
                                     PIN_ODR_HIGH(GPIOA_XP) |             \
                                     PIN_ODR_HIGH(GPIOA_YP) |             \
                                     PIN_ODR_HIGH(GPIOA_MCO) |     \
                                     PIN_ODR_LOW(GPIOA_USART1_TX) | \
                                     PIN_ODR_LOW(GPIOA_USART1_RX) | \
                                     PIN_ODR_HIGH(GPIOA_USB_DM) |   \
                                     PIN_ODR_HIGH(GPIOA_USB_DP) |   \
                                     PIN_ODR_HIGH(GPIOA_JTMS) |     \
                                     PIN_ODR_HIGH(GPIOA_JTCK) |     \
                                     PIN_ODR_HIGH(GPIOA_LCD_RESET))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(0, 0) |           \
                                     PIN_AFIO_AF(1, 0) |           \
                                     PIN_AFIO_AF(2, 0) |       \
                                     PIN_AFIO_AF(3, 0) |       \
                                     PIN_AFIO_AF(4, 0) |           \
                                     PIN_AFIO_AF(GPIOA_DAC2, 0) |           \
                                     PIN_AFIO_AF(GPIOA_XP, 0) |           \
                                     PIN_AFIO_AF(GPIOA_YP, 0))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_MCO, 0) |           \
                                     PIN_AFIO_AF(GPIOA_USART1_TX, 7) |    \
                                     PIN_AFIO_AF(GPIOA_USART1_RX, 7) |    \
                                     PIN_AFIO_AF(GPIOA_USB_DM, 14) |     \
                                     PIN_AFIO_AF(GPIOA_USB_DP, 14) |     \
                                     PIN_AFIO_AF(GPIOA_JTMS, 0) |          \
                                     PIN_AFIO_AF(GPIOA_JTCK, 0) |          \
                                     PIN_AFIO_AF(GPIOA_LCD_RESET, 0))

/*
 * GPIOB setup:
 * PB0  - XN                        analog
 * PB1  - YN                        analog
 * PB3  - SPI1_SCLK                 (alternate 5).
 * PB2  - free (SD GP1)
 * PB4  - SPI1_MISO                 (alternate 5).
 * PB5  - SPI1_MOSI                 (alternate 5).
 * PB6  - LCD CS
 * PB7  - LCD CD
 * PB8  - I2C1_SCL                  (alternate 4).
 * PB9  - I2C1_SDA                  (alternate 4).
 * PB10 - free (SD GP2)
 * PB11 - SD CS
 * PB12 - I2S2_WCLK                 (alternate 5).
 * PB13 - I2S2_BCLK                 (alternate 5).
 * PB14 - free
 * PB15 - I2S2_MOSI                 (alternate 5).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ANALOG(GPIOB_XN)          | \
                                     PIN_MODE_ANALOG(GPIOB_YN)          | \
                                     PIN_MODE_OUTPUT(2)                 | \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI_SCLK) | \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI_MISO) | \
                                     PIN_MODE_ALTERNATE(GPIOB_SPI_MOSI) | \
                                     PIN_MODE_OUTPUT(GPIOB_LCD_CS)      | \
                                     PIN_MODE_OUTPUT(GPIOB_LCD_CD)      | \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SCL) | \
                                     PIN_MODE_ALTERNATE(GPIOB_I2C1_SDA) | \
                                     PIN_MODE_OUTPUT(10)                | \
                                     PIN_MODE_OUTPUT(GPIOB_SD_CS)       | \
                                     PIN_MODE_ALTERNATE(GPIOB_I2S2_WCLK)| \
                                     PIN_MODE_ALTERNATE(GPIOB_I2S2_BCLK)| \
                                     PIN_MODE_ALTERNATE(GPIOB_I2S2_MISO)| \
                                     PIN_MODE_ALTERNATE(GPIOB_I2S2_MOSI))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_XN)       | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_YN)       | \
                                     PIN_OTYPE_PUSHPULL(2)              | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI_SCLK) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI_MISO) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SPI_MOSI) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LCD_CS)   | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_LCD_CD)   | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_I2C1_SCL) | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_I2C1_SDA) | \
                                     PIN_OTYPE_PUSHPULL(10)             | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_SD_CS)    | \
                                     PIN_OTYPE_PUSHPULL(GPIOB_I2S2_WCLK)| \
                                     PIN_OTYPE_PUSHPULL(GPIOB_I2S2_BCLK)| \
                                     PIN_OTYPE_PUSHPULL(GPIOB_I2S2_MISO)| \
                                     PIN_OTYPE_PUSHPULL(GPIOB_I2S2_MOSI))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_2M(GPIOB_XN)         | \
                                     PIN_OSPEED_2M(GPIOB_YN)         | \
                                     PIN_OSPEED_100M(2)              | \
                                     PIN_OSPEED_100M(GPIOB_SPI_SCLK) | \
                                     PIN_OSPEED_100M(GPIOB_SPI_MISO) | \
                                     PIN_OSPEED_100M(GPIOB_SPI_MOSI) | \
                                     PIN_OSPEED_100M(GPIOB_LCD_CS)   | \
                                     PIN_OSPEED_100M(GPIOB_LCD_CD)   | \
                                     PIN_OSPEED_100M(GPIOB_I2C1_SCL) | \
                                     PIN_OSPEED_100M(GPIOB_I2C1_SDA) | \
                                     PIN_OSPEED_100M(10)             | \
                                     PIN_OSPEED_100M(GPIOB_SD_CS)    | \
                                     PIN_OSPEED_100M(GPIOB_I2S2_WCLK)| \
                                     PIN_OSPEED_100M(GPIOB_I2S2_BCLK)| \
                                     PIN_OSPEED_100M(GPIOB_I2S2_MISO)| \
                                     PIN_OSPEED_100M(GPIOB_I2S2_MOSI))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_XN)     | \
                                     PIN_PUPDR_FLOATING(GPIOB_YN)     | \
                                     PIN_PUPDR_PULLUP(2)              | \
                                     PIN_PUPDR_PULLUP(GPIOB_SPI_SCLK) | \
                                     PIN_PUPDR_PULLUP(GPIOB_SPI_MISO) | \
                                     PIN_PUPDR_PULLUP(GPIOB_SPI_MOSI) | \
                                     PIN_PUPDR_PULLUP(GPIOB_LCD_CS)   | \
                                     PIN_PUPDR_PULLUP(GPIOB_LCD_CD)   | \
                                     PIN_PUPDR_PULLUP(GPIOB_I2C1_SCL) | \
                                     PIN_PUPDR_PULLUP(GPIOB_I2C1_SDA) | \
                                     PIN_PUPDR_PULLUP(10)             | \
                                     PIN_PUPDR_PULLUP(GPIOB_SD_CS)    | \
                                     PIN_PUPDR_PULLUP(GPIOB_I2S2_WCLK)| \
                                     PIN_PUPDR_PULLUP(GPIOB_I2S2_BCLK)| \
                                     PIN_PUPDR_PULLUP(GPIOB_I2S2_MISO)| \
                                     PIN_PUPDR_PULLUP(GPIOB_I2S2_MOSI))
#define VAL_GPIOB_ODR               (PIN_ODR_HIGH(GPIOB_XN)       | \
                                     PIN_ODR_HIGH(GPIOB_YN)       | \
                                     PIN_ODR_HIGH(2)              | \
                                     PIN_ODR_HIGH(GPIOB_SPI_SCLK) | \
                                     PIN_ODR_HIGH(GPIOB_SPI_MISO) | \
                                     PIN_ODR_HIGH(GPIOB_SPI_MOSI) | \
                                     PIN_ODR_HIGH(GPIOB_LCD_CS)   | \
                                     PIN_ODR_HIGH(GPIOB_LCD_CD)   | \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SCL) | \
                                     PIN_ODR_HIGH(GPIOB_I2C1_SDA) | \
                                     PIN_ODR_HIGH(10) |             \
                                     PIN_ODR_HIGH(GPIOB_SD_CS)    | \
                                     PIN_ODR_HIGH(GPIOB_I2S2_WCLK)| \
                                     PIN_ODR_HIGH(GPIOB_I2S2_BCLK)| \
                                     PIN_ODR_HIGH(GPIOB_I2S2_MISO)| \
                                     PIN_ODR_HIGH(GPIOB_I2S2_MOSI))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_XN, 0)       | \
                                     PIN_AFIO_AF(GPIOB_YN, 0)       | \
                                     PIN_AFIO_AF(2, 0)              | \
                                     PIN_AFIO_AF(GPIOB_SPI_SCLK, 5) | \
                                     PIN_AFIO_AF(GPIOB_SPI_MISO, 5) | \
                                     PIN_AFIO_AF(GPIOB_SPI_MOSI, 5) | \
                                     PIN_AFIO_AF(GPIOB_LCD_CS,   0) | \
                                     PIN_AFIO_AF(GPIOB_LCD_CD,   0))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_I2C1_SCL, 4) | \
                                     PIN_AFIO_AF(GPIOB_I2C1_SDA, 4) | \
                                     PIN_AFIO_AF(10, 0)             | \
                                     PIN_AFIO_AF(GPIOB_SD_CS,     0)| \
                                     PIN_AFIO_AF(GPIOB_I2S2_WCLK, 5)| \
                                     PIN_AFIO_AF(GPIOB_I2S2_BCLK, 5)| \
                                     PIN_AFIO_AF(GPIOB_I2S2_MISO, 0)| \
                                     PIN_AFIO_AF(GPIOB_I2S2_MOSI, 5))
/*
 * GPIOC setup:
 *
 * PC13 - LED                   (output pushpull maximum).
 * PC14 - OSC32_IN              (input pushpull maximum).
 * PC15 - OSC32_OUT             (input pushpull maximum).
 */
#define GPIOC_LED               13

#define VAL_GPIOC_MODER             (PIN_MODE_INPUT(0) |           \
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
                                     PIN_MODE_OUTPUT(GPIOC_LED) | \
                                     PIN_MODE_INPUT(GPIOC_OSC32_IN) | \
                                     PIN_MODE_INPUT(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(0) |       \
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
                                     PIN_OTYPE_PUSHPULL(GPIOC_LED) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_IN) | \
                                     PIN_OTYPE_PUSHPULL(GPIOC_OSC32_OUT))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_100M(0) |          \
                                     PIN_OSPEED_100M(1) |          \
                                     PIN_OSPEED_100M(2) |          \
                                     PIN_OSPEED_100M(3) |          \
                                     PIN_OSPEED_100M(4) |          \
                                     PIN_OSPEED_100M(5) |          \
                                     PIN_OSPEED_100M(6) |          \
                                     PIN_OSPEED_100M(7) |          \
                                     PIN_OSPEED_100M(8) |          \
                                     PIN_OSPEED_100M(9) |          \
                                     PIN_OSPEED_100M(10) |         \
                                     PIN_OSPEED_100M(11) |         \
                                     PIN_OSPEED_100M(12) |         \
                                     PIN_OSPEED_100M(GPIOC_LED) | \
                                     PIN_OSPEED_100M(GPIOC_OSC32_IN) | \
                                     PIN_OSPEED_100M(GPIOC_OSC32_OUT))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_PULLUP(0) |         \
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
                                     PIN_PUPDR_FLOATING(GPIOC_LED) | \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_IN) |        \
                                     PIN_PUPDR_FLOATING(GPIOC_OSC32_OUT))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(0) |             \
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
                                     PIN_ODR_HIGH(GPIOC_LED) | \
                                     PIN_ODR_HIGH(GPIOC_OSC32_IN) | \
                                     PIN_ODR_HIGH(GPIOC_OSC32_OUT))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(0, 0) |           \
                                     PIN_AFIO_AF(1, 0) |           \
                                     PIN_AFIO_AF(2, 0) |           \
                                     PIN_AFIO_AF(3, 0) |           \
                                     PIN_AFIO_AF(4, 0) |           \
                                     PIN_AFIO_AF(5, 0) |           \
                                     PIN_AFIO_AF(6, 0) |           \
                                     PIN_AFIO_AF(7, 0))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(8, 0) |           \
                                     PIN_AFIO_AF(9, 0) |           \
                                     PIN_AFIO_AF(10, 0) |          \
                                     PIN_AFIO_AF(11, 0) |          \
                                     PIN_AFIO_AF(12, 0) |          \
                                     PIN_AFIO_AF(GPIOC_LED, 0) | \
                                     PIN_AFIO_AF(GPIOC_OSC32_IN, 0) | \
                                     PIN_AFIO_AF(GPIOC_OSC32_OUT, 0))

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
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_100M(0) |          \
                                     PIN_OSPEED_100M(1) |          \
                                     PIN_OSPEED_100M(2) |          \
                                     PIN_OSPEED_100M(3) |          \
                                     PIN_OSPEED_100M(4) |          \
                                     PIN_OSPEED_100M(5) |          \
                                     PIN_OSPEED_100M(6) |          \
                                     PIN_OSPEED_100M(7) |          \
                                     PIN_OSPEED_100M(8) |          \
                                     PIN_OSPEED_100M(9) |          \
                                     PIN_OSPEED_100M(10) |         \
                                     PIN_OSPEED_100M(11) |         \
                                     PIN_OSPEED_100M(12) |         \
                                     PIN_OSPEED_100M(13) |         \
                                     PIN_OSPEED_100M(14) |         \
                                     PIN_OSPEED_100M(15))
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
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_100M(0) |          \
                                     PIN_OSPEED_100M(1) |          \
                                     PIN_OSPEED_100M(2) |          \
                                     PIN_OSPEED_100M(3) |          \
                                     PIN_OSPEED_100M(4) |          \
                                     PIN_OSPEED_100M(5) |          \
                                     PIN_OSPEED_100M(6) |          \
                                     PIN_OSPEED_100M(7) |          \
                                     PIN_OSPEED_100M(8) |          \
                                     PIN_OSPEED_100M(9) |          \
                                     PIN_OSPEED_100M(10) |         \
                                     PIN_OSPEED_100M(11) |         \
                                     PIN_OSPEED_100M(12) |         \
                                     PIN_OSPEED_100M(13) |         \
                                     PIN_OSPEED_100M(14) |         \
                                     PIN_OSPEED_100M(15))
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
 */
#define VAL_GPIOF_MODER             (PIN_MODE_INPUT(0) |           \
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
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_PUSHPULL(0) |       \
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
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_100M(0) |          \
                                     PIN_OSPEED_100M(1) |          \
                                     PIN_OSPEED_100M(2) |          \
                                     PIN_OSPEED_100M(3) |          \
                                     PIN_OSPEED_100M(4) |          \
                                     PIN_OSPEED_100M(5) |          \
                                     PIN_OSPEED_100M(6) |          \
                                     PIN_OSPEED_100M(7) |          \
                                     PIN_OSPEED_100M(8) |          \
                                     PIN_OSPEED_100M(9) |          \
                                     PIN_OSPEED_100M(10) |         \
                                     PIN_OSPEED_100M(11) |         \
                                     PIN_OSPEED_100M(12) |         \
                                     PIN_OSPEED_100M(13) |         \
                                     PIN_OSPEED_100M(14) |         \
                                     PIN_OSPEED_100M(15))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(0) |       \
                                     PIN_PUPDR_FLOATING(1) |       \
                                     PIN_PUPDR_FLOATING(2) |       \
                                     PIN_PUPDR_FLOATING(3) |       \
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
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(0) |             \
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
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(0, 0) |           \
                                     PIN_AFIO_AF(1, 0) |           \
                                     PIN_AFIO_AF(2, 0) |           \
                                     PIN_AFIO_AF(3, 0) |           \
                                     PIN_AFIO_AF(4, 0) |           \
                                     PIN_AFIO_AF(5, 0) |           \
                                     PIN_AFIO_AF(6, 0) |           \
                                     PIN_AFIO_AF(7, 0))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(8, 0) |           \
                                     PIN_AFIO_AF(9, 0) |           \
                                     PIN_AFIO_AF(10, 0) |          \
                                     PIN_AFIO_AF(11, 0) |          \
                                     PIN_AFIO_AF(12, 0) |          \
                                     PIN_AFIO_AF(13, 0) |          \
                                     PIN_AFIO_AF(14, 0) |          \
                                     PIN_AFIO_AF(15, 0))

#define USB_DP_R_VDD

#ifdef NANOVNA_F303
/*
 * USB bus activation/de-activation macro, required by the USB driver.
 */
#ifdef USB_DP_R_PA10
#define usb_lld_connect_bus(usbp) palSetPad(GPIOA, GPIOA_USB_DISC)
#define usb_lld_disconnect_bus(usbp) palClearPad(GPIO, GPIOA_USB_DISC)
#endif

#ifdef  USB_DP_R_VDD
#define usb_lld_connect_bus(usbp)    {palSetPadMode(GPIOA, GPIOA_USB_DP, PAL_MODE_ALTERNATE(14));}
#define usb_lld_disconnect_bus(usbp) {palSetPadMode(GPIOA, GPIOA_USB_DP, PAL_STM32_MODE_OUTPUT); palClearPad(GPIOA, GPIOA_USB_DP);}
#else // USB_DP connect to VDD by 1.5K R, and USB_DP short with PA10
#define usb_lld_connect_bus(usbp) palSetPadMode(GPIOA, GPIOA_USB_DISC, PAL_MODE_INPUT)
#define usb_lld_disconnect_bus(usbp) palClearPad(GPIOA, GPIOA_USB_DISC)
#endif
#endif

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
