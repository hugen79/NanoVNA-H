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

#include "hal.h"

#if HAL_USE_PAL || defined(__DOXYGEN__)
/**
 * @brief   PAL setup.
 * @details Digital I/O ports static configuration as defined in @p board.h.
 *          This variable is used by the HAL when initializing the PAL driver.
 */
const PALConfig pal_default_config = {
#if STM32_HAS_GPIOA
  {VAL_GPIOA_MODER, VAL_GPIOA_OTYPER, VAL_GPIOA_OSPEEDR, VAL_GPIOA_PUPDR,
   VAL_GPIOA_ODR,   VAL_GPIOA_AFRL,   VAL_GPIOA_AFRH},
#endif
#if STM32_HAS_GPIOB
  {VAL_GPIOB_MODER, VAL_GPIOB_OTYPER, VAL_GPIOB_OSPEEDR, VAL_GPIOB_PUPDR,
   VAL_GPIOB_ODR,   VAL_GPIOB_AFRL,   VAL_GPIOB_AFRH},
#endif
#if STM32_HAS_GPIOC
  {VAL_GPIOC_MODER, VAL_GPIOC_OTYPER, VAL_GPIOC_OSPEEDR, VAL_GPIOC_PUPDR,
   VAL_GPIOC_ODR,   VAL_GPIOC_AFRL,   VAL_GPIOC_AFRH},
#endif
#if STM32_HAS_GPIOD
  {VAL_GPIOD_MODER, VAL_GPIOD_OTYPER, VAL_GPIOD_OSPEEDR, VAL_GPIOD_PUPDR,
   VAL_GPIOD_ODR,   VAL_GPIOD_AFRL,   VAL_GPIOD_AFRH},
#endif
#if STM32_HAS_GPIOE
  {VAL_GPIOE_MODER, VAL_GPIOE_OTYPER, VAL_GPIOE_OSPEEDR, VAL_GPIOE_PUPDR,
   VAL_GPIOE_ODR,   VAL_GPIOE_AFRL,   VAL_GPIOE_AFRH},
#endif
#if STM32_HAS_GPIOF
  {VAL_GPIOF_MODER, VAL_GPIOF_OTYPER, VAL_GPIOF_OSPEEDR, VAL_GPIOF_PUPDR,
   VAL_GPIOF_ODR,   VAL_GPIOF_AFRL,   VAL_GPIOF_AFRH},
#endif
#if STM32_HAS_GPIOG
  {VAL_GPIOG_MODER, VAL_GPIOG_OTYPER, VAL_GPIOG_OSPEEDR, VAL_GPIOG_PUPDR,
   VAL_GPIOG_ODR,   VAL_GPIOG_AFRL,   VAL_GPIOG_AFRH},
#endif
#if STM32_HAS_GPIOH
  {VAL_GPIOH_MODER, VAL_GPIOH_OTYPER, VAL_GPIOH_OSPEEDR, VAL_GPIOH_PUPDR,
   VAL_GPIOH_ODR,   VAL_GPIOH_AFRL,   VAL_GPIOH_AFRH},
#endif
#if STM32_HAS_GPIOI
  {VAL_GPIOI_MODER, VAL_GPIOI_OTYPER, VAL_GPIOI_OSPEEDR, VAL_GPIOI_PUPDR,
   VAL_GPIOI_ODR,   VAL_GPIOI_AFRL,   VAL_GPIOI_AFRH}
#endif
};
#endif

/*
 * Early initialization code.
 * This initialization must be performed just after stack setup and before
 * any other initialization.
 */
void __early_init(void) {
  // Refer to thess pages for how to start dfu from software
  // https://community.st.com/s/question/0D50X00009XkeeWSAR/stm32l476rg-jump-to-bootloader-from-software
  // https://stm32f4-discovery.net/2017/04/tutorial-jump-system-memory-software-stm32/
#if 0
  if ( *((unsigned long *)BOOT_FROM_SYTEM_MEMORY_MAGIC_ADDRESS) == BOOT_FROM_SYTEM_MEMORY_MAGIC ) {
    // require irq
    // __enable_irq();
    // reset magic bytes
    *((unsigned long *)BOOT_FROM_SYTEM_MEMORY_MAGIC_ADDRESS) = 0;
   #if 1
    // https://stm32f4-discovery.net/2017/04/tutorial-jump-system-memory-software-stm32/
    // Step: Disable systick timer and reset it to default values
    #if 0
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;
    #endif
    // Step: Disable all interrupts
    __disable_irq();
    // Remap system memory to address 0x0000 0000 in address space
    typedef void (*pFunction)(void);
    pFunction bootloader;
    uint32_t msp;
    uint32_t foo = SYSCFG->CFGR1;
    foo = (foo & ~SYSCFG_CFGR1_MEM_MODE) || SYSCFG_CFGR1_MEM_MODE_0;
    SYSCFG->CFGR1 = foo;
    //foo = SYSCFG->CFGR1;
    __DSB();
    __ISB();
    //__DSB();
    //__ISB();
    #if 1
    bootloader = (void (*)(void)) (*((uint32_t *)(STM32F303xC_SYSTEM_MEMORY+4)));
    //msp = *(uint32_t *) STM32F303xC_SYSTEM_MEMORY;
    msp = 0x20002250;
    #else
    bootloader = (void (*)(void)) (*((uint32_t *)(4)));
    //msp = *(uint32_t *) 0;
    msp = 0x20002250;
    #endif
    __set_MSP(msp);
    bootloader();
    while(1);
   #else
    __set_MSP(SYSTEM_BOOT_MSP); 
    ( (void (*)(void)) (*((uint32_t *)(STM32F303xC_SYSTEM_MEMORY+4))) )();
    while(1);
   #endif
  }
#endif

  stm32_clock_init();
}

/*
 * Board-specific initialization code.
 */
void boardInit(void) {
  // Speedup flash latency
  FLASH->ACR= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_0;
}
