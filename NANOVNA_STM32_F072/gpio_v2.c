/*
 * Copyright (c) 2019-2020, Dmitry (DiSlord) dislordlive@gmail.com
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

typedef struct {
  uint32_t              moder;
  uint32_t              otyper;
  uint32_t              ospeedr;
  uint32_t              pupdr;
  uint32_t              odr;
  uint32_t              afrl;
  uint32_t              afrh;
} pal_conf_t;

const pal_conf_t pal_config[] = {
#if STM32_HAS_GPIOA
  {VAL_GPIOA_MODER, VAL_GPIOA_OTYPER, VAL_GPIOA_OSPEEDR, VAL_GPIOA_PUPDR, VAL_GPIOA_ODR,   VAL_GPIOA_AFRL,   VAL_GPIOA_AFRH},
#endif
#if STM32_HAS_GPIOB
  {VAL_GPIOB_MODER, VAL_GPIOB_OTYPER, VAL_GPIOB_OSPEEDR, VAL_GPIOB_PUPDR, VAL_GPIOB_ODR,   VAL_GPIOB_AFRL,   VAL_GPIOB_AFRH},
#endif
#if STM32_HAS_GPIOC
  {VAL_GPIOC_MODER, VAL_GPIOC_OTYPER, VAL_GPIOC_OSPEEDR, VAL_GPIOC_PUPDR, VAL_GPIOC_ODR,   VAL_GPIOC_AFRL,   VAL_GPIOC_AFRH},
#endif
/*
#if STM32_HAS_GPIOD
  {VAL_GPIOD_MODER, VAL_GPIOD_OTYPER, VAL_GPIOD_OSPEEDR, VAL_GPIOD_PUPDR, VAL_GPIOD_ODR,   VAL_GPIOD_AFRL,   VAL_GPIOD_AFRH},
#endif
#if STM32_HAS_GPIOE
  {VAL_GPIOE_MODER, VAL_GPIOE_OTYPER, VAL_GPIOE_OSPEEDR, VAL_GPIOE_PUPDR, VAL_GPIOE_ODR,   VAL_GPIOE_AFRL,   VAL_GPIOE_AFRH},
#endif
#if STM32_HAS_GPIOF
  {VAL_GPIOF_MODER, VAL_GPIOF_OTYPER, VAL_GPIOF_OSPEEDR, VAL_GPIOF_PUPDR, VAL_GPIOF_ODR,   VAL_GPIOF_AFRL,   VAL_GPIOF_AFRH},
#endif
#if STM32_HAS_GPIOG
  {VAL_GPIOG_MODER, VAL_GPIOG_OTYPER, VAL_GPIOG_OSPEEDR, VAL_GPIOG_PUPDR, VAL_GPIOG_ODR,   VAL_GPIOG_AFRL,   VAL_GPIOG_AFRH},
#endif
#if STM32_HAS_GPIOH
  {VAL_GPIOH_MODER, VAL_GPIOH_OTYPER, VAL_GPIOH_OSPEEDR, VAL_GPIOH_PUPDR, VAL_GPIOH_ODR,   VAL_GPIOH_AFRL,   VAL_GPIOH_AFRH},
#endif
#if STM32_HAS_GPIOI
  {VAL_GPIOI_MODER, VAL_GPIOI_OTYPER, VAL_GPIOI_OSPEEDR, VAL_GPIOI_PUPDR, VAL_GPIOI_ODR,   VAL_GPIOI_AFRL,   VAL_GPIOI_AFRH}
#endif
*/
};

static void initgpio(GPIO_TypeDef *port, const pal_conf_t *config) {
  port->OTYPER  = config->otyper;
  port->OSPEEDR = config->ospeedr;
  port->PUPDR   = config->pupdr;
  port->ODR     = config->odr;
  port->AFR[0]  = config->afrl;
  port->AFR[1]  = config->afrh;
  port->MODER   = config->moder;
}

void initPal(void) {
  rccEnableAHB(STM32_GPIO_EN_MASK, FALSE);
  initgpio(GPIOA, &pal_config[0]);
  initgpio(GPIOB, &pal_config[1]);
  initgpio(GPIOC, &pal_config[2]);
//  initgpio(GPIOD, &pal_config[3]);
}

void palSetPadGroupMode(GPIO_TypeDef *port, uint32_t mask, uint32_t mode) {
  uint32_t moder   = (mode & PAL_STM32_MODE_MASK) >> 0;
  uint32_t otyper  = (mode & PAL_STM32_OTYPE_MASK) >> 2;
  uint32_t ospeedr = (mode & PAL_STM32_OSPEED_MASK) >> 3;
  uint32_t pupdr   = (mode & PAL_STM32_PUPDR_MASK) >> 5;
  uint32_t altr    = (mode & PAL_STM32_ALTERNATE_MASK) >> 7;
  uint32_t bit     = 0;
  while (true) {
    if ((mask & 1) != 0) {
      uint32_t altrmask, m1, m2, m4;

      altrmask = altr << ((bit & 7) * 4);
      m1 = 1 << bit;
      m2 = 3 << (bit * 2);
      m4 = 15 << ((bit & 7) * 4);
      port->OTYPER  = (port->OTYPER  & ~m1) | otyper;
      port->OSPEEDR = (port->OSPEEDR & ~m2) | ospeedr;
      port->PUPDR   = (port->PUPDR   & ~m2) | pupdr;
      if (moder == PAL_STM32_MODE_ALTERNATE) {
        // If going in alternate mode then the alternate number is set before switching mode in order to avoid glitches.
        port->AFR[(bit>>3)&1] = (port->AFR[(bit>>3)&1] & ~m4) | altrmask;
        port->MODER   = (port->MODER & ~m2) | moder;
      }
      else {
        // If going into a non-alternate mode then the mode is switched before setting the alternate mode in order to avoid glitches.
        port->MODER   = (port->MODER & ~m2) | moder;
        port->AFR[(bit>>3)&1] = (port->AFR[(bit>>3)&1] & ~m4) | altrmask;
      }
    }
    mask >>= 1;
    if (!mask)
      return;
    otyper <<= 1;
    ospeedr <<= 2;
    pupdr <<= 2;
    moder <<= 2;
    bit++;
  }
}

void palSetPadMode(GPIO_TypeDef *port, int bit, uint32_t mode) {
  uint32_t otyper  = ((mode & PAL_STM32_OTYPE_MASK)  >> 2)<<(bit*1);
  uint32_t ospeedr = ((mode & PAL_STM32_OSPEED_MASK) >> 3)<<(bit*2);
  uint32_t pupdr   = ((mode & PAL_STM32_PUPDR_MASK)  >> 5)<<(bit*2);
  uint32_t moder   = ((mode & PAL_STM32_MODE_MASK)   >> 0)<<(bit*2);
  uint32_t m1 = 0x1 << (bit * 1);
  uint32_t m2 = 0x3 << (bit * 2);

  port->OTYPER  = (port->OTYPER  & ~m1) | otyper;
  port->OSPEEDR = (port->OSPEEDR & ~m2) | ospeedr;
  port->PUPDR   = (port->PUPDR   & ~m2) | pupdr;
#if 0
  port->MODER   = (port->MODER & ~m2) | moder;
#else
  uint32_t altrmask = ((mode & PAL_STM32_ALTERNATE_MASK) >> 7) << ((bit&7) * 4);
  uint32_t m4       = 0xF << ((bit&7) * 4);
  if (moder == PAL_STM32_MODE_ALTERNATE) {
    // If going in alternate mode then the alternate number is set before switching mode in order to avoid glitches.
    port->AFR[(bit>>3)&1] = (port->AFR[(bit>>3)&1] & ~m4) | altrmask;
    port->MODER = (port->MODER   & ~m2) | moder;
  }
  else {
    // If going into a non-alternate mode then the mode is switched before setting the alternate mode in order to avoid glitches.
    port->MODER = (port->MODER   & ~m2) | moder;
    port->AFR[(bit>>3)&1] = (port->AFR[(bit>>3)&1] & ~m4) | altrmask;
  }
#endif
}
