/*
 * Copyright (c) 2019-2020, Dmitry (DiSlord) dislordlive@gmail.com
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
// F072 Ext interrupts handler function
#if HAL_USE_EXT == FALSE
extern void handle_button_interrupt(uint16_t channel);

//#define EXT_CH0_HANDLER_FUNC
#define EXT_CH1_HANDLER_FUNC     handle_button_interrupt
#define EXT_CH2_HANDLER_FUNC     handle_button_interrupt
#define EXT_CH3_HANDLER_FUNC     handle_button_interrupt
//#define EXT_CH4_HANDLER_FUNC
//#define EXT_CH5_HANDLER_FUNC
//#define EXT_CH6_HANDLER_FUNC
//#define EXT_CH7_HANDLER_FUNC
//#define EXT_CH8_HANDLER_FUNC
//#define EXT_CH9_HANDLER_FUNC
//#define EXT_CH10_HANDLER_FUNC
//#define EXT_CH11_HANDLER_FUNC
//#define EXT_CH12_HANDLER_FUNC
//#define EXT_CH13_HANDLER_FUNC
//#define EXT_CH14_HANDLER_FUNC
//#define EXT_CH15_HANDLER_FUNC

#define STM32_EXT_EXTI0_IRQ_PRIORITY        6
#define STM32_EXT_EXTI1_IRQ_PRIORITY        6
#define STM32_EXT_EXTI2_IRQ_PRIORITY        6
#define STM32_EXT_EXTI3_IRQ_PRIORITY        6
#define STM32_EXT_EXTI4_IRQ_PRIORITY        6
#define STM32_EXT_EXTI5_9_IRQ_PRIORITY      6
#define STM32_EXT_EXTI10_15_IRQ_PRIORITY    6

#if defined(EXT_CH0_HANDLER_FUNC)
OSAL_IRQ_HANDLER(Vector58) {  // EXTI[0] interrupt handler.
  uint32_t pr = EXTI->PR & (1U << 0);
  EXTI->PR = pr;
#ifdef EXT_CH0_HANDLER_FUNC
  if (pr & (1U << 0)) EXT_CH0_HANDLER_FUNC(0);
#endif
}
#endif

#if defined(EXT_CH1_HANDLER_FUNC)
OSAL_IRQ_HANDLER(Vector5C) {  // EXTI[1] interrupt handler.
  uint32_t pr = EXTI->PR & (1U << 1);
  EXTI->PR = pr;
#ifdef EXT_CH1_HANDLER_FUNC
  if (pr & (1U << 1)) EXT_CH1_HANDLER_FUNC(1);
#endif
}
#endif

#if defined(EXT_CH2_HANDLER_FUNC)
OSAL_IRQ_HANDLER(Vector60) {  // EXTI[2] interrupt handler.
  uint32_t pr = EXTI->PR & (1U << 2);
  EXTI->PR = pr;
#ifdef EXT_CH2_HANDLER_FUNC
  if (pr & (1U << 2)) EXT_CH2_HANDLER_FUNC(2);
#endif
}
#endif

#if defined(EXT_CH3_HANDLER_FUNC)
OSAL_IRQ_HANDLER(Vector64) {  // EXTI[3] interrupt handler.
  uint32_t pr = EXTI->PR & (1U << 3);
  EXTI->PR = pr;
#ifdef EXT_CH3_HANDLER_FUNC
  if (pr & (1U << 3)) EXT_CH3_HANDLER_FUNC(3);
#endif
}
#endif

#if defined(EXT_CH4_HANDLER_FUNC)
OSAL_IRQ_HANDLER(Vector68) {  // EXTI[4] interrupt handler.
  uint32_t pr = EXTI->PR & (1U << 4);
  EXTI->PR = pr;
#ifdef EXT_CH4_HANDLER_FUNC
  if (pr & (1U << 4)) EXT_CH4_HANDLER_FUNC(4);
#endif
}
#endif


#if defined(EXT_CH5_HANDLER_FUNC) || defined(EXT_CH6_HANDLER_FUNC) || defined(EXT_CH7_HANDLER_FUNC) || \
    defined(EXT_CH8_HANDLER_FUNC) || defined(EXT_CH9_HANDLER_FUNC)
OSAL_IRQ_HANDLER(Vector9C) {  // EXTI[5]...EXTI[9] interrupt handler
  uint32_t pr = EXTI->PR & ((1U << 5) | (1U << 6) | (1U << 7) | (1U << 8) | (1U << 9));
#ifdef EXT_CH5_HANDLER_FUNC
  if (pr & (1U << 5)) EXT_CH5_HANDLER_FUNC(5);
#endif
#ifdef EXT_CH6_HANDLER_FUNC
  if (pr & (1U << 6)) EXT_CH6_HANDLER_FUNC(6);
#endif
#ifdef EXT_CH7_HANDLER_FUNC
  if (pr & (1U << 7)) EXT_CH7_HANDLER_FUNC(7);
#endif
#ifdef EXT_CH8_HANDLER_FUNC
  if (pr & (1U << 8)) EXT_CH8_HANDLER_FUNC(8);
#endif
#ifdef EXT_CH9_HANDLER_FUNC
  if (pr & (1U << 9)) EXT_CH9_HANDLER_FUNC(9);
#endif
}
#endif

#if defined(EXT_CH10_HANDLER_FUNC) || defined(EXT_CH11_HANDLER_FUNC) || defined(EXT_CH12_HANDLER_FUNC) || \
    defined(EXT_CH13_HANDLER_FUNC) || defined(EXT_CH14_HANDLER_FUNC) || defined(EXTI_CH15_HANDLER_FUNC)
OSAL_IRQ_HANDLER(VectorE0) {  // EXTI[4]...EXTI[15] interrupt handler
  uint32_t pr = EXTI->PR & ((1U << 10) | (1U << 11) | (1U << 12) |
                            (1U << 13) | (1U << 14) | (1U << 15));
#ifdef EXT_CH10_HANDLER_FUNC
  if (pr & (1U << 10)) EXT_CH10_HANDLER_FUNC(10);
#endif
#ifdef EXT_CH11_HANDLER_FUNC
  if (pr & (1U << 11)) EXT_CH11_HANDLER_FUNC(11);
#endif
#ifdef EXT_CH12_HANDLER_FUNC
  if (pr & (1U << 12)) EXT_CH12_HANDLER_FUNC(12);
#endif
#ifdef EXT_CH13_HANDLER_FUNC
  if (pr & (1U << 13)) EXT_CH13_HANDLER_FUNC(13);
#endif
#ifdef EXT_CH14_HANDLER_FUNC
  if (pr & (1U << 14)) EXT_CH14_HANDLER_FUNC(14);
#endif
#ifdef EXT_CH15_HANDLER_FUNC
  if (pr & (1U << 15)) EXT_CH15_HANDLER_FUNC(15);
#endif
}
#endif

void extStart(void) {
#ifdef EXT_CH0_HANDLER_FUNC
  nvicEnableVector(EXTI0_IRQn, STM32_EXT_EXTI0_IRQ_PRIORITY);
#endif
#ifdef EXT_CH1_HANDLER_FUNC
  nvicEnableVector(EXTI1_IRQn, STM32_EXT_EXTI1_IRQ_PRIORITY);
#endif
#ifdef EXT_CH2_HANDLER_FUNC
  nvicEnableVector(EXTI2_TSC_IRQn, STM32_EXT_EXTI2_IRQ_PRIORITY);
#endif
#ifdef EXT_CH3_HANDLER_FUNC
  nvicEnableVector(EXTI3_IRQn, STM32_EXT_EXTI3_IRQ_PRIORITY);
#endif
#ifdef EXT_CH4_HANDLER_FUNC
  nvicEnableVector(EXTI4_IRQn, STM32_EXT_EXTI4_IRQ_PRIORITY);
#endif
#if defined(EXT_CH5_HANDLER_FUNC) || defined(EXT_CH6_HANDLER_FUNC) || defined(EXT_CH7_HANDLER_FUNC) || \
    defined(EXT_CH8_HANDLER_FUNC) || defined(EXT_CH9_HANDLER_FUNC)
  nvicEnableVector(EXTI9_5_IRQn, STM32_EXT_EXTI5_9_IRQ_PRIORITY);
#endif
#if defined(EXT_CH10_HANDLER_FUNC) || defined(EXT_CH11_HANDLER_FUNC) || defined(EXT_CH12_HANDLER_FUNC) || \
    defined(EXT_CH13_HANDLER_FUNC) || defined(EXT_CH14_HANDLER_FUNC) || defined(EXTI_CH15_HANDLER_FUNC)
  nvicEnableVector(EXTI15_10_IRQn, STM32_EXT_EXTI10_15_IRQ_PRIORITY);
#endif
//  nvicEnableVector(PVD_IRQn, STM32_EXT_EXTI16_IRQ_PRIORITY);
//  nvicEnableVector(RTC_Alarm_IRQn, STM32_EXT_EXTI17_IRQ_PRIORITY);
}

void ext_channel_enable(uint16_t channel, uint16_t mode) {
  // Setting the associated GPIO for external channels.
  if (channel < 16) {
    uint16_t port =  (mode & EXT_MODE_GPIO_MASK) >> EXT_MODE_GPIO_OFF;
    uint32_t old_reg = SYSCFG->EXTICR[channel>>2] & ~(0xF << ((channel & 3) * 4));
    SYSCFG->EXTICR[channel>>2] = old_reg          |  (port<< ((channel & 3) * 4));
  }
  uint32_t cmask = (1 << (channel & 0x1F));
  // Programming edge registers.
  if (mode & EXT_CH_MODE_RISING_EDGE)  EXTI->RTSR|= cmask;
  else                                 EXTI->RTSR&=~cmask;
  if (mode & EXT_CH_MODE_FALLING_EDGE) EXTI->FTSR|= cmask;
  else                                 EXTI->FTSR&=~cmask;
  // Programming interrupt and event registers.
  EXTI->IMR|= cmask;
  EXTI->EMR&=~cmask;
}
#endif
