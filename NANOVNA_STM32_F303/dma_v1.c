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
// Add all handlers (for reset DMA interrupt)
//#define DMA1_USE_ALL_HANDLERS

// F303 DMA1 interrupts handler function
#if defined(DMA1_CH1_HANDLER_FUNC) || defined(DMA1_USE_ALL_HANDLERS)
OSAL_IRQ_HANDLER(STM32_DMA1_CH1_HANDLER) {
  uint32_t flags = DMA1->ISR; DMA1->IFCR = flags;  // reset interrupt vector
#ifdef DMA1_CH1_HANDLER_FUNC
  if (flags & (STM32_DMA_ISR_MASK<<0)) DMA1_CH1_HANDLER_FUNC((flags>>0)&STM32_DMA_ISR_MASK); // DMA Channel 1 handler
#endif
}
#endif

#if defined(DMA1_CH2_HANDLER_FUNC) || defined(DMA1_USE_ALL_HANDLERS)
OSAL_IRQ_HANDLER(STM32_DMA1_CH2_HANDLER) {
  uint32_t flags = DMA1->ISR; DMA1->IFCR = flags;  // reset interrupt vector
#ifdef DMA1_CH2_HANDLER_FUNC
  if (flags & (STM32_DMA_ISR_MASK<<4)) DMA1_CH2_HANDLER_FUNC((flags>>4)&STM32_DMA_ISR_MASK); // DMA Channel 2 handler
#endif
}
#endif

#if defined(DMA1_CH3_HANDLER_FUNC) || defined(DMA1_USE_ALL_HANDLERS)
OSAL_IRQ_HANDLER(STM32_DMA1_CH3_HANDLER) {
  uint32_t flags = DMA1->ISR; DMA1->IFCR = flags;  // reset interrupt vector
#ifdef DMA1_CH3_HANDLER_FUNC
  if (flags & (STM32_DMA_ISR_MASK<<8)) DMA1_CH3_HANDLER_FUNC((flags>>8)&STM32_DMA_ISR_MASK); // DMA Channel 3 handler
#endif
}
#endif

#if defined(DMA1_CH4_HANDLER_FUNC) || defined(DMA1_USE_ALL_HANDLERS)
OSAL_IRQ_HANDLER(STM32_DMA1_CH4_HANDLER) {
  uint32_t flags = DMA1->ISR; DMA1->IFCR = flags;  // reset interrupt vector
#ifdef DMA1_CH4_HANDLER_FUNC
  if (flags & (STM32_DMA_ISR_MASK<<12)) DMA1_CH4_HANDLER_FUNC((flags>>12)&STM32_DMA_ISR_MASK); // DMA Channel 4 handler
#endif
}
#endif

#if defined(DMA1_CH5_HANDLER_FUNC) || defined(DMA1_USE_ALL_HANDLERS)
OSAL_IRQ_HANDLER(STM32_DMA1_CH5_HANDLER) {
  uint32_t flags = DMA1->ISR; DMA1->IFCR = flags;  // reset interrupt vector
#ifdef DMA1_CH5_HANDLER_FUNC
  if (flags & (STM32_DMA_ISR_MASK<<16)) DMA1_CH5_HANDLER_FUNC((flags>>16)&STM32_DMA_ISR_MASK); // DMA Channel 5 handler
#endif
}
#endif

#if defined(DMA1_CH6_HANDLER_FUNC) || defined(DMA1_USE_ALL_HANDLERS)
OSAL_IRQ_HANDLER(STM32_DMA1_CH6_HANDLER) {
  uint32_t flags = DMA1->ISR; DMA1->IFCR = flags;  // reset interrupt vector
#ifdef DMA1_CH6_HANDLER_FUNC
  if (flags & (STM32_DMA_ISR_MASK<<20)) DMA1_CH6_HANDLER_FUNC((flags>>20)&STM32_DMA_ISR_MASK); // DMA Channel 6 handler
#endif
}
#endif

#if defined(DMA1_CH7_HANDLER_FUNC) || defined(DMA1_USE_ALL_HANDLERS)
OSAL_IRQ_HANDLER(STM32_DMA1_CH7_HANDLER) {
  uint32_t flags = DMA1->ISR; DMA1->IFCR = flags;  // reset interrupt vector
#ifdef DMA1_CH7_HANDLER_FUNC
  if (flags & (STM32_DMA_ISR_MASK<<24)) DMA1_CH7_HANDLER_FUNC((flags>>24)&STM32_DMA_ISR_MASK); // DMA Channel 7 handler
#endif
}
#endif
