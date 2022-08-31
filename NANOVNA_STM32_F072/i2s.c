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

// F072 SPI2 RX DMA1 interrupt
#define STM32_SPI2_RX_DMA_IRQ_NUMBER           DMA1_Channel4_5_6_7_IRQn

#define SPI_I2S_FHILLIPS_MODE  (                   0)
#define SPI_I2S_MSB_MODE       (SPI_I2SCFGR_I2SSTD_0)
#define SPI_I2S_LSB_MODE       (SPI_I2SCFGR_I2SSTD_1)
#define SPI_I2S_PCM_MODE       (SPI_I2SCFGR_I2SSTD_0 | SPI_I2SCFGR_I2SSTD_1)

/*
 * Run I2S bus in Circular mode, fill buffer, and handle read in I2S DMA RX interrupt
 */
void initI2S(void *buffer, uint16_t count) {
  const uint16_t  I2S_DMA_RX_ccr = 0
    | STM32_DMA_CR_PL(3)       // 3 - Very High
    | STM32_DMA_CR_PSIZE_HWORD // 16 bit
    | STM32_DMA_CR_MSIZE_HWORD // 16 bit
    | STM32_DMA_CR_DIR_P2M     // Read from peripheral
    | STM32_DMA_CR_MINC        // Memory increment mode
    | STM32_DMA_CR_CIRC        // Circular mode
    | STM32_DMA_CR_HTIE        // Half transfer complete interrupt enable
    | STM32_DMA_CR_TCIE        // Full transfer complete interrupt enable
//  | STM32_DMA_CR_TEIE        // Transfer error interrupt enable
    ;
  // I2S RX DMA setup.
  nvicEnableVector(STM32_SPI2_RX_DMA_IRQ_NUMBER, STM32_I2S_SPI2_IRQ_PRIORITY);
  dmaChannelSetTransactionSize(I2S_DMA_RX, count);                 // number of data register
  dmaChannelSetPeripheral(I2S_DMA_RX, &SPI2->DR);                  // peripheral address register
  dmaChannelSetMemory(I2S_DMA_RX, buffer);                         // memory address register
  dmaChannelSetMode(I2S_DMA_RX, I2S_DMA_RX_ccr | STM32_DMA_CR_EN); // configuration register

  // Starting I2S
  rccEnableSPI2(FALSE);           // Enabling I2S unit clock.
  SPI2->CR1 = 0;                  // CRs settings
  SPI2->CR2 = SPI_CR2_RXDMAEN;    // Enable RX DMA
  SPI2->I2SPR   = 0;              // I2S (re)configuration.
  SPI2->I2SCFGR = 0
    | SPI_I2SCFGR_I2SCFG_0        // 01: Slave - receive
//  | SPI_I2SCFGR_I2SCFG_1        //
    | SPI_I2SCFGR_I2SMOD          // I2S mode is selected
    | SPI_I2S_PCM_MODE            // I2S PCM standard (aic3204 use DSP mode, short sync)
    | SPI_I2SCFGR_PCMSYNC         // Short sync
    | SPI_I2SCFGR_I2SE            // I2S enable
    ;
}
