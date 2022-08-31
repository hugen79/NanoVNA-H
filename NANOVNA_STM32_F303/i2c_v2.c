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

#define VNA_I2C                         I2C1
#define I2C_CR2_SADD_7BIT_SHIFT         1
#define I2C_CR2_NBYTES_SHIFT            16

void i2c_set_timings(uint32_t timings) {
  VNA_I2C->CR1&=~I2C_CR1_PE;
  VNA_I2C->TIMINGR = timings;
  VNA_I2C->CR1|= I2C_CR1_PE;
}

void i2c_start(void) {
  rccEnableI2C1(FALSE);
  i2c_set_timings(STM32_I2C_INIT_T);
}

// I2C TX only (compact version)
bool i2c_transfer(uint8_t addr, const uint8_t *w, size_t wn)
{
  //if (wn == 0) return false;
  while(VNA_I2C->ISR & I2C_ISR_BUSY); // wait last transaction
  VNA_I2C->CR1|= I2C_CR1_PE;
  VNA_I2C->CR2 = (addr << I2C_CR2_SADD_7BIT_SHIFT) | (wn << I2C_CR2_NBYTES_SHIFT) | I2C_CR2_AUTOEND | I2C_CR2_START;
  do {
    while ((VNA_I2C->ISR & (I2C_ISR_TXE|I2C_ISR_NACKF)) == 0);
    if (VNA_I2C->ISR & I2C_ISR_NACKF) {VNA_I2C->CR1 = 0; return false;}  // NO ASK error
    VNA_I2C->TXDR = *w++;
  } while (--wn);
  return true;
}

// I2C TX and RX variant
bool i2c_receive(uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn)
{
  VNA_I2C->CR1|= I2C_CR1_PE;
  if (wn) {
    VNA_I2C->CR2 = (addr << I2C_CR2_SADD_7BIT_SHIFT) | (wn << I2C_CR2_NBYTES_SHIFT);
    if (rn == 0) VNA_I2C->CR2|= I2C_CR2_AUTOEND;
    VNA_I2C->CR2|= I2C_CR2_START;
    do {
      while ((VNA_I2C->ISR & (I2C_ISR_TXE|I2C_ISR_NACKF)) == 0);
      if (VNA_I2C->ISR & I2C_ISR_NACKF) {VNA_I2C->CR1 = 0; return false;}  // NO ASK error
      VNA_I2C->TXDR = *w++;
    } while (--wn);
  }

  if (rn) {
    while ((VNA_I2C->ISR & I2C_ISR_TC) == 0); // wait transfer completion if need
    VNA_I2C->CR2 = (addr << I2C_CR2_SADD_7BIT_SHIFT) | (rn << I2C_CR2_NBYTES_SHIFT) | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND;   // start transfer
    //VNA_I2C->CR2|= I2C_CR2_AUTOEND; // important to do it afterwards to do a proper repeated start!
    do {
      while((VNA_I2C->ISR & (I2C_ISR_RXNE|I2C_ISR_NACKF)) == 0);
      if (VNA_I2C->ISR & I2C_ISR_NACKF) {VNA_I2C->CR1 = 0; return false;}  // NO ASK error
      *r++ = VNA_I2C->RXDR;
    } while (--rn);
  }
  return true;
}

