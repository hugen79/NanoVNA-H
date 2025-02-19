/*
 * Copyright (c) 2019-2024, Dmitry (DiSlord) dislordlive@gmail.com
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
#include "ch.h"
#include "hal.h"
#include "nanovna.h"
#include "chprintf.h"
#include "spi.h"

// Pin macros for LCD
#define LCD_CS_LOW        palClearPad(GPIOB, GPIOB_LCD_CS)
#define LCD_CS_HIGH       palSetPad(GPIOB, GPIOB_LCD_CS)
#define LCD_RESET_ASSERT  palClearPad(GPIOA, GPIOA_LCD_RESET)
#define LCD_RESET_NEGATE  palSetPad(GPIOA, GPIOA_LCD_RESET)
#define LCD_DC_CMD        palClearPad(GPIOB, GPIOB_LCD_CD)
#define LCD_DC_DATA       palSetPad(GPIOB, GPIOB_LCD_CD)

// SPI bus for LCD
#define LCD_SPI           SPI1
#ifdef __USE_DISPLAY_DMA__
// DMA channels for used in LCD SPI bus
#define LCD_DMA_RX        DMA1_Channel2    // DMA1 channel 2 use for SPI1 rx
#define LCD_DMA_TX        DMA1_Channel3    // DMA1 channel 3 use for SPI1 tx
#endif

// Custom display definition
#if defined(LCD_DRIVER_ILI9341) || defined(LCD_DRIVER_ST7789)
 // Set SPI bus speed for LCD
 #define LCD_SPI_SPEED            SPI_BR_DIV2
 // Read speed, need more slow, not define if need use some as Tx speed
 #define ILI9341_SPI_RX_SPEED     SPI_BR_DIV2
 // Read speed, need more slow, not define if need use some as Tx speed
 #define ST7789V_SPI_RX_SPEED     SPI_BR_DIV8
 // Allow enable DMA for read display data (can not stable on full speed, on less speed slower)
 #define __USE_DISPLAY_DMA_RX__
#elif defined(LCD_DRIVER_ST7796S)
 // Set SPI bus speed for LCD
 #define LCD_SPI_SPEED    SPI_BR_DIV2
 // Read speed, need more slow, not define if need use some as Tx speed
 #define LCD_SPI_RX_SPEED SPI_BR_DIV4
 // Allow enable DMA for read display data
 #define __USE_DISPLAY_DMA_RX__
#endif

// Disable DMA rx on disabled DMA tx
#ifndef __USE_DISPLAY_DMA__
 #undef __USE_DISPLAY_DMA_RX__
#endif

// LCD display buffer
pixel_t spi_buffer[SPI_BUFFER_SIZE];
// Default foreground & background colors
pixel_t foreground_color = 0;
pixel_t background_color = 0;

//*****************************************************
// SPI functions, settings and data
//*****************************************************
void spi_TxByte(const uint8_t data) {
  while (SPI_TX_IS_NOT_EMPTY(LCD_SPI));
  SPI_WRITE_8BIT(LCD_SPI, data);
}
// Transmit buffer to SPI bus  (len should be > 0)
void spi_TxBuffer(const uint8_t *buffer, uint16_t len) {
  while(len--) {
    while (SPI_TX_IS_NOT_EMPTY(LCD_SPI));
    SPI_WRITE_8BIT(LCD_SPI, *buffer++);
  }
}

// Receive byte from SPI bus
uint8_t spi_RxByte(void) {
  // Start RX clock (by sending data)
  SPI_WRITE_8BIT(LCD_SPI, 0xFF);
  while (SPI_RX_IS_EMPTY(LCD_SPI));
  return SPI_READ_8BIT(LCD_SPI);
}

// Receive buffer from SPI bus (len should be > 0)
void spi_RxBuffer(uint8_t *buffer, uint16_t len) {
  do{
    SPI_WRITE_8BIT(LCD_SPI, 0xFF);
    while (SPI_RX_IS_EMPTY(LCD_SPI));
    *buffer++ = SPI_READ_8BIT(LCD_SPI);
  }while(--len);
}

void spi_DropRx(void) {
  // Drop Rx buffer after tx and wait tx complete
#if 1
  while (SPI_RX_IS_NOT_EMPTY(LCD_SPI)||SPI_IS_BUSY(LCD_SPI))
    (void)SPI_READ_8BIT(LCD_SPI);
  (void)SPI_READ_8BIT(LCD_SPI);
#else
  while(SPI_IS_BUSY(LCD_SPI));
  (void)SPI_READ_16BIT(LCD_SPI);
  (void)SPI_READ_16BIT(LCD_SPI);
#endif
}

//*****************************************************
// SPI DMA settings and data
//*****************************************************
#ifdef __USE_DISPLAY_DMA__
static const uint32_t txdmamode = 0
    | STM32_DMA_CR_PL(STM32_SPI_SPI1_DMA_PRIORITY)  // Set priority
    | STM32_DMA_CR_DIR_M2P;                         // Memory to Spi

static const uint32_t rxdmamode = 0
    | STM32_DMA_CR_PL(STM32_SPI_SPI1_DMA_PRIORITY)  // Set priority
    | STM32_DMA_CR_DIR_P2M;                         // SPI to Memory

// SPI transmit byte buffer use DMA (65535 bytes limit)
static inline void spi_DMATxBuffer(const uint8_t *buffer, uint16_t len, bool wait) {
  dmaChannelSetMemory(LCD_DMA_TX, buffer);
  dmaChannelSetTransactionSize(LCD_DMA_TX, len);
  dmaChannelSetMode(LCD_DMA_TX, txdmamode | STM32_DMA_CR_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_EN);
  if (wait)
    dmaChannelWaitCompletion(LCD_DMA_TX);
}

// Wait DMA Rx completion
static void dmaChannelWaitCompletionRxTx(void) {
  dmaChannelWaitCompletion(LCD_DMA_TX);
  dmaChannelWaitCompletion(LCD_DMA_RX);
//  while (SPI_IS_BUSY(LCD_SPI));   // Wait SPI tx/rx
}

// SPI receive byte buffer use DMA
static const uint16_t dummy_tx = 0xFFFF;
static inline void spi_DMARxBuffer(uint8_t *buffer, uint16_t len, bool wait) {
  // Init Rx DMA buffer, size, mode (spi and mem data size is 8 bit), and start
  dmaChannelSetMemory(LCD_DMA_RX, buffer);
  dmaChannelSetTransactionSize(LCD_DMA_RX, len);
  dmaChannelSetMode(LCD_DMA_RX, rxdmamode | STM32_DMA_CR_BYTE | STM32_DMA_CR_MINC | STM32_DMA_CR_EN);
  // Init dummy Tx DMA (for rx clock), size, mode (spi and mem data size is 8 bit), and start
  dmaChannelSetMemory(LCD_DMA_TX, &dummy_tx);
  dmaChannelSetTransactionSize(LCD_DMA_TX, len);
  dmaChannelSetMode(LCD_DMA_TX, txdmamode | STM32_DMA_CR_BYTE | STM32_DMA_CR_EN);
  if (wait)
    dmaChannelWaitCompletionRxTx();
}
#else
// Replace DMA function vs no DMA
#define dmaChannelWaitCompletionRxTx() {}
#define spi_DMATxBuffer(buffer, len, flag) spi_TxBuffer(buffer, len)
#define spi_DMARxBuffer(buffer, len, flag) spi_RxBuffer(buffer, len)
#endif // __USE_DISPLAY_DMA__

static void spi_init(void) {
  rccEnableSPI1(FALSE);
  LCD_SPI->CR1 = 0;
  LCD_SPI->CR1 = SPI_CR1_MSTR      // SPI is MASTER
               | SPI_CR1_SSM       // Software slave management (The external NSS pin is free for other application uses)
               | SPI_CR1_SSI       // Internal slave select (This bit has an effect only when the SSM bit is set. Allow use NSS pin as I/O)
               | LCD_SPI_SPEED     // Baud rate control
               | SPI_CR1_CPHA      // Clock Phase
               | SPI_CR1_CPOL      // Clock Polarity
               ;
  LCD_SPI->CR2 = SPI_CR2_8BIT      // SPI data size, set to 8 bit
               | SPI_CR2_FRXTH     // SPI_SR_RXNE generated every 8 bit data
//             | SPI_CR2_SSOE      //
#ifdef __USE_DISPLAY_DMA__
               | SPI_CR2_TXDMAEN   // Tx DMA enable
#ifdef __USE_DISPLAY_DMA_RX__
               | SPI_CR2_RXDMAEN   // Rx DMA enable
#endif
#endif
               ;
// Init SPI DMA Peripheral
#ifdef __USE_DISPLAY_DMA__
  dmaChannelSetPeripheral(LCD_DMA_TX, &LCD_SPI->DR); // DMA Peripheral Tx
#ifdef __USE_DISPLAY_DMA_RX__
  dmaChannelSetPeripheral(LCD_DMA_RX, &LCD_SPI->DR); // DMA Peripheral Rx
#endif
#endif
  // Enable DMA on SPI
  LCD_SPI->CR1|= SPI_CR1_SPE;       //SPI enable
}

//******************************************************************************
//           All LCD (ILI9341, ST7789V, ST9996s) level 1 commands
//******************************************************************************
#define LCD_NOP             0x00  // No operation
#define LCD_SWRESET         0x01  // Software reset
#define LCD_RDDID           0x04  // Read display ID
#define LCD_RDNUMED         0x05  // Read Number of the Errors on DSI (only ST7796s)
#define LCD_RDDST           0x09  // Read display status
#define LCD_RDDPM           0x0A  // Read Display Power Mode
#define LCD_RDD_MADCTL      0x0B  // Read Display MADCTL
#define LCD_RDDCOLMOD       0x0C  // Read Display Pixel Format
#define LCD_RDDIM           0x0D  // Read Display Image Mode
#define LCD_RDDSM           0x0E  // Read Display Signal Mode
#define LCD_RDDSDR          0x0F  // Read Display Self-Diagnostic Result
#define LCD_SLPIN           0x10  // Sleep in
#define LCD_SLPOUT          0x11  // Sleep Out
#define LCD_PTLON           0x12  // Partial Display Mode On
#define LCD_NORON           0x13  // Normal Display Mode On
#define LCD_INVOFF          0x20  // Display Inversion Off
#define LCD_INVON           0x21  // Display Inversion On
#define LCD_GAMSET          0x26  // Gamma Set (only ILI9341 and ST7789V)
#define LCD_DISPOFF         0x28  // Display Off
#define LCD_DISPON          0x29  // Display On
#define LCD_CASET           0x2A  // Column Address Set
#define LCD_RASET           0x2B  // Row Address Set
#define LCD_RAMWR           0x2C  // Memory Write
#define LCD_RGBSET          0x2D  // Color Set  (only ILI9341)
#define LCD_RAMRD           0x2E  // Memory Read
#define LCD_PTLAR           0x30  // Partial Area
#define LCD_VSCRDEF         0x33  // Vertical Scrolling Definition
#define LCD_TEOFF           0x34  // Tearing Effect Line OFF
#define LCD_TEON            0x35  // Tearing Effect Line On
#define LCD_MADCTL          0x36  // Memory Data Access Control
#define LCD_VSCSAD          0x37  // Vertical Scroll Start Address of RAM
#define LCD_IDMOFF          0x38  // Idle Mode Off
#define LCD_IDMON           0x39  // Idle mode on
#define LCD_COLMOD          0x3A  // Interface Pixel Format
#define LCD_WRMEMC          0x3C  // Write_Memory_Continue (only ILI9341)
#define LCD_RDMEMC          0x3E  // Read Memory Continue
#define LCD_STE             0x44  // Set Tear Scanline
#define LCD_GSCAN           0x45  // Get Scanline
#define LCD_WRDISBV         0x51  // Write Display Brightness
#define LCD_RDDISBV         0x52  // Read Display Brightness Value
#define LCD_WRCTRLD         0x53  // Write CTRL Display
#define LCD_RDCTRLD         0x54  // Read CTRL Value Display
#define LCD_WRCACE          0x55  // Write Content Adaptive Brightness Control and Color Enhancement
#define LCD_RDCABC          0x56  // Read Content Adaptive Brightness Control
#define LCD_WRCABCMB        0x5E  // Write CABC Minimum Brightness
#define LCD_RDCABCMB        0x5F  // Read CABC Minimum Brightness
#define LCD_RDID1           0xDA  // Read ID1
#define LCD_RDID2           0xDB  // Read ID2
#define LCD_RDID3           0xDC  // Read ID3

// MEMORY_ACCESS_CONTROL register
#define LCD_MADCTL_MH       0x04
#define LCD_MADCTL_BGR      0x08
#define LCD_MADCTL_RGB      0x00
#define LCD_MADCTL_ML       0x10
#define LCD_MADCTL_MV       0x20
#define LCD_MADCTL_MX       0x40
#define LCD_MADCTL_MY       0x80
// Display rotation enum
enum {
 DISPLAY_ROTATION_0 = 0,
 DISPLAY_ROTATION_90,
 DISPLAY_ROTATION_180,
 DISPLAY_ROTATION_270,
};

//******************************************************************************
// Custom ILI9391 level 2 commands
//******************************************************************************
#define ILI9341_IFMODE      0xB0  // RGB Interface Signal Control
#define ILI9341_FRMCTR1     0xB1  // Frame Rate Control (In Normal Mode/Full Colors)
#define ILI9341_FRMCTR2     0xB2  // Frame Rate Control (In Idle Mode/8 colors)
#define ILI9341_FRMCTR3     0xB3  // Frame Rate control (In Partial Mode/Full Colors)
#define ILI9341_INVTR       0xB4  // Display Inversion Control
#define ILI9341_PRCTR       0xB5  // Blanking Porch Control
#define ILI9341_DISCTRL     0xB6  // Display Function Control
#define ILI9341_ETMOD       0xB7  // Entry Mode Set
#define ILI9341_BKLTCTRL1   0xB8  // Backlight Control 1
#define ILI9341_BKLTCTRL2   0xB9  // Backlight Control 2
#define ILI9341_BKLTCTRL3   0xBA  // Backlight Control 3
#define ILI9341_BKLTCTRL4   0xBB  // Backlight Control 4
#define ILI9341_BKLTCTRL5   0xBC  // Backlight Control 5
#define ILI9341_BKLTCTRL7   0xBE  // Backlight Control 7
#define ILI9341_BKLTCTRL8   0xBF  // Backlight Control 8
#define ILI9341_PWCTRL1     0xC0  // Power Control 1
#define ILI9341_PWCTRL2     0xC1  // Power Control 2
#define ILI9341_VMCTRL1     0xC5  // VCOM Control 1
#define ILI9341_VMCTRL2     0xC7  // VCOM Control 2
#define ILI9341_NVMWR       0xD0  // NV Memory Write
#define ILI9341_NVMPKEY     0xD1  // NV Memory Protection Key
#define ILI9341_RDNVM       0xD2  // NV Memory Status Read
#define ILI9341_RDID4       0xD3  // Read ID4
#define ILI9341_PGAMCTRL    0xE0  // Positive Gamma Correction
#define ILI9341_NGAMCTRL    0xE1  // Negative Gamma Correction
#define ILI9341_DGAMCTRL1   0xE2  // Digital Gamma Control 1
#define ILI9341_DGAMCTRL2   0xE3  // Digital Gamma Control 2
#define ILI9341_IFCTL       0xF6  // Interface Control
// Extend register commands
#define ILI9341_POWERA      0xCB  // Power control A
#define ILI9341_POWERB      0xCF  // Power control B
#define ILI9341_DTCA        0xE8  // Driver timing control A
#define ILI9341_DTCB        0xEA  // Driver timing control B
#define ILI9341_POWER_SEQ   0xED  // Power on sequence control
#define ILI9341_3GAMMA_EN   0xF2  // Enable 3G
#define ILI9341_PUMPCTRL    0xF7  // Pump ratio control

//******************************************************************************
// Custom ST7789V level 2 commands
//******************************************************************************
#define ST7789V_RAMCTRL     0xB0  // RAM Control
#define ST7789V_RGBCTRL     0xB1  // RGB Interface Control
#define ST7789V_PORCTRL     0xB2  // Porch Setting
#define ST7789V_FRCTRL1     0xB3  // Frame Rate Control 1 (In partial mode/ idle colors)
#define ST7789V_INVTR       0xB4  // Display Inversion Control (only ILI9341)
#define ST7789V_PARCTRL     0xB5  // Partial Control
#define ST7789V_GCTRL       0xB7  // Gate Control
#define ST7789V_GTADJ       0xB8  // Gate On Timing Adjustment
#define ST7789V_DGMEN       0xBA  // Digital Gamma Enable
#define ST7789V_VCOMS       0xBB  // VCOM Setting
#define ST7789V_POWSAVE     0xBC  // Power Saving Mode
#define ST7789V_DLPOFFSAVE  0xBD  // Display off power save
#define ST7789V_LCMCTRL     0xC0  // LCM Control
#define ST7789V_IDSET       0xC1  // ID Code Setting
#define ST7789V_VDVVRHEN    0xC2  // VDV and VRH Command Enable
#define ST7789V_VRHS        0xC3  // VRH Set
#define ST7789V_VDVS        0xC4  // VDV Set
#define ST7789V_VCMOFSET    0xC5  // VCOM Offset Set
#define ST7789V_FRCTRL2     0xC6  // Frame Rate Control in Normal Mode
#define ST7789V_CABCCTRL    0xC7  // CABC Control
#define ST7789V_REGSEL1     0xC8  // Register Value Selection 1
#define ST7789V_REGSEL2     0xCA  // Register Value Selection 2
#define ST7789V_PWMFRSEL    0xCC  // PWM Frequency Selection
#define ST7789V_PWCTRL1     0xD0  // Power Control 1
#define ST7789V_VAPVANEN    0xD2  // Enable VAP/VAN signal output
#define ST7789V_CMD2EN      0xDF  // Command 2 Enable
#define ST7789V_PVGAMCTRL   0xE0  // Positive Voltage Gamma Control
#define ST7789V_NVGAMCTRL   0xE1  // Negative Voltage Gamma Control
#define ST7789V_DGMLUTR     0xE2  // Digital Gamma Look-up Table for Red
#define ST7789V_DGMLUTB     0xE3  // Digital Gamma Look-up Table for Blue
#define ST7789V_GATECTRL    0xE4  // Gate Control
#define ST7789V_SPI2EN      0xE7  // SPI2 Enable
#define ST7789V_PWCTRL2     0xE8  // Power Control 2
#define ST7789V_EQCTRL      0xE9  // Equalize time control
#define ST7789V_PROMCTRL    0xEC  // Program Mode Control
#define ST7789V_PROMEN      0xFA  // Program Mode Enable
#define ST7789V_NVMSET      0xFC  // NVM Setting
#define ST7789V_PROMACT     0xFE  // Program action

//******************************************************************************
// Custom ST7796s level 2 commands
//******************************************************************************
#define ST7796S_IFMODE      0xB0  // Interface Mode Control
#define ST7796S_FRMCTR1     0xB1  // Frame Rate Control (In Normal Mode/Full Colors)
#define ST7796S_FRMCTR2     0xB2  // Frame Rate Control 2 (In Idle Mode/8 colors)
#define ST7796S_FRMCTR3     0xB3  // Frame Rate Control3 (In Partial Mode/Full Colors)
#define ST7796S_DIC         0xB4  // Display Inversion Control
#define ST7796S_BPC         0xB5  // Blanking Porch Control
#define ST7796S_DFC         0xB6  // Display Function Control
#define ST7796S_EM          0xB7  // Entry Mode Set
#define ST7796S_PWR1        0xC0  // Power Control 1
#define ST7796S_PWR2        0xC1  // Power Control 2
#define ST7796S_PWR3        0xC2  // Power Control 3
#define ST7796S_VCMPCTL     0xC5  // VCOM Control
#define ST7796S_VCMOFFSET   0xC6  // Vcom Offset Registe
#define ST7796S_NVMADW      0xD0  // NVM Address/Data Write
#define ST7796S_NVMBPROG    0xD1  // NVM Byte Program
#define ST7796S_NVMSR       0xD2  // Status Read
#define ST7796S_RDID4       0xD3  // Read ID4
#define ST7796S_PGC         0xE0  // Positive Gamma Control
#define ST7796S_NGC         0xE1  // Negative Gamma Control
#define ST7796S_DGC1        0xE2  // Digital Gamma Control 1
#define ST7796S_DGC2        0xE2  // Digital Gamma Control 2
#define ST7796S_DOCA        0xE8  // Display Output Ctrl Adjust
#define ST7796S_CSCON       0xF0  // Command Set Control
#define ST7796S_SPI         0xFB  // Read Control

//******************************************************************************
// Low level Display driver functions
//******************************************************************************
// Used only in double buffer mode
#ifndef lcd_get_cell_buffer
#define LCD_BUFFER_1    0x01
#define LCD_DMA_RUN     0x02
static uint8_t LCD_dma_status = 0;

// Return free buffer for render
pixel_t *lcd_get_cell_buffer(void) {
  return &spi_buffer[(LCD_dma_status&LCD_BUFFER_1) ? SPI_BUFFER_SIZE/2 : 0];
}
#endif

// Disable inline for this function
static void lcd_send_command(uint8_t cmd, uint16_t len, const uint8_t *data) {
// Uncomment on low speed SPI (possible get here before previous tx complete)
  while (SPI_IS_BUSY(LCD_SPI));
  LCD_CS_LOW;
  LCD_DC_CMD;
  SPI_WRITE_8BIT(LCD_SPI, cmd);
  // Need wait transfer complete and set data bit
  while (SPI_IS_BUSY(LCD_SPI));
  LCD_DC_DATA;
  spi_TxBuffer(data, len);
//  while (SPI_IN_TX_RX(LCD_SPI));
  //LCD_CS_HIGH;
}

// Send command to LCD and read 32bit answer
// LCD_RDDID command, need shift result right by 7 bit
// 0x00858552 for ST7789V (9.1.3 RDDID (04h): Read Display ID)
// 0x006BFFFF for ST7796S ?? no id description in datasheet
// 0x00000000 for ili9341 ?? no id description in datasheet
uint32_t lcd_send_register(uint8_t cmd, uint8_t len, const uint8_t *data) {
  lcd_bulk_finish();
  SPI_BR_SET(LCD_SPI, SPI_BR_DIV16);   // Set most safe read speed
  lcd_send_command(cmd, len, data);    // Send command
  spi_DropRx();                        // Skip data from rx buffer
  uint32_t ret;
  ret = spi_RxByte();ret<<=8;
  ret|= spi_RxByte();ret<<=8;
  ret|= spi_RxByte();ret<<=8;
  ret|= spi_RxByte();
  LCD_CS_HIGH;
  SPI_BR_SET(LCD_SPI, LCD_SPI_SPEED);
  return ret;
}

//******************************************************************************
// Display driver init sequence and hardware depend functions
//******************************************************************************
// ILI9341 and ST7789V Lcd init sequence + lcd depend image rotate function
#if defined(LCD_DRIVER_ILI9341) || defined(LCD_DRIVER_ST7789)
typedef enum {ili9341_type = 0, st7789v} lcd_type_t;
static lcd_type_t lcd_type = ili9341_type;
static const uint8_t ili9341_init_seq[] = {               // ILI9341 init sequence
  // cmd,           len, data...,
  LCD_SWRESET,        0,                                  // SW reset
  LCD_DISPOFF,        0,                                  // display off
//ILI9341_POWERB,     3, 0x00, 0xC1, 0x30,                // Power control B
//ILI9341_POWER_SEQ,  4, 0x64, 0x03, 0x12, 0x81,          // Power on sequence control
//ILI9341_DTCA,       3, 0x85, 0x00, 0x78,                // Driver timing control A
//ILI9341_POWERA,     5, 0x39, 0x2C, 0x00, 0x34, 0x02,    // Power control A
//ILI9341_PUMPCTRL,   1, 0x20,                            // Pump ratio control
//ILI9341_DTCB,       2, 0x00, 0x00,                      // Driver timing control B
  ILI9341_PWCTRL1,    1, 0x23,                            // POWER_CONTROL_1
  ILI9341_PWCTRL2,    1, 0x10,                            // POWER_CONTROL_2
  ILI9341_VMCTRL1,    2, 0x3e, 0x28,                      // VCOM_CONTROL_1
  ILI9341_VMCTRL2,    1, 0xBE,                            // VCOM_CONTROL_2
  LCD_MADCTL,         1, LCD_MADCTL_MV | LCD_MADCTL_BGR,  // landscape
  LCD_COLMOD,         1, 0x55,                            // COLMOD_PIXEL_FORMAT_SET : 16 bit pixel
  ILI9341_FRMCTR1,    2, 0x00, 0x18,                      // Frame Rate
//ILI9341_3GAMMA_EN,  1, 0x00,                            // Gamma Function Disable
  LCD_GAMSET,         1, 0x01,                            // gamma set for curve 01/2/04/08
  ILI9341_PGAMCTRL,  15, 0x0F,  0x31,  0x2B,  0x0C,  0x0E,  0x08,  0x4E,  0xF1,  0x37,  0x07,  0x10,  0x03,  0x0E, 0x09,  0x00, // positive gamma correction
  ILI9341_NGAMCTRL,  15, 0x00,  0x0E,  0x14,  0x03,  0x11,  0x07,  0x31,  0xC1,  0x48,  0x08,  0x0F,  0x0C,  0x31, 0x36,  0x0F, // negative gamma correction
//LCD_CASET,          4, 0x00, 0x00, 0x01, 0x3f,          // Column Address Set: x = 0, width 320
//LCD_RASET,          4, 0x00, 0x00, 0x00, 0xef,          // Page Address Set: y = 0, height 240
  ILI9341_ETMOD,      1, 0x06,                            // entry mode
  ILI9341_DISCTRL,    3, 0x08, 0x82, 0x27,                // display function control
  ILI9341_IFCTL,      3, 0x00, 0x00, 0x00,                // Interface Control (set WEMODE=0)
  LCD_SLPOUT,         0,                                  // sleep out
  LCD_DISPON,         0,                                  // display on
  0                                                       // sentinel
};

// ST7789 LCD_RDDID read return 0x42C2A97F (need shift right by 7 bit, so ID1 = 0x85, ID2 = 0x85, ID3 = 0x52)
#define ST7789V_ID        0x858552
static const uint8_t ST7789V_init_seq[] = {               // ST7789V init sequence
  // cmd,           len, data...,
  LCD_SWRESET,        0,                                  // SW reset
  LCD_DISPOFF,        0,                                  // display off
  LCD_MADCTL,         1, LCD_MADCTL_MX | LCD_MADCTL_MV | LCD_MADCTL_RGB,
  LCD_COLMOD,         1, 0x55,                            // COLMOD_PIXEL_FORMAT_SET : 16 bit pixel
//ST7789V_PORCTRL,    5, 0x0C, 0x0C, 0x00, 0x33, 0x33,
//ST7789V_GCTRL,      1, 0x35,
  ST7789V_VCOMS,      1, 0x1F,                            // default 0x20
//ST7789V_LCMCTRL,    1, 0x2C,
  ST7789V_VDVVRHEN,   2, 0x01, 0xC3,                      // default 0x01, 0xFF !!! why need C3? datasheet say 0xFF
//ST7789V_VDVS,       1, 0x20,
//ST7789V_FRCTRL2,    1, 0x0F,
//ST7789V_PWCTRL1,    2, 0xA4, 0xA1,
  LCD_SLPOUT,         0,                                  // sleep out
  LCD_DISPON,         0,                                  // display on
  0                                                       // sentinel
};

// Read display ID and detect type
static const uint8_t *get_lcd_init(void) {
  uint32_t id = lcd_send_register(LCD_RDDID, 0, 0) >> 7;
  if (id == ST7789V_ID) lcd_type = st7789v;
  return lcd_type == ili9341_type ? ili9341_init_seq : ST7789V_init_seq;
}

void lcd_set_rotation(uint8_t r) {
  static const uint8_t lcd_rotation_const[]={
    // ILI9341 LCD_MADCTL rotation settings
    (LCD_MADCTL_MV | LCD_MADCTL_BGR),
    (LCD_MADCTL_MY | LCD_MADCTL_BGR),
    (LCD_MADCTL_MX | LCD_MADCTL_MY | LCD_MADCTL_MV | LCD_MADCTL_BGR),
    (LCD_MADCTL_MX | LCD_MADCTL_BGR),
    // ST7789 LCD_MADCTL rotation settings
    (LCD_MADCTL_MX | LCD_MADCTL_MV | LCD_MADCTL_RGB),
    (                                LCD_MADCTL_RGB),
    (LCD_MADCTL_MY | LCD_MADCTL_MV | LCD_MADCTL_RGB),
    (LCD_MADCTL_MX | LCD_MADCTL_MY | LCD_MADCTL_RGB)
  };
  lcd_send_command(LCD_MADCTL, 1, &lcd_rotation_const[lcd_type * 4 + r]);
}

#endif

#ifdef LCD_DRIVER_ST7796S
static const uint8_t ST7796S_init_seq[] = {               // ST7996s init sequence
  // cmd,           len, data...,
  LCD_SWRESET,        0,                                  // SW reset
  LCD_DISPOFF,        0,                                  // display off
  ST7796S_IFMODE,     1, 0x00,                            // Interface Mode Control
  ST7796S_FRMCTR1,    1, 0x0A,                            // Frame Rate
  ST7796S_DIC,        1, 0x02,                            // Display Inversion Control , 2 Dot
  ST7796S_DFC,        3, 0x02, 0x02, 0x3B,                // RGB/MCU Interface Control
  ST7796S_EM,         1, 0xC6,                            // EntryMode
  ST7796S_PWR1,       2, 0x17, 0x15,                      // Power Control 1
  ST7796S_PWR2,       1, 0x41,                            // Power Control 2
//ST7796S_VCMPCTL,    3, 0x00, 0x4D, 0x90,
  ST7796S_VCMPCTL,    3, 0x00, 0x12, 0x80,                // VCOM Control
  LCD_MADCTL,         1, LCD_MADCTL_MV | LCD_MADCTL_BGR,  // landscape, BGR
  LCD_COLMOD,         1, 0x55,                            // Interface Pixel Format, 16bpp
//ST7796S_PGC,       15, 0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F,  // P-Gamma
//ST7796S_NGC,       15, 0x00, 0X16, 0X19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F,  // N-Gamma
//0xE9,               1, 0x00,                            // Set Image Func
  LCD_WRDISBV,        1, 0xFF,                            // Set Brightness to Max
//0xF7,               4, 0xA9, 0x51, 0x2C, 0x82,          // Adjust Control ??
  LCD_SLPOUT,         0,                                  // sleep out
  LCD_DISPON,         0,                                  // display on
  0                                                       // sentinel
};

static const uint8_t *get_lcd_init(void) {
  return ST7796S_init_seq;
}

void lcd_set_rotation(uint8_t r) {
  static const uint8_t ST7796S_rotation_const[]={
          (LCD_MADCTL_MV | LCD_MADCTL_BGR),
          (LCD_MADCTL_MY | LCD_MADCTL_BGR),
          (LCD_MADCTL_MX | LCD_MADCTL_MY | LCD_MADCTL_MV | LCD_MADCTL_BGR),
          (LCD_MADCTL_MX | LCD_MADCTL_BGR)
  };
  lcd_send_command(LCD_MADCTL, 1, &ST7796S_rotation_const[r]);
}
#endif

void lcd_init(void) {
  spi_init();
  LCD_RESET_ASSERT;
  chThdSleepMilliseconds(5);
  LCD_RESET_NEGATE;
  chThdSleepMilliseconds(5); // need time before LCD ready after reset
  const uint8_t *p = get_lcd_init();
  while (*p) {
    lcd_send_command(p[0], p[1], &p[2]);
    p += 2 + p[1];
    chThdSleepMilliseconds(2);
  }
  lcd_clear_screen();
}

void lcd_setWindow(int x, int y, int w, int h, uint16_t cmd) {
// Any LCD exchange start from this
  dmaChannelWaitCompletionRxTx();
//uint8_t xx[4] = { x >> 8, x, (x+w-1) >> 8, (x+w-1) };
//uint8_t yy[4] = { y >> 8, y, (y+h-1) >> 8, (y+h-1) };
  uint32_t xx = __REV16(x | ((x + w - 1) << 16));
  uint32_t yy = __REV16(y | ((y + h - 1) << 16));
  lcd_send_command(LCD_CASET, 4, (uint8_t *)&xx);
  lcd_send_command(LCD_RASET, 4, (uint8_t *)&yy);
  lcd_send_command(cmd, 0, NULL);
}

// Set DMA data size, depend from pixel size
#define LCD_DMA_MODE (LCD_PIXEL_SIZE == 2 ? STM32_DMA_CR_HWORD : STM32_DMA_CR_BYTE)

//
// LCD read data functions (Copy screen data to buffer)
//
#if defined(LCD_DRIVER_ILI9341) || defined(LCD_DRIVER_ST7789)
// ILI9341 or ST7789 send data in RGB888 format, need parse it
void lcd_read_memory(int x, int y, int w, int h, uint16_t *out) {
  uint16_t len = w * h;
  lcd_setWindow(x, y, w, h, LCD_RAMRD);
  // Set read speed (if different from write speed)
  if (lcd_type == st7789v && ST7789V_SPI_RX_SPEED != LCD_SPI_SPEED) SPI_BR_SET(LCD_SPI, ST7789V_SPI_RX_SPEED);
  else if (                  ILI9341_SPI_RX_SPEED != LCD_SPI_SPEED) SPI_BR_SET(LCD_SPI, ILI9341_SPI_RX_SPEED);
  spi_DropRx();                                       // Skip data from SPI rx buffer
  spi_RxByte();                                       // require 8bit dummy clock
  uint8_t *rgbbuf = (uint8_t *)out;                   // receive pixel data to buffer
#ifndef __USE_DISPLAY_DMA_RX__
  spi_RxBuffer(rgbbuf, len * LCD_RX_PIXEL_SIZE);
  do {                                                // Parse received data to RGB565 format
    *out++ = RGB565(rgbbuf[0], rgbbuf[1], rgbbuf[2]); // read data is always 18bit
    rgbbuf+= LCD_RX_PIXEL_SIZE;
  } while(--len);
#else
  len*= LCD_RX_PIXEL_SIZE;                     // Set data size for DMA read
  spi_DMARxBuffer(rgbbuf, len, false);         // Start DMA read, and not wait completion
  do {                                         // Parse received data to RGB565 format while data receive by DMA
    uint16_t left = dmaChannelGetTransactionSize(LCD_DMA_RX)+LCD_RX_PIXEL_SIZE; // Get DMA data left
    if (left > len) continue;                  // Next pixel RGB data not ready
    do {                                       // Process completed by DMA data
      *out++ = RGB565(rgbbuf[0], rgbbuf[1], rgbbuf[2]);
      rgbbuf+= LCD_RX_PIXEL_SIZE;
      len   -= LCD_RX_PIXEL_SIZE;
    } while (left < len);
  } while(len);
  dmaChannelWaitCompletionRxTx();              // Stop DMA transfer
#endif
  SPI_BR_SET(LCD_SPI, LCD_SPI_SPEED);          // restore SPI speed
  LCD_CS_HIGH;                                 // stop read
}
#elif defined(LCD_DRIVER_ST7796S)
// ST7796S send data in RGB565 format, not need parse
void lcd_read_memory(int x, int y, int w, int h, uint16_t *out) {
  uint16_t len = w * h;
  lcd_setWindow(x, y, w, h, LCD_RAMRD);
  // Set read speed (if need different)
  if (LCD_SPI_RX_SPEED != LCD_SPI_SPEED) SPI_BR_SET(LCD_SPI, LCD_SPI_RX_SPEED);
  spi_DropRx();         // Skip data from rx buffer
  spi_RxByte();         // require 8bit dummy clock
  // receive pixel data to buffer
#ifndef __USE_DISPLAY_DMA_RX__
  spi_RxBuffer((uint8_t *)out, len * 2);
#else
  spi_DMARxBuffer((uint8_t *)out, len * 2, true);
#endif
  // restore speed if need
  if (LCD_SPI_RX_SPEED != LCD_SPI_SPEED) SPI_BR_SET(LCD_SPI, LCD_SPI_SPEED);
  LCD_CS_HIGH;
}
#endif

void lcd_set_flip(bool flip) {
  dmaChannelWaitCompletionRxTx();
  lcd_set_rotation(flip ? DISPLAY_ROTATION_180 : DISPLAY_ROTATION_0);
}

// Wait completion before next data send
#ifndef lcd_bulk_finish
void lcd_bulk_finish(void) {
  dmaChannelWaitCompletion(LCD_DMA_TX);  // Wait DMA
//while (SPI_IN_TX_RX(LCD_SPI));         // Wait tx
}
#endif

static void lcd_bulk_buffer(int x, int y, int w, int h, pixel_t *buffer) {
  lcd_setWindow(x, y, w, h, LCD_RAMWR);
#ifdef __USE_DISPLAY_DMA__
  dmaChannelSetMemory(LCD_DMA_TX, buffer);
  dmaChannelSetTransactionSize(LCD_DMA_TX, w * h);
  dmaChannelSetMode(LCD_DMA_TX, txdmamode | LCD_DMA_MODE | STM32_DMA_CR_MINC | STM32_DMA_CR_EN);
#else
  spi_TxBuffer((uint8_t *)buffer, w * h * sizeof(pixel_t));
#endif

#ifdef __REMOTE_DESKTOP__
  if (sweep_mode & SWEEP_REMOTE) {
    remote_region_t rd = {"bulk\r\n", x, y, w, h};
    send_region(&rd, (uint8_t *)buffer, w * h * sizeof(pixel_t));
  }
#endif
}

// Copy part of spi_buffer to region, no wait completion after if buffer count !=1
#ifndef lcd_bulk_continue
void lcd_bulk_continue(int x, int y, int w, int h) {
  lcd_bulk_buffer(x, y, w, h, lcd_get_cell_buffer());  // Send new cell data
  LCD_dma_status^=LCD_BUFFER_1;                        // Switch buffer
}
#endif

// Copy spi_buffer to region, wait completion after
void lcd_bulk(int x, int y, int w, int h) {
  lcd_bulk_buffer(x, y, w, h, spi_buffer);  // Send data
  lcd_bulk_finish();                        // Wait
}

//******************************************************************************
//   Display draw functions
//******************************************************************************
// Fill region by some color
void lcd_fill(int x, int y, int w, int h) {
  lcd_setWindow(x, y, w, h, LCD_RAMWR);
  uint32_t len = w * h;
#ifdef __USE_DISPLAY_DMA__
  dmaChannelSetMemory(LCD_DMA_TX, &background_color);
  while(len) {
    uint32_t delta = len > 0xFFFF ? 0xFFFF : len; // DMA can send only 65535 data in one run
    dmaChannelSetTransactionSize(LCD_DMA_TX, delta);
    dmaChannelSetMode(LCD_DMA_TX, txdmamode | LCD_DMA_MODE | STM32_DMA_CR_EN);
    dmaChannelWaitCompletion(LCD_DMA_TX);
    len-=delta;
  }
#else
  do {
    while (SPI_TX_IS_NOT_EMPTY(LCD_SPI))
      ;
    if (LCD_PIXEL_SIZE == 2) SPI_WRITE_16BIT(LCD_SPI, background_color);
    else                     SPI_WRITE_8BIT(LCD_SPI,  background_color);
  } while(--len);
#endif

#ifdef __REMOTE_DESKTOP__
  if (sweep_mode & SWEEP_REMOTE) {
    remote_region_t rd = {"fill\r\n", x, y, w, h};
    send_region(&rd, (uint8_t *)&background_color, sizeof(pixel_t));
  }
#endif
}

#if 0
static void lcd_pixel(int x, int y, uint16_t color) {
  lcd_setWindow(x, y, 1, 1, LCD_RAMWR);
  while (SPI_TX_IS_NOT_EMPTY(LCD_SPI));
  SPI_WRITE_16BIT(LCD_SPI, color);
}
#endif

void lcd_line(int x0, int y0, int x1, int y1) {
  // Modified Bresenham's line algorithm
  if (x1 < x0) { SWAP(int, x0, x1); SWAP(int, y0, y1); }      // Need draw from left to right
  int dx =-(x1 - x0), sx = 1;
  int dy = (y1 - y0), sy = 1; if (dy < 0) {dy = -dy; sy = -1;}
  int err = -((dx + dy) < 0 ? dx : dy) / 2;
  while (1) {
    lcd_setWindow(x0, y0, LCD_WIDTH-x0, 1, LCD_RAMWR);        // prepare send Horizontal line
    while (1) {
      while (SPI_TX_IS_NOT_EMPTY(LCD_SPI));
      SPI_WRITE_16BIT(LCD_SPI, foreground_color);             // Send color
      if (x0 == x1 && y0 == y1)
        return;
      int e2 = err;
      if (e2 > dx) { err-= dy; x0+= sx; }
      if (e2 < dy) { err-= dx; y0+= sy; break;}               // Y coordinate change, next horizontal line
    }
  }
}

void lcd_clear_screen(void) {
  lcd_fill(0, 0, LCD_WIDTH, LCD_HEIGHT);
}

void lcd_set_foreground(uint16_t fg_idx) {
  foreground_color = GET_PALTETTE_COLOR(fg_idx);
}

void lcd_set_background(uint16_t bg_idx) {
  background_color = GET_PALTETTE_COLOR(bg_idx);
}

void lcd_set_colors(uint16_t fg_idx, uint16_t bg_idx) {
  foreground_color = GET_PALTETTE_COLOR(fg_idx);
  background_color = GET_PALTETTE_COLOR(bg_idx);
}

void lcd_blitBitmap(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *b) {
#if 1 // Use this for remote desktop (in this case bulk operation send to remote)
  pixel_t *buf = spi_buffer;
  uint8_t bits = 0;
  for (uint32_t c = 0; c < height; c++) {
    for (uint32_t r = 0; r < width; r++) {
      if ((r&7) == 0) bits = *b++;
      *buf++ = (0x80 & bits) ? foreground_color : background_color;
      bits <<= 1;
    }
  }
  lcd_bulk(x, y, width, height);
#else
  uint8_t bits = 0;
  lcd_setWindow(x, y, width, height, LCD_RAMWR);
  for (uint32_t c = 0; c < height; c++) {
    for (uint32_t r = 0; r < width; r++) {
      if ((r&7) == 0) bits = *b++;
      while (SPI_TX_IS_NOT_EMPTY(LCD_SPI));
      SPI_WRITE_16BIT(LCD_SPI, (0x80 & bits) ? foreground_color : background_color);
      bits <<= 1;
    }
  }
#endif
}

void lcd_drawchar(uint8_t ch, int x, int y) {
  lcd_blitBitmap(x, y, FONT_GET_WIDTH(ch), FONT_GET_HEIGHT, FONT_GET_DATA(ch));
}

#ifndef lcd_drawstring
void lcd_drawstring(int16_t x, int16_t y, const char *str)
{
  int x_pos = x;
  while (*str) {
    uint8_t ch = *str++;
    if (ch == '\n') {x = x_pos; y+=FONT_STR_HEIGHT; continue;}
    const uint8_t *char_buf = FONT_GET_DATA(ch);
    uint16_t w = FONT_GET_WIDTH(ch);
    lcd_blitBitmap(x, y, w, FONT_GET_HEIGHT, char_buf);
    x += w;
  }
}
#endif

typedef struct {
  const void *vmt;
  int16_t start_x, start_y;
  int16_t x, y;
  uint16_t state;
} lcdPrintStream;

static void put_normal(lcdPrintStream *ps, uint8_t ch) {
  if (ch == '\n') {ps->x = ps->start_x; ps->y+=FONT_STR_HEIGHT; return;}
  uint16_t w = FONT_GET_WIDTH(ch);
#if _USE_FONT_ < 3
  lcd_blitBitmap(ps->x, ps->y, w, FONT_GET_HEIGHT, FONT_GET_DATA(ch));
#else
  lcd_blitBitmap(ps->x, ps->y, w < 9 ? 9 : w, FONT_GET_HEIGHT, FONT_GET_DATA(ch));
#endif
  ps->x+= w;
}

#if _USE_FONT_ != _USE_SMALL_FONT_
typedef void (*font_put_t)(lcdPrintStream *ps, uint8_t ch);
static font_put_t put_char = put_normal;
static void put_small(lcdPrintStream *ps, uint8_t ch) {
  if (ch == '\n') {ps->x = ps->start_x; ps->y+=sFONT_STR_HEIGHT; return;}
  uint16_t w = sFONT_GET_WIDTH(ch);
#if _USE_SMALL_FONT_ < 3
  lcd_blitBitmap(ps->x, ps->y, w, sFONT_GET_HEIGHT, sFONT_GET_DATA(ch));
#else
  lcd_blitBitmap(ps->x, ps->y, w < 9 ? 9 : w, sFONT_GET_HEIGHT, sFONT_GET_DATA(ch));
#endif
  ps->x+= w;
}
void lcd_set_font(int type) {put_char = type == FONT_SMALL ? put_small : put_normal;}

#else
#define put_char    put_normal
#endif

static msg_t lcd_put(void *ip, uint8_t ch) {
  lcdPrintStream *ps = ip;
  if (ps->state) {
         if (ps->state == R_BGCOLOR[0]) lcd_set_background(ch);
    else if (ps->state == R_FGCOLOR[0]) lcd_set_foreground(ch);
    ps->state = 0;
    return MSG_OK;
  } else if (ch < 0x09) {
    ps->state = ch;
    return MSG_OK;
  }
  put_char(ps, ch);
  return MSG_OK;
}

// Simple print in buffer function
int lcd_printf(int16_t x, int16_t y, const char *fmt, ...) {
  // Init small lcd print stream
  struct lcd_printStreamVMT {
    _base_sequential_stream_methods
  } lcd_vmt = {NULL, NULL, lcd_put, NULL};
  lcdPrintStream ps = {&lcd_vmt, x, y, x, y, 0};
  // Performing the print operation using the common code.
  va_list ap;
  va_start(ap, fmt);
  int retval = chvprintf((BaseSequentialStream *)(void *)&ps, fmt, ap);
  va_end(ap);
  // Return number of bytes that would have been written.
  return retval;
}

int lcd_printfV(int16_t x, int16_t y, const char *fmt, ...) {
  // Init small lcd print stream
  struct lcd_printStreamVMT {
    _base_sequential_stream_methods
  } lcd_vmt = {NULL, NULL, lcd_put, NULL};
  lcdPrintStream ps = {&lcd_vmt, x, y, x, y, 0};
  lcd_set_foreground(LCD_FG_COLOR);
  lcd_set_background(LCD_BG_COLOR);
  lcd_set_rotation(DISPLAY_ROTATION_270);
  // Performing the print operation using the common code.
  va_list ap;
  va_start(ap, fmt);
  int retval = chvprintf((BaseSequentialStream *)(void *)&ps, fmt, ap);
  va_end(ap);
  lcd_set_rotation(DISPLAY_ROTATION_0);
  // Return number of bytes that would have been written.
  return retval;
}

void lcd_blitBitmapScale(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t size, const uint8_t *b) {
  lcd_setWindow(x, y, w * size, h * size, LCD_RAMWR);
  for (int c = 0; c < h; c++) {
    const uint8_t *ptr = b; uint8_t bits = 0;
    for (int i = 0; i < size; i++) {
      ptr = b;
      for (int r = 0; r < w; r++, bits <<= 1) {
        if ((r&7) == 0) bits = *ptr++;
        for (int j = 0; j < size; j++) {
          while (SPI_TX_IS_NOT_EMPTY(LCD_SPI));
          SPI_WRITE_16BIT(LCD_SPI, (0x80 & bits) ? foreground_color : background_color);
        }
      }
    }
    b = ptr;
  }
}

int lcd_drawchar_size(uint8_t ch, int x, int y, uint8_t size) {
  const uint8_t *char_buf = FONT_GET_DATA(ch);
  uint16_t w = FONT_GET_WIDTH(ch);
#if 1 // Use this for remote desctop (in this case bulk operation send to remote)
  pixel_t *buf = spi_buffer;
  for (uint32_t c = 0; c < FONT_GET_HEIGHT; c++, char_buf++) {
    for (uint32_t i = 0; i < size; i++) {
      uint8_t bits = *char_buf;
      for (uint32_t r = 0; r < w; r++, bits <<= 1)
        for (uint32_t j = 0; j < size; j++)
          *buf++ = (0x80 & bits) ? foreground_color : background_color;
    }
  }
  lcd_bulk(x, y, w * size, FONT_GET_HEIGHT * size);
#else
  lcd_setWindow(x, y, w * size, FONT_GET_HEIGHT * size, LCD_RAMWR);
  for (int c = 0; c < FONT_GET_HEIGHT; c++, char_buf++) {
    for (int i = 0; i < size; i++) {
      uint8_t bits = *char_buf;
      for (int r = 0; r < w; r++, bits <<= 1)
        for (int j = 0; j < size; j++) {
          while (SPI_TX_IS_NOT_EMPTY(LCD_SPI));
          SPI_WRITE_16BIT(LCD_SPI, (0x80 & bits) ? foreground_color : background_color);
        }
    }
  }
#endif
  return w * size;
}

void lcd_drawfont(uint8_t ch, int x, int y) {
  lcd_blitBitmap(x, y, NUM_FONT_GET_WIDTH, NUM_FONT_GET_HEIGHT, NUM_FONT_GET_DATA(ch));
}

void lcd_drawstring_size(const char *str, int x, int y, uint8_t size) {
  while (*str)
    x += lcd_drawchar_size(*str++, x, y, size);
}

void lcd_vector_draw(int x, int y, const vector_data *v) {
  while (v->shift_x || v->shift_y) {
    int x1 = x + (int)v->shift_x;
    int y1 = y + (int)v->shift_y;
    if (!v->transparent)
      lcd_line(x, y, x1, y1);
    x = x1; y = y1;
    v++;
  }
}

#if 0
static const uint16_t colormap[] = {
  RGBHEX(0x00ff00), RGBHEX(0x0000ff), RGBHEX(0xff0000),
  RGBHEX(0x00ffff), RGBHEX(0xff00ff), RGBHEX(0xffff00)
};

void ili9341_test(int mode) {
  int x, y;
  int i;
  switch (mode) {
    default:
#if 1
    lcd_fill(0, 0, LCD_WIDTH, LCD_HEIGHT, 0);
    for (y = 0; y < LCD_HEIGHT; y++) {
      lcd_fill(0, y, LCD_WIDTH, 1, RGB(LCD_HEIGHT-y, y, (y + 120) % 256));
    }
    break;
    case 1:
      lcd_fill(0, 0, LCD_WIDTH, LCD_HEIGHT, 0);
      for (y = 0; y < LCD_HEIGHT; y++) {
        for (x = 0; x < LCD_WIDTH; x++) {
          ili9341_pixel(x, y, (y<<8)|x);
        }
      }
      break;
    case 2:
      //lcd_send_command(0x55, 0xff00);
      ili9341_pixel(64, 64, 0xaa55);
    break;
#endif
#if 1
    case 3:
      for (i = 0; i < 10; i++)
        lcd_drawfont(i, i*20, 120);
    break;
#endif
#if 0
    case 4:
      draw_grid(10, 8, 29, 29, 15, 0, 0xffff, 0);
    break;
#endif
    case 4:
      lcd_line(0, 0, 15, 100);
      lcd_line(0, 0, 100, 100);
      lcd_line(0, 15, 100, 0);
      lcd_line(0, 100, 100, 0);
    break;
  }
}
#endif

#ifdef __USE_SD_CARD__
//*****************************************************
//* SD functions and definitions
//*****************************************************
// Definitions for MMC/SDC command
#define CMD0     (0x40+0)     // GO_IDLE_STATE
#define CMD1     (0x40+1)     // SEND_OP_COND
#define CMD8     (0x40+8)     // SEND_IF_COND
#define CMD9     (0x40+9)     // SEND_CSD
#define CMD10    (0x40+10)    // SEND_CID
#define CMD12    (0x40+12)    // STOP_TRANSMISSION
#define CMD13    (0x40+13)    // SEND_STATUS
#define CMD16    (0x40+16)    // SET_BLOCKLEN
#define CMD17    (0x40+17)    // READ_SINGLE_BLOCK
#define CMD18    (0x40+18)    // READ_MULTIPLE_BLOCK
#define CMD23    (0x40+23)    // SET_BLOCK_COUNT
#define CMD24    (0x40+24)    // WRITE_BLOCK
#define CMD25    (0x40+25)    // WRITE_MULTIPLE_BLOCK
#define CMD55    (0x40+55)    // APP_CMD
#define CMD58    (0x40+58)    // READ_OCR
#define CMD59    (0x40+59)    // CRC_ON_OFF
// Then send after CMD55 (APP_CMD) interpret as ACMD
#define ACMD41   (0xC0+41)    // SEND_OP_COND (ACMD)

// MMC card type and status flags
#define CT_MMC       0x01      // MMC v3
#define CT_SD1       0x02      // SDv1
#define CT_SD2       0x04      // SDv2
#define CT_SDC       0x06      // SD
#define CT_BLOCK     0x08      // Block addressing
#define CT_WRPROTECT 0x40      // Write protect flag
#define CT_POWER_ON  0x80      // Power ON flag

// 7.3.2 Responses
// 7.3.2.1 Format R1 (1 byte)
// This response token is sent by the card after every command with the exception of SEND_STATUS commands.
#define SD_R1_IDLE                                 ((uint8_t)0x01) // The card is in idle state
#define SD_R1_ERASE_RESET                          ((uint8_t)0x02) // erase reset
#define SD_R1_ILLEGAL_CMD                          ((uint8_t)0x04) // Illegal command
#define SD_R1_CRC_ERROR                            ((uint8_t)0x08) // The CRC check of the last command failed
#define SD_R1_ERR_ERASE_CLR                        ((uint8_t)0x10) // error in the sequence of erase commands
#define SD_R1_ADDR_ERROR                           ((uint8_t)0x20) // Incorrect address specified
#define SD_R1_PARAM_ERROR                          ((uint8_t)0x40) // Parameter error
#define SD_R1_NOT_R1                               ((uint8_t)0x80) // Not R1 register
// 7.3.2.2 Format R1b (R1 + Busy)
// The busy signal token can be any number of bytes. A zero value indicates card is busy.
// A non-zero value indicates the card is ready for the next command.
// 7.3.2.3 Format R2  (2 byte)
// This response token is two bytes long and sent as a response to the SEND_STATUS command.
// 1 byte - some as R1
// 2 byte -

// 7.3.2.4 Format R3 (R1 + OCR, 5 bytes)
// This response token is sent by the card when a READ_OCR command is received.
// 1 byte - some as R1
// 2-5 byte - OCR
// On Send byte order in SendCommand send MSB first!!
// Received byte order MSB last!!
#define _OCR(dword) (((dword&0x000000FF)<<24)|((dword&0x0000FF00)<<8)|((dword&0x00FF0000)>>8)|((dword&0xFF000000)>>24))
#define SD_OCR_LOW_VOLTAGE                         ((uint32_t)0x00000080) // Reserved for Low Voltage Range
#define SD_OCR_27_VOLTAGE                          ((uint32_t)0x00008000) // VDD Voltage Window 2.7-2.8V
#define SD_OCR_28_VOLTAGE                          ((uint32_t)0x00010000) // VDD Voltage Window 2.8-2.9V
#define SD_OCR_29_VOLTAGE                          ((uint32_t)0x00020000) // VDD Voltage Window 2.9-3.0V
#define SD_OCR_30_VOLTAGE                          ((uint32_t)0x00040000) // VDD Voltage Window 3.0-3.1V
#define SD_OCR_31_VOLTAGE                          ((uint32_t)0x00080000) // VDD Voltage Window 3.1-3.2V
#define SD_OCR_32_VOLTAGE                          ((uint32_t)0x00100000) // VDD Voltage Window 3.2-3.3V
#define SD_OCR_33_VOLTAGE                          ((uint32_t)0x00200000) // VDD Voltage Window 3.3-3.4V
#define SD_OCR_34_VOLTAGE                          ((uint32_t)0x00400000) // VDD Voltage Window 3.4-3.8V
#define SD_OCR_35_VOLTAGE                          ((uint32_t)0x00800000) // VDD Voltage Window 3.5-3.6V
#define SD_OCR_18_VOLTAGE                          ((uint32_t)0x01000000) // VDD Voltage switch to 1.8V (UHS-I only)
#define SD_OCR_CAPACITY                            ((uint32_t)0x40000000) // Card Capacity Status (CCS)
#define SD_OCR_BUSY                                ((uint32_t)0x80000000) // Card power up status bit (busy)

// 5.3 CSD Register
//  16GB Kingston  40 0E 00 32 5B 59 00 00 73 A7 7F 80 0A 40 00 EB   //  29608 * 512 kB
//  32GB Samsung   40 0E 00 32 5B 59 00 00 EE 7F 7F 80 0A 40 40 55   //  61056 * 512 kB
// 128GB Samsung   40 0E 00 32 5B 59 00 03 B9 FF 7F 80 0A 40 40 AB   // 244224 * 512 kB
#define CSD_0_STRUCTURE                          0b11000000
#define CSD_1_TAAC                               0b11111111
#define CSD_2_NSAC                               0b11111111
#define CSD_3_TRAN_SPEED                         0b11111111
#define CSD_4_CCC                                0b11111111
#define CSD_5_CCC                                0b11110000
#define CSD_5_READ_BL_LEN                        0b00001111
#define CSD_6_READ_BL_PARTIAL                    0b10000000
#define CSD_6_WRITE_BLK_MISALIGN                 0b01000000
#define CSD_6_READ_BLK_MISALIGN                  0b00100000
#define CSD_6_DSR_IMP                            0b00010000
#define CSD_7_C_SIZE                             0b00111111
#define CSD_8_C_SIZE                             0b11111111
#define CSD_9_C_SIZE                             0b11111111
#define CSD_10_ERASE_BLK_EN                      0b01000000
#define CSD_10_SECTOR_SIZE                       0b00111111
#define CSD_11_SECTOR_SIZE                       0b10000000
#define CSD_11_WP_GRP_SIZE                       0b01111111
#define CSD_12_WP_GRP_ENABLE                     0b10000000
#define CSD_12_R2W_FACTOR                        0b00011100
#define CSD_12_WRITE_BL_LEN                      0b00000011
#define CSD_13_WRITE_BL_LEN                      0b11000000
#define CSD_13_WRITE_BL_PARTIAL                  0b00100000
#define CSD_14_FILE_FORMAT_GRP                   0b10000000
#define CSD_14_COPY                              0b01000000
#define CSD_14_PERM_WRITE_PROTECT                0b00100000
#define CSD_14_TMP_WRITE_PROTECT                 0b00010000
#define CSD_14_FILE_FORMAT                       0b00001100
#define CSD_15_CRC                               0b11111110
// 7.3.3.1 Data Response Token
#define SD_TOKEN_DATA_ACCEPTED                     ((uint8_t)0x05) // Data accepted
#define SD_TOKEN_WRITE_CRC_ERROR                   ((uint8_t)0x0b) // Data rejected due to a CRC error
#define SD_TOKEN_WRITE_ERROR                       ((uint8_t)0x0d) // Data rejected due to a write error
// 7.3.3.2 Start Block Tokens and Stop Tran Token
#define SD_TOKEN_START_BLOCK                       ((uint8_t)0xfe) // Start block (single tx, single/multiple rx)
#define SD_TOKEN_START_M_BLOCK                     ((uint8_t)0xfc) // Start multiple block tx
#define SD_TOKEN_STOP_M_BLOCK                      ((uint8_t)0xfd) // Stop multiple block tx
// 7.3.3.3 Data Error Token
#define SD_TOKEN_READ_ERROR                        ((uint8_t)0x01) // Data read error
#define SD_TOKEN_READ_CC_ERROR                     ((uint8_t)0x02) // Internal card controller error
#define SD_TOKEN_READ_ECC_ERROR                    ((uint8_t)0x04) // Card ECC failed
#define SD_TOKEN_READ_RANGE_ERROR                  ((uint8_t)0x08) // Read address out of range

//*****************************************************
//             SD card module settings
//*****************************************************
// Use for enable CRC check of Tx and Rx data on SPI
// If enable both CRC check, on initialization send SD command - CRC_ON_OFF vs ON
// And Card begin check received data and answer on CRC errors
//#define SD_USE_COMMAND_CRC
//#define SD_USE_DATA_CRC

// Use DMA on sector data Tx to SD card (only if enabled Tx DMA for LCD)
#ifdef __USE_DISPLAY_DMA__
#define __USE_SDCARD_DMA__
#endif

// Use DMA on sector data Rx from SD card (only if enabled Rx DMA for LCD)
#ifdef __USE_DISPLAY_DMA_RX__
#define __USE_SDCARD_DMA_RX__
#endif

// Define sector size
#define SD_SECTOR_SIZE      512
// SD card spi bus
#define SD_SPI              SPI1
#define SD_DMA_RX           DMA1_Channel2    // DMA1 channel 2 use for SPI1 rx
#define SD_DMA_TX           DMA1_Channel3    // DMA1 channel 3 use for SPI1 tx
// Define SD SPI speed on work
#define SD_SPI_SPEED        SPI_BR_DIV2
// div4 give less error and high speed for Rx
#define SD_SPI_RX_SPEED     SPI_BR_DIV2

// Define SD SPI speed on initialization (100-400kHz need)
#define SD_INIT_SPI_SPEED   SPI_BR_DIV256

// Local values for SD card state
static uint8_t CardStatus  = 0;      // Status: power on, write protect and Type 0:MMC, 1:SDC, 2:Block addressing

// Debug functions, 0 to disable
#define DEBUG    0
int shell_printf(const char *fmt, ...);
#define DEBUG_PRINT(...) do { if (DEBUG) shell_printf(__VA_ARGS__); } while (0)
#if DEBUG == 1
uint32_t w_cnt;
uint32_t w_time;
uint32_t r_cnt;
uint32_t r_time;
uint32_t total_time;
uint32_t crc_time;
void testLog(void){
  DEBUG_PRINT(" Read  speed = %d Byte/s (count %d, time %d)\r\n", r_cnt*512*100000/r_time, r_cnt, r_time);
  DEBUG_PRINT(" Write speed = %d Byte/s (count %d, time %d)\r\n", w_cnt*512*100000/w_time, w_cnt, w_time);
  DEBUG_PRINT(" Total time = %d\r\n", chVTGetSystemTimeX() - total_time);
  DEBUG_PRINT(" CRC16 time %d\r\n", crc_time);
}
#endif

//*******************************************************
//               SD card SPI functions
//*******************************************************
#define SD_CS_LOW     palClearPad(GPIOB, GPIOB_SD_CS)
#define SD_CS_HIGH    palSetPad(GPIOB, GPIOB_SD_CS)

static void SD_Select_SPI(uint32_t speed) {
  while (SPI_IS_BUSY(LCD_SPI));
  LCD_CS_HIGH;               // Unselect LCD
  SPI_BR_SET(SD_SPI, speed); // Set Baud rate control for SD card
  SD_CS_LOW;                 // Select SD Card
}

static void SD_Unselect_SPI(void) {
  while (SPI_IS_BUSY(SD_SPI));
  SD_CS_HIGH;                         // Unselect SD Card
  spi_RxByte();                       // Dummy read/write one Byte recommend for SD after CS up
  SPI_BR_SET(LCD_SPI, LCD_SPI_SPEED); // Restore Baud rate for LCD
}

//*******************************************************
//*           SD functions
//*******************************************************
// CRC7 used for commands
#ifdef SD_USE_COMMAND_CRC
#define CRC7_POLY  0x89
#define CRC7_INIT  0x00
//                                        7   3
// CRC7 it's a 7 bit CRC with polynomial x + x + 1
static uint8_t crc7(const uint8_t *ptr, uint16_t count) {
  uint8_t crc = CRC7_INIT;
  uint8_t i = 0;
  while (count--){
    crc ^= *ptr++;
    do{
      if (crc & 0x80) crc^=CRC7_POLY;
      crc = crc << 1;
    } while((++i)&0x7);
  }
  return crc;
}
#endif
// CRC16 used for data
#ifdef SD_USE_DATA_CRC
#define CRC16_POLY  0x1021
#define CRC16_INIT  0x0000
//                                      16   12   5
// This is the CCITT CRC 16 polynomial X  + X  + X  + 1.
static uint16_t crc16(const uint8_t *ptr, uint16_t count) {
  uint16_t crc = CRC16_INIT;
#if DEBUG == 1
  crc_time-= chVTGetSystemTimeX();
#endif
  #if 0
  uint8_t i = 0;
  while(count--){
    crc^= ((uint16_t) *ptr++ << 8);
    do{
      if (crc & 0x8000)
        crc = (crc << 1) ^ CRC16_POLY;
      else
        crc = crc << 1;
    } while((++i)&0x7);
  }
  return __REVSH(crc); // swap bytes
  #else
  while (count--){
    crc^= *ptr++;
    crc^= (crc>> 4)&0x000F;
    crc^= (crc<<12);
    crc^= (crc<< 5)&0x1FE0;
    crc = __REVSH(crc); // swap bytes
  }
#if DEBUG == 1
  crc_time+= chVTGetSystemTimeX();
#endif
  return crc;
  #endif
}
#endif

// Wait and read R1 answer from SD
static uint8_t SD_ReadR1(uint32_t cnt) {
  uint8_t r1;
   // 8th bit R1 always zero, check it
  spi_DropRx();
  while(((r1=spi_RxByte())&0x80) && --cnt)
    ;
  return r1;
}

// Wait SD ready token answer (wait time in systick)
static bool SD_WaitDataToken(uint8_t token, uint32_t wait_time) {
  uint8_t res;
  uint32_t time = chVTGetSystemTimeX();
  spi_DropRx();
  while((res = spi_RxByte()) != token && chVTGetSystemTimeX() - time < wait_time)
    ;
  return res == token;
}

static uint8_t SD_WaitDataAccept(uint32_t cnt) {
  uint8_t res;
  spi_DropRx();
  while((res = spi_RxByte()) == 0xFF && --cnt)
    ;
  return res&0x1F;
}

// Wait no Busy answer from SD (wait time in systick)
static uint8_t SD_WaitNotBusy(uint32_t wait_time) {
  uint8_t res;
  uint32_t time = chVTGetSystemTimeX();
  spi_DropRx();
  do {
    if ((res = spi_RxByte()) == 0xFF)
      return res;
  } while(chVTGetSystemTimeX() - time < wait_time);
  return 0;
}

// Receive data block from SD
static bool SD_RxDataBlock(uint8_t *buff, uint16_t len, uint8_t token) {
  // loop until receive read response token or timeout ~100ms
  if (!SD_WaitDataToken(token, MS2ST(100))) {
    DEBUG_PRINT(" rx SD_WaitDataToken err\r\n");
    return FALSE;
  }
  // Receive data
#ifdef __USE_SDCARD_DMA_RX__
  spi_DMARxBuffer(buff, len, true);
#else
  spi_RxBuffer(buff, len);
#endif
  // Read and check CRC (if enabled)
  uint16_t crc; spi_RxBuffer((uint8_t*)&crc, 2);
#ifdef SD_USE_DATA_CRC
  uint16_t bcrc = crc16(buff, len);
  if (crc != bcrc){
    DEBUG_PRINT("CRC = %04x , calc = %04x\r\n", (uint32_t)crc, (uint32_t)bcrc);
    return FALSE;
  }
#endif
  return TRUE;
}

// Transmit data block to SD
static bool SD_TxDataBlock(const uint8_t *buff, uint16_t len, uint8_t token) {
  uint8_t r1;
  // Transmit token
  spi_TxByte(token);
#ifdef __USE_SDCARD_DMA__
  spi_DMATxBuffer(buff, len, false);
#else
  spi_TxBuffer((uint8_t*)buff, len);
#endif
  // Calculate and Send CRC
#ifdef  SD_USE_DATA_CRC
  uint16_t bcrc = crc16(buff, len);
#else
  uint16_t bcrc = 0xFFFF;
#endif
#ifdef __USE_SDCARD_DMA__
  dmaChannelWaitCompletion(SD_DMA_TX);
#endif
  spi_TxByte((bcrc>>0) & 0xFF); // Send CRC
  spi_TxByte((bcrc>>8) & 0xFF);
  // Receive transmit data response token on next 8 bytes
  if ((r1 = SD_WaitDataAccept(100)) != SD_TOKEN_DATA_ACCEPTED) {
    DEBUG_PRINT(" Tx accept error = %04x\n", (uint32_t)r1);
    return FALSE;
  }
#if 0  // Wait busy in block transfer (recommended timeout is 250ms (500ms for SDXC) set 250ms
  if (token == SD_TOKEN_START_M_BLOCK && (r1 = SD_WaitNotBusy(MS2ST(250))) != 0xFF) {
    DEBUG_PRINT(" Tx busy error = %04\n", (uint32_t)r1);
    return FALSE;
  }
#endif
  // Continue execute, wait not busy on next command
  return TRUE;
}

// Transmit command to SD
static uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg) {
  uint8_t buf[6];
  uint8_t r1;
  // Advanced command (ACMD__), need send CMD55 before
  if ((cmd & 0x80) && (r1 = SD_SendCmd(CMD55, 0)) > 1) return r1;
  // wait SD ready after last Tx (recommended timeout is 250ms (500ms for SDXC) set 250ms
  if ((r1 = SD_WaitNotBusy(MS2ST(500))) != 0xFF) {
    DEBUG_PRINT(" SD_WaitNotBusy CMD%d err, %02x\r\n", cmd-0x40, (uint32_t)r1);
    return 0xFF;
  }
  // Transmit command
  buf[0] = cmd & 0x7F;
  buf[1] = (arg >> 24)&0xFF;
  buf[2] = (arg >> 16)&0xFF;
  buf[3] = (arg >>  8)&0xFF;
  buf[4] = (arg >>  0)&0xFF;
#ifdef SD_USE_COMMAND_CRC
  buf[5] = crc7(buf, 5)|0x01;
#else
  uint8_t crc = 0x01;              // Dummy CRC + Stop
       if (cmd == CMD0) crc = 0x95;// Valid CRC for CMD0(0)
  else if (cmd == CMD8) crc = 0x87;// Valid CRC for CMD8(0x1AA)
  buf[5] = crc;
#endif
  spi_TxBuffer(buf, 6);
// Skip a stuff byte when STOP_TRANSMISSION
  if (cmd == CMD12) spi_RxByte();
  // Receive response register r1 (need max 8 cycles, in tests answer on next read)
  r1 = SD_ReadR1(100);
  if (r1&(SD_R1_NOT_R1|SD_R1_CRC_ERROR|SD_R1_ERASE_RESET|SD_R1_ERR_ERASE_CLR)){
    DEBUG_PRINT(" SD_SendCmd err CMD%d, 0x%x, 0x%08x\r\n", (uint32_t)cmd-0x40, (uint32_t)r1, arg);
    return r1;
  }
  if (r1&(~SD_R1_IDLE))
    DEBUG_PRINT(" SD_SendCmd CMD%d, 0x%x, 0x%08x\r\n", (uint32_t)cmd-0x40, (uint32_t)r1, arg);
  return r1;
}

//*******************************************************
//       diskio.c functions for file system library
//*******************************************************
// If enable RTC - get RTC time
#if FF_FS_NORTC == 0
DWORD get_fattime (void) {
  return rtc_get_FAT();
}
#endif

// diskio.c - Initialize SD
DSTATUS disk_initialize(BYTE pdrv) {
// Debug counters
#if DEBUG == 1
  w_cnt = 0;
  w_time = 0;
  r_cnt = 0;
  r_time = 0;
  crc_time = 0;
  total_time = chVTGetSystemTimeX();
#endif
  if (pdrv != 0) return disk_status(pdrv);
  // Start init SD card
  CardStatus = 0;
  LCD_CS_HIGH;
  // Power on, try detect on bus, set card to idle state:
  //   Dummy TxRx 80 bits for power up SD
  for(int n = 0; n < 10; n++)
    spi_RxByte();
  // check disk type
  uint8_t  type = 0;
  uint32_t cnt = 100;
  // Set low SPI bus speed = PLL/256 (on 72MHz =281.250kHz)
  SD_Select_SPI(SD_INIT_SPI_SPEED);
  // send GO_IDLE_STATE command
  if (SD_SendCmd(CMD0, 0) == SD_R1_IDLE) {
    DEBUG_PRINT(" CMD0 Ok\r\n");
    // SDC V2+ accept CMD8 command, http://elm-chan.org/docs/mmc/mmc_e.html
    if (SD_SendCmd(CMD8, 0x00001AAU) == SD_R1_IDLE) {
      uint32_t ocr; spi_RxBuffer((uint8_t *)&ocr, 4);
      DEBUG_PRINT(" CMD8 0x%x\r\n", ocr);
      // operation condition register voltage range 2.7-3.6V
      if (ocr == _OCR(0x00001AAU)) {
        // ACMD41 with HCS bit can be up to 200ms wait
        while (SD_SendCmd(ACMD41, SD_OCR_CAPACITY) != 0 && --cnt)   // Check OCR
          chThdSleepMilliseconds(10);
        DEBUG_PRINT(" ACMD41 %d\r\n", cnt);
        // READ_OCR
        if (cnt && SD_SendCmd(CMD58, 0) == 0) {
          DWORD ocr; spi_RxBuffer((uint8_t *)&ocr, 4);
          DEBUG_PRINT(" CMD58 OCR = 0x%08x\r\n", _OCR(ocr));
          // Check CCS bit, SDv2 (HC or SC)
          type = (ocr & _OCR(SD_OCR_CAPACITY)) ? CT_SD2 | CT_BLOCK : CT_SD2;
        }
      }
#if defined(SD_USE_COMMAND_CRC) && defined(SD_USE_DATA_CRC)
      SD_SendCmd(CMD59, 1); // Enable CRC check on card
#endif
//      uint8_t csd[16];
//      if (SD_SendCmd(CMD9, 0) == 0 && SD_RxDataBlock(csd, 16, SD_TOKEN_START_BLOCK)){
//        DEBUG_PRINT(" CSD =");
//        for (int i = 0; i<16; i++)
//          DEBUG_PRINT(" %02x", csd[i]);
//        DEBUG_PRINT("\r\n");
//      }
    } else { // SDC V1 or MMC
      uint8_t cmd = (SD_SendCmd(ACMD41, 0) <= 1) ? ACMD41 : CMD1; // cmd for idle state
      DEBUG_PRINT(" CMD8 Fail, cmd = 0x%02x\r\n", cmd);
      while(SD_SendCmd(cmd, 0) && --cnt)                          // Wait idle state (depend from card type)
        chThdSleepMilliseconds(10);
      if (cnt && SD_SendCmd(CMD16, SD_SECTOR_SIZE) == 0)          // SET_BLOCKLEN and set type
        type = cmd == ACMD41 ? CT_SD1 : CT_MMC;
      DEBUG_PRINT(" CMD16 %d %d\r\n", cnt, type);
    }
  }
  SD_Unselect_SPI();
  DEBUG_PRINT("CardType %d\r\n", type);
  if (type)
    CardStatus = CT_POWER_ON | type;
  return disk_status(pdrv);
}

// diskio.c - Return disk status
DSTATUS disk_status(BYTE pdrv) {
  if (pdrv != 0) return STA_NOINIT;
  return CardStatus == 0 ? STA_NOINIT : 0;
}

// diskio.c - Read sector
DRESULT disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count) {
  // No disk or wrong block count
  if (pdrv != 0 || !(CardStatus & CT_POWER_ON)) return RES_NOTRDY;
#if DEBUG == 1
  r_cnt+= count;
  r_time-= chVTGetSystemTimeX();
#endif
  SD_Select_SPI(SD_SPI_RX_SPEED);
  // convert to byte address if no block mode
  if (!(CardStatus & CT_BLOCK)) sector*= SD_SECTOR_SIZE;
  if (count == 1) {                               // Read single block
    if (SD_SendCmd(CMD17, sector) == 0 && SD_RxDataBlock(buff, SD_SECTOR_SIZE, SD_TOKEN_START_BLOCK))
      count--;
  } else if (SD_SendCmd(CMD18, sector) == 0) {    // Read multiple blocks
    while(SD_RxDataBlock(buff, SD_SECTOR_SIZE, SD_TOKEN_START_BLOCK) && --count)
      buff+= SD_SECTOR_SIZE;
    SD_SendCmd(CMD12, 0);                         // Finish multiple block read
  }
  SD_Unselect_SPI();
#if DEBUG == 1
  r_time+= chVTGetSystemTimeX();
  if (count)
    DEBUG_PRINT(" err READ_BLOCK %d 0x%08x\r\n", count, sector);
#if 0
  else{
    DEBUG_PRINT("Sector read 0x%08x %d \r\n", sector, count);
    for (UINT j = 0; j < 32; j++){
      for (UINT i = 0; i < 16; i++)
        DEBUG_PRINT(" 0x%02x", buff[j*16 + i]);
      DEBUG_PRINT("\r\n");
    }
  }
#endif
#endif
  return count ? RES_ERROR : RES_OK;
}

// diskio.c - Write sector
DRESULT disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count) {
  // No disk or wrong block count
  if (pdrv != 0 || !(CardStatus & CT_POWER_ON)) return RES_NOTRDY;
  // Write protection
  if (CardStatus & CT_WRPROTECT) return RES_WRPRT;
#if DEBUG == 1
#if 0
    DEBUG_PRINT("Sector write 0x%08x, %d\r\n", sector, count);
    for (UINT j = 0; j < 32; j++){
      for (UINT i = 0; i < 16; i++)
        DEBUG_PRINT(" 0x%02x", buff[j*16 + i]);
      DEBUG_PRINT("\r\n");
    }
#endif
  w_cnt+=count;
  w_time-= chVTGetSystemTimeX();
#endif
  SD_Select_SPI(SD_SPI_SPEED);
#if 1                                                 // Write multiple block mode
  // convert to byte address if no block mode
  if (!(CardStatus & CT_BLOCK)) sector*= SD_SECTOR_SIZE;
  if (count == 1) {                                   // Write single block
    if (SD_SendCmd(CMD24, sector) == 0 && SD_TxDataBlock(buff, SD_SECTOR_SIZE, SD_TOKEN_START_BLOCK))
      count--;
  } else if (SD_SendCmd(CMD25, sector) == 0) {        // Write multiple blocks, wait busy
    while (SD_TxDataBlock(buff, SD_SECTOR_SIZE, SD_TOKEN_START_M_BLOCK) && SD_WaitNotBusy(MS2ST(250)) == 0xFF && --count)
      buff+= SD_SECTOR_SIZE;
    spi_TxByte(SD_TOKEN_STOP_M_BLOCK);                // Finish multiple block write
  }
#else
  while(SD_SendCmd(CMD24, (CardStatus & CT_BLOCK) ? sector : sector * SD_SECTOR_SIZE) == 0 && // Write block command
        SD_TxDataBlock(buff, SD_SECTOR_SIZE, SD_TOKEN_START_BLOCK) && --count) {              // Write data, dec count
    sector++;
    buff+= SD_SECTOR_SIZE;
  }
#endif
  SD_Unselect_SPI();
#if DEBUG == 1
  w_time+= chVTGetSystemTimeX();
  if (count)
    DEBUG_PRINT(" WRITE_BLOCK %d 0x%08x\r\n", count, sector);
#endif

  return count ? RES_ERROR : RES_OK;
}

// The disk_ioctl function is called to control device specific features and miscellaneous functions other than generic read/write.
// Implement only five device independent commands used by FatFS module
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void* buff) {
  (void)buff;
  DRESULT res = RES_PARERR;
  // No disk or not ready
  if (pdrv != 0 || !(CardStatus & CT_POWER_ON)) return RES_NOTRDY;
  SD_Select_SPI(SD_SPI_RX_SPEED);
  switch (cmd){
    // Makes sure that the device has finished pending write process.
    // If the disk I/O layer or storage device has a write-back cache,
    // the dirty cache data must be committed to media immediately.
    // Nothing to do for this command if each write operation to the media is completed
    // within the disk_write function.
    case CTRL_SYNC:
      if (SD_WaitNotBusy(MS2ST(200)) == 0xFF) res = RES_OK;
    break;
#if FF_USE_TRIM == 1
    // Informs the device the data on the block of sectors is no longer needed and it can be erased.
    // The sector block is specified in an LBA_t array {<Start LBA>, <End LBA>} pointed by buff.
    // This is an identical command to Trim of ATA device. Nothing to do for this command if this function
    // is not supported or not a flash memory device. FatFs does not check the result code and the file function
    // is not affected even if the sector block was not erased well. This command is called on remove a cluster chain
    // and in the f_mkfs function. It is required when FF_USE_TRIM == 1.
    case CTRL_TRIM:
    break;
#endif
#if FF_MAX_SS > FF_MIN_SS
    // Retrieves sector size used for read/write function into the WORD variable pointed by buff.
    // Valid sector sizes are 512, 1024, 2048 and 4096. This command is required only if FF_MAX_SS > FF_MIN_SS.
    // When FF_MAX_SS == FF_MIN_SS, this command will be never used and the read/write function must work in FF_MAX_SS bytes/sector only.
    case GET_SECTOR_SIZE:
      *(uint16_t*) buff = SD_SECTOR_SIZE;
      res = RES_OK;
    break;
#endif
#if FF_USE_MKFS == 1
    // Retrieves erase block size of the flash memory media in unit of sector into the DWORD variable pointed by buff.
    // The allowable value is 1 to 32768 in power of 2. Return 1 if the erase block size is unknown or non flash memory media.
    // This command is used by only f_mkfs function and it attempts to align data area on the erase block boundary.
    // It is required when FF_USE_MKFS == 1.
    case GET_BLOCK_SIZE:
   	 *(uint16_t*) buff = ;//SD_SECTOR_SIZE;
      res = RES_OK;
    break;
    // Retrieves number of available sectors, the largest allowable LBA + 1, on the drive into the LBA_t variable pointed by buff.
    // This command is used by f_mkfs and f_fdisk function to determine the size of volume/partition to be created.
    // It is required when FF_USE_MKFS == 1.
    case GET_SECTOR_COUNT:
    {
      // SEND_CSD
      uint8_t csd[16];
      if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16, SD_TOKEN_START_BLOCK)) {
        uint32_t n, csize;
        if ((csd[0] >> 6) == 1)  {  // SDC V2
          csize = ((uint32_t)csd[7]<<16)|((uint32_t)csd[8]<< 8)|((uint32_t)csd[9]<< 0);
          n = 10;
        }
        else {                      // MMC or SDC V1
          csize = ((uint32_t)csd[8]>>6)+((uint32_t)csd[7]<<2)+((uint32_t)(csd[6]&0x03)<<10);
          n = (csd[5]&0x0F)+((csd[10]&0x80)>>7)+((csd[9]&0x03)<<1) + 2 - 9;
        }
        *(uint32_t*)buff = (csize+1)<<n;
        res = RES_OK;
      }
    }
    break;
#endif
  }
  SD_Unselect_SPI();
  DEBUG_PRINT("disk_ioctl(%d) = %d,\r\n", cmd, res);
#if DEBUG == 1
  testLog();
#endif
  return res;
}
#endif //__USE_SD_CARD__
