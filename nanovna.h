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
#pragma once
#include "ch.h"

//#define __MS5351__
#define __ZEETK__

// Define LCD display driver and size
#if defined(NANOVNA_F303)
#define LCD_DRIVER_ST7796S
#define LCD_480x320
#else
// Used auto detect from ILI9341 or ST7789
#define LCD_DRIVER_ILI9341
#define LCD_DRIVER_ST7789
#define LCD_320x240
#endif

// Enable DMA mode for send data to LCD (Need enable HAL_USE_SPI in halconf.h)
#define __USE_DISPLAY_DMA__
// LCD or hardware allow change brightness, add menu item for this
#if defined(NANOVNA_F303)
#define __LCD_BRIGHTNESS__
#else
//#define __LCD_BRIGHTNESS__
#endif
// Use DAC (in H4 used for brightness used DAC, so need enable __LCD_BRIGHTNESS__ for it)
//#define __VNA_ENABLE_DAC__
// Allow enter to DFU from menu or command
#define __DFU_SOFTWARE_MODE__
// Add RTC clock support
#define __USE_RTC__
// Add RTC backup registers support
#define __USE_BACKUP__
// Add SD card support, req enable RTC (additional settings for file system see FatFS lib ffconf.h)
#define __USE_SD_CARD__
// Use unique serial string for USB
#define __USB_UID__
// If enabled serial in halconf.h, possible enable serial console control
#define __USE_SERIAL_CONSOLE__
// Add show y grid line values option
#define __USE_GRID_VALUES__
// Add remote desktop option
#define __REMOTE_DESKTOP__
// Add RLE8 compression capture image format
#define __CAPTURE_RLE8__
// Allow flip display
#define __FLIP_DISPLAY__
// Add shadow on text in plot area (improve readable, but little slowdown render)
#define _USE_SHADOW_TEXT_
// Faster draw line in cell algorithm (better clipping and faster)
// #define __VNA_FAST_LINES__
// Use build in table for sin/cos calculation, allow save a lot of flash space (this table also use for FFT), max sin/cos error = 4e-7
#define __VNA_USE_MATH_TABLES__
// Use custom fast/compact approximation for some math functions in calculations (vna_ ...), use it carefully
#define __USE_VNA_MATH__
// Use cache for window function used by FFT (but need FFT_SIZE*sizeof(float) RAM)
//#define USE_FFT_WINDOW_BUFFER
// Enable data smooth option
#define __USE_SMOOTH__
// Enable optional change digit separator for locales (dot or comma, need for correct work some external software)
#define __DIGIT_SEPARATOR__
// Use table for frequency list (if disabled use real time calc)
//#define __USE_FREQ_TABLE__
// Enable DSP instruction (support only by Cortex M4 and higher)
#ifdef ARM_MATH_CM4
#define __USE_DSP__
#endif
// Add measure module option (allow made some measure calculations on data)
#define __VNA_MEASURE_MODULE__
// Add Z normalization feature
//#define __VNA_Z_RENORMALIZATION__

/*
 * Submodules defines
 */
// If SD card enabled
#ifdef __USE_SD_CARD__
// Allow run commands from SD card (config.ini in root)
#define __SD_CARD_LOAD__
// Allow screenshots in TIFF format
#define __SD_CARD_DUMP_TIFF__
// Allow dump firmware to SD card
#define __SD_CARD_DUMP_FIRMWARE__
// Enable SD card file browser, and allow load files from it
#define __SD_FILE_BROWSER__
#endif

// If measure module enabled, add submodules
#ifdef __VNA_MEASURE_MODULE__
// Add LC match function
#define __USE_LC_MATCHING__
// Enable Series measure option
#define __S21_MEASURE__
// Enable S11 cable measure option
#define __S11_CABLE_MEASURE__
// Enable S11 resonance search option
#define __S11_RESONANCE_MEASURE__
#endif

/*
 * Hardware depends options for VNA
 */
#if defined(NANOVNA_F303)
// Define ADC sample rate in kilobyte (can be 48k, 96k, 192k, 384k)
//#define AUDIO_ADC_FREQ_K        768
//#define AUDIO_ADC_FREQ_K        384
#define AUDIO_ADC_FREQ_K        192
//#define AUDIO_ADC_FREQ_K        96
//#define AUDIO_ADC_FREQ_K        48

// Define sample count for one step measure
#define AUDIO_SAMPLES_COUNT   (48)
//#define AUDIO_SAMPLES_COUNT   (96)
//#define AUDIO_SAMPLES_COUNT   (192)

// Frequency offset, depend from AUDIO_ADC_FREQ settings (need aligned table)
// Use real time build table (undef for use constant, see comments)
// Constant tables build only for AUDIO_SAMPLES_COUNT = 48
#define USE_VARIABLE_OFFSET

// Maximum sweep point count (limit by flash and RAM size)
#define SWEEP_POINTS_MAX         401

#define AUDIO_ADC_FREQ_K1        384
#else
//#define AUDIO_ADC_FREQ_K        768
//#define AUDIO_ADC_FREQ_K        384
#define AUDIO_ADC_FREQ_K        192
//#define AUDIO_ADC_FREQ_K        96
//#define AUDIO_ADC_FREQ_K        48

// Define sample count for one step measure
#define AUDIO_SAMPLES_COUNT   (48)
//#define AUDIO_SAMPLES_COUNT   (96)
//#define AUDIO_SAMPLES_COUNT   (192)

// Frequency offset, depend from AUDIO_ADC_FREQ settings (need aligned table)
// Use real time build table (undef for use constant, see comments)
// Constant tables build only for AUDIO_SAMPLES_COUNT = 48
#define USE_VARIABLE_OFFSET

// Maximum sweep point count (limit by flash and RAM size)
#define SWEEP_POINTS_MAX         101
#endif
// Minimum sweep point count
#define SWEEP_POINTS_MIN         21

// Dirty hack for H4 ADC speed in version screen (Need for correct work NanoVNA-App)
#ifndef AUDIO_ADC_FREQ_K1
#define AUDIO_ADC_FREQ_K1        AUDIO_ADC_FREQ_K
#endif

/*
 * main.c
 */
// Minimum frequency set
#ifdef __ZEETK__
#define FREQUENCY_MIN            600
#else
#define FREQUENCY_MIN            1600
#endif

// Maximum frequency set
#define FREQUENCY_MAX            2000000000U

// Frequency threshold (max frequency for si5351, harmonic mode after)
#ifdef __ZEETK__
#define FREQUENCY_THRESHOLD      300000110U
#else
#define FREQUENCY_THRESHOLD      290000110U
#endif

// XTAL frequency on si5351
#define XTALFREQ                 26000000U
// Define i2c bus speed, add predefined for 400k, 600k, 900k
#define STM32_I2C_SPEED          900
// Define default src impedance for xtal calculations
#define MEASURE_DEFAULT_R        50.0f

// Add IF select menu in expert settings
#ifdef USE_VARIABLE_OFFSET
#define USE_VARIABLE_OFFSET_MENU
#endif

#if AUDIO_ADC_FREQ_K == 768
#define FREQUENCY_OFFSET_STEP    16000
// For 768k ADC    (16k step for 48 samples)
#define FREQUENCY_IF_K          8    // only  96 samples and variable table
//#define FREQUENCY_IF_K         12  // only 192 samples and variable table
//#define FREQUENCY_IF_K         16
//#define FREQUENCY_IF_K         32
//#define FREQUENCY_IF_K         48
//#define FREQUENCY_IF_K         64

#elif AUDIO_ADC_FREQ_K == 384
#define FREQUENCY_OFFSET_STEP    4000
// For 384k ADC    (8k step for 48 samples)
//#define FREQUENCY_IF_K          8
#define FREQUENCY_IF_K         12  // only 96 samples and variable table
//#define FREQUENCY_IF_K         16
//#define FREQUENCY_IF_K         20  // only 96 samples and variable table
//#define FREQUENCY_IF_K         24
//#define FREQUENCY_IF_K         32

#elif AUDIO_ADC_FREQ_K == 192
#define FREQUENCY_OFFSET_STEP    4000
// For 192k ADC (sin_cos table in dsp.c generated for 8k, 12k, 16k, 20k, 24k if change need create new table )
//#define FREQUENCY_IF_K          8
#define FREQUENCY_IF_K         12
//#define FREQUENCY_IF_K         16
//#define FREQUENCY_IF_K         20
//#define FREQUENCY_IF_K         24
//#define FREQUENCY_IF_K         28

#elif AUDIO_ADC_FREQ_K == 96
#define FREQUENCY_OFFSET_STEP    2000
// For 96k ADC (sin_cos table in dsp.c generated for 6k, 8k, 10k, 12k if change need create new table )
//#define FREQUENCY_IF_K          6
//#define FREQUENCY_IF_K          8
//#define FREQUENCY_IF_K         10
#define FREQUENCY_IF_K         12

#elif AUDIO_ADC_FREQ_K == 48
#define FREQUENCY_OFFSET_STEP    1000
// For 48k ADC (sin_cos table in dsp.c generated for 3k, 4k, 5k, 6k, if change need create new table )
//#define FREQUENCY_IF_K          3
//#define FREQUENCY_IF_K          4
//#define FREQUENCY_IF_K          5
#define FREQUENCY_IF_K          6
//#define FREQUENCY_IF_K          7
#endif

/*
 * CPU Hardware depend functions declaration
 */
#include "hardware.h"

#define AUDIO_ADC_FREQ       (AUDIO_ADC_FREQ_K*1000)
#define FREQUENCY_OFFSET     (FREQUENCY_IF_K*1000)

// Apply calibration after made sweep, (if set 1, then calibration move out from sweep cycle)
#define APPLY_CALIBRATION_AFTER_SWEEP 0

// Speed of light const
#define SPEED_OF_LIGHT           299792458

// pi const
#define VNA_PI                   3.14159265358979323846f
#define VNA_TWOPI                6.28318530717958647692f

// Define frequency range (can be unsigned)
typedef uint32_t freq_t;

// Optional sweep point (in UI menu)
#if SWEEP_POINTS_MAX >=401
#define POINTS_SET_COUNT       5
#define POINTS_SET             {51, 101, 201, 301, SWEEP_POINTS_MAX}
#define POINTS_COUNT_DEFAULT   101
#elif SWEEP_POINTS_MAX >=301
#define POINTS_SET_COUNT       4
#define POINTS_SET             {51, 101, 201, SWEEP_POINTS_MAX}
#define POINTS_COUNT_DEFAULT   101
#elif SWEEP_POINTS_MAX >=201
#define POINTS_SET_COUNT       3
#define POINTS_SET             {51, 101, SWEEP_POINTS_MAX}
#define POINTS_COUNT_DEFAULT   101
#elif SWEEP_POINTS_MAX >=101
#define POINTS_SET_COUNT       2
#define POINTS_SET             {51, SWEEP_POINTS_MAX}
#define POINTS_COUNT_DEFAULT   SWEEP_POINTS_MAX
#endif

extern float measured[2][SWEEP_POINTS_MAX][2];

#define CAL_TYPE_COUNT  5
#define CAL_LOAD        0
#define CAL_OPEN        1
#define CAL_SHORT       2
#define CAL_THRU        3
#define CAL_ISOLN       4

#define CALSTAT_LOAD (1<<0)
#define CALSTAT_OPEN (1<<1)
#define CALSTAT_SHORT (1<<2)
#define CALSTAT_THRU (1<<3)
#define CALSTAT_ISOLN (1<<4)
#define CALSTAT_ES (1<<5)
#define CALSTAT_ER (1<<6)
#define CALSTAT_ET (1<<7)
#define CALSTAT_ED CALSTAT_LOAD
#define CALSTAT_EX CALSTAT_ISOLN
#define CALSTAT_APPLY (1<<8)
#define CALSTAT_INTERPOLATED (1<<9)
#define CALSTAT_ENHANCED_RESPONSE (1<<10)

#define ETERM_ED 0 /* error term directivity */
#define ETERM_ES 1 /* error term source match */
#define ETERM_ER 2 /* error term refrection tracking */
#define ETERM_ET 3 /* error term transmission tracking */
#define ETERM_EX 4 /* error term isolation */

#if   SWEEP_POINTS_MAX <= 256
#define FFT_SIZE   256
#elif SWEEP_POINTS_MAX <= 512
#define FFT_SIZE   512
#endif

void cal_collect(uint16_t type);
void cal_done(void);

#define MAX_FREQ_TYPE 5
enum stimulus_type {
  ST_START=0, ST_STOP, ST_CENTER, ST_CW, ST_SPAN, ST_STEP, ST_VAR
};

freq_t getFrequency(uint16_t idx);
freq_t getFrequencyStep(void);

void   set_marker_index(int m, int idx);
freq_t get_marker_frequency(int marker);

void   reset_sweep_frequency(void);
void   set_sweep_frequency(uint16_t type, freq_t frequency);

void set_bandwidth(uint16_t bw_count);
uint32_t get_bandwidth_frequency(uint16_t bw_freq);

void set_power(uint8_t value);

void    set_smooth_factor(uint8_t factor);
uint8_t get_smooth_factor(void);

int32_t  my_atoi(const char *p);
uint32_t my_atoui(const char *p);
float    my_atof(const char *p);
bool strcmpi(const char *t1, const char *t2);
int get_str_index(const char *v, const char *list);
int parse_line(char *line, char* args[], int max_cnt);
void swap_bytes(uint16_t *buf, int size);
int packbits(char *source, char *dest, int size);
void _delay_8t(uint32_t cycles);
inline void delayMicroseconds(uint32_t us) {_delay_8t(us*STM32_CORE_CLOCK/8);}
inline void delayMilliseconds(uint32_t ms) {_delay_8t(ms*125*STM32_CORE_CLOCK);}

void pause_sweep(void);
void toggle_sweep(void);
int  load_properties(uint32_t id);

#ifdef __USE_BACKUP__
void update_backup_data(void);
#endif

void set_sweep_points(uint16_t points);

bool sd_card_load_config(void);
void VNAShell_executeCMDLine(char *line);

#ifdef __REMOTE_DESKTOP__
// State flags for remote touch state
#define REMOTE_NONE     0
#define REMOTE_PRESS    1
#define REMOTE_RELEASE  2
typedef struct {
  char new_str[6];
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
} remote_region_t;
void remote_touch_set(uint16_t state, int16_t x, int16_t y);
void send_region(remote_region_t *rd, uint8_t * buf, uint16_t size);
#endif

#define SWEEP_ENABLE  0x01
#define SWEEP_ONCE    0x02
#define SWEEP_BINARY  0x08
#define SWEEP_REMOTE  0x40
#define SWEEP_UI_MODE 0x80

extern  uint8_t sweep_mode;
extern const char *info_about[];

/*
 * Measure timings for si5351 generator, after ready
 */
// Enable si5351 timing command, used for debug align times
//#define ENABLE_SI5351_TIMINGS
#if defined(NANOVNA_F303)
// Generator ready delays, values in us
#define DELAY_BAND_1_2           US2ST( 100)   // Delay for bands 1-2
#define DELAY_BAND_3_4           US2ST( 200)   // Delay for bands 3-4
#define DELAY_BANDCHANGE         US2ST(5000)   // Band changes need set additional delay after reset PLL
#define DELAY_CHANNEL_CHANGE     US2ST( 400)   // Switch channel delay
#define DELAY_SWEEP_START        US2ST(2000)   // Delay at sweep start
// Delay after set new PLL values in ms, and send reset
#define DELAY_RESET_PLL_BEFORE            0    //    0 (0 for disabled)
#define DELAY_RESET_PLL_AFTER          4000    // 4000 (0 for disabled)
#else
// Generator ready delays, values in us
#define DELAY_BAND_1_2           US2ST( 100)   // 0 Delay for bands 1-2
#define DELAY_BAND_3_4           US2ST( 200)   // 1 Delay for bands 3-4
#define DELAY_BANDCHANGE         US2ST( 5000)   // 2 Band changes need set additional delay after reset PLL
#define DELAY_CHANNEL_CHANGE     US2ST( 400)   // 3 Switch channel delay
#define DELAY_SWEEP_START        US2ST( 2000)   // 4 Delay at sweep start
// Delay after before/after set new PLL values in ms
#define DELAY_RESET_PLL_BEFORE           0    // 5    0 (0 for disabled)
#define DELAY_RESET_PLL_AFTER          4000    // 6 4000 (0 for disabled)
#endif

/*
 * dsp.c
 */
// Define aic3204 source clock frequency (on 8MHz used fractional multiplier, and possible little phase error)
#define AUDIO_CLOCK_REF       (8000000U)
// Define aic3204 source clock frequency (on 12288000U used integer multiplier)
//#define AUDIO_CLOCK_REF       (12288000U)
// Disable AIC PLL clock, use input as CODEC_CLKIN (not stable on some devices, on long work)
//#define AUDIO_CLOCK_REF       (98304000U)

// Buffer contain left and right channel samples (need x2)
#define AUDIO_BUFFER_LEN      (AUDIO_SAMPLES_COUNT*2)

// Bandwidth depend from AUDIO_SAMPLES_COUNT and audio ADC frequency
// for AUDIO_SAMPLES_COUNT = 48 and ADC =  48kHz one measure give  48000/48=1000Hz
// for AUDIO_SAMPLES_COUNT = 48 and ADC =  96kHz one measure give  96000/48=2000Hz
// for AUDIO_SAMPLES_COUNT = 48 and ADC = 192kHz one measure give 192000/48=4000Hz
// Define additional measure count for menus
#if AUDIO_ADC_FREQ/AUDIO_SAMPLES_COUNT == 16000
#define BANDWIDTH_8000            (  1 - 1)
#define BANDWIDTH_4000            (  2 - 1)
#define BANDWIDTH_1000            (  8 - 1)
#define BANDWIDTH_333             ( 24 - 1)
#define BANDWIDTH_100             ( 80 - 1)
#define BANDWIDTH_30              (256 - 1)
#elif AUDIO_ADC_FREQ/AUDIO_SAMPLES_COUNT == 8000
#define BANDWIDTH_8000            (  1 - 1)
#define BANDWIDTH_4000            (  2 - 1)
#define BANDWIDTH_1000            (  8 - 1)
#define BANDWIDTH_333             ( 24 - 1)
#define BANDWIDTH_100             ( 80 - 1)
#define BANDWIDTH_30              (256 - 1)
#elif AUDIO_ADC_FREQ/AUDIO_SAMPLES_COUNT == 4000
#define BANDWIDTH_4000            (  1 - 1)
#define BANDWIDTH_2000            (  2 - 1)
#define BANDWIDTH_1000            (  4 - 1)
#define BANDWIDTH_333             ( 12 - 1)
#define BANDWIDTH_100             ( 40 - 1)
#define BANDWIDTH_30              (132 - 1)
#elif AUDIO_ADC_FREQ/AUDIO_SAMPLES_COUNT == 2000
#define BANDWIDTH_2000            (  1 - 1)
#define BANDWIDTH_1000            (  2 - 1)
#define BANDWIDTH_333             (  6 - 1)
#define BANDWIDTH_100             ( 20 - 1)
#define BANDWIDTH_30              ( 66 - 1)
#define BANDWIDTH_10              (200 - 1)
#elif AUDIO_ADC_FREQ/AUDIO_SAMPLES_COUNT == 1000
#define BANDWIDTH_1000            (  1 - 1)
#define BANDWIDTH_333             (  3 - 1)
#define BANDWIDTH_100             ( 10 - 1)
#define BANDWIDTH_30              ( 33 - 1)
#define BANDWIDTH_10              (100 - 1)
#endif

typedef int16_t  audio_sample_t;
void dsp_process(audio_sample_t *src, size_t len);
void reset_dsp_accumerator(void);
void calculate_gamma(float *gamma);
void fetch_amplitude(float *gamma);
void fetch_amplitude_ref(float *gamma);
void generate_DSP_Table(int offset);

/*
 * tlv320aic3204.c
 */

void tlv320aic3204_init(void);
void tlv320aic3204_set_gain(uint8_t lgain, uint8_t rgain);
void tlv320aic3204_select(uint8_t channel);
void tlv320aic3204_write_reg(uint8_t page, uint8_t reg, uint8_t data);

/*
 * vna_math.c
 */
#include "vna_math.h"

/*
 * plot.c
 */
// LCD display size settings
#ifdef LCD_320x240 // 320x240 display plot definitions
#define LCD_WIDTH                   320
#define LCD_HEIGHT                  240

// Define maximum distance in pixel for pickup marker (can be bigger for big displays)
#define MARKER_PICKUP_DISTANCE       20
// Used marker image settings
#define _USE_MARKER_SET_              1
// Used font settings
#define _USE_FONT_                    1
#define _USE_SMALL_FONT_              0

// Plot area size settings
// Offset of plot area (size of additional info at left side)
#define OFFSETX                      10
#define OFFSETY                       0

// Grid count, must divide
//#define NGRIDY                     10
#define NGRIDY                        8

// Plot area WIDTH better be n*(POINTS_COUNT-1)
#define WIDTH                       300
// Plot area HEIGHT = NGRIDY * GRIDY
#define HEIGHT                      232

// GRIDX calculated depends from frequency span
// GRIDY depend from HEIGHT and NGRIDY, must be integer
#define GRIDY                       (HEIGHT / NGRIDY)

// Need for reference marker draw
#define CELLOFFSETX                   5
#define AREA_WIDTH_NORMAL           (CELLOFFSETX + WIDTH  + 1 + 4)
#define AREA_HEIGHT_NORMAL          (              HEIGHT + 1)

// Smith/polar chart
#define P_CENTER_X                  (CELLOFFSETX + WIDTH/2)
#define P_CENTER_Y                  (HEIGHT/2)
#define P_RADIUS                    (HEIGHT/2)

// Other settings (battery/calibration/frequency text position)
// Battery icon position
#define BATTERY_ICON_POSX             1
#define BATTERY_ICON_POSY             1

// Calibration text coordinates
#define CALIBRATION_INFO_POSX         0
#define CALIBRATION_INFO_POSY       100

#define FREQUENCIES_XPOS1           OFFSETX
#define FREQUENCIES_XPOS2           (LCD_WIDTH - 23 * sFONT_WIDTH)
#define FREQUENCIES_XPOS3           (LCD_WIDTH/2 + OFFSETX - 16 * sFONT_WIDTH / 2)
#define FREQUENCIES_YPOS            (AREA_HEIGHT_NORMAL)
#endif // end 320x240 display plot definitions

#ifdef LCD_480x320 // 480x320 display definitions
#define LCD_WIDTH                   480
#define LCD_HEIGHT                  320

// Define maximum distance in pixel for pickup marker (can be bigger for big displays)
#define MARKER_PICKUP_DISTANCE       30
// Used marker image settings
#define _USE_MARKER_SET_              2
// Used font settings
#define _USE_FONT_                    2
#define _USE_SMALL_FONT_              2

// Plot area size settings
// Offset of plot area (size of additional info at left side)
#define OFFSETX                      15
#define OFFSETY                       0

// Grid count, must divide
//#define NGRIDY                     10
#define NGRIDY                        8

// Plot area WIDTH better be n*(POINTS_COUNT-1)
#define WIDTH                       455
// Plot area HEIGHT = NGRIDY * GRIDY
#define HEIGHT                      304

// GRIDX calculated depends from frequency span
// GRIDY depend from HEIGHT and NGRIDY, must be integer
#define GRIDY                       (HEIGHT / NGRIDY)

// Need for reference marker draw
#define CELLOFFSETX                   5
#define AREA_WIDTH_NORMAL           (CELLOFFSETX + WIDTH  + 1 + 4)
#define AREA_HEIGHT_NORMAL          (              HEIGHT + 1)

// Smith/polar chart
#define P_CENTER_X                  (CELLOFFSETX + WIDTH/2)
#define P_CENTER_Y                  (HEIGHT/2)
#define P_RADIUS                    (HEIGHT/2)

// Other settings (battery/calibration/frequency text position)
// Battery icon position
#define BATTERY_ICON_POSX             3
#define BATTERY_ICON_POSY             2

// Calibration text coordinates
#define CALIBRATION_INFO_POSX         0
#define CALIBRATION_INFO_POSY       100

#define FREQUENCIES_XPOS1           OFFSETX
#define FREQUENCIES_XPOS2           (LCD_WIDTH - 22 * sFONT_WIDTH)
#define FREQUENCIES_XPOS3           (LCD_WIDTH/2 + OFFSETX - 14 * sFONT_WIDTH / 2)
#define FREQUENCIES_YPOS            (AREA_HEIGHT_NORMAL + 2)
#endif // end 480x320 display plot definitions

// UI size defines
// Text offset in menu
#define MENU_TEXT_OFFSET              6
#define MENU_ICON_OFFSET              4
// Scale / ref quick touch position
#define UI_SCALE_REF_X0             (OFFSETX - 5)
#define UI_SCALE_REF_X1             (OFFSETX + CELLOFFSETX + 10)
// Leveler Marker mode select
#define UI_MARKER_Y0                 30
// Maximum menu buttons count
#define MENU_BUTTON_MAX              16
#define MENU_BUTTON_MIN               8
// Menu buttons y offset
#define MENU_BUTTON_Y_OFFSET          1
// Menu buttons size = 21 for icon and 10 chars
#define MENU_BUTTON_WIDTH           (7 + 12 * FONT_WIDTH)
#define MENU_BUTTON_HEIGHT(n)       (AREA_HEIGHT_NORMAL/(n))
#define MENU_BUTTON_BORDER            1
#define KEYBOARD_BUTTON_BORDER        1
#define BROWSER_BUTTON_BORDER         1
// Browser window settings
#define FILES_COLUMNS               (LCD_WIDTH/160)                                // columns in browser
#define FILES_ROWS                   10                                            // rows in browser
#define FILES_PER_PAGE              (FILES_COLUMNS*FILES_ROWS)                     // FILES_ROWS * FILES_COLUMNS
#define FILE_BOTTOM_HEIGHT           20                                            // Height of bottom buttons (< > X)
#define FILE_BUTTON_HEIGHT          ((LCD_HEIGHT - FILE_BOTTOM_HEIGHT)/FILES_ROWS) // Height of file buttons

// Define message box width
#define MESSAGE_BOX_WIDTH           180

// Height of numerical input field (at bottom)
#define NUM_INPUT_HEIGHT             32
// On screen keyboard button size
#if 1 // Full screen keyboard
#define KP_WIDTH                  (LCD_WIDTH / 4)                                  // numeric keypad button width
#define KP_HEIGHT                 ((LCD_HEIGHT - NUM_INPUT_HEIGHT) / 4)            // numeric keypad button height
#define KP_X_OFFSET               0                                                // numeric keypad X offset
#define KP_Y_OFFSET               0                                                // numeric keypad Y offset
#define KPF_WIDTH                 (LCD_WIDTH / 10)                                 // text keypad button width
#define KPF_HEIGHT                KPF_WIDTH                                        // text keypad button height
#define KPF_X_OFFSET              0                                                // text keypad X offset
#define KPF_Y_OFFSET              (LCD_HEIGHT - NUM_INPUT_HEIGHT - 4 * KPF_HEIGHT) // text keypad Y offset
#else // 64 pixel size keyboard
#define KP_WIDTH                 64                                                // numeric keypad button width
#define KP_HEIGHT                64                                                // numeric keypad button height
#define KP_X_OFFSET              (LCD_WIDTH-MENU_BUTTON_WIDTH-16-KP_WIDTH*4)       // numeric keypad X offset
#define KP_Y_OFFSET              20                                                // numeric keypad Y offset
#define KPF_WIDTH                (LCD_WIDTH / 10)                                  // text keypad button width
#define KPF_HEIGHT               KPF_WIDTH                                         // text keypad button height
#define KPF_X_OFFSET              0                                                // text keypad X offset
#define KPF_Y_OFFSET             (LCD_HEIGHT - NUM_INPUT_HEIGHT - 4 * KPF_HEIGHT)  // text keypad Y offset
#endif

/*
 * Font size defines
 */
#define FONT_SMALL           0
#define FONT_NORMAL          1

#if _USE_FONT_ == 0
extern const uint8_t x5x7_bits[];
#define FONT_START_CHAR   0x16
#define FONT_WIDTH           5
#define FONT_GET_HEIGHT      7
#define FONT_STR_HEIGHT      8
#define FONT_GET_DATA(ch)    (  &x5x7_bits[(ch-FONT_START_CHAR)*FONT_GET_HEIGHT])
#define FONT_GET_WIDTH(ch)   (8-(x5x7_bits[(ch-FONT_START_CHAR)*FONT_GET_HEIGHT]&0x7))

#elif _USE_FONT_ == 1
extern const uint8_t x6x10_bits[];
#define FONT_START_CHAR   0x16
#define FONT_WIDTH           6
#define FONT_GET_HEIGHT     10
#define FONT_STR_HEIGHT     11
#define FONT_GET_DATA(ch)   (  &x6x10_bits[(ch-FONT_START_CHAR)*FONT_GET_HEIGHT])
#define FONT_GET_WIDTH(ch)  (8-(x6x10_bits[(ch-FONT_START_CHAR)*FONT_GET_HEIGHT]&0x7))

#elif _USE_FONT_ == 2
extern const uint8_t x7x11b_bits[];
#define FONT_START_CHAR   0x16
#define FONT_WIDTH           7
#define FONT_GET_HEIGHT     11
#define FONT_STR_HEIGHT     11
#define FONT_GET_DATA(ch)   (  &x7x11b_bits[(ch-FONT_START_CHAR)*FONT_GET_HEIGHT])
#define FONT_GET_WIDTH(ch)  (8-(x7x11b_bits[(ch-FONT_START_CHAR)*FONT_GET_HEIGHT]&7))

#elif _USE_FONT_ == 3
extern const uint8_t x11x14_bits[];
#define FONT_START_CHAR   0x16
#define FONT_WIDTH          11
#define FONT_GET_HEIGHT     14
#define FONT_STR_HEIGHT     16
#define FONT_GET_DATA(ch)   (   &x11x14_bits[(ch-FONT_START_CHAR)*2*FONT_GET_HEIGHT  ])
#define FONT_GET_WIDTH(ch)  (14-(x11x14_bits[(ch-FONT_START_CHAR)*2*FONT_GET_HEIGHT+1]&0x7))
#endif

#if _USE_SMALL_FONT_ == 0
extern const uint8_t x5x7_bits[];
#define sFONT_START_CHAR   0x16
#define sFONT_WIDTH           5
#define sFONT_GET_HEIGHT      7
#define sFONT_STR_HEIGHT      8
#define sFONT_GET_DATA(ch)    (  &x5x7_bits[(ch-sFONT_START_CHAR)*sFONT_GET_HEIGHT])
#define sFONT_GET_WIDTH(ch)   (8-(x5x7_bits[(ch-sFONT_START_CHAR)*sFONT_GET_HEIGHT]&0x7))

#elif _USE_SMALL_FONT_ == 1
extern const uint8_t x6x10_bits[];
#define sFONT_START_CHAR   0x16
#define sFONT_WIDTH           6
#define sFONT_GET_HEIGHT     10
#define sFONT_STR_HEIGHT     11
#define sFONT_GET_DATA(ch)   (  &x6x10_bits[(ch-sFONT_START_CHAR)*sFONT_GET_HEIGHT])
#define sFONT_GET_WIDTH(ch)  (8-(x6x10_bits[(ch-sFONT_START_CHAR)*sFONT_GET_HEIGHT]&0x7))

#elif _USE_SMALL_FONT_ == 2
extern const uint8_t x7x11b_bits[];
#define sFONT_START_CHAR   0x16
#define sFONT_WIDTH           7
#define sFONT_GET_HEIGHT     11
#define sFONT_STR_HEIGHT     11
#define sFONT_GET_DATA(ch)   (  &x7x11b_bits[(ch-sFONT_START_CHAR)*sFONT_GET_HEIGHT])
#define sFONT_GET_WIDTH(ch)  (8-(x7x11b_bits[(ch-sFONT_START_CHAR)*sFONT_GET_HEIGHT]&7))

#elif _USE_SMALL_FONT_ == 3
extern const uint8_t x11x14_bits[];
#define sFONT_START_CHAR   0x16
#define sFONT_WIDTH          11
#define sFONT_GET_HEIGHT     14
#define sFONT_STR_HEIGHT     16
#define sFONT_GET_DATA(ch)   (   &x11x14_bits[(ch-sFONT_START_CHAR)*2*sFONT_GET_HEIGHT  ])
#define sFONT_GET_WIDTH(ch)  (14-(x11x14_bits[(ch-sFONT_START_CHAR)*2*sFONT_GET_HEIGHT+1]&0x7))
#endif

#if _USE_FONT_ != _USE_SMALL_FONT_
void    lcd_set_font(int type);
#else
#define lcd_set_font(type) {}
#endif

extern const uint8_t numfont16x22[];
#define NUM_FONT_GET_WIDTH      16
#define NUM_FONT_GET_HEIGHT     22
#define NUM_FONT_GET_DATA(ch)   (&numfont16x22[ch*2*NUM_FONT_GET_HEIGHT])

// Glyph names from numfont16x22.c
enum {
  KP_0 = 0, KP_1, KP_2, KP_3, KP_4, KP_5, KP_6, KP_7, KP_8, KP_9,
  KP_PERIOD,
  KP_MINUS,
  KP_BS,
  KP_k, KP_M, KP_G,
  KP_m, KP_u, KP_n, KP_p,
  KP_X1, KP_ENTER, KP_PERCENT, // Enter values
#if 0
  KP_INF,
  KP_DB,
  KP_PLUSMINUS,
  KP_KEYPAD,
  KP_SPACE,
  KP_PLUS,
#endif
  // Special uint8_t buttons
  KP_EMPTY = 255  // Empty button
};

/*
 * LC match text output settings
 */
#ifdef __VNA_MEASURE_MODULE__
// X and Y offset to L/C match text
 #define STR_MEASURE_X      (OFFSETX +  0)
// Better be aligned by cell (cell height = 32)
 #define STR_MEASURE_Y      (OFFSETY + 80)
// 1/3 Width of text (use 3 column for data)
 #define STR_MEASURE_WIDTH  (FONT_WIDTH * 10)
// String Height (need 2 + 0..4 string)
 #define STR_MEASURE_HEIGHT (FONT_STR_HEIGHT + 1)
#endif

#ifdef __USE_GRID_VALUES__
#define GRID_X_TEXT   (WIDTH - 5 * sFONT_WIDTH)
#endif

// Render control chars
#define R_BGCOLOR  "\001"  // hex 0x01 set background color
#define R_FGCOLOR  "\002"  // hex 0x02 set foreground color

#define R_TEXT_COLOR "\002\001" // set  1 color index as foreground
#define R_LINK_COLOR "\002\031" // set 25 color index as foreground

// Additional chars in fonts
#define S_ENTER    "\026"  // hex 0x16
#define S_DELTA    "\027"  // hex 0x17
#define S_SARROW   "\030"  // hex 0x18
#define S_INFINITY "\031"  // hex 0x19
#define S_LARROW   "\032"  // hex 0x1A
#define S_RARROW   "\033"  // hex 0x1B
#define S_PI       "\034"  // hex 0x1C
#define S_MICRO    '\035'  // hex 0x1D
#define S_OHM      "\036"  // hex 0x1E
#define S_DEGREE   "\037"  // hex 0x1F
#define S_SIEMENS  "S"     //
#define S_dB       "dB"    //
#define S_Hz       "Hz"    //
#define S_FARAD    "F"     //
#define S_HENRY    "H"     //
#define S_SECOND   "s"     //
#define S_METRE    "m"     //
#define S_VOLT     "V"     //
#define S_AMPER    "A"     //
#define S_PPM      "ppm"   //

// Max palette indexes in config
#define MAX_PALETTE     32

// trace 
#define MAX_TRACE_TYPE 30
enum trace_type {
  TRC_LOGMAG=0, TRC_PHASE, TRC_DELAY, TRC_SMITH, TRC_POLAR, TRC_LINEAR, TRC_SWR, TRC_REAL, TRC_IMAG,
  TRC_R, TRC_X, TRC_Z, TRC_ZPHASE,
  TRC_G, TRC_B, TRC_Y, TRC_Rp, TRC_Xp,
  TRC_sC, TRC_sL,
  TRC_pC, TRC_pL,
  TRC_Q,
  TRC_Rser, TRC_Xser, TRC_Zser,
  TRC_Rsh, TRC_Xsh, TRC_Zsh,
  TRC_Qs21
};

// Mask for define rectangular plot
#define RECTANGULAR_GRID_MASK ((1<<TRC_LOGMAG)|(1<<TRC_PHASE)|(1<<TRC_DELAY)|(1<<TRC_LINEAR)|(1<<TRC_SWR)|(1<<TRC_REAL)|(1<<TRC_IMAG)\
                              |(1<<TRC_R)|(1<<TRC_X)|(1<<TRC_Z)|(1<<TRC_ZPHASE)\
                              |(1<<TRC_G)|(1<<TRC_B)|(1<<TRC_Y)|(1<<TRC_Rp)|(1<<TRC_Xp)\
                              |(1<<TRC_sC)|(1<<TRC_sL)\
                              |(1<<TRC_pC)|(1<<TRC_pL)\
                              |(1<<TRC_Q)\
                              |(1<<TRC_Rser)|(1<<TRC_Xser)|(1<<TRC_Zser)\
                              |(1<<TRC_Rsh)|(1<<TRC_Xsh)|(1<<TRC_Zsh)\
                              |(1<<TRC_Qs21))

// complex graph type (polar / smith / admit)
#define ROUND_GRID_MASK ((1<<TRC_POLAR)|(1<<TRC_SMITH))
// Scale / Amplitude input in nano/pico values graph type
#define NANO_TYPE_MASK        ((1<<TRC_DELAY)|(1<<TRC_sC)|(1<<TRC_sL)|(1<<TRC_pC)|(1<<TRC_pL))

// Trace info description structure
typedef float (*get_value_cb_t)(int idx, const float *v); // get value callback
typedef struct trace_info {
  const char *name;            // Trace name
  const char *format;          // trace value printf format for marker output
  const char *dformat;         // delta value printf format
  const char *symbol;          // value symbol
  float refpos;                // default refpos
  float scale_unit;            // default scale
  get_value_cb_t get_value_cb; // get value callback (can be NULL, in this case need add custom calculations)
} trace_info_t;
// Trace render options in plot.c
extern const trace_info_t trace_info_list[MAX_TRACE_TYPE];

// marker smith value format
enum {MS_LIN, MS_LOG, MS_REIM, MS_RX, MS_RLC, MS_GB, MS_GLC, MS_RpXp, MS_RpLC, MS_SHUNT_RX, MS_SHUNT_RLC, MS_SERIES_RX, MS_SERIES_RLC, MS_END};
#define ADMIT_MARKER_VALUE(v)    ((1<<(v))&((1<<MS_GB)|(1<<MS_GLC)|(1<<MS_RpXp)|(1<<MS_RpLC)))
#define LC_MARKER_VALUE(v)       ((1<<(v))&((1<<MS_RLC)|(1<<MS_GLC)|(1<<MS_RpLC)|(1<<MS_SHUNT_RLC)|(1<<MS_SERIES_RLC)))

#define S11_SMITH_VALUE(v)       ((1<<(v))&((1<<MS_LIN)|(1<<MS_LOG)|(1<<MS_REIM)|(1<<MS_RX)|(1<<MS_RLC)|(1<<MS_GB)|(1<<MS_GLC)|(1<<MS_RpXp)|(1<<MS_RpLC)))
#define S21_SMITH_VALUE(v)       ((1<<(v))&((1<<MS_LIN)|(1<<MS_LOG)|(1<<MS_REIM)|(1<<MS_SHUNT_RX)|(1<<MS_SHUNT_RLC)|(1<<MS_SERIES_RX)|(1<<MS_SERIES_RLC)))

typedef struct {
  const char *name;         // Trace name
  const char *format;       // trace value printf format for marker output
  get_value_cb_t get_re_cb; // get real value callback
  get_value_cb_t get_im_cb; // get imag value callback (can be NULL, in this case need add custom calculations)
} marker_info_t;
extern const marker_info_t marker_info_list[MS_END];

// lever_mode
enum {LM_MARKER, LM_SEARCH, LM_FREQ_0, LM_FREQ_1, LM_EDELAY};

#define MARKER_INVALID       -1
#define TRACE_INVALID        -1

// properties flags
#define DOMAIN_MODE             (1<<0)
#define DOMAIN_FREQ             (0<<0)
#define DOMAIN_TIME             (1<<0)
// Time domain function
#define TD_FUNC                 (0b11<<1)
#define TD_FUNC_BANDPASS        (0b00<<1)
#define TD_FUNC_LOWPASS_IMPULSE (0b01<<1)
#define TD_FUNC_LOWPASS_STEP    (0b10<<1)
// Time domain window
#define TD_WINDOW               (0b11<<3)
#define TD_WINDOW_NORMAL        (0b00<<3)
#define TD_WINDOW_MINIMUM       (0b01<<3)
#define TD_WINDOW_MAXIMUM       (0b10<<3)
// Sweep mode
#define TD_START_STOP           (0<<0)
#define TD_CENTER_SPAN          (1<<6)
// Marker track
#define TD_MARKER_TRACK         (1<<7)
// Marker delta
#define TD_MARKER_DELTA         (1<<8)
// Marker delta
//#define TD_MARKER_LOCK          (1<<9) // reserved

//
// config.vna_mode flags (16 bit field)
//
enum {
  VNA_MODE_AUTO_NAME = 0,// Auto name for files
#ifdef __USE_SMOOTH__
  VNA_MODE_SMOOTH,       // Smooth function (0: Geom, 1: Arith)
#endif
#ifdef __USE_SERIAL_CONSOLE__
  VNA_MODE_CONNECTION,   // Connection flag (0: USB, 1: SERIAL)
#endif
  VNA_MODE_SEARCH,       // Marker search mode (0: max, 1: min)
  VNA_MODE_SHOW_GRID,    // Show grid values
  VNA_MODE_DOT_GRID,     // Dotted grid lines
#ifdef __USE_BACKUP__
  VNA_MODE_BACKUP,       // Made backup settings (save some settings after power off)
#endif
#ifdef __FLIP_DISPLAY__
  VNA_MODE_FLIP_DISPLAY, // Flip display
#endif
#ifdef __DIGIT_SEPARATOR__
  VNA_MODE_SEPARATOR,    // Comma or dot digit separator (0: dot, 1: comma)
#endif
#ifdef __SD_CARD_DUMP_TIFF__
  VNA_MODE_TIFF,         // Save screenshot format (0: bmp, 1: tiff)
#endif
#ifdef __USB_UID__
  VNA_MODE_USB_UID       // Use unique serial string for USB
#endif
};

// Update config._vna_mode flags function
typedef enum {VNA_MODE_CLR = 0, VNA_MODE_SET, VNA_MODE_TOGGLE} vna_mode_ops;
void apply_VNA_mode(uint16_t idx, vna_mode_ops operation);

#ifdef __VNA_MEASURE_MODULE__
// Measure option mode
enum {
  MEASURE_NONE = 0,
#ifdef __USE_LC_MATCHING__
  MEASURE_LC_MATH,
#endif
#ifdef __S21_MEASURE__
  MEASURE_SHUNT_LC,
  MEASURE_SERIES_LC,
  MEASURE_SERIES_XTAL,
  MEASURE_FILTER,
#endif
#ifdef __S11_CABLE_MEASURE__
  MEASURE_S11_CABLE,
#endif
#ifdef __S11_RESONANCE_MEASURE__
  MEASURE_S11_RESONANCE,
#endif
  MEASURE_END
};
#endif

#define STORED_TRACES  1
#define TRACES_MAX     4
#define TRACE_INDEX_COUNT (TRACES_MAX+STORED_TRACES)

typedef struct trace {
  uint8_t enabled;
  uint8_t type;
  uint8_t channel;
  uint8_t smith_format;
  float scale;
  float refpos;
} trace_t;

// marker 1 to 8
#define MARKERS_MAX 8
typedef struct marker {
  uint8_t  enabled;
  uint8_t  reserved;
  uint16_t index;
  freq_t   frequency;
} marker_t;

typedef struct config {
  uint32_t magic;
  uint32_t _harmonic_freq_threshold;
  int32_t  _IF_freq;
  int16_t  _touch_cal[4];
  uint16_t _vna_mode;
  uint16_t _dac_value;
  uint16_t _vbat_offset;
  uint16_t _bandwidth;
  uint8_t  _lever_mode;
  uint8_t  _brightness;
  uint16_t _lcd_palette[MAX_PALETTE];
  uint32_t _serial_speed;
  uint32_t _xtal_freq;
  float    _measure_r;
  uint8_t  _band_mode;
  uint8_t  _reserved[3];
  uint32_t checksum;
} config_t;

typedef struct properties {
  uint32_t magic;
  freq_t   _frequency0;          // sweep start frequency
  freq_t   _frequency1;          // sweep stop frequency
  freq_t   _cal_frequency0;      // calibration start frequency
  freq_t   _cal_frequency1;      // calibration stop frequency
  freq_t   _var_freq;            // frequency step by leveler (0 for auto)
  uint16_t _mode;                // timed domain option flag and some others flags
  uint16_t _sweep_points;        // points used in measure sweep
  int8_t   _current_trace;       // 0..(TRACES_MAX -1) (TRACE_INVALID  for disabled)
  int8_t   _active_marker;       // 0..(MARKERS_MAX-1) (MARKER_INVALID for disabled)
  int8_t   _previous_marker;     // 0..(MARKERS_MAX-1) (MARKER_INVALID for disabled)
  uint8_t  _power;               // 0 ... 3 current output power settings
  uint8_t  _cal_power;           // 0 ... 3 Power used in calibration
  uint8_t  _measure;             // additional trace data calculations
  uint16_t _cal_sweep_points;    // points used in calibration
  uint16_t _cal_status;          // calibration data collected flags
  trace_t  _trace[TRACES_MAX];
  marker_t _markers[MARKERS_MAX];
  uint8_t  _reserved;
  uint8_t  _velocity_factor;     // 0 .. 100 %
  float    _electrical_delay[2]; // delays for S11 and S21 traces in seconds
  float    _var_delay;           // electrical delay step by leveler
  float    _s21_offset;          // additional external attenuator for S21 measures
  float    _portz;               // Used for port-z renormalization
  float    _cal_load_r;          // Used as calibration standard LOAD R value (calculated in renormalization procedure)
  uint32_t _reserved1[7];
  float    _cal_data[CAL_TYPE_COUNT][SWEEP_POINTS_MAX][2]; // Put at the end for faster access to others data from struct
  uint32_t checksum;
} properties_t;

extern config_t config;
extern properties_t current_props;

void set_trace_type(int t, int type, int channel);
void set_trace_channel(int t, int channel);
void set_trace_scale(int t, float scale);
void set_active_trace(int t);
void set_trace_refpos(int t, float refpos);
void set_trace_enable(int t, bool enable);
const char *get_trace_chname(int t);

//
// Shell config functions and macros for Serial connect, not used if Serial mode disabled
void shell_update_speed(uint32_t speed);
void shell_reset_console(void);
void  set_electrical_delay(int ch, float seconds);
float get_electrical_delay(void);
void set_s21_offset(float offset);
float groupdelay_from_array(int i, const float *v);

void plot_init(void);
void update_grid(freq_t fstart, freq_t fstop);
void request_to_redraw(uint16_t mask);
void request_to_draw_cells_behind_menu(void);
void request_to_draw_cells_behind_numeric_input(void);
void request_to_draw_marker(uint16_t idx);
void redraw_marker(int8_t marker);
void draw_all(void);
void set_area_size(uint16_t w, uint16_t h);
void plot_set_measure_mode(uint8_t mode);
uint16_t plot_get_measure_channels(void);

int distance_to_index(int8_t t, uint16_t idx, int16_t x, int16_t y);
int search_nearest_index(int x, int y, int t);

void toogleStoredTrace(int idx);
uint8_t getStoredTraces(void);

const char *get_trace_typename(int t, int marker_smith_format);
const char *get_smith_format_names(int m);

// Marker search functions
#define MK_SEARCH_LEFT    -1
#define MK_SEARCH_RIGHT    1
void marker_search(void);
void marker_search_dir(int16_t from, int16_t dir);

// _request flag for update screen
#define REDRAW_PLOT       (1<<0)
#define REDRAW_FREQUENCY  (1<<1)
#define REDRAW_CAL_STATUS (1<<2)
#define REDRAW_MARKER     (1<<3)
#define REDRAW_REFERENCE  (1<<4)
#define REDRAW_GRID_VALUE (1<<5)
#define REDRAW_BATTERY    (1<<6)
#define REDRAW_AREA       (1<<7)
#define REDRAW_CLRSCR     (1<<8)
#define REDRAW_BACKUP     (1<<9)
#define REDRAW_CELLS      (1<<10)

/*
 * ili9341.c
 */
// Set display buffers count for cell render (if use 2 and DMA, possible send data and prepare new in some time)
#ifdef __USE_DISPLAY_DMA__
// Cell size = sizeof(spi_buffer), but need wait while cell data send to LCD
//#define DISPLAY_CELL_BUFFER_COUNT     1
// Cell size = sizeof(spi_buffer)/2, while one cell send to LCD by DMA, CPU render to next cell
#define DISPLAY_CELL_BUFFER_COUNT     2
#else
// Always one if no DMA mode
#define DISPLAY_CELL_BUFFER_COUNT     1
#endif

// Custom display driver panel definitions for ILI9341
#if defined(LCD_DRIVER_ILI9341) || defined(LCD_DRIVER_ST7789)
// LCD touch settings
#define DEFAULT_TOUCH_CONFIG {530, 795, 3460, 3350}    // 2.8 inch LCD panel
// Define LCD pixel format (8 or 16 bit)
//#define LCD_8BIT_MODE
#define LCD_16BIT_MODE
// Default LCD brightness if display support it
#define DEFAULT_BRIGHTNESS  70
// Data size for one pixel data read from display in bytes
#define LCD_RX_PIXEL_SIZE  3
#endif

// Custom display driver panel definitions for ST7796S
#ifdef LCD_DRIVER_ST7796S
// LCD touch settings
#define DEFAULT_TOUCH_CONFIG {380, 665, 3600, 3450 }  // 4.0 inch LCD panel
// Define LCD pixel format (8 or 16 bit)
//#define LCD_8BIT_MODE
#define LCD_16BIT_MODE
// Default LCD brightness if display support it
#define DEFAULT_BRIGHTNESS  70
// Data size for one pixel data read from display in bytes
#define LCD_RX_PIXEL_SIZE  2
#endif

// For 8 bit color displays pixel data definitions
#ifdef LCD_8BIT_MODE
typedef uint8_t pixel_t;
//  8-bit RRRGGGBB
//#define RGB332(r,g,b)  ( (((r)&0xE0)>>0) | (((g)&0xE0)>>3) | (((b)&0xC0)>>5))
#define RGB565(r,g,b)  ( (((r)&0xE0)>>0) | (((g)&0xE0)>>3) | (((b)&0xC0)>>6))
#define RGBHEX(hex)    ( (((hex)&0xE00000)>>16) | (((hex)&0x00E000)>>11) | (((hex)&0x0000C0)>>6) )
#define HEXRGB(hex)    ( (((hex)<<16)&0xE00000) | (((hex)<<11)&0x00E000) | (((hex)<<6)&0x0000C0) )
#define LCD_PIXEL_SIZE        1
// Cell size, depends from spi_buffer size, CELLWIDTH*CELLHEIGHT*sizeof(pixel) <= sizeof(spi_buffer)
#define CELLWIDTH  (64/DISPLAY_CELL_BUFFER_COUNT)
#define CELLHEIGHT (64)
#endif

// For 16 bit color displays pixel data definitions
#ifdef LCD_16BIT_MODE
typedef uint16_t pixel_t;
// SPI bus revert byte order
// 16-bit gggBBBbb RRRrrGGG
#define RGB565(r,g,b)  ( (((g)&0x1c)<<11) | (((b)&0xf8)<<5) | ((r)&0xf8) | (((g)&0xe0)>>5) )
#define RGBHEX(hex) ( (((hex)&0x001c00)<<3) | (((hex)&0x0000f8)<<5) | (((hex)&0xf80000)>>16) | (((hex)&0x00e000)>>13) )
#define HEXRGB(hex) ( (((hex)>>3)&0x001c00) | (((hex)>>5)&0x0000f8) | (((hex)<<16)&0xf80000) | (((hex)<<13)&0x00e000) )
#define LCD_PIXEL_SIZE        2
// Cell size, depends from spi_buffer size, CELLWIDTH*CELLHEIGHT*sizeof(pixel) <= sizeof(spi_buffer)
#define CELLWIDTH  (64/DISPLAY_CELL_BUFFER_COUNT)
#define CELLHEIGHT (32)
#endif

// Define size of screen buffer in pixel_t (need for cell w * h * count)
#define SPI_BUFFER_SIZE             (CELLWIDTH * CELLHEIGHT * DISPLAY_CELL_BUFFER_COUNT)

#ifndef SPI_BUFFER_SIZE
#error "Define LCD pixel format"
#endif

enum {
  LCD_BG_COLOR = 0,       // background
  LCD_FG_COLOR,           // foreground (in most cases text on background)
  LCD_GRID_COLOR,         // grid lines color
  LCD_MENU_COLOR,         // UI menu color
  LCD_MENU_TEXT_COLOR,    // UI menu text color
  LCD_MENU_ACTIVE_COLOR,  // UI selected menu color
  LCD_TRACE_1_COLOR,      // Trace 1 color
  LCD_TRACE_2_COLOR,      // Trace 2 color
  LCD_TRACE_3_COLOR,      // Trace 3 color
  LCD_TRACE_4_COLOR,      // Trace 4 color
  LCD_TRACE_5_COLOR,      // Stored trace A color
  LCD_TRACE_6_COLOR,      // Stored trace B color
  LCD_NORMAL_BAT_COLOR,   // Normal battery icon color
  LCD_LOW_BAT_COLOR,      // Low battery icon color
  LCD_SPEC_INPUT_COLOR,   // Not used, for future
  LCD_RISE_EDGE_COLOR,    // UI menu button rise edge color
  LCD_FALLEN_EDGE_COLOR,  // UI menu button fallen edge color
  LCD_SWEEP_LINE_COLOR,   // Sweep line color
  LCD_BW_TEXT_COLOR,      // Bandwidth text color
  LCD_INPUT_TEXT_COLOR,   // Keyboard Input text color
  LCD_INPUT_BG_COLOR,     // Keyboard Input text background color
  LCD_MEASURE_COLOR,      // Measure text color
  LCD_GRID_VALUE_COLOR,   // Not used, for future
  LCD_INTERP_CAL_COLOR,   // Calibration state on interpolation color
  LCD_DISABLE_CAL_COLOR,  // Calibration state on disable color
  LCD_LINK_COLOR,         // UI menu button text for values color
  LCD_TXT_SHADOW_COLOR,   // Plot area text border color
};

#define LCD_DEFAULT_PALETTE {\
[LCD_BG_COLOR         ] = RGB565(  0,  0,  0), \
[LCD_FG_COLOR         ] = RGB565(255,255,255), \
[LCD_GRID_COLOR       ] = RGB565(128,128,128), \
[LCD_MENU_COLOR       ] = RGB565(230,230,230), \
[LCD_MENU_TEXT_COLOR  ] = RGB565(  0,  0,  0), \
[LCD_MENU_ACTIVE_COLOR] = RGB565(210,210,210), \
[LCD_TRACE_1_COLOR    ] = RGB565(255,255,  0), \
[LCD_TRACE_2_COLOR    ] = RGB565(  0,255,255), \
[LCD_TRACE_3_COLOR    ] = RGB565(  0,255,  0), \
[LCD_TRACE_4_COLOR    ] = RGB565(255,  0,255), \
[LCD_TRACE_5_COLOR    ] = RGB565(255,  0,  0), \
[LCD_TRACE_6_COLOR    ] = RGB565(  0,  0,255), \
[LCD_NORMAL_BAT_COLOR ] = RGB565( 31,227,  0), \
[LCD_LOW_BAT_COLOR    ] = RGB565(255,  0,  0), \
[LCD_SPEC_INPUT_COLOR ] = RGB565(128,255,128), \
[LCD_RISE_EDGE_COLOR  ] = RGB565(255,255,255), \
[LCD_FALLEN_EDGE_COLOR] = RGB565(128,128,128), \
[LCD_SWEEP_LINE_COLOR ] = RGB565(  0,  0,255), \
[LCD_BW_TEXT_COLOR    ] = RGB565(196,196,196), \
[LCD_INPUT_TEXT_COLOR ] = RGB565(  0,  0,  0), \
[LCD_INPUT_BG_COLOR   ] = RGB565(255,255,255), \
[LCD_MEASURE_COLOR    ] = RGB565(255,255,255), \
[LCD_GRID_VALUE_COLOR ] = RGB565( 96, 96, 96), \
[LCD_INTERP_CAL_COLOR ] = RGB565( 31,227,  0), \
[LCD_DISABLE_CAL_COLOR] = RGB565(255,  0,  0), \
[LCD_LINK_COLOR       ] = RGB565(  0,  0,192), \
[LCD_TXT_SHADOW_COLOR ] = RGB565(  0,  0,  0), \
}

#define GET_PALTETTE_COLOR(idx)  config._lcd_palette[idx]

extern pixel_t foreground_color;
extern pixel_t background_color;

extern pixel_t spi_buffer[SPI_BUFFER_SIZE];

typedef struct {
  uint8_t transparent : 1;
  int8_t shift_x : 7;
  int8_t shift_y : 8;
} vector_data;

// Used for easy define big Bitmap as 0bXXXXXXXXX image
#define _BMP8(d)                                                        ((d)&0xFF)
#define _BMP16(d)                                      (((d)>>8)&0xFF), ((d)&0xFF)
#define _BMP24(d)                    (((d)>>16)&0xFF), (((d)>>8)&0xFF), ((d)&0xFF)
#define _BMP32(d)  (((d)>>24)&0xFF), (((d)>>16)&0xFF), (((d)>>8)&0xFF), ((d)&0xFF)

void lcd_init(void);
void lcd_bulk(int x, int y, int w, int h);
void lcd_fill(int x, int y, int w, int h);

#if DISPLAY_CELL_BUFFER_COUNT == 1
#define lcd_get_cell_buffer()             spi_buffer
#define lcd_bulk_continue                 lcd_bulk
#define lcd_bulk_finish()                 {}
#else
pixel_t *lcd_get_cell_buffer(void);                     // get buffer for cell render
void lcd_bulk_continue(int x, int y, int w, int h);     // send data to display, in DMA mode use it, no wait DMA complete
void lcd_bulk_finish(void);                             // wait DMA complete (need call at end)
#endif

void lcd_set_foreground(uint16_t fg_idx);
void lcd_set_background(uint16_t bg_idx);
void lcd_set_colors(uint16_t fg_idx, uint16_t bg_idx);
void lcd_clear_screen(void);
void lcd_blitBitmap(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *bitmap);
void lcd_blitBitmapScale(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t size, const uint8_t *b);
void lcd_drawchar(uint8_t ch, int x, int y);
#if 0
void lcd_drawstring(int16_t x, int16_t y, const char *str);
#else
// use printf for draw string
#define lcd_drawstring lcd_printf
#endif
int  lcd_printf(int16_t x, int16_t y, const char *fmt, ...);
int  lcd_printfV(int16_t x, int16_t y, const char *fmt, ...);
int  lcd_drawchar_size(uint8_t ch, int x, int y, uint8_t size);
void lcd_drawstring_size(const char *str, int x, int y, uint8_t size);
void lcd_drawfont(uint8_t ch, int x, int y);
void lcd_read_memory(int x, int y, int w, int h, uint16_t* out);
void lcd_line(int x0, int y0, int x1, int y1);
void lcd_vector_draw(int x, int y, const vector_data *v);

uint32_t lcd_send_register(uint8_t cmd, uint8_t len, const uint8_t *data);
void     lcd_set_flip(bool flip);

// SD Card support, discio functions for FatFS lib implemented in ili9341.c
#ifdef  __USE_SD_CARD__
#include "../FatFs/ff.h"
#include "../FatFs/diskio.h"

// Buffers for SD card use spi_buffer
#if SPI_BUFFER_SIZE < 2048
#error "SPI_BUFFER_SIZE for SD card support need size >= 2048"
#else
// Fat file system work area (at the end of spi_buffer)
#define fs_volume    (FATFS *)(((uint8_t*)(&spi_buffer[SPI_BUFFER_SIZE])) - sizeof(FATFS))
// FatFS file object (at the end of spi_buffer)
#define fs_file      (   FIL*)(((uint8_t*)(&spi_buffer[SPI_BUFFER_SIZE])) - sizeof(FATFS) - sizeof(FIL))
#endif

void testLog(void);        // debug log
#endif

/*
 * flash.c
 */
#define CONFIG_MAGIC      0x434f4e56 // Config magic value (allow reset on new config version)
#define PROPERTIES_MAGIC  0x434f4e54 // Properties magic value (allow reset on new properties version)

#define NO_SAVE_SLOT      ((uint16_t)(-1))
extern uint16_t lastsaveid;

#define frequency0          current_props._frequency0
#define frequency1          current_props._frequency1
#define cal_frequency0      current_props._cal_frequency0
#define cal_frequency1      current_props._cal_frequency1
#define var_freq            current_props._var_freq
#define sweep_points        current_props._sweep_points
#define cal_sweep_points    current_props._cal_sweep_points
#define cal_power           current_props._cal_power
#define cal_status          current_props._cal_status
#define cal_data            current_props._cal_data
#define electrical_delayS11 current_props._electrical_delay[0]
#define electrical_delayS21 current_props._electrical_delay[1]
#define s21_offset          current_props._s21_offset
#define velocity_factor     current_props._velocity_factor
#define trace               current_props._trace
#define current_trace       current_props._current_trace
#define markers             current_props._markers
#define active_marker       current_props._active_marker
#define previous_marker     current_props._previous_marker
#ifdef __VNA_Z_RENORMALIZATION__
 #define cal_load_r         current_props._cal_load_r
#else
 #define cal_load_r         50.0f
#endif

#define props_mode          current_props._mode
#define domain_window      (props_mode&TD_WINDOW)
#define domain_func        (props_mode&TD_FUNC)

#define FREQ_STARTSTOP()       {props_mode&=~TD_CENTER_SPAN;}
#define FREQ_CENTERSPAN()      {props_mode|= TD_CENTER_SPAN;}
#define FREQ_IS_STARTSTOP()  (!(props_mode&TD_CENTER_SPAN))
#define FREQ_IS_CENTERSPAN()   (props_mode&TD_CENTER_SPAN)
#define FREQ_IS_CW()           (frequency0 == frequency1)

#define get_trace_scale(t)      current_props._trace[t].scale
#define get_trace_refpos(t)     current_props._trace[t].refpos

#define VNA_MODE(idx)        (config._vna_mode&(1<<idx))
#define lever_mode           config._lever_mode
#define IF_OFFSET            config._IF_freq
#ifdef __DIGIT_SEPARATOR__
#define DIGIT_SEPARATOR      (VNA_MODE(VNA_MODE_SEPARATOR) ? ',' : '.')
#else
#define DIGIT_SEPARATOR      '.'
#endif

inline freq_t
get_sweep_frequency(uint16_t type)
{
  switch (type) {
    case ST_START:  return frequency0;
    case ST_STOP:   return frequency1;
    case ST_CENTER: return (frequency0>>1) + (frequency1>>1) + (frequency0&1);
    case ST_SPAN:   return frequency1 - frequency0;
    case ST_CW:     return frequency0;
  }
  return 0;
}

int caldata_save(uint32_t id);
int caldata_recall(uint32_t id);
const properties_t *get_properties(uint32_t id);

int config_save(void);
int config_recall(void);

void clear_all_config_prop_data(void);

/*
 * ui.c
 */
// Enter in leveler search mode after search click
//#define UI_USE_LEVELER_SEARCH_MODE

void ui_init(void);
void ui_process(void);

void handle_touch_interrupt(void);

void ui_touch_cal_exec(void);
void ui_touch_draw_test(void);
void ui_enter_dfu(void);

void ui_message_box(const char *header, const char *text, uint32_t delay);

// Irq operation process set
#define OP_NONE       0x00
#define OP_LEVER      0x01
#define OP_TOUCH      0x02
#define OP_CONSOLE    0x04
extern uint8_t operation_requested;

#define TOUCH_THRESHOLD 2000
/*
 * misclinous
 */
int plot_printf(char *str, int, const char *fmt, ...);
#define PULSE do { palClearPad(GPIOC, GPIOC_LED); palSetPad(GPIOC, GPIOC_LED);} while(0)

#define ARRAY_COUNT(a)    (sizeof(a)/sizeof(*(a)))
// Speed profile definition
#define START_PROFILE   systime_t time = chVTGetSystemTimeX();
#define STOP_PROFILE    {lcd_printfV(1, 1, "T:%08d", chVTGetSystemTimeX() - time);}
// Macros for convert define value to string
#define STR1(x)  #x
#define define_to_STR(x)  STR1(x)
#define SWAP(type, x, y) {type t = x; x=y; y=t;}
/*EOF*/
