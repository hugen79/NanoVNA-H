/*
 * Copyright (c) 2019-2021, Dmitry (DiSlord) dislordlive@gmail.com
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

#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "si5351.h"
#include "nanovna.h"

#include <chprintf.h>
#include <string.h>

/*
 *  Shell settings
 */
// If need run shell as thread (use more amount of memory fore stack), after
// enable this need reduce spi_buffer size, by default shell run in main thread
// #define VNA_SHELL_THREAD

static BaseSequentialStream *shell_stream = 0;

// Shell new line
#define VNA_SHELL_NEWLINE_STR    "\r\n"
// Shell command promt
#define VNA_SHELL_PROMPT_STR     "ch> "
// Shell max arguments
#define VNA_SHELL_MAX_ARGUMENTS   4
// Shell max command line size
#define VNA_SHELL_MAX_LENGTH     48

// Shell command functions prototypes
typedef void (*vna_shellcmd_t)(int argc, char *argv[]);
#define VNA_SHELL_FUNCTION(command_name) \
      static void command_name(int argc, char *argv[])

// Shell command line buffer, args, nargs, and function ptr
static char shell_line[VNA_SHELL_MAX_LENGTH];
static char *shell_args[VNA_SHELL_MAX_ARGUMENTS + 1];
static uint16_t shell_nargs;
static volatile vna_shellcmd_t  shell_function = 0;

#define ENABLED_DUMP_COMMAND
// Allow get threads debug info
//#define ENABLE_THREADS_COMMAND
// Enable vbat_offset command, allow change battery voltage correction in config
#define ENABLE_VBAT_OFFSET_COMMAND
// Info about NanoVNA, need fore soft
#define ENABLE_INFO_COMMAND
// Enable color command, allow change config color for traces, grid, menu
#define ENABLE_COLOR_COMMAND
// Enable transform command
#define ENABLE_TRANSFORM_COMMAND
// Enable sample command
//#define ENABLE_SAMPLE_COMMAND
// Enable I2C command for send data to AIC3204, used for debug
//#define ENABLE_I2C_COMMAND
// Enable LCD command for send data to LCD screen, used for debug
//#define ENABLE_LCD_COMMAND
// Enable output debug data on screen on hard fault
//#define ENABLE_HARD_FAULT_HANDLER_DEBUG
// Enable test command, used for debug
//#define ENABLE_TEST_COMMAND
// Enable stat command, used for debug
//#define ENABLE_STAT_COMMAND
// Enable gain command, used for debug
//#define ENABLE_GAIN_COMMAND
// Enable port command, used for debug
//#define ENABLE_PORT_COMMAND
// Enable si5351 register write, used for debug
//#define ENABLE_SI5351_REG_WRITE
// Enable i2c timing command, used for debug
//#define ENABLE_I2C_TIMINGS
// Enable band setting command, used for debug
//#define ENABLE_BAND_COMMAND
// Enable scan_bin command (need use ex scan in future)
#define ENABLE_SCANBIN_COMMAND
// Enable debug for console command
//#define DEBUG_CONSOLE_SHOW
// Enable usart command
#define ENABLE_USART_COMMAND
#ifdef __USE_SD_CARD__
// Enable SD card console command
#define ENABLE_SD_CARD_CMD
#endif

static void apply_CH0_error_term(float data[4], float c_data[CAL_TYPE_COUNT][2]);
static void apply_CH1_error_term(float data[4], float c_data[CAL_TYPE_COUNT][2]);
static void cal_interpolate(int idx, freq_t f, float data[CAL_TYPE_COUNT][2]);

static uint16_t get_sweep_mask(void);
static void update_frequencies(void);
static int  set_frequency(freq_t freq);
static void set_frequencies(freq_t start, freq_t stop, uint16_t points);
static bool sweep(bool break_on_operation, uint16_t ch_mask);
static void transform_domain(uint16_t ch_mask);

uint8_t sweep_mode = SWEEP_ENABLE;

// sweep operation variables
volatile uint16_t wait_count = 0;
// current sweep point (used for continue sweep if user break)
static uint16_t p_sweep = 0;
// ChibiOS i2s buffer must be 2x size (for process one while next buffer filled by DMA)
static int16_t rx_buffer[AUDIO_BUFFER_LEN * 2];
// Sweep measured data
float measured[2][POINTS_COUNT][2];

//Buffer for fast apply FFT window function
#ifdef USE_FFT_WINDOW_BUFFER
static float kaiser_data[FFT_SIZE];
#endif

#undef VERSION
#define VERSION "1.1.01"

// Version text, displayed in Config->Version menu, also send by info command
const char *info_about[]={
  "Board: " BOARD_NAME,
  "2019-2021 Copyright NanoVNA.com",
  "based on  @DiSlord @edy555 ... source",
  "Licensed under GPL.",
  "Version: " VERSION " ["\
  "p:"define_to_STR(POINTS_COUNT)", "\
  "IF:"define_to_STR(FREQUENCY_IF_K)"k, "\
  "ADC:"define_to_STR(AUDIO_ADC_FREQ_K)"k, "\
  "Lcd:"define_to_STR(LCD_WIDTH)"x"define_to_STR(LCD_HEIGHT)\
  "]",  "Build Time: " __DATE__ " - " __TIME__,
//  "Kernel: " CH_KERNEL_VERSION,
//  "Compiler: " PORT_COMPILER_NAME,
  "Architecture: " PORT_ARCHITECTURE_NAME " Core Variant: " PORT_CORE_VARIANT_NAME,
//  "Port Info: " PORT_INFO,
  "Platform: " PLATFORM_NAME,
  0 // sentinel
};

// Allow draw some debug on LCD
#ifdef DEBUG_CONSOLE_SHOW
void my_debug_log(int offs, char *log){
  static uint16_t shell_line_y = 0;
  lcd_set_foreground(LCD_FG_COLOR);
  lcd_set_background(LCD_BG_COLOR);
  lcd_fill(FREQUENCIES_XPOS1, shell_line_y, LCD_WIDTH-FREQUENCIES_XPOS1, 2 * FONT_GET_HEIGHT);
  lcd_drawstring(FREQUENCIES_XPOS1 + offs, shell_line_y, log);
  shell_line_y+=FONT_STR_HEIGHT;
  if (shell_line_y >= LCD_HEIGHT - FONT_STR_HEIGHT*4) shell_line_y=0;
}
#define DEBUG_LOG(offs, text)    my_debug_log(offs, text);
#else
#define DEBUG_LOG(offs, text)
#endif

#ifdef __USE_SMOOTH__
static float arifmetic_mean(float v0, float v1, float v2){
  return (v0+2*v1+v2)/4;
}

static float geometry_mean(float v0, float v1, float v2){
  float v = vna_cbrtf(vna_fabsf(v0*v1*v2));
  if (v0+v1+v2 < 0) v = -v;
  return v;
}

uint8_t smooth_factor = 0;
void set_smooth_factor(uint8_t factor){
  if (factor > 8) factor = 8;
  smooth_factor = factor;
  request_to_redraw(REDRAW_CAL_STATUS);
}
uint8_t get_smooth_factor(void) {
  return smooth_factor;
}

// Allow smooth complex data point array (this remove noise, smooth power depend form count)
// see https://terpconnect.umd.edu/~toh/spectrum/Smoothing.html
static void measurementDataSmooth(uint16_t ch_mask){
  int j;
//  ch_mask = 2;
//  memcpy(measured[0], measured[1], sizeof(measured[0]));
  float (*smooth_func)(float v0, float v1, float v2) = (VNA_mode & VNA_SMOOTH_FUNCTION) ? arifmetic_mean : geometry_mean;
  for (int ch = 0; ch < 2; ch++,ch_mask>>=1) {
    if ((ch_mask&1)==0) continue;
    int count = 1<<(smooth_factor-1), n;
    float *data = measured[ch][0];
    for (n = 0; n < count; n++){
      float prev_re = data[2*0  ];
      float prev_im = data[2*0+1];
// first point smooth (use first and second points), disabled it made phase shift
//      data[0] = smooth_func(prev_re, prev_re, data[2  ]);
//      data[1] = smooth_func(prev_im, prev_im, data[2+1]);
// simple data smooth on 3 points
      for (j = 1; j < sweep_points - 1; j++){
        float old_re = data[2*j  ]; // save current data point for next point smooth
        float old_im = data[2*j+1];
        data[2*j  ] = smooth_func(prev_re, data[2*j  ], data[2*j+2]);
        data[2*j+1] = smooth_func(prev_im, data[2*j+1], data[2*j+3]);
        prev_re = old_re;
        prev_im = old_im;
      }
// last point smooth, disabled it made phase shift
//      data[2*j  ] = smooth_func(data[2*j  ], data[2*j  ], prev_re);
//      data[2*j+1] = smooth_func(data[2*j+1], data[2*j+1], prev_im);
    }
  }
}
#endif

static THD_WORKING_AREA(waThread1, 768);
static THD_FUNCTION(Thread1, arg)
{
  (void)arg;
  chRegSetThreadName("sweep");
/*
 * UI (menu, touch, buttons) and plot initialize
 */
  ui_init();
  //Initialize graph plotting
  plot_init();
/*
 * Set LCD display brightness
 */
#ifdef  __LCD_BRIGHTNESS__
  lcd_setBrightness(config._brightness);
#endif

  while (1) {
    bool completed = false;
    uint16_t mask = get_sweep_mask();
    if (sweep_mode&(SWEEP_ENABLE|SWEEP_ONCE)) {
      completed = sweep(true, mask);
      sweep_mode&=~SWEEP_ONCE;
    } else {
      __WFI();
    }
    // Run Shell command in sweep thread
    if (shell_function) {
      shell_function(shell_nargs - 1, &shell_args[1]);
      shell_function = 0;
      chThdSleepMilliseconds(10);
      continue;
    }
    // Process UI inputs
    sweep_mode|= SWEEP_UI_MODE;
    ui_process();
    sweep_mode&=~SWEEP_UI_MODE;
    // Process collected data, calculate trace coordinates and plot only if scan completed
    if ((sweep_mode & SWEEP_ENABLE) && completed) {
#ifdef __USE_SMOOTH__
//    START_PROFILE;
      if (smooth_factor)
        measurementDataSmooth(mask);
//    STOP_PROFILE;
#endif
//      START_PROFILE
      if ((props_mode & DOMAIN_MODE) == DOMAIN_TIME) transform_domain(mask);
//      STOP_PROFILE;
      // Prepare draw graphics, cache all lines, mark screen cells for redraw
      plot_into_index(measured);
    }
    request_to_redraw(REDRAW_BATTERY);
#ifndef DEBUG_CONSOLE_SHOW
    // plot trace and other indications as raster
    draw_all(completed);  // flush markmap only if scan completed to prevent remaining traces
#endif
  }
}

static inline void
pause_sweep(void)
{
  sweep_mode &= ~SWEEP_ENABLE;
}

static inline void
resume_sweep(void)
{
  sweep_mode |= SWEEP_ENABLE;
}

void
toggle_sweep(void)
{
  sweep_mode ^= SWEEP_ENABLE;
}

#if 0
//
// Original kaiser_window functions
//
static float
bessel0(float x)
{
  const float eps = 0.0001;

  float ret = 0;
  float term = 1;
  float m = 0;

  while (term  > eps * ret) {
    ret += term;
    ++m;
    term *= (x*x) / (4*m*m);
  }
  return ret;
}

static float
kaiser_window(float k, float n, float beta)
{
  if (beta == 0.0) return 1.0;
  float r = (2 * k) / (n - 1) - 1;
  return bessel0(beta * vna_sqrtf(1 - r * r)) / bessel0(beta);
}
#else
//             Zero-order Bessel function
//     (x/2)^(2n)
// 1 + ----------
//       (n!)^2
// set input as (x/2)^2 (input range 0 .. beta*beta/4)
//       x^n           x       x^2      x^3      x^4     x^5
// 1 + ------ = 1 + ------ + ------ + ------ + ------ + ------  ......
//     (n!)^2          1        4       36       576    14400
// first 2 (0 and 1) precalculated as simple.

// Precalculated multiplier step for (n!)^2  max SIZE 16 (first 2 use as init)
static const float div[] = {/*0, 1,*/ 4, 9, 16, 25, 36, 49, 64, 81, 100, 121, 144, 169, 196, 225, 256};
float bessel0_ext(float x_pow_2)
{
// set calculated count, more SIZE - less error but longer (bigger beta also need more size for less error)
// For beta =  6 SIZE =  (11-2) no error
// For beta = 13 SIZE =  (13-2) max error 0.0002 (use as default, use constant size faster then check every time limits in float)
#define SIZE     (13-2)
  int i = SIZE;
  float term = x_pow_2;
  float ret = 1.0f + term;
  do {
    term*= x_pow_2 / div[SIZE - i];
    ret += term;
  }while(--i);
  return ret;
}
// Move out constant divider:  bessel0(beta)
// Made calculation optimization (in integer)
// x = (2*k)/(n-1) - 1 = (set n=n-1) = 2*k/n - 1 = (2*k-n)/n
// calculate kaiser window vs bessel0(w) there:
//                                    n*n - (2*k-n)*(2*k-n)              4*k*(n-k)
// w = beta*sqrt(1 - x*x) = beta*sqrt(---------------------) = beta*sqrt(---------)
//                                           n*n                            n*n
// bessel0(w) = bessel0_ext(z) (there z = (w/2)^2 for speed)
// return = bessel0_ext(z)
static float
kaiser_window_ext(uint32_t k, uint32_t n, uint16_t beta)
{
  if (beta == 0) return 1.0;
  n = n - 1;
  k = k * (n - k) * beta * beta;
  n = n * n;
  return bessel0_ext((float)k / n);
}
#endif

static void
transform_domain(uint16_t ch_mask)
{
  // use spi_buffer as temporary buffer and calculate ifft for time domain
  // Need 2 * sizeof(float) * FFT_SIZE bytes for work
#if 2*4*FFT_SIZE > (SPI_BUFFER_SIZE * LCD_PIXEL_SIZE)
#error "Need increase spi_buffer or use less FFT_SIZE value"
#endif
  int i;
  uint16_t offset = 0;
  uint8_t is_lowpass = FALSE;
  switch (domain_func) {
//  case TD_FUNC_BANDPASS:
//    break;
    case TD_FUNC_LOWPASS_IMPULSE:
    case TD_FUNC_LOWPASS_STEP:
      is_lowpass = TRUE;
      offset = sweep_points;
      break;
  }
  uint16_t window_size = sweep_points + offset;
  uint16_t beta = 0;
  switch (domain_window) {
//    case TD_WINDOW_MINIMUM:
//    beta = 0;  // this is rectangular
//      break;
    case TD_WINDOW_NORMAL:
      beta = 6;
      break;
    case TD_WINDOW_MAXIMUM:
      beta = 13;
      break;
  }
  // Add amplitude correction for not full size FFT data and also add computed default scale
  // recalculate the scale factor if any window details are changed. The scale factor is to compensate for windowing.
  // Add constant multiplier for kaiser_window_ext use 1.0f / bessel0_ext(beta*beta/4.0f)
  // Add constant multiplier  1.0f / FFT_SIZE
  static float window_scale = 0;
  static uint16_t td_cache = 0;
  // Check mode cache data
  uint16_t td_check = (props_mode & (TD_WINDOW|TD_FUNC))|(sweep_points<<5);
  if (td_cache!=td_check){
    td_cache = td_check;
    if (domain_func == TD_FUNC_LOWPASS_STEP)
      window_scale = 1.0f / (FFT_SIZE * bessel0_ext(beta*beta/4.0f));
    else {
      window_scale = 0.0f;
      for (int i = 0; i < sweep_points; i++)
        window_scale += kaiser_window_ext(i + offset, window_size, beta);
      if (domain_func == TD_FUNC_BANDPASS) window_scale = 1.0f / (  window_scale);
      else                                 window_scale = 1.0f / (2*window_scale);
//    window_scale*= FFT_SIZE               // add correction from kaiser_window
//    window_scale/= FFT_SIZE               // add defaut from FFT_SIZE
//    window_scale*= bessel0_ext(beta*beta/4.0f) // for get result as kaiser_window
//    window_scale/= bessel0_ext(beta*beta/4.0f) // for set correction on calculated kaiser_window for value
    }
#ifdef USE_FFT_WINDOW_BUFFER
    // Cache window function data to buffer
    for (i = 0; i < sweep_points; i++)
      kaiser_data[i] = kaiser_window_ext(i + offset, window_size, beta) * window_scale;
#endif
  }
  // Made Time Domain Calculations
  for (int ch = 0; ch < 2; ch++,ch_mask>>=1) {
    if ((ch_mask&1)==0) continue;
    // Prepare data in tmp buffer (use spi_buffer), apply window function and constant correction factor
    float* tmp  = (float*)spi_buffer;
    float *data = measured[ch][0];
    for (i = 0; i < sweep_points; i++) {
#ifdef USE_FFT_WINDOW_BUFFER
      float w = kaiser_data[i];
#else
      float w = kaiser_window_ext(i + offset, window_size, beta) * window_scale;
#endif
      tmp[i * 2 + 0] = data[i * 2 + 0] * w;
      tmp[i * 2 + 1] = data[i * 2 + 1] * w;
    }
    // Fill zeroes last
    for (; i < FFT_SIZE; i++) {
      tmp[i * 2 + 0] = 0.0;
      tmp[i * 2 + 1] = 0.0;
    }
    // For lowpass mode swap
    if (is_lowpass) {
      for (i = 1; i < sweep_points; i++) {
        tmp[(FFT_SIZE - i) * 2 + 0] =  tmp[i * 2 + 0];
        tmp[(FFT_SIZE - i) * 2 + 1] = -tmp[i * 2 + 1];
      }
    }
    // Made iFFT in temp buffer
    fft_inverse((float(*)[2])tmp);
    // set img part as zero
    if (is_lowpass){
      for (i = 0; i < sweep_points; i++)
        tmp[i*2+1] = 0.0f;
    }
    if (domain_func == TD_FUNC_LOWPASS_STEP) {
      for (i = 1; i < sweep_points; i++) {
        tmp[i*2+0]+= tmp[i*2+0-2];
//      tmp[i*2+1]+= tmp[i*2+1-2];  // already zero as is_lowpass
      }
    }
    // Copy data back
    memcpy(measured[ch], tmp, sizeof(measured[0]));
  }
}

// Shell commands output
int shell_printf(const char *fmt, ...)
{
  if (shell_stream == NULL) return 0;
  va_list ap;
  int formatted_bytes;
  va_start(ap, fmt);
  formatted_bytes = chvprintf(shell_stream, fmt, ap);
  va_end(ap);
  return formatted_bytes;
}

#ifdef __USE_SERIAL_CONSOLE__
// Serial Shell commands output
int serial_shell_printf(const char *fmt, ...)
{
  va_list ap;
  int formatted_bytes;
  va_start(ap, fmt);
  formatted_bytes = chvprintf((BaseSequentialStream *)&SD1, fmt, ap);
  va_end(ap);
  return formatted_bytes;
}
#endif

//
// Function used for search substring v in list
// Example need search parameter "center" in "start|stop|center|span|cw" getStringIndex return 2
// If not found return -1
// Used for easy parse command arguments
static int get_str_index(const char *v, const char *list)
{
  int i = 0;
  while (1) {
    const char *p = v;
    while (1) {
      char c = *list;
      if (c == '|') c = 0;
      if (c == *p++) {
        // Found, return index
        if (c == 0) return i;
        list++;    // Compare next symbol
        continue;
      }
      break;  // Not equal, break
    }
    // Set new substring ptr
    while (1) {
      // End of string, not found
      if (*list == 0) return -1;
      if (*list++ == '|') break;
    }
    i++;
  }
  return -1;
}

VNA_SHELL_FUNCTION(cmd_pause)
{
  (void)argc;
  (void)argv;
  pause_sweep();
}

VNA_SHELL_FUNCTION(cmd_resume)
{
  (void)argc;
  (void)argv;

  // restore frequencies array and cal
  update_frequencies();
  resume_sweep();
}

VNA_SHELL_FUNCTION(cmd_reset)
{
  (void)argc;
  (void)argv;
#ifdef __DFU_SOFTWARE_MODE__
  if (argc == 1) {
    if (get_str_index(argv[0], "dfu") == 0) {
      shell_printf("Performing reset to DFU mode\r\n");
      enter_dfu();
      return;
    }
  }
#endif
  shell_printf("Performing reset\r\n");

  rccEnableWWDG(FALSE);
  WWDG->CFR = 0x60;
  WWDG->CR = 0xff;

  /* wait forever */
  while (1)
    ;
}

// Use macro, std isdigit more big
#define _isdigit(c) (c >= '0' && c <= '9')
// Rewrite universal standart str to value functions to more compact
//
// Convert string to int32
int32_t my_atoi(const char *p)
{
  int32_t value = 0;
  uint32_t c;
  bool neg = false;

  if (*p == '-') {neg = true; p++;}
  if (*p == '+') p++;
  while ((c = *p++ - '0') < 10)
    value = value * 10 + c;
  return neg ? -value : value;
}

// Convert string to uint32
//  0x - for hex radix
//  0o - for oct radix
//  0b - for bin radix
//  default dec radix
uint32_t my_atoui(const char *p)
{
  uint32_t value = 0, radix = 10, c;
  if (*p == '+') p++;
  if (*p == '0') {
    switch (p[1]) {
      case 'x': radix = 16; break;
      case 'o': radix =  8; break;
      case 'b': radix =  2; break;
      default:  goto calculate;
    }
    p+=2;
  }
calculate:
  while (1) {
    c = *p++ - '0';
    // c = to_upper(*p) - 'A' + 10
    if (c >= 'A' - '0') c = (c&(~0x20)) - ('A' - '0') + 10;
    if (c >= radix) return value;
    value = value * radix + c;
  }
}

float
my_atof(const char *p)
{
  int neg = FALSE;
  if (*p == '-')
    neg = TRUE;
  if (*p == '-' || *p == '+')
    p++;
  float x = my_atoi(p);
  while (_isdigit((int)*p))
    p++;
  if (*p == '.') {
    float d = 1.0f;
    p++;
    while (_isdigit((int)*p)) {
      d /= 10.0f;
      x += d * (*p - '0');
      p++;
    }
  }
  if (*p == 'e' || *p == 'E') {
    p++;
    int exp = my_atoi(p);
    while (exp > 0) {
      x *= 10;
      exp--;
    }
    while (exp < 0) {
      x /= 10;
      exp++;
    }
  }
  if (neg)
    x = -x;
  return x;
}

#ifdef __USE_SMOOTH__
VNA_SHELL_FUNCTION(cmd_smooth)
{
  if (argc == 1) {
    set_smooth_factor(my_atoui(argv[0]));
    return;
  }
  shell_printf("smooth = %d\r\n", smooth_factor);
}
#endif

#ifdef USE_VARIABLE_OFFSET
VNA_SHELL_FUNCTION(cmd_offset)
{
  if (argc != 1) {
    shell_printf("usage: offset {frequency offset(Hz)}\r\n");
    return;
  }
  si5351_set_frequency_offset(my_atoi(argv[0]));
}
#endif

VNA_SHELL_FUNCTION(cmd_freq)
{
  if (argc != 1) {
    goto usage;
  }
  uint32_t freq = my_atoui(argv[0]);

  pause_sweep();
  set_frequency(freq);
  return;
usage:
  shell_printf("usage: freq {frequency(Hz)}\r\n");
}

void set_power(uint8_t value){
  request_to_redraw(REDRAW_CAL_STATUS);
  if (value > SI5351_CLK_DRIVE_STRENGTH_8MA) value = SI5351_CLK_DRIVE_STRENGTH_AUTO;
  if (current_props._power == value) return;
  current_props._power = value;
  // Update power if pause, need for generation in CW mode
  if (!(sweep_mode&SWEEP_ENABLE)) si5351_set_power(value);
}

VNA_SHELL_FUNCTION(cmd_power)
{
  if (argc == 0) {
    shell_printf("power: %d\r\n", current_props._power);
    return;
  }
  if (argc != 1) {
    shell_printf("usage: power {0-3}|{255 - auto}\r\n");
    return;
  }
  set_power(my_atoi(argv[0]));
//  set_frequency(frequency);
}

#ifdef __USE_RTC__
VNA_SHELL_FUNCTION(cmd_time)
{
  (void)argc;
  (void)argv;
  uint32_t  dt_buf[2];
  dt_buf[0] = rtc_get_tr_bcd(); // TR should be read first for sync
  dt_buf[1] = rtc_get_dr_bcd(); // DR should be read second
  static const uint8_t idx_to_time[] = {6,5,4,2,  1,  0};
  static const char       time_cmd[] = "y|m|d|h|min|sec";
  //            0    1   2       4      5     6
  // time[] ={sec, min, hr, 0, day, month, year, 0}
  uint8_t   *time = (uint8_t*)dt_buf;
  if (argc == 3 &&  get_str_index(argv[0], "b") == 0){
    rtc_set_time(my_atoui(argv[1]), my_atoui(argv[2]));
    return;
  }
  if (argc!=2) goto usage;
  int idx = get_str_index(argv[0], time_cmd);
  uint32_t val = my_atoui(argv[1]);
  if (idx < 0 || val > 99)
    goto usage;
  // Write byte value in struct
  time[idx_to_time[idx]] = ((val/10)<<4)|(val%10); // value in bcd format
  rtc_set_time(dt_buf[1], dt_buf[0]);
  return;
usage:
  shell_printf("20%02x/%02x/%02x %02x:%02x:%02x\r\n"\
               "usage: time {[%s] 0-99} or {b 0xYYMMDD 0xHHMMSS}\r\n", time[6], time[5], time[4], time[2], time[1], time[0], time_cmd);
}
#endif

#ifdef __VNA_ENABLE_DAC__
// Check DAC enabled in ChibiOS
#if HAL_USE_DAC == TRUE
#error "Need disable HAL_USE_DAC in halconf.h for use VNA_DAC"
#endif

static void dac_init(void) {
  rccEnableDAC1(false); // Use DAC1
  DAC->CR|= DAC_CR_EN2; // Enable DAC1 ch2
}

VNA_SHELL_FUNCTION(cmd_dac)
{
  if (argc != 1) {
    shell_printf("usage: dac {value(0-4095)}\r\n"\
                 "current value: %d\r\n", config._dac_value);
    return;
  }
  DAC->DHR12R2 = my_atoui(argv[0])&0xFFF;
}
#endif

VNA_SHELL_FUNCTION(cmd_threshold)
{
  uint32_t value;
  if (argc != 1) {
    shell_printf("usage: threshold {frequency in harmonic mode}\r\n"\
                 "current: %d\r\n", config._harmonic_freq_threshold);
    return;
  }
  value = my_atoui(argv[0]);
  config._harmonic_freq_threshold = value;
}

VNA_SHELL_FUNCTION(cmd_saveconfig)
{
  (void)argc;
  (void)argv;
  config_save();
  shell_printf("Config saved.\r\n");
}

VNA_SHELL_FUNCTION(cmd_clearconfig)
{
  if (argc != 1) {
    shell_printf("usage: clearconfig {protection key}\r\n");
    return;
  }

  if (get_str_index(argv[0], "1234") != 0) {
    shell_printf("Key unmatched.\r\n");
    return;
  }

  clear_all_config_prop_data();
  shell_printf("Config and all cal data cleared.\r\n"\
               "Do reset manually to take effect. Then do touch cal and save.\r\n");
}

VNA_SHELL_FUNCTION(cmd_data)
{
  int i;
  int sel = 0;
  float (*array)[2];
  if (argc == 1)
    sel = my_atoi(argv[0]);
  if (sel < 0 || sel >=7)
    goto usage;

  array = sel < 2 ? measured[sel] : cal_data[sel-2];

  for (i = 0; i < sweep_points; i++)
    shell_printf("%f %f\r\n", array[i][0], array[i][1]);
  return;
usage:
  shell_printf("usage: data [array]\r\n");
}

VNA_SHELL_FUNCTION(cmd_capture)
{
// read pixel count at one time (PART*2 bytes required for read buffer)
  (void)argc;
  (void)argv;
  int y;
// Check buffer limits, if less possible reduce rows count
#define READ_ROWS 2
#if (SPI_BUFFER_SIZE*LCD_PIXEL_SIZE) < (LCD_RX_PIXEL_SIZE*LCD_WIDTH*READ_ROWS)
#error "Low size of spi_buffer for cmd_capture"
#endif
  // read 2 row pixel time
  for (y = 0; y < LCD_HEIGHT; y += READ_ROWS) {
    // use uint16_t spi_buffer[2048] (defined in ili9341) for read buffer
    lcd_read_memory(0, y, LCD_WIDTH, READ_ROWS, (uint16_t *)spi_buffer);
    streamWrite(shell_stream, (void*)spi_buffer, READ_ROWS * LCD_WIDTH * sizeof(uint16_t));
  }
}

#if 0
VNA_SHELL_FUNCTION(cmd_gamma)
{
  float gamma[2];
  (void)argc;
  (void)argv;
  
  pause_sweep();
  chMtxLock(&mutex);
  wait_dsp(4);  
  calculate_gamma(gamma);
  chMtxUnlock(&mutex);

  shell_printf("%d %d\r\n", gamma[0], gamma[1]);
}
#endif

static void (*sample_func)(float *gamma) = calculate_gamma;
#ifdef ENABLE_SAMPLE_COMMAND
VNA_SHELL_FUNCTION(cmd_sample)
{
  if (argc != 1) goto usage;
  //                                         0    1   2
  static const char cmd_sample_list[] = "gamma|ampl|ref";
  switch (get_str_index(argv[0], cmd_sample_list)) {
    case 0:
      sample_func = calculate_gamma;
      return;
    case 1:
      sample_func = fetch_amplitude;
      return;
    case 2:
      sample_func = fetch_amplitude_ref;
      return;
    default:
      break;
  }
usage:
  shell_printf("usage: sample {%s}\r\n", cmd_sample_list);
}
#endif

config_t config = {
  .magic       = CONFIG_MAGIC,
  ._harmonic_freq_threshold = FREQUENCY_THRESHOLD,
  ._IF_freq    = FREQUENCY_OFFSET,
  ._touch_cal  = DEFAULT_TOUCH_CONFIG,
  ._vna_mode   = VNA_MODE_USB | VNA_MODE_SEARCH_MAX,
  ._brightness = DEFAULT_BRIGHTNESS,
  ._dac_value   = 1922,
  ._vbat_offset = 320,
  ._bandwidth = BANDWIDTH_1000,
  ._lcd_palette = LCD_DEFAULT_PALETTE,
  ._serial_speed = SERIAL_DEFAULT_BITRATE,
  ._xtal_freq = XTALFREQ,
  ._measure_r = MEASURE_DEFAULT_R,
  ._lever_mode = LM_MARKER,
  ._digit_separator = '.',
#ifdef __BAND_MODE__
#ifdef __MS5351__
  ._band_mode = 1,
#else
  ._band_mode = 0,
#endif
#endif
};

properties_t current_props;

// NanoVNA Default settings
static const trace_t def_trace[TRACES_MAX] = {//enable, type, channel, reserved, scale, refpos
  { 1, TRC_LOGMAG, 0, 0, 10.0, NGRIDY-1 },
  { 1, TRC_LOGMAG, 1, 0, 10.0, NGRIDY-1 },
  { 1, TRC_SMITH,  0, 0, 1.0, 0 },
  { 1, TRC_PHASE,  1, 0, 90.0, NGRIDY/2 }
};

static const marker_t def_markers[MARKERS_MAX] = {
  { 1, 0, 30*POINTS_COUNT_DEFAULT/100-1, 0 },
#if MARKERS_MAX > 1
  { 0, 0, 40*POINTS_COUNT_DEFAULT/100-1, 0 },
#endif
#if MARKERS_MAX > 2
  { 0, 0, 50*POINTS_COUNT_DEFAULT/100-1, 0 },
#endif
#if MARKERS_MAX > 3
  { 0, 0, 60*POINTS_COUNT_DEFAULT/100-1, 0 },
#endif
#if MARKERS_MAX > 4
  { 0, 0, 70*POINTS_COUNT_DEFAULT/100-1, 0 },
#endif
#if MARKERS_MAX > 5
  { 0, 0, 80*POINTS_COUNT_DEFAULT/100-1, 0 },
#endif
#if MARKERS_MAX > 6
  { 0, 0, 90*POINTS_COUNT_DEFAULT/100-1, 0 },
#endif
#if MARKERS_MAX > 7
  { 0, 0,100*POINTS_COUNT_DEFAULT/100-1, 0 },
#endif
};

// Load propeties default settings
void load_default_properties(void)
{
//Magic add on caldata_save
//current_props.magic = CONFIG_MAGIC;
  current_props._frequency0       =     50000;    // start =  50kHz
  current_props._frequency1       = 900000000;    // end   = 900MHz
  current_props._var_freq         = 0;
  current_props._sweep_points     = POINTS_COUNT_DEFAULT; // Set default points count
  current_props._cal_frequency0   =     50000;    // calibration start =  50kHz
  current_props._cal_frequency1   = 900000000;    // calibration end   = 900MHz
  current_props._cal_sweep_points = POINTS_COUNT_DEFAULT; // Set calibration default points count
  current_props._cal_status   = 0;
//=============================================
  memcpy(current_props._trace, def_trace, sizeof(def_trace));
  memcpy(current_props._markers, def_markers, sizeof(def_markers));
//=============================================
  current_props._electrical_delay = 0.0;
  current_props._portz = 50.0f;
  current_props._velocity_factor = 70;
  current_props._current_trace   = 0;
  current_props._active_marker   = 0;
  current_props._previous_marker = MARKER_INVALID;
  current_props._mode            = 0;
  current_props._marker_smith_format = MS_RLC;
  current_props._power     = SI5351_CLK_DRIVE_STRENGTH_AUTO;
  current_props._cal_power = SI5351_CLK_DRIVE_STRENGTH_AUTO;
  current_props._measure   = MEASURE_NONE;
//This data not loaded by default
//current_props._cal_data[5][POINTS_COUNT][2];
//Checksum add on caldata_save
//current_props.checksum = 0;
}

//
// Backup registers support, allow save data on power off (while vbat power enabled)
//
#ifdef __USE_BACKUP__
#if POINTS_COUNT > 511 || SAVEAREA_MAX > 15
#error "Check backup data limits!!"
#endif
// backup_0 bitfield
typedef union {
  struct {
    uint32_t points   : 9; //  9 !! limit 511 points!!
    uint32_t bw       : 9; // 18 !! limit 511
    uint32_t id       : 4; // 22 !! 15 save slots
    uint32_t leveler  : 3; // 25
  };
  uint32_t v;
} backup_0;

void update_backup_data(void) {
  backup_0 bk = {
    .points   = sweep_points,
    .bw       = config._bandwidth,
    .id       = lastsaveid,
    .leveler  = lever_mode,
  };
  RTC->BKP0R = bk.v;
  RTC->BKP1R = frequency0;
  RTC->BKP2R = frequency1;
  RTC->BKP3R = var_freq;
}

static void load_start_properties(void) {
  if (VNA_mode & VNA_MODE_BACKUP) {
    backup_0 bk = {.v = RTC->BKP0R};
    if (bk.v != 0 && bk.id < SAVEAREA_MAX) { // if backup data valid, and slot valid
      if (caldata_recall(bk.id) == 0) {      // Load ok
        sweep_points = bk.points;            // Restore settings depend from calibration data
        lever_mode   = bk.leveler;
        set_bandwidth(bk.bw);
        frequency0 = RTC->BKP1R;
        frequency1 = RTC->BKP2R;
        var_freq   = RTC->BKP3R;
      }
      // Here need restore settings not depend from cal data
    }
    else
      caldata_recall(0);
  }
  else
    caldata_recall(0);
  update_frequencies();
#ifdef __VNA_MEASURE_MODULE__
  plot_set_measure_mode(current_props._measure);
#endif
}
#else
static void load_start_properties(void) {
  load_properties(0);
}
#endif

int load_properties(uint32_t id){
  int r = caldata_recall(id);
  update_frequencies();
#ifdef __VNA_MEASURE_MODULE__
  plot_set_measure_mode(current_props._measure);
#endif
  return r;
}

#ifdef ENABLED_DUMP_COMMAND
int16_t *dump_buffer;
volatile int16_t dump_len = 0;
int16_t dump_selection = 0;
static void
duplicate_buffer_to_dump(int16_t *p, size_t n)
{
  p+=dump_selection;
  while (n) {
    if (dump_len == 0) return;
    dump_len--;
    *dump_buffer++ = *p;
    p+=2;
    n-=2;
  }
}
#endif

//
// DMA i2s callback function, called on get 'half' and 'full' buffer size data
// need for process data, while DMA fill next buffer
static volatile systime_t ready_time = 0;

void i2s_end_callback(I2SDriver *i2sp, size_t offset, size_t n)
{
  int16_t *p = &rx_buffer[offset];
  (void)i2sp;
  if (wait_count == 0 || chVTGetSystemTimeX() < ready_time) return;
  if (wait_count == config._bandwidth+2)      // At this moment in buffer exist noise data, reset and wait next clean buffer
    reset_dsp_accumerator();
  else if (wait_count <= config._bandwidth+1) // Clean data ready, process it
    dsp_process(p, n);
#ifdef ENABLED_DUMP_COMMAND
  duplicate_buffer_to_dump(p, n);
#endif
  --wait_count;
//  stat.callback_count++;
}

static const I2SConfig i2sconfig = {
  NULL,                   // TX Buffer
  rx_buffer,              // RX Buffer
  AUDIO_BUFFER_LEN * 2,   // RX Buffer size
  NULL,                   // tx callback
  i2s_end_callback,       // rx callback
  0,                      // i2scfgr
  0                       // i2spr
};

#ifdef ENABLE_SI5351_TIMINGS
extern uint16_t timings[16];
#undef DELAY_CHANNEL_CHANGE
#undef DELAY_SWEEP_START
#define DELAY_CHANNEL_CHANGE  timings[3]
#define DELAY_SWEEP_START     timings[4]
#endif

#define DSP_START(delay) {ready_time = chVTGetSystemTimeX() + delay; wait_count = config._bandwidth+2;}
#define DSP_WAIT         while (wait_count) {__WFI();}
#define RESET_SWEEP      {p_sweep = 0;}

#define SWEEP_CH0_MEASURE           1
#define SWEEP_CH1_MEASURE           2
#define SWEEP_APPLY_EDELAY          4
#define SWEEP_APPLY_CALIBRATION     8
#define SWEEP_USE_INTERPOLATION    16
#define SWEEP_USE_RENORMALIZATION  32

static uint16_t get_sweep_mask(void){
  uint16_t ch_mask = 0;
  int t;
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    if (trace[t].channel == 0) ch_mask|=SWEEP_CH0_MEASURE;
    if (trace[t].channel == 1) ch_mask|=SWEEP_CH1_MEASURE;
  }
#ifdef __VNA_MEASURE_MODULE__
  // For measure calculations need data
  ch_mask|= plot_get_measure_channels();
#endif
#ifdef __VNA_Z_RENORMALIZATION__
  if (current_props._portz != 50.0f)
    ch_mask|= SWEEP_USE_RENORMALIZATION;
#endif
  if (cal_status & CALSTAT_APPLY)        ch_mask|= SWEEP_APPLY_CALIBRATION;
  if (cal_status & CALSTAT_INTERPOLATED) ch_mask|= SWEEP_USE_INTERPOLATION;
  if (electrical_delay)                  ch_mask|= SWEEP_APPLY_EDELAY;
  return ch_mask;
}

static void applyEDelay(float data[2], float s, float c){
  float real = data[0];
  float imag = data[1];
  data[0] = real * c - imag * s;
  data[1] = imag * c + real * s;
}

#ifdef __VNA_Z_RENORMALIZATION__
#include "vna_modules/vna_renorm.c"
#endif

// main loop for measurement
static bool sweep(bool break_on_operation, uint16_t mask)
{
  if (p_sweep>=sweep_points || break_on_operation == false) RESET_SWEEP;
  if (break_on_operation && mask == 0)
    return false;
  float s, c;
  float data[4];
  float c_data[CAL_TYPE_COUNT][2];
  // Blink LED while scanning
  palClearPad(GPIOC, GPIOC_LED);
  int delay = 0;
//  START_PROFILE;
  lcd_set_background(LCD_SWEEP_LINE_COLOR);
  // Wait some time for stable power
  int st_delay = DELAY_SWEEP_START;
  int bar_start = 0;
  int interpolation_idx;

  for (; p_sweep < sweep_points; p_sweep++) {
    freq_t frequency = getFrequency(p_sweep);
    // Need made measure - set frequency
    if (mask & (SWEEP_CH0_MEASURE|SWEEP_CH1_MEASURE)) {
      delay = set_frequency(frequency);
      interpolation_idx = mask & SWEEP_USE_INTERPOLATION ? -1 : p_sweep;
      // Edelay calibration
      if (mask & SWEEP_APPLY_EDELAY)
        vna_sincosf(electrical_delay * frequency * 1E-12, &s, &c);
      // Set invalid value for check
      c_data[0][0] = INFINITY;
    }
    // CH0:REFLECTION, reset and begin measure
    if (mask & SWEEP_CH0_MEASURE){
      tlv320aic3204_select(0);
      DSP_START(delay+st_delay);
      delay = DELAY_CHANNEL_CHANGE;
      // Get calibration data
      if (mask & SWEEP_APPLY_CALIBRATION)
        cal_interpolate(interpolation_idx, frequency, c_data);
      //================================================
      // Place some code thats need execute while delay
      //================================================
      DSP_WAIT;
      (*sample_func)(&data[0]);             // calculate reflection coefficient
      if (mask & SWEEP_APPLY_CALIBRATION)   // Apply calibration
        apply_CH0_error_term(data, c_data);
      if (mask & SWEEP_APPLY_EDELAY)        // Apply e-delay
        applyEDelay(&data[0], s, c);
    }
    // CH1:TRANSMISSION, reset and begin measure
    if (mask & SWEEP_CH1_MEASURE){
      tlv320aic3204_select(1);
      DSP_START(delay+st_delay);
      // Get calibration data
      if ((mask & SWEEP_APPLY_CALIBRATION) && c_data[0][0] == INFINITY)
        cal_interpolate(interpolation_idx, frequency, c_data);
      //================================================
      // Place some code thats need execute while delay
      //================================================
      DSP_WAIT;
      (*sample_func)(&data[2]);              // Measure transmission coefficient
      if (mask & SWEEP_APPLY_CALIBRATION)    // Apply calibration
        apply_CH1_error_term(data, c_data);
      if (mask & SWEEP_APPLY_EDELAY)         // Apply e-delay
        applyEDelay(&data[2], s, c);
    }
#ifdef __VNA_Z_RENORMALIZATION__
    if (mask & SWEEP_USE_RENORMALIZATION)
      apply_renormalization(data, mask);
#endif
    if (p_sweep < POINTS_COUNT){
      if (mask & SWEEP_CH0_MEASURE){
        measured[0][p_sweep][0] = data[0];
        measured[0][p_sweep][1] = data[1];
      }
      if (mask & SWEEP_CH1_MEASURE){
        measured[1][p_sweep][0] = data[2];
        measured[1][p_sweep][1] = data[3];
      }
    }
    if (operation_requested && break_on_operation) break;
    st_delay = 0;
    // Display SPI made noise on measurement (can see in CW mode), use reduced update
    if (config._bandwidth >= BANDWIDTH_100){
      int current_bar =  (p_sweep * WIDTH)/(sweep_points-1);
      if (current_bar - bar_start > 0){
        lcd_fill(OFFSETX+CELLOFFSETX + bar_start, OFFSETY, current_bar - bar_start, 1);
        bar_start = current_bar;
      }
    }
  }
  if (bar_start){
    lcd_set_background(LCD_GRID_COLOR);
    lcd_fill(OFFSETX+CELLOFFSETX, OFFSETY, bar_start, 1);
  }
//  STOP_PROFILE;
  // blink LED while scanning
  palSetPad(GPIOC, GPIOC_LED);
  return p_sweep == sweep_points;
}

#ifdef ENABLED_DUMP_COMMAND
VNA_SHELL_FUNCTION(cmd_dump)
{
  int i, j;
  int16_t dump[96*2];
  dump_buffer = dump;
  dump_len = sizeof(dump) / sizeof(int16_t);
  int len = dump_len;
  if (argc == 1)
    dump_selection = my_atoi(argv[0]) == 1 ? 0 : 1;

  tlv320aic3204_select(0);
  DSP_START(DELAY_SWEEP_START);
  while (dump_len > 0) {__WFI();}
  for (i = 0, j = 0; i < len; i++) {
    shell_printf("%6d ", dump[i]);
    if (++j == 12) {
      shell_printf("\r\n");
      j = 0;
    }
  }
}
#endif

#ifdef ENABLE_GAIN_COMMAND
VNA_SHELL_FUNCTION(cmd_gain)
{
  int rvalue = 0;
  int lvalue = 0;
  if (argc == 0 && argc > 2) {
    shell_printf("usage: gain {lgain(0-95)} [rgain(0-95)]\r\n");
    return;
  };
  lvalue = rvalue = my_atoui(argv[0]);
  if (argc == 3)
    rvalue = my_atoui(argv[1]);
  tlv320aic3204_set_gain(lvalue, rvalue);
}
#endif

static int set_frequency(freq_t freq)
{
  return si5351_set_frequency(freq, current_props._power);
}

void set_bandwidth(uint16_t bw_count){
  config._bandwidth = bw_count&0x1FF;
  request_to_redraw(REDRAW_BACKUP | REDRAW_FREQUENCY);
}

uint32_t get_bandwidth_frequency(uint16_t bw_freq){
  return (AUDIO_ADC_FREQ/AUDIO_SAMPLES_COUNT)/(bw_freq+1);
}

#define MAX_BANDWIDTH      (AUDIO_ADC_FREQ/AUDIO_SAMPLES_COUNT)
#define MIN_BANDWIDTH      ((AUDIO_ADC_FREQ/AUDIO_SAMPLES_COUNT)/512 + 1)

VNA_SHELL_FUNCTION(cmd_bandwidth)
{
  uint16_t user_bw;
  if (argc == 1)
    user_bw = my_atoui(argv[0]);
  else if (argc == 2){
    uint16_t f = my_atoui(argv[0]);
         if (f > MAX_BANDWIDTH) user_bw = 0;
    else if (f < MIN_BANDWIDTH) user_bw = 511;
    else user_bw = ((AUDIO_ADC_FREQ+AUDIO_SAMPLES_COUNT/2)/AUDIO_SAMPLES_COUNT)/f - 1;
  }
  else
    goto result;
  set_bandwidth(user_bw);
result:
  shell_printf("bandwidth %d (%uHz)\r\n", config._bandwidth, get_bandwidth_frequency(config._bandwidth));
}

void set_sweep_points(uint16_t points){
  if (points == sweep_points || points > POINTS_COUNT)
    return;

  sweep_points = points;
  update_frequencies();
}

/*
 * Frequency list functions
 */
#ifdef __USE_FREQ_TABLE__
static freq_t frequencies[POINTS_COUNT];
static void
set_frequencies(freq_t start, freq_t stop, uint16_t points)
{
  uint32_t i;
  freq_t step = (points - 1);
  freq_t span = stop - start;
  freq_t delta = span / step;
  freq_t error = span % step;
  freq_t f = start, df = step>>1;
  for (i = 0; i <= step; i++, f+=delta) {
    frequencies[i] = f;
    if ((df+=error) >= step) {f++; df-= step;}
  }
  // disable at out of sweep range
  for (; i < POINTS_COUNT; i++)
    frequencies[i] = 0;
}
#define _c_start    frequencies[0]
#define _c_stop     frequencies[sweep_points-1]
#define _c_points   (sweep_points)

freq_t getFrequency(uint16_t idx) {return frequencies[idx];}
#else
static freq_t   _f_start;
static freq_t   _f_delta;
static freq_t   _f_error;
static uint16_t _f_points;

static void
set_frequencies(freq_t start, freq_t stop, uint16_t points)
{
  freq_t span = stop - start;
  _f_start  = start;
  _f_points = (points - 1);
  _f_delta  = span / _f_points;
  _f_error  = span % _f_points;
}
freq_t getFrequency(uint16_t idx) {return _f_start + _f_delta * idx + (_f_points / 2 + _f_error * idx) / _f_points;}
freq_t getFrequencyStep(void) {return _f_delta;}
#endif

static bool needInterpolate(freq_t start, freq_t stop, uint16_t points){
  return start != cal_frequency0 || stop != cal_frequency1 || points != cal_sweep_points;
}

#define SCAN_MASK_OUT_FREQ       0b00000001
#define SCAN_MASK_OUT_DATA0      0b00000010
#define SCAN_MASK_OUT_DATA1      0b00000100
#define SCAN_MASK_NO_CALIBRATION 0b00001000
#define SCAN_MASK_NO_EDELAY      0b00010000
#define SCAN_MASK_BINARY         0b10000000

VNA_SHELL_FUNCTION(cmd_scan)
{
  freq_t start, stop;
  uint16_t points = sweep_points;
  if (argc < 2 || argc > 4) {
    shell_printf("usage: scan {start(Hz)} {stop(Hz)} [points] [outmask]\r\n");
    return;
  }

  start = my_atoui(argv[0]);
  stop = my_atoui(argv[1]);
  if (start == 0 || stop == 0 || start > stop) {
      shell_printf("frequency range is invalid\r\n");
      return;
  }
  if (argc >= 3) {
    points = my_atoui(argv[2]);
    if (points == 0 || points > POINTS_COUNT) {
      shell_printf("sweep points exceeds range "define_to_STR(POINTS_COUNT)"\r\n");
      return;
    }
    sweep_points = points;
  }
  uint16_t mask = 0;
  uint16_t sweep_ch = SWEEP_CH0_MEASURE|SWEEP_CH1_MEASURE;

#ifdef ENABLE_SCANBIN_COMMAND
  if (argc == 4) {
    mask = my_atoui(argv[3]);
    if (sweep_mode&SWEEP_BINARY) mask|=SCAN_MASK_BINARY;
    sweep_ch = (mask>>1)&3;
  }
  sweep_mode&=~(SWEEP_BINARY);
#else
  if (argc == 4) {
    mask = my_atoui(argv[3]);
    sweep_ch = (mask>>1)&3;
  }
#endif

  if ((cal_status & CALSTAT_APPLY) && !(mask&SCAN_MASK_NO_CALIBRATION)) sweep_ch|= SWEEP_APPLY_CALIBRATION;
  if (electrical_delay             && !(mask&SCAN_MASK_NO_EDELAY     )) sweep_ch|= SWEEP_APPLY_EDELAY;
  if (needInterpolate(start, stop, sweep_points))
    sweep_ch|= SWEEP_USE_INTERPOLATION;

  sweep_points = points;
  set_frequencies(start, stop, points);
  if (sweep_ch & (SWEEP_CH0_MEASURE|SWEEP_CH1_MEASURE))
    sweep(false, sweep_ch);
  pause_sweep();
  // Output data after if set (faster data receive)
  if (mask) {
    if (mask&SCAN_MASK_BINARY){
      streamWrite(shell_stream, (void *)&mask, sizeof(uint16_t));
      streamWrite(shell_stream, (void *)&points, sizeof(uint16_t));
      for (int i = 0; i < points; i++) {
        if (mask & SCAN_MASK_OUT_FREQ ) {freq_t f = getFrequency(i); streamWrite(shell_stream, (void *)&f, sizeof(freq_t));}    // 4 bytes .. frequency
        if (mask & SCAN_MASK_OUT_DATA0) streamWrite(shell_stream, (void *)&measured[0][i][0], sizeof(float)* 2);  // 4+4 bytes .. S11 real/imag
        if (mask & SCAN_MASK_OUT_DATA1) streamWrite(shell_stream, (void *)&measured[1][i][0], sizeof(float)* 2);  // 4+4 bytes .. S21 real/imag
      }
    }
    else{
      for (int i = 0; i < points; i++) {
        if (mask & SCAN_MASK_OUT_FREQ ) shell_printf("%u ", getFrequency(i));
        if (mask & SCAN_MASK_OUT_DATA0) shell_printf("%f %f ", measured[0][i][0], measured[0][i][1]);
        if (mask & SCAN_MASK_OUT_DATA1) shell_printf("%f %f ", measured[1][i][0], measured[1][i][1]);
        shell_printf("\r\n");
      }
    }
  }
}

#ifdef ENABLE_SCANBIN_COMMAND
VNA_SHELL_FUNCTION(cmd_scan_bin)
{
  sweep_mode|= SWEEP_BINARY;
  cmd_scan(argc, argv);
  sweep_mode&=~(SWEEP_BINARY);
}
#endif

VNA_SHELL_FUNCTION(cmd_tcxo)
{
  if (argc == 1)
    si5351_set_tcxo(my_atoui(argv[0]));
  shell_printf("tcxo = %u Hz\r\n", config._xtal_freq);
}

void set_marker_index(int m, int idx)
{
  if (m == MARKER_INVALID || (uint32_t)idx >= sweep_points) return;
  markers[m].index = idx;
  markers[m].frequency = getFrequency(idx);
}

freq_t get_marker_frequency(int marker)
{
  if ((uint32_t)marker >= MARKERS_MAX)
    return 0;
  return markers[marker].frequency;
}

static void
update_marker_index(void)
{
  int m, idx;
  freq_t fstart = get_sweep_frequency(ST_START);
  freq_t fstop  = get_sweep_frequency(ST_STOP);
  for (m = 0; m < MARKERS_MAX; m++) {
    // Update index for all markers !!
    uint32_t f = markers[m].frequency;
    if (f == 0) idx = markers[m].index; // Not need update index in no freq
    else if (f < fstart) idx = 0;
    else if (f >= fstop) idx = sweep_points-1;
    else { // Search frequency index for marker frequency
#if 0
      for (idx = 1; idx < sweep_points; idx++) {
        if (frequencies[idx] <= f) continue;
        if (f < (frequencies[idx-1]/2 + frequencies[idx]/2)) idx--; // Correct closest idx
        break;
      }
#else
      float r = ((float)(f - fstart))/(fstop - fstart);
      idx = r * (sweep_points-1);
#endif
    }
    set_marker_index(m, idx);
  }
}

static void
update_frequencies(void)
{
  freq_t start = get_sweep_frequency(ST_START);
  freq_t stop  = get_sweep_frequency(ST_STOP);

  set_frequencies(start, stop, sweep_points);
  update_marker_index();
  // set grid layout
  update_grid();
  // Update interpolation flag
  if (needInterpolate(start, stop, sweep_points))
    cal_status|= CALSTAT_INTERPOLATED;
  else
    cal_status&= ~CALSTAT_INTERPOLATED;

  request_to_redraw(REDRAW_BACKUP | REDRAW_CAL_STATUS | REDRAW_FREQUENCY | REDRAW_AREA);
  RESET_SWEEP;
}

void
set_sweep_frequency(int type, freq_t freq)
{
  // Check frequency for out of bounds (minimum SPAN can be any value)
  if (type < ST_SPAN && freq < START_MIN)
    freq = START_MIN;
  if (freq > STOP_MAX)
    freq = STOP_MAX;
  freq_t center, span;
  switch (type) {
    case ST_START:
      FREQ_STARTSTOP();
      frequency0 = freq;
      // if start > stop then make start = stop
      if (frequency1 < freq) frequency1 = freq;
      break;
    case ST_STOP:
      FREQ_STARTSTOP()
      frequency1 = freq;
        // if start > stop then make start = stop
      if (frequency0 > freq) frequency0 = freq;
      break;
    case ST_CENTER:
      FREQ_CENTERSPAN();
      center = freq;
      span   = (frequency1 - frequency0)>>1;
      if (span > center - START_MIN)
        span = (center - START_MIN);
      if (span > STOP_MAX - center)
        span = (STOP_MAX - center);
      frequency0 = center - span;
      frequency1 = center + span;
      break;
    case ST_SPAN:
      FREQ_CENTERSPAN();
      center = (frequency0>>1) + (frequency1>>1);
      span = freq>>1;
      if (center < START_MIN + span)
        center = START_MIN + span;
      if (center > STOP_MAX - span)
        center = STOP_MAX - span;
      frequency0 = center - span;
      frequency1 = center + span;
      break;
    case ST_CW:
      FREQ_CENTERSPAN();
      frequency0 = freq;
      frequency1 = freq;
      break;
    case ST_VAR:
      var_freq = freq;
      request_to_redraw(REDRAW_BACKUP);
      return;
  }
  update_frequencies();
}

void reset_sweep_frequency(void){
  frequency0 = cal_frequency0;
  frequency1 = cal_frequency1;
  sweep_points = cal_sweep_points;
  update_frequencies();
}

freq_t
get_sweep_frequency(int type)
{
  switch (type) {
    case ST_START:  return frequency0;
    case ST_STOP:   return frequency1;
    case ST_CENTER: return frequency0/2 + frequency1/2;
    case ST_SPAN:   return frequency1 - frequency0;
    case ST_CW:     return frequency0;
  }
  return 0;
}

VNA_SHELL_FUNCTION(cmd_sweep)
{
  if (argc == 0) {
    shell_printf("%u %u %d\r\n", get_sweep_frequency(ST_START), get_sweep_frequency(ST_STOP), sweep_points);
    return;
  } else if (argc > 3) {
    goto usage;
  }
  freq_t   value0 = 0;
  freq_t   value1 = 0;
  uint32_t value2 = 0;
  if (argc >= 1) value0 = my_atoui(argv[0]);
  if (argc >= 2) value1 = my_atoui(argv[1]);
  if (argc >= 3) value2 = my_atoui(argv[2]);
#if MAX_FREQ_TYPE != 5
#error "Sweep mode possibly changed, check cmd_sweep function"
#endif
  // Parse sweep {start|stop|center|span|cw} {freq(Hz)}
  // get enum ST_START, ST_STOP, ST_CENTER, ST_SPAN, ST_CW
  static const char sweep_cmd[] = "start|stop|center|span|cw";
  if (argc == 2 && value0 == 0) {
    int type = get_str_index(argv[0], sweep_cmd);
    if (type == -1)
      goto usage;
    set_sweep_frequency(type, value1);
    return;
  }
  //  Parse sweep {start(Hz)} [stop(Hz)]
  if (value0)
    set_sweep_frequency(ST_START, value0);
  if (value1)
    set_sweep_frequency(ST_STOP, value1);
  if (value2)
    set_sweep_points(value2);
  return;
usage:
  shell_printf("usage: sweep {start(Hz)} [stop(Hz)] [points]\r\n"\
               "\tsweep {%s} {freq(Hz)}\r\n", sweep_cmd);
}


static void
eterm_set(int term, float re, float im)
{
  int i;
  for (i = 0; i < sweep_points; i++) {
    cal_data[term][i][0] = re;
    cal_data[term][i][1] = im;
  }
}

static void
eterm_copy(int dst, int src)
{
  memcpy(cal_data[dst], cal_data[src], sizeof cal_data[dst]);
}

#if 0
const struct open_model {
  float c0;
  float c1;
  float c2;
  float c3;
} open_model = { 50, 0, -300, 27 };
#endif

#if 0
static void
adjust_ed(void)
{
  int i;
  for (i = 0; i < sweep_points; i++) {
    // z=1/(jwc*z0) = 1/(2*pi*f*c*z0)  Note: normalized with Z0
    // s11ao = (z-1)/(z+1) = (1-1/z)/(1+1/z) = (1-jwcz0)/(1+jwcz0)
    // prepare 1/s11ao to avoid dividing complex
    float c = 1000e-15;
    float z0 = 50;
    //float z = 2 * VNA_PI * frequencies[i] * c * z0;
    float z = 0.02;
    cal_data[ETERM_ED][i][0] += z;
  }
}
#endif

static void
eterm_calc_es(void)
{
  int i;
  for (i = 0; i < sweep_points; i++) {
    // z=1/(jwc*z0) = 1/(2*pi*f*c*z0)  Note: normalized with Z0
    // s11ao = (z-1)/(z+1) = (1-1/z)/(1+1/z) = (1-jwcz0)/(1+jwcz0)
    // prepare 1/s11ao for effeiciency
#if 0
    float c = 50e-15;
    //float c = 1.707e-12;
    float z0 = 50;
    float z = 2 * VNA_PI * frequencies[i] * c * z0;
    float sq = 1 + z*z;
    float s11aor = (1 - z*z) / sq;
    float s11aoi = 2*z / sq;
#else
    float s11aor = 1.0f;
    float s11aoi = 0.0f;
#endif
    // S11mo’= S11mo - Ed
    // S11ms’= S11ms - Ed
    float s11or = cal_data[CAL_OPEN][i][0] - cal_data[ETERM_ED][i][0];
    float s11oi = cal_data[CAL_OPEN][i][1] - cal_data[ETERM_ED][i][1];
    float s11sr = cal_data[CAL_SHORT][i][0] - cal_data[ETERM_ED][i][0];
    float s11si = cal_data[CAL_SHORT][i][1] - cal_data[ETERM_ED][i][1];
    // Es = (S11mo'/s11ao + S11ms’)/(S11mo' - S11ms’)
    float numr = s11sr + s11or * s11aor - s11oi * s11aoi;
    float numi = s11si + s11oi * s11aor + s11or * s11aoi;
    float denomr = s11or - s11sr;
    float denomi = s11oi - s11si;
    float d = denomr*denomr+denomi*denomi;
    cal_data[ETERM_ES][i][0] = (numr*denomr + numi*denomi)/d;
    cal_data[ETERM_ES][i][1] = (numi*denomr - numr*denomi)/d;
  }
  cal_status &= ~CALSTAT_OPEN;
  cal_status |= CALSTAT_ES;
}

static void
eterm_calc_er(int sign)
{
  int i;
  for (i = 0; i < sweep_points; i++) {
    // Er = sign*(1-sign*Es)S11ms'
    float s11sr = cal_data[CAL_SHORT][i][0] - cal_data[ETERM_ED][i][0];
    float s11si = cal_data[CAL_SHORT][i][1] - cal_data[ETERM_ED][i][1];
    float esr = cal_data[ETERM_ES][i][0];
    float esi = cal_data[ETERM_ES][i][1];
    if (sign > 0) {
      esr = -esr;
      esi = -esi;
    }
    esr = 1 + esr;
    float err = esr * s11sr - esi * s11si;
    float eri = esr * s11si + esi * s11sr;
    if (sign < 0) {
      err = -err;
      eri = -eri;
    }
    cal_data[ETERM_ER][i][0] = err;
    cal_data[ETERM_ER][i][1] = eri;
  }
  cal_status &= ~CALSTAT_SHORT;
  cal_status |= CALSTAT_ER;
}

// CAUTION: Et is inversed for efficiency
static void
eterm_calc_et(void)
{
  int i;
  for (i = 0; i < sweep_points; i++) {
    // Et = 1/(S21mt - Ex)
    float etr = cal_data[CAL_THRU][i][0] - cal_data[CAL_ISOLN][i][0];
    float eti = cal_data[CAL_THRU][i][1] - cal_data[CAL_ISOLN][i][1];
    float sq = etr*etr + eti*eti;
    float invr =  etr / sq;
    float invi = -eti / sq;
    cal_data[ETERM_ET][i][0] = invr;
    cal_data[ETERM_ET][i][1] = invi;
  }
  cal_status &= ~CALSTAT_THRU;
  cal_status |= CALSTAT_ET;
}

#if 0
void apply_error_term(void)
{
  int i;
  for (i = 0; i < sweep_points; i++) {
    // S11m' = S11m - Ed
    // S11a = S11m' / (Er + Es S11m')
    float s11mr = measured[0][i][0] - cal_data[ETERM_ED][i][0];
    float s11mi = measured[0][i][1] - cal_data[ETERM_ED][i][1];
    float err = cal_data[ETERM_ER][i][0] + s11mr * cal_data[ETERM_ES][i][0] - s11mi * cal_data[ETERM_ES][i][1];
    float eri = cal_data[ETERM_ER][i][1] + s11mr * cal_data[ETERM_ES][i][1] + s11mi * cal_data[ETERM_ES][i][0];
    float sq = err*err + eri*eri;
    float s11ar = (s11mr * err + s11mi * eri) / sq;
    float s11ai = (s11mi * err - s11mr * eri) / sq;
    measured[0][i][0] = s11ar;
    measured[0][i][1] = s11ai;

    // CAUTION: Et is inversed for efficiency
    // S21m' = S21m - Ex
    // S21a = S21m' (1-EsS11a)Et
    float s21mr = measured[1][i][0] - cal_data[ETERM_EX][i][0];
    float s21mi = measured[1][i][1] - cal_data[ETERM_EX][i][1];
    float esr = 1 - (cal_data[ETERM_ES][i][0] * s11ar - cal_data[ETERM_ES][i][1] * s11ai);
    float esi = - (cal_data[ETERM_ES][i][1] * s11ar + cal_data[ETERM_ES][i][0] * s11ai);
    float etr = esr * cal_data[ETERM_ET][i][0] - esi * cal_data[ETERM_ET][i][1];
    float eti = esr * cal_data[ETERM_ET][i][1] + esi * cal_data[ETERM_ET][i][0];
    float s21ar = s21mr * etr - s21mi * eti;
    float s21ai = s21mi * etr + s21mr * eti;
    measured[1][i][0] = s21ar;
    measured[1][i][1] = s21ai;
  }
}

static void apply_error_term_at(int i)
{
    // S11m' = S11m - Ed
    // S11a = S11m' / (Er + Es S11m')
    float s11mr = measured[0][i][0] - cal_data[ETERM_ED][i][0];
    float s11mi = measured[0][i][1] - cal_data[ETERM_ED][i][1];
    float err = cal_data[ETERM_ER][i][0] + s11mr * cal_data[ETERM_ES][i][0] - s11mi * cal_data[ETERM_ES][i][1];
    float eri = cal_data[ETERM_ER][i][1] + s11mr * cal_data[ETERM_ES][i][1] + s11mi * cal_data[ETERM_ES][i][0];
    float sq = err*err + eri*eri;
    float s11ar = (s11mr * err + s11mi * eri) / sq;
    float s11ai = (s11mi * err - s11mr * eri) / sq;
    measured[0][i][0] = s11ar;
    measured[0][i][1] = s11ai;

    // CAUTION: Et is inversed for efficiency
    // S21m' = S21m - Ex
    // S21a = S21m' (1-EsS11a)Et
    float s21mr = measured[1][i][0] - cal_data[ETERM_EX][i][0];
    float s21mi = measured[1][i][1] - cal_data[ETERM_EX][i][1];
#if 1
    float esr = 1 - (cal_data[ETERM_ES][i][0] * s11ar - cal_data[ETERM_ES][i][1] * s11ai);
    float esi = 0 - (cal_data[ETERM_ES][i][1] * s11ar + cal_data[ETERM_ES][i][0] * s11ai);
    float etr = esr * cal_data[ETERM_ET][i][0] - esi * cal_data[ETERM_ET][i][1];
    float eti = esr * cal_data[ETERM_ET][i][1] + esi * cal_data[ETERM_ET][i][0];
    float s21ar = s21mr * etr - s21mi * eti;
    float s21ai = s21mi * etr + s21mr * eti;
#else
    // Not made CH1 correction by CH0 data
    float s21ar = s21mr * cal_data[ETERM_ET][i][0] - s21mi * cal_data[ETERM_ET][i][1];
    float s21ai = s21mi * cal_data[ETERM_ET][i][0] + s21mr * cal_data[ETERM_ET][i][1];
#endif
    measured[1][i][0] = s21ar;
    measured[1][i][1] = s21ai;
}
#endif

static void apply_CH0_error_term(float data[4], float c_data[CAL_TYPE_COUNT][2])
{
  // S11m' = S11m - Ed
  // S11a = S11m' / (Er + Es S11m')
  float s11mr = data[0] - c_data[ETERM_ED][0];
  float s11mi = data[1] - c_data[ETERM_ED][1];
  float err = c_data[ETERM_ER][0] + s11mr * c_data[ETERM_ES][0] - s11mi * c_data[ETERM_ES][1];
  float eri = c_data[ETERM_ER][1] + s11mr * c_data[ETERM_ES][1] + s11mi * c_data[ETERM_ES][0];
  float sq = err*err + eri*eri;
  data[0] = (s11mr * err + s11mi * eri) / sq;
  data[1] = (s11mi * err - s11mr * eri) / sq;
}

static void apply_CH1_error_term(float data[4], float c_data[CAL_TYPE_COUNT][2])
{
  // CAUTION: Et is inversed for efficiency
  // S21a = (S21m - Ex) * Et
  float s21mr = data[2] - c_data[ETERM_EX][0];
  float s21mi = data[3] - c_data[ETERM_EX][1];
  // Not made CH1 correction by CH0 data
  data[2] = s21mr * c_data[ETERM_ET][0] - s21mi * c_data[ETERM_ET][1];
  data[3] = s21mi * c_data[ETERM_ET][0] + s21mr * c_data[ETERM_ET][1];
}

void
cal_collect(uint16_t type)
{
  uint16_t dst, src;

  static const struct {
    uint16_t set_flag;
    uint16_t clr_flag;
    uint8_t dst;
    uint8_t src;
 } calibration_set[]={
//    type       set data flag                              reset flag  destination source
    [CAL_LOAD] = {CALSTAT_LOAD,  ~(                      CALSTAT_APPLY), CAL_LOAD,  0},
    [CAL_OPEN] = {CALSTAT_OPEN,  ~(CALSTAT_ES|CALSTAT_ER|CALSTAT_APPLY), CAL_OPEN,  0}, // Reset Es and Er state
    [CAL_SHORT]= {CALSTAT_SHORT, ~(CALSTAT_ES|CALSTAT_ER|CALSTAT_APPLY), CAL_SHORT, 0}, // Reset Es and Er state
    [CAL_THRU] = {CALSTAT_THRU,  ~(           CALSTAT_ET|CALSTAT_APPLY), CAL_THRU,  1}, // Reset Et state
    [CAL_ISOLN]= {CALSTAT_ISOLN, ~(                      CALSTAT_APPLY), CAL_ISOLN, 1},
  };
  if (type >= ARRAY_COUNT(calibration_set)) return;

  // reset old calibration if frequency range/points not some
  if (needInterpolate(frequency0, frequency1, sweep_points)){
    cal_status = 0;
    cal_frequency0 = frequency0;
    cal_frequency1 = frequency1;
    cal_sweep_points = sweep_points;
  }
  cal_power = current_props._power;

  cal_status&=calibration_set[type].clr_flag;
  cal_status|=calibration_set[type].set_flag;
  dst = calibration_set[type].dst;
  src = calibration_set[type].src;

  // Run sweep for collect data (use minimum BANDWIDTH_30, or bigger if set)
  uint8_t bw = config._bandwidth;  // store current setting
  if (bw < BANDWIDTH_100)
    config._bandwidth = BANDWIDTH_100;

  // Set MAX settings for sweep_points on calibrate
//  if (sweep_points != POINTS_COUNT)
//    set_sweep_points(POINTS_COUNT);

  // Measure calibration data
  sweep(false, (src == 0) ? SWEEP_CH0_MEASURE : SWEEP_CH1_MEASURE);
  // Copy calibration data
  memcpy(cal_data[dst], measured[src], sizeof measured[0]);

  // Made average if need
  int count = 1, i, j;
  for (i = 1; i < count; i ++){
    sweep(false, (src == 0) ? SWEEP_CH0_MEASURE : SWEEP_CH1_MEASURE);
    for (j = 0; j < sweep_points; j++){
      cal_data[dst][j][0]+=measured[src][j][0];
      cal_data[dst][j][1]+=measured[src][j][1];
    }
  }
  if (i!=1){
    for (j = 0; j < sweep_points; j++){
      cal_data[dst][j][0]/=i;
      cal_data[dst][j][1]/=i;
    }
  }

  config._bandwidth = bw;          // restore
  request_to_redraw(REDRAW_CAL_STATUS);
}

void
cal_done(void)
{
  // Set Load/Ed to default if not calculated
  if (!(cal_status & CALSTAT_LOAD))
    eterm_set(ETERM_ED, 0.0, 0.0);
  // Set Isoln/Ex to default if not measured
  if (!(cal_status & CALSTAT_ISOLN))
    eterm_set(ETERM_EX, 0.0, 0.0);

  //adjust_ed();
  // Precalculate Es and Er from Short and Open (and use Load/Ed data)
  if ((cal_status & CALSTAT_SHORT) && (cal_status & CALSTAT_OPEN)) {
    eterm_calc_es();
    eterm_calc_er(-1);
  } else if (cal_status & CALSTAT_OPEN) {
    eterm_copy(CAL_SHORT, CAL_OPEN);
    cal_status &=~ CALSTAT_OPEN;
    eterm_set(ETERM_ES, 0.0, 0.0);
    eterm_calc_er(1);
  } else if (cal_status & CALSTAT_SHORT) {
    eterm_set(ETERM_ES, 0.0, 0.0);
    eterm_calc_er(-1);
  }

  // Apply Et
  if (cal_status & CALSTAT_THRU)
    eterm_calc_et();

  // Set other fields to default if not set
  if (!(cal_status & CALSTAT_ET))
    eterm_set(ETERM_ET, 1.0, 0.0);
  if (!(cal_status & CALSTAT_ER))
    eterm_set(ETERM_ER, 1.0, 0.0);
  if (!(cal_status & CALSTAT_ES))
    eterm_set(ETERM_ES, 0.0, 0.0);

  cal_status|= CALSTAT_APPLY;
  lastsaveid = NO_SAVE_SLOT;
  request_to_redraw(REDRAW_BACKUP | REDRAW_CAL_STATUS);
}

static void cal_interpolate(int idx, freq_t f, float data[CAL_TYPE_COUNT][2]){
  int eterm;
  uint16_t src_points = cal_sweep_points - 1;
  if (idx >= 0)
    goto copy_point;
  if (f <= cal_frequency0){
    idx = 0;
    goto copy_point;
  }
  if (f >= cal_frequency1){
    idx = src_points;
    goto copy_point;
  }
  // Search k1
  freq_t span = cal_frequency1 - cal_frequency0;
  idx = (uint64_t)(f - cal_frequency0) * (uint64_t)src_points / span;
  uint64_t v = (uint64_t)span * idx + src_points/2;
  freq_t src_f0 = cal_frequency0 + (v       ) / src_points;
  freq_t src_f1 = cal_frequency0 + (v + span) / src_points;

  freq_t delta = src_f1 - src_f0;
  // Not need interpolate
  if (f == src_f0) goto copy_point;

  float k1 = (delta == 0) ? 0.0 : (float)(f - src_f0) / delta;
  // avoid glitch between freqs in different harmonics mode
  uint32_t hf0 = si5351_get_harmonic_lvl(src_f0);
  if (hf0 != si5351_get_harmonic_lvl(src_f1)) {
    // f in prev harmonic, need extrapolate from prev 2 points
    if (hf0 == si5351_get_harmonic_lvl(f)){
      if (idx < 1) goto copy_point; // point limit
      idx--;
      k1+= 1.0f;
    }
    // f in next harmonic, need extrapolate from next 2 points
    else {
      if (idx >= src_points) goto copy_point; // point limit
      idx++;
      k1-= 1.0f;
    }
  }
  // Interpolate by k1
  float k0 = 1.0 - k1;
  for (eterm = 0; eterm < CAL_TYPE_COUNT; eterm++) {
    data[eterm][0] = cal_data[eterm][idx][0] * k0 + cal_data[eterm][idx+1][0] * k1;
    data[eterm][1] = cal_data[eterm][idx][1] * k0 + cal_data[eterm][idx+1][1] * k1;
  }
  return;
  // Direct point copy
copy_point:
  for (eterm = 0; eterm < CAL_TYPE_COUNT; eterm++) {
    data[eterm][0] = cal_data[eterm][idx][0];
    data[eterm][1] = cal_data[eterm][idx][1];
  }
  return;
}

VNA_SHELL_FUNCTION(cmd_cal)
{
  static const char *items[] = { "load", "open", "short", "thru", "isoln", "Es", "Er", "Et", "cal'ed" };

  if (argc == 0) {
    int i;
    for (i = 0; i < 9; i++) {
      if (cal_status & (1<<i))
        shell_printf("%s ", items[i]);
    }
    shell_printf("\r\n");
    return;
  }
  request_to_redraw(REDRAW_CAL_STATUS);
  //                                     0    1     2    3     4    5  6   7     8
  static const char cmd_cal_list[] = "load|open|short|thru|isoln|done|on|off|reset";
  switch (get_str_index(argv[0], cmd_cal_list)) {
    case 0:
      cal_collect(CAL_LOAD);
      return;
    case 1:
      cal_collect(CAL_OPEN);
      return;
    case 2:
      cal_collect(CAL_SHORT);
      return;
    case 3:
      cal_collect(CAL_THRU);
      return;
    case 4:
      cal_collect(CAL_ISOLN);
      return;
    case 5:
      cal_done();
      return;
    case 6:
      cal_status |= CALSTAT_APPLY;
      return;
    case 7:
      cal_status &= ~CALSTAT_APPLY;
      return;
    case 8:
      cal_status = 0;
      return;
    default:
      break;
  }
  shell_printf("usage: cal [%s]\r\n", cmd_cal_list);
}

VNA_SHELL_FUNCTION(cmd_save)
{
  if (argc != 1)
    goto usage;

  int id = my_atoi(argv[0]);
  if (id < 0 || id >= SAVEAREA_MAX)
    goto usage;
  caldata_save(id);
  request_to_redraw(REDRAW_CAL_STATUS);
  return;

 usage:
  shell_printf("save {id}\r\n");
}

VNA_SHELL_FUNCTION(cmd_recall)
{
  if (argc != 1)
    goto usage;

  int id = my_atoi(argv[0]);
  if (id < 0 || id >= SAVEAREA_MAX)
    goto usage;
  // Check for success
  if (load_properties(id))
    shell_printf("Err, default load\r\n");
  return;
 usage:
  shell_printf("recall {id}\r\n");
}

static const char * const trc_channel_name[] = {
  "S11", "S21"
};

const char *get_trace_chname(int t)
{
  return trc_channel_name[trace[t].channel];
}

void set_trace_type(int t, int type)
{
  int enabled = type != TRC_OFF;
  int force = FALSE;

  if (trace[t].enabled != enabled) {
    trace[t].enabled = enabled;
    force = TRUE;
  }
  if (trace[t].type != type && enabled) {
    trace[t].type = type;
    // Set default trace refpos
    trace[t].refpos = trace_info_list[type].refpos;
    // Set default trace scale
    trace[t].scale  = trace_info_list[type].scale_unit;
    force = TRUE;
  }
  if (force)
    plot_into_index(measured);
  request_to_redraw(REDRAW_AREA);
}

void set_trace_channel(int t, int channel)
{
  if (trace[t].channel != channel) {
    trace[t].channel = channel;
    plot_into_index(measured);
  }
}

void set_trace_scale(int t, float scale)
{
  if (trace[t].scale != scale) {
    trace[t].scale = scale;
    plot_into_index(measured);
  }
}

void set_trace_refpos(int t, float refpos)
{
  if (trace[t].refpos != refpos) {
    trace[t].refpos = refpos;
    plot_into_index(measured);
  }
}

void set_electrical_delay(float picoseconds)
{
  if (electrical_delay != picoseconds) {
    electrical_delay = picoseconds;
    request_to_redraw(REDRAW_AREA);
  }
}

VNA_SHELL_FUNCTION(cmd_trace)
{
  int t;
  if (argc == 0) {
    for (t = 0; t < TRACES_MAX; t++) {
      if (trace[t].enabled) {
        const char *type = get_trace_typename(trace[t].type);
        const char *channel = get_trace_chname(t);
        float scale = get_trace_scale(t);
        float refpos = get_trace_refpos(t);
        shell_printf("%d %s %s %f %f\r\n", t, type, channel, scale, refpos);
      }
    }
    return;
  }

  if (get_str_index(argv[0], "all") == 0 &&
      argc > 1 && get_str_index(argv[1], "off") == 0) {
    for (t = 0; t < TRACES_MAX; t++)
      set_trace_type(t, TRC_OFF);
    return;
  }

  t = my_atoi(argv[0]);
  if (t < 0 || t >= TRACES_MAX)
    goto usage;
  if (argc == 1) {
    const char *type = get_trace_typename(trace[t].type);
    const char *channel = get_trace_chname(t);
    shell_printf("%d %s %s\r\n", t, type, channel);
    return;
  }
#if MAX_TRACE_TYPE != 22
#error "Trace type enum possibly changed, check cmd_trace function"
#endif
  // enum TRC_LOGMAG, TRC_PHASE, TRC_DELAY, TRC_SMITH, TRC_POLAR, TRC_LINEAR, TRC_SWR, TRC_REAL, TRC_IMAG, TRC_R, TRC_X, TRC_Z, TRC_G, TRC_B, TRC_Y, TRC_Rp, TRC_Xp, TRC_sC, TRC_sL, TRC_pC, TRC_pL, TRC_Q, TRC_OFF
  static const char cmd_type_list[] = "logmag|phase|delay|smith|polar|linear|swr|real|imag|r|x|z|g|b|y|rp|xp|sc|sl|pc|pl|q|off";
  int type = get_str_index(argv[1], cmd_type_list);
  if (type >= 0) {
    if (argc > 2) {
      int src = my_atoi(argv[2]);
      if (src != 0 && src != 1)
        goto usage;
      set_trace_channel(t, src);
    }
    set_trace_type(t, type);
    return;
  }
  //                                            0      1
  static const char cmd_scale_ref_list[] = "scale|refpos";
  if (argc >= 3) {
    switch (get_str_index(argv[1], cmd_scale_ref_list)) {
      case 0: set_trace_scale(t, my_atof(argv[2])); break;
      case 1: set_trace_refpos(t, my_atof(argv[2])); break;
      default:
        goto usage;
    }
  }
  return;
usage:
  shell_printf("trace {0|1|2|3|all} [%s] [src]\r\n"\
               "trace {0|1|2|3} {%s} {value}\r\n", cmd_type_list, cmd_scale_ref_list);
}

VNA_SHELL_FUNCTION(cmd_edelay)
{
  if (argc != 1) {
    shell_printf("%f\r\n", electrical_delay);
    return;
  }
  set_electrical_delay(my_atof(argv[0]));
}


VNA_SHELL_FUNCTION(cmd_marker)
{
  static const char cmd_marker_list[] = "on|off";
  static const char cmd_marker_smith[] = "lin|log|ri|rx|rlc|gb|glc|rpxp|rplc";
  int t;
  if (argc == 0) {
    for (t = 0; t < MARKERS_MAX; t++) {
      if (markers[t].enabled) {
        shell_printf("%d %d %u\r\n", t+1, markers[t].index, markers[t].frequency);
      }
    }
    return;
  }
  request_to_redraw(REDRAW_MARKER|REDRAW_AREA);
  // Marker on|off command
  int enable = get_str_index(argv[0], cmd_marker_list);
  if (enable >= 0) { // string found: 0 - on, 1 - off
    active_marker = enable == 1 ? MARKER_INVALID : 0;
    for (t = 0; t < MARKERS_MAX; t++)
      markers[t].enabled = enable == 0;
    return;
  }
  // Set marker smith format
  int format = get_str_index(argv[0], cmd_marker_smith);
  if (format >=0){
    marker_smith_format = format;
    return;
  }
  t = my_atoi(argv[0])-1;
  if (t < 0 || t >= MARKERS_MAX)
    goto usage;
  if (argc == 1) {
    shell_printf("%d %d %u\r\n", t+1, markers[t].index, markers[t].frequency);
    active_marker = t;
    // select active marker
    markers[t].enabled = TRUE;
    return;
  }

  switch (get_str_index(argv[1], cmd_marker_list)) {
    case 0: markers[t].enabled = TRUE; active_marker = t; return;
    case 1: markers[t].enabled =FALSE; if (active_marker == t) active_marker = MARKER_INVALID; return;
    default:
      // select active marker and move to index
      markers[t].enabled = TRUE;
      int index = my_atoi(argv[1]);
      set_marker_index(t, index);
      active_marker = t;
      return;
  }
 usage:
  shell_printf("marker [n] [%s|{index}]\r\n"
               "marker [%s]\r\n", cmd_marker_list, cmd_marker_smith);
}

#if 0
VNA_SHELL_FUNCTION(cmd_grid)
{
  if (argc != 1) {
    shell_printf("grid %s\r\n", config._mode&VNA_MODE_SHOW_GRID ? "on" : "off");
    return;
  }
  if (my_atoi(argv[0]))
    config._mode|= VNA_MODE_SHOW_GRID;
  else
    config._mode&=~VNA_MODE_SHOW_GRID;
}
#endif

VNA_SHELL_FUNCTION(cmd_touchcal)
{
  (void)argc;
  (void)argv;
  //extern int16_t touch_cal[4];
  int i;

  shell_printf("first touch upper left, then lower right...");
  touch_cal_exec();
  shell_printf("done\r\n");

  shell_printf("touch cal params:");
  for (i = 0; i < 4; i++) {
    shell_printf(" %d", config._touch_cal[i]);
  }
  shell_printf("\r\n");
  request_to_redraw(REDRAW_CLRSCR | REDRAW_AREA | REDRAW_BATTERY | REDRAW_CAL_STATUS | REDRAW_FREQUENCY);
}

VNA_SHELL_FUNCTION(cmd_touchtest)
{
  (void)argc;
  (void)argv;
  touch_draw_test();
}

VNA_SHELL_FUNCTION(cmd_frequencies)
{
  int i;
  (void)argc;
  (void)argv;
  for (i = 0; i < sweep_points; i++) {
    shell_printf("%u\r\n", getFrequency(i));
  }
}

#ifdef ENABLE_TRANSFORM_COMMAND
static void
set_domain_mode(int mode) // accept DOMAIN_FREQ or DOMAIN_TIME
{
  if (mode != (props_mode & DOMAIN_MODE)) {
    props_mode = (props_mode & ~DOMAIN_MODE) | (mode & DOMAIN_MODE);
    request_to_redraw(REDRAW_FREQUENCY | REDRAW_MARKER);
    lever_mode = LM_MARKER;
  }
}

static inline void
set_timedomain_func(uint32_t func) // accept TD_FUNC_LOWPASS_IMPULSE, TD_FUNC_LOWPASS_STEP or TD_FUNC_BANDPASS
{
  props_mode = (props_mode & ~TD_FUNC) | func;
}

static inline void
set_timedomain_window(uint32_t func) // accept TD_WINDOW_MINIMUM/TD_WINDOW_NORMAL/TD_WINDOW_MAXIMUM
{
  props_mode = (props_mode & ~TD_WINDOW) | func;
}

VNA_SHELL_FUNCTION(cmd_transform)
{
  int i;
  if (argc == 0) {
    goto usage;
  }
  //                                         0   1       2    3        4       5      6       7
  static const char cmd_transform_list[] = "on|off|impulse|step|bandpass|minimum|normal|maximum";
  for (i = 0; i < argc; i++) {
    switch (get_str_index(argv[i], cmd_transform_list)) {
      case 0:
        set_domain_mode(DOMAIN_TIME);
        return;
      case 1:
        set_domain_mode(DOMAIN_FREQ);
        return;
      case 2:
        set_timedomain_func(TD_FUNC_LOWPASS_IMPULSE);
        return;
      case 3:
        set_timedomain_func(TD_FUNC_LOWPASS_STEP);
        return;
      case 4:
        set_timedomain_func(TD_FUNC_BANDPASS);
        return;
      case 5:
        set_timedomain_window(TD_WINDOW_MINIMUM);
        return;
      case 6:
        set_timedomain_window(TD_WINDOW_NORMAL);
        return;
      case 7:
        set_timedomain_window(TD_WINDOW_MAXIMUM);
        return;
      default:
        goto usage;
    }
  }
  return;
usage:
  shell_printf("usage: transform {%s} [...]\r\n", cmd_transform_list);
}
#endif

#ifdef ENABLE_TEST_COMMAND
VNA_SHELL_FUNCTION(cmd_test)
{
  (void)argc;
  (void)argv;

#if 0
  int i;
  for (i = 0; i < 100; i++) {
    palClearPad(GPIOC, GPIOC_LED);
    set_frequency(10000000);
    palSetPad(GPIOC, GPIOC_LED);
    chThdSleepMilliseconds(50);

    palClearPad(GPIOC, GPIOC_LED);
    set_frequency(90000000);
    palSetPad(GPIOC, GPIOC_LED);
    chThdSleepMilliseconds(50);
  }
#endif

#if 0
  int i;
  int mode = 0;
  if (argc >= 1)
    mode = my_atoi(argv[0]);

  for (i = 0; i < 20; i++) {
    palClearPad(GPIOC, GPIOC_LED);
    ili9341_test(mode);
    palSetPad(GPIOC, GPIOC_LED);
    chThdSleepMilliseconds(50);
  }
#endif

#if 0
  //extern adcsample_t adc_samples[2];
  //shell_printf("adc: %d %d\r\n", adc_samples[0], adc_samples[1]);
  int i;
  int x, y;
  for (i = 0; i < 50; i++) {
    test_touch(&x, &y);
    shell_printf("adc: %d %d\r\n", x, y);
    chThdSleepMilliseconds(200);
  }
  //extern int touch_x, touch_y;
  //shell_printf("adc: %d %d\r\n", touch_x, touch_y);
#endif
#if 0
  while (argc > 1) {
    int16_t x, y;
    touch_position(&x, &y);
    shell_printf("touch: %d %d\r\n", x, y);
    chThdSleepMilliseconds(200);
  }
#endif
}
#endif

#ifdef ENABLE_PORT_COMMAND
VNA_SHELL_FUNCTION(cmd_port)
{
  int port;
  if (argc != 1) {
    shell_printf("usage: port {0:TX 1:RX}\r\n");
    return;
  }
  port = my_atoi(argv[0]);
  tlv320aic3204_select(port);
}
#endif

#ifdef ENABLE_STAT_COMMAND
static struct {
  int16_t rms[2];
  int16_t ave[2];
#if 0
  int callback_count;
  int32_t last_counter_value;
  int32_t interval_cycles;
  int32_t busy_cycles;
#endif
} stat;

VNA_SHELL_FUNCTION(cmd_stat)
{
  int16_t *p = &rx_buffer[0];
  int32_t acc0, acc1;
  int32_t ave0, ave1;
//  float sample[2], ref[2];
//  minr, maxr,  mins, maxs;
  int32_t count = AUDIO_BUFFER_LEN;
  int i;
  (void)argc;
  (void)argv;
  for (int ch=0;ch<2;ch++){
    tlv320aic3204_select(ch);
    DSP_START(4);
    DSP_WAIT;
//    reset_dsp_accumerator();
//    dsp_process(&p[               0], AUDIO_BUFFER_LEN);
//    dsp_process(&p[AUDIO_BUFFER_LEN], AUDIO_BUFFER_LEN);

    acc0 = acc1 = 0;
    for (i = 0; i < AUDIO_BUFFER_LEN*2; i += 2) {
      acc0 += p[i  ];
      acc1 += p[i+1];
    }
    ave0 = acc0 / count;
    ave1 = acc1 / count;
    acc0 = acc1 = 0;
//    minr  = maxr = 0;
//    mins  = maxs = 0;
    for (i = 0; i < AUDIO_BUFFER_LEN*2; i += 2) {
      acc0 += (p[i  ] - ave0)*(p[i  ] - ave0);
      acc1 += (p[i+1] - ave1)*(p[i+1] - ave1);
//      if (minr < p[i  ]) minr = p[i  ];
//      if (maxr > p[i  ]) maxr = p[i  ];
//      if (mins < p[i+1]) mins = p[i+1];
//      if (maxs > p[i+1]) maxs = p[i+1];
    }
    stat.rms[0] = vna_sqrtf(acc0 / count);
    stat.rms[1] = vna_sqrtf(acc1 / count);
    stat.ave[0] = ave0;
    stat.ave[1] = ave1;
    shell_printf("Ch: %d\r\n", ch);
    shell_printf("average:   r: %6d s: %6d\r\n", stat.ave[0], stat.ave[1]);
    shell_printf("rms:       r: %6d s: %6d\r\n", stat.rms[0], stat.rms[1]);
//    shell_printf("min:     ref %6d ch %6d\r\n", minr, mins);
//    shell_printf("max:     ref %6d ch %6d\r\n", maxr, maxs);
  }
  //shell_printf("callback count: %d\r\n", stat.callback_count);
  //shell_printf("interval cycle: %d\r\n", stat.interval_cycles);
  //shell_printf("busy cycle: %d\r\n", stat.busy_cycles);
  //shell_printf("load: %d\r\n", stat.busy_cycles * 100 / stat.interval_cycles);
//  extern int awd_count;
//  shell_printf("awd: %d\r\n", awd_count);
}
#endif

#ifndef VERSION
#define VERSION "unknown"
#endif

const char NANOVNA_VERSION[] = VERSION;

VNA_SHELL_FUNCTION(cmd_version)
{
  (void)argc;
  (void)argv;
  shell_printf("%s\r\n", NANOVNA_VERSION);
}

VNA_SHELL_FUNCTION(cmd_vbat)
{
  (void)argc;
  (void)argv;
  shell_printf("%d mV\r\n", adc_vbat_read());
}

#ifdef ENABLE_VBAT_OFFSET_COMMAND
VNA_SHELL_FUNCTION(cmd_vbat_offset)
{
  if (argc != 1) {
    shell_printf("%d\r\n", config._vbat_offset);
    return;
  }
  config._vbat_offset = (int16_t)my_atoi(argv[0]);
}
#endif

#ifdef ENABLE_SI5351_TIMINGS
VNA_SHELL_FUNCTION(cmd_si5351time)
{
  (void)argc;
  int idx = my_atoui(argv[0]);
  uint16_t value = my_atoui(argv[1]);
  si5351_set_timing(idx, value);
}
#endif

#ifdef ENABLE_SI5351_REG_WRITE
VNA_SHELL_FUNCTION(cmd_si5351reg)
{
  if (argc != 2) {
    shell_printf("usage: si reg data\r\n");
    return;
  }
  uint8_t reg = my_atoui(argv[0]);
  uint8_t dat = my_atoui(argv[1]);
  uint8_t buf[] = { reg, dat };
  si5351_bulk_write(buf, 2);
}
#endif

static void set_I2C_timings(uint32_t timings) {
  I2CD1.i2c->CR1 &=~I2C_CR1_PE;
  I2CD1.i2c->TIMINGR = timings;
  I2CD1.i2c->CR1 |= I2C_CR1_PE;
}

#ifdef ENABLE_I2C_TIMINGS
VNA_SHELL_FUNCTION(cmd_i2ctime)
{
  (void)argc;
  uint32_t tim =  STM32_TIMINGR_PRESC(0U)  |
                  STM32_TIMINGR_SCLDEL(my_atoui(argv[0])) | STM32_TIMINGR_SDADEL(my_atoui(argv[1])) |
                  STM32_TIMINGR_SCLH(my_atoui(argv[2])) | STM32_TIMINGR_SCLL(my_atoui(argv[3]));
  set_I2C_timings(tim);
}
#endif

#ifdef ENABLE_INFO_COMMAND
VNA_SHELL_FUNCTION(cmd_info)
{
  (void)argc;
  (void)argv;
  int i = 0;
  while (info_about[i])
    shell_printf("%s\r\n", info_about[i++]);
}
#endif

#ifdef ENABLE_COLOR_COMMAND
VNA_SHELL_FUNCTION(cmd_color)
{
  uint32_t color;
  uint16_t i;
  if (argc != 2) {
    shell_printf("usage: color {id} {rgb24}\r\n");
    for (i=0; i < MAX_PALETTE; i++) {
      color = GET_PALTETTE_COLOR(i);
      color = HEXRGB(color);
      shell_printf(" %2d: 0x%06x\r\n", i, color);
    }
    return;
  }
  i = my_atoui(argv[0]);
  if (i >= MAX_PALETTE)
    return;
  color = RGBHEX(my_atoui(argv[1]));
  config._lcd_palette[i] = color;
  // Redraw all
  request_to_redraw(REDRAW_CLRSCR | REDRAW_AREA | REDRAW_CAL_STATUS | REDRAW_BATTERY | REDRAW_FREQUENCY);
}
#endif

#ifdef ENABLE_I2C_COMMAND
VNA_SHELL_FUNCTION(cmd_i2c){
  if (argc != 3) {
    shell_printf("usage: i2c page reg data\r\n");
    return;
  }
  uint8_t page = my_atoui(argv[0]);
  uint8_t reg  = my_atoui(argv[1]);
  uint8_t data = my_atoui(argv[2]);
  tlv320aic3204_write_reg(page, reg, data);
}
#endif

#ifdef ENABLE_BAND_COMMAND
VNA_SHELL_FUNCTION(cmd_band){
  static const char cmd_sweep_list[] = "mode|freq|div|mul|omul|pow|opow|l|r|lr|adj";
  if (argc != 3){
    shell_printf("cmd error\r\n");
    return;
  }
  int idx = my_atoui(argv[0]);
  int pidx = get_str_index(argv[1], cmd_sweep_list);
  si5351_update_band_config(idx, pidx, my_atoui(argv[2]));
}
#endif

#ifdef ENABLE_LCD_COMMAND
VNA_SHELL_FUNCTION(cmd_lcd){
  uint8_t d[VNA_SHELL_MAX_ARGUMENTS];
  if (argc == 0) return;
  for (int i=0;i<argc;i++)
    d[i] =  my_atoui(argv[i]);
  uint32_t ret = lcd_send_command(d[0], argc-1, &d[1]);
  shell_printf("ret = 0x%08X\r\n", ret);
  chThdSleepMilliseconds(5);
}
#endif

#ifdef ENABLE_THREADS_COMMAND
#if CH_CFG_USE_REGISTRY == FALSE
#error "Threads Requite enabled CH_CFG_USE_REGISTRY in chconf.h"
#endif
VNA_SHELL_FUNCTION(cmd_threads) 
{
  static const char *states[] = {CH_STATE_NAMES};
  thread_t *tp;
  (void)argc;
  (void)argv;
  shell_printf("stklimit|   stack|stk free|    addr|refs|prio|    state|        name"VNA_SHELL_NEWLINE_STR);
  tp = chRegFirstThread();
  do {
    uint32_t max_stack_use = 0U;
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE)
    uint32_t stklimit = (uint32_t)tp->wabase;
#if CH_DBG_FILL_THREADS == TRUE
    uint8_t *p = (uint8_t *)tp->wabase; while(p[max_stack_use]==CH_DBG_STACK_FILL_VALUE) max_stack_use++;
#endif
#else
    uint32_t stklimit = 0U;
#endif
    shell_printf("%08x|%08x|%08x|%08x|%4u|%4u|%9s|%12s"VNA_SHELL_NEWLINE_STR,
             stklimit, (uint32_t)tp->ctx.sp, max_stack_use, (uint32_t)tp,
             (uint32_t)tp->refs - 1, (uint32_t)tp->prio, states[tp->state],
             tp->name == NULL ? "" : tp->name);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}
#endif

#ifdef __USE_SERIAL_CONSOLE__
#ifdef ENABLE_USART_COMMAND
VNA_SHELL_FUNCTION(cmd_usart_cfg)
{
  if (argc != 1) goto result;
  uint32_t speed = my_atoui(argv[0]);
  if (speed < 300) speed = 300;
  config._serial_speed = speed;
  shell_update_speed();
result:
  shell_printf("Serial: %u baud\r\n", config._serial_speed);
}

VNA_SHELL_FUNCTION(cmd_usart)
{
  uint32_t time = 2000; // 200ms wait answer by default
  if (argc == 0 || argc > 2 || (VNA_mode & VNA_MODE_SERIAL)) return;
  if (argc == 2) time = my_atoui(argv[1])*10;
  sdWriteTimeout(&SD1, (uint8_t *)argv[0], strlen(argv[0]), time);
  sdWriteTimeout(&SD1, (uint8_t *)VNA_SHELL_NEWLINE_STR, sizeof(VNA_SHELL_NEWLINE_STR)-1, time);
  uint32_t size;
  uint8_t buffer[64];
  while ((size = sdReadTimeout(&SD1, buffer, sizeof(buffer), time)))
    streamWrite(&SDU1, buffer, size);
}
#endif
#endif

#ifdef __REMOTE_DESKTOP__
void send_region(remote_region_t *rd, uint8_t * buf, uint16_t size)
{
  if (SDU1.config->usbp->state == USB_ACTIVE) {
    streamWrite(shell_stream, (void*) rd, sizeof(remote_region_t));
    streamWrite(shell_stream, (void*) buf, size);
    streamWrite(shell_stream, (void*)"ch> \r\n", 6);
  }
  else
    sweep_mode&=~SWEEP_REMOTE;
}

VNA_SHELL_FUNCTION(cmd_refresh)
{
  static const char cmd_enable_list[] = "on|off";
  if (argc != 1) return;
  int enable = get_str_index(argv[0], cmd_enable_list);
       if (enable == 0) sweep_mode|= SWEEP_REMOTE;
  else if (enable == 1) sweep_mode&=~SWEEP_REMOTE;
  // redraw all on screen
  request_to_redraw(REDRAW_FREQUENCY | REDRAW_CAL_STATUS | REDRAW_AREA | REDRAW_BATTERY);
}

VNA_SHELL_FUNCTION(cmd_touch)
{
  if (argc != 2) return;
  remote_touch_set(REMOTE_PRESS, my_atoi(argv[0]), my_atoi(argv[1]));
}

VNA_SHELL_FUNCTION(cmd_release)
{
  int16_t x = -1, y = -1;
  if (argc == 2) {
    x = my_atoi(argv[0]);
    y = my_atoi(argv[1]);
  }
  remote_touch_set(REMOTE_RELEASE, x, y);
}
#endif

#ifdef ENABLE_SD_CARD_CMD
#ifndef __USE_SD_CARD__
#error "Need enable SD card support __USE_SD_CARD__ in nanovna.h, for use ENABLE_SD_CARD_CMD"
#endif

static FRESULT cmd_sd_card_mount(void){
  const FRESULT res = f_mount(fs_volume, "", 1);
  if (res != FR_OK)
    shell_printf("err: no card\r\n");
  return res;
}

VNA_SHELL_FUNCTION(cmd_sd_list)
{
  (void)argc;
  (void)argv;

  DIR dj;
  FILINFO fno;
  FRESULT res;
  if (cmd_sd_card_mount() != FR_OK)
    return;
  char *search;
  switch (argc){
    case 0: search =   "*.*";break;
    case 1: search = argv[0];break;
    default: shell_printf("usage: sd_list {pattern}\r\n"); return;
  }
  shell_printf("sd_list:\r\n");
  res = f_findfirst(&dj, &fno, "", search);
  while (res == FR_OK && fno.fname[0])
  {
    shell_printf("%s %u\r\n", fno.fname, fno.fsize);
    res = f_findnext(&dj, &fno);
  }
  f_closedir(&dj);
}

VNA_SHELL_FUNCTION(cmd_sd_read)
{
  char *buf = (char *)spi_buffer;
  if (argc != 1)
  {
     shell_printf("usage: sd_read {filename}\r\n");
     return;
  }
  const char *filename = argv[0];
  if (cmd_sd_card_mount() != FR_OK)
    return;

  if (f_open(fs_file, filename, FA_OPEN_EXISTING | FA_READ) != FR_OK){
    shell_printf("err: no file\r\n");
    return;
  }
  // shell_printf("sd_read: %s\r\n", filename);
  // number of bytes to follow (file size)
  uint32_t filesize = f_size(fs_file);
  streamWrite(shell_stream, (void *)&filesize, 4);
  UINT size = 0;
  // file data (send all data from file)
  while (f_read(fs_file, buf, 512, &size) == FR_OK && size > 0)
    streamWrite(shell_stream, (void *)buf, size);

  f_close(fs_file);
  return;
}

VNA_SHELL_FUNCTION(cmd_sd_delete)
{
  FRESULT res;
  if (argc != 1) {
     shell_printf("usage: sd_delete {filename}\r\n");
     return;
  }
  if (cmd_sd_card_mount() != FR_OK)
    return;
  const char *filename = argv[0];
  res = f_unlink(filename);
  shell_printf("delete: %s %s\r\n", filename, res == FR_OK ? "OK" : "err");
  return;
}
#endif

//=============================================================================
VNA_SHELL_FUNCTION(cmd_help);

#pragma pack(push, 2)
typedef struct {
  const char           *sc_name;
  vna_shellcmd_t    sc_function;
  uint16_t flags;
} VNAShellCommand;
#pragma pack(pop)

// Some commands can executed only in sweep thread, not in main cycle
#define CMD_WAIT_MUTEX  1
// Command execution need in sweep thread, and need break sweep for run
#define CMD_BREAK_SWEEP 2
// Command can run in shell thread (if sweep thread process UI, not sweep)
#define CMD_RUN_IN_UI   4
// Command can run in load script
#define CMD_RUN_IN_LOAD 8

static const VNAShellCommand commands[] =
{
    {"scan"        , cmd_scan        , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
#ifdef ENABLE_SCANBIN_COMMAND
    {"scan_bin"    , cmd_scan_bin    , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
#endif
    {"data"        , cmd_data        , 0},
    {"frequencies" , cmd_frequencies , 0},
    {"freq"        , cmd_freq        , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP|CMD_RUN_IN_UI|CMD_RUN_IN_LOAD},
    {"sweep"       , cmd_sweep       , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP|CMD_RUN_IN_UI|CMD_RUN_IN_LOAD},
    {"power"       , cmd_power       , CMD_RUN_IN_LOAD},
#ifdef USE_VARIABLE_OFFSET
    {"offset"      , cmd_offset      , CMD_WAIT_MUTEX|CMD_RUN_IN_UI|CMD_RUN_IN_LOAD},
#endif
    {"bandwidth"   , cmd_bandwidth   , CMD_RUN_IN_LOAD},
#ifdef __USE_RTC__
    {"time"        , cmd_time        , CMD_RUN_IN_UI},
#endif
#ifdef ENABLE_SD_CARD_CMD
    { "sd_list",   cmd_sd_list,   CMD_WAIT_MUTEX|CMD_BREAK_SWEEP|CMD_RUN_IN_UI},
    { "sd_read",   cmd_sd_read,   CMD_WAIT_MUTEX|CMD_BREAK_SWEEP|CMD_RUN_IN_UI},
    { "sd_delete", cmd_sd_delete, CMD_WAIT_MUTEX|CMD_BREAK_SWEEP|CMD_RUN_IN_UI},
#endif
#ifdef __VNA_ENABLE_DAC__
    {"dac"         , cmd_dac         , CMD_RUN_IN_LOAD},
#endif
    {"saveconfig"  , cmd_saveconfig  , CMD_RUN_IN_LOAD},
    {"clearconfig" , cmd_clearconfig , CMD_RUN_IN_LOAD},
#ifdef ENABLED_DUMP_COMMAND
    {"dump"        , cmd_dump        , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
#endif
#ifdef ENABLE_PORT_COMMAND
    {"port"        , cmd_port        , CMD_RUN_IN_LOAD},
#endif
#ifdef ENABLE_STAT_COMMAND
    {"stat"        , cmd_stat        , CMD_WAIT_MUTEX},
#endif
#ifdef ENABLE_GAIN_COMMAND
    {"gain"        , cmd_gain        , CMD_WAIT_MUTEX},
#endif
#ifdef ENABLE_SAMPLE_COMMAND
    {"sample"      , cmd_sample      , 0},
#endif
#ifdef ENABLE_TEST_COMMAND
    {"test"        , cmd_test        , 0},
#endif
    {"touchcal"    , cmd_touchcal    , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
    {"touchtest"   , cmd_touchtest   , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
    {"pause"       , cmd_pause       , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP|CMD_RUN_IN_UI|CMD_RUN_IN_LOAD},
    {"resume"      , cmd_resume      , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP|CMD_RUN_IN_UI|CMD_RUN_IN_LOAD},
    {"cal"         , cmd_cal         , CMD_WAIT_MUTEX},
    {"save"        , cmd_save        , CMD_RUN_IN_LOAD},
    {"recall"      , cmd_recall      , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP|CMD_RUN_IN_UI|CMD_RUN_IN_LOAD},
    {"trace"       , cmd_trace       , CMD_RUN_IN_LOAD},
    {"marker"      , cmd_marker      , CMD_RUN_IN_LOAD},
//  {"grid"        , cmd_grid        , CMD_RUN_IN_LOAD},
    {"edelay"      , cmd_edelay      , CMD_RUN_IN_LOAD},
    {"capture"     , cmd_capture     , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP|CMD_RUN_IN_UI},
#ifdef __REMOTE_DESKTOP__
    {"refresh"     , cmd_refresh     , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP|CMD_RUN_IN_UI},
    {"touch"       , cmd_touch       , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP|CMD_RUN_IN_UI},
    {"release"     , cmd_release     , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP|CMD_RUN_IN_UI},
#endif
    {"vbat"        , cmd_vbat        , CMD_RUN_IN_LOAD},
    {"tcxo"        , cmd_tcxo        , CMD_RUN_IN_LOAD},
    {"reset"       , cmd_reset       , CMD_RUN_IN_LOAD},
#ifdef __USE_SMOOTH__
    {"smooth"      , cmd_smooth      , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP|CMD_RUN_IN_UI|CMD_RUN_IN_LOAD},
#endif
#ifdef __USE_SERIAL_CONSOLE__
#ifdef ENABLE_USART_COMMAND
    {"usart_cfg"   , cmd_usart_cfg   , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP|CMD_RUN_IN_UI|CMD_RUN_IN_LOAD},
    {"usart"       , cmd_usart       , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP|CMD_RUN_IN_UI|CMD_RUN_IN_LOAD},
#endif
#endif
#ifdef ENABLE_VBAT_OFFSET_COMMAND
    {"vbat_offset" , cmd_vbat_offset , CMD_RUN_IN_LOAD},
#endif
#ifdef ENABLE_TRANSFORM_COMMAND
    {"transform"   , cmd_transform   , CMD_RUN_IN_LOAD},
#endif
    {"threshold"   , cmd_threshold   , CMD_RUN_IN_LOAD},
    {"help"        , cmd_help        , 0},
#ifdef ENABLE_INFO_COMMAND
    {"info"        , cmd_info        , 0},
#endif
    {"version"     , cmd_version     , 0},
#ifdef ENABLE_COLOR_COMMAND
    {"color"       , cmd_color       , CMD_RUN_IN_LOAD},
#endif
#ifdef ENABLE_I2C_COMMAND
    {"i2c"         , cmd_i2c         , CMD_WAIT_MUTEX},
#endif
#ifdef ENABLE_SI5351_REG_WRITE
    {"si"          , cmd_si5351reg   , CMD_WAIT_MUTEX},
#endif
#ifdef ENABLE_LCD_COMMAND
    {"lcd"         , cmd_lcd         , CMD_WAIT_MUTEX},
#endif
#ifdef ENABLE_THREADS_COMMAND
    {"threads"     , cmd_threads     , 0},
#endif
#ifdef ENABLE_SI5351_TIMINGS
    {"t"           , cmd_si5351time  , CMD_WAIT_MUTEX},
#endif
#ifdef ENABLE_I2C_TIMINGS
    {"i"           , cmd_i2ctime     , CMD_WAIT_MUTEX},
#endif
#ifdef ENABLE_BAND_COMMAND
    {"b"           , cmd_band        , CMD_WAIT_MUTEX},
#endif
    {NULL          , NULL            , 0}
};

VNA_SHELL_FUNCTION(cmd_help)
{
  (void)argc;
  (void)argv;
  const VNAShellCommand *scp = commands;
  shell_printf("Commands:");
  while (scp->sc_name != NULL) {
    shell_printf(" %s", scp->sc_name);
    scp++;
  }
  shell_printf(VNA_SHELL_NEWLINE_STR);
  return;
}

/*
 * VNA shell functions
 */

// Check Serial connection requirements
#ifdef __USE_SERIAL_CONSOLE__
#if HAL_USE_SERIAL == FALSE
#error "For serial console need HAL_USE_SERIAL as TRUE in halconf.h"
#endif

// Before start process command from shell, need select input stream
#define PREPARE_STREAM shell_stream = (VNA_mode & VNA_MODE_SERIAL) ? (BaseSequentialStream *)&SD1 : (BaseSequentialStream *)&SDU1;

// Update Serial connection speed and settings
void shell_update_speed(void){
  // Update Serial speed settings
  SerialConfig s_config = {config._serial_speed, 0, USART_CR2_STOP1_BITS, 0 };
  sdStop(&SD1);
  sdStart(&SD1, &s_config);  // USART config
}

// Check USB connection status
static bool usb_IsActive(void){
  return usbGetDriverStateI(&USBD1) == USB_ACTIVE;
}

// Reset shell I/O queue
void shell_reset_console(void){
  // Reset I/O queue over USB (for USB need also connect/disconnect)
  if (usb_IsActive()){
    if (VNA_mode & VNA_MODE_SERIAL)
      sduDisconnectI(&SDU1);
    else
      sduConfigureHookI(&SDU1);
  }
  // Reset I/O queue over Serial
  oqResetI(&SD1.oqueue);
  iqResetI(&SD1.iqueue);
}

// Check active connection for Shell
static bool shell_check_connect(void){
  // Serial connection always active
  if (VNA_mode & VNA_MODE_SERIAL)
    return true;
  // USB connection can be USB_SUSPENDED
  return usb_IsActive();
}

static void shell_init_connection(void){
/*
 * Initializes and start serial-over-USB CDC driver SDU1, connected to USBD1
 */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

/*
 * Set Serial speed settings for SD1
 */
  shell_update_speed();

/*
 * Activates the USB driver and then the USB bus pull-up on D+.
 * Note, a delay is inserted in order to not have to disconnect the cable
 * after a reset.
 */
  usbDisconnectBus(&USBD1);
  chThdSleepMilliseconds(100);
  usbStart(&USBD1, &usbcfg);
  usbConnectBus(&USBD1);

/*
 *  Set I/O stream (SDU1 or SD1) for shell
 */
  PREPARE_STREAM;
}

#else
// Only USB console, shell_stream always on USB
#define PREPARE_STREAM shell_stream = (BaseSequentialStream *)&SDU1;

// Check connection as Active, if no suspend input
static bool shell_check_connect(void){
  return SDU1.config->usbp->state == USB_ACTIVE;
}

// Init shell I/O connection over USB
static void shell_init_connection(void){
/*
 * Initializes and start serial-over-USB CDC driver SDU1, connected to USBD1
 */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

/*
 * Activates the USB driver and then the USB bus pull-up on D+.
 * Note, a delay is inserted in order to not have to disconnect the cable
 * after a reset.
 */
  usbDisconnectBus(&USBD1);
  chThdSleepMilliseconds(100);
  usbStart(&USBD1, &usbcfg);
  usbConnectBus(&USBD1);

/*
 *  Set I/O stream SDU1 for shell
 */
  PREPARE_STREAM;
}
#endif

static const VNAShellCommand *VNAShell_parceLine(char *line){
  // Parse and execute line
  char *lp = line, *ep;
  shell_nargs = 0;
  while (*lp != 0) {
    // Skipping white space and tabs at string begin.
    while (*lp == ' ' || *lp == '\t') lp++;
    // If an argument starts with a double quote then its delimiter is another quote, else
    // delimiter is white space.
    ep = (*lp == '"') ? strpbrk(++lp, "\"") : strpbrk(lp, " \t");
    // Store in args string
    shell_args[shell_nargs++] = lp;
    // Stop, end of input string
    if ((lp = ep) == NULL) break;
    // Argument limits check
    if (shell_nargs > VNA_SHELL_MAX_ARGUMENTS) {
      shell_printf("too many arguments, max " define_to_STR(VNA_SHELL_MAX_ARGUMENTS) "" VNA_SHELL_NEWLINE_STR);
      return NULL;
    }
    // Set zero at the end of string and continue check
    *lp++ = 0;
  }
  if (shell_nargs){
    const VNAShellCommand *scp;
    for (scp = commands; scp->sc_name != NULL; scp++)
      if (get_str_index(scp->sc_name, shell_args[0]) == 0)
        return scp;
  }
  return NULL;
}

//
// Read command line from shell_stream
//
static int VNAShell_readLine(char *line, int max_size)
{
  // send backspace, space for erase, backspace again
  char backspace[] = {0x08, 0x20, 0x08, 0x00};
  uint8_t c;
  // Prepare I/O for shell_stream
  PREPARE_STREAM;
  uint16_t j = 0;
  // Return 0 only if stream not active
  while (streamRead(shell_stream, &c, 1)) {
    // Backspace or Delete
    if (c == 0x08 || c == 0x7f) {
      if (j > 0) {shell_printf(backspace); j--;}
      continue;
    }
    // New line (Enter)
    if (c == '\r') {
      shell_printf(VNA_SHELL_NEWLINE_STR);
      line[j] = 0;
      return 1;
    }
    // Others (skip) or too long - skip
    if (c < ' ' || j >= max_size - 1) continue;
    streamPut(shell_stream, c); // Echo
    line[j++] = (char)c;
  }
  return 0;
}

//
// Parse and run command line
//
static void VNAShell_executeLine(char *line)
{
//  DEBUG_LOG(0, line); // debug console log
  // Execute line
  const VNAShellCommand *scp = VNAShell_parceLine(line);
  if (scp) {
    uint16_t cmd_flag = scp->flags;
    // Skip wait mutex if process UI
    if ((cmd_flag& CMD_RUN_IN_UI) && (sweep_mode&SWEEP_UI_MODE)) cmd_flag&=~CMD_WAIT_MUTEX;
    // Wait
    if (cmd_flag & CMD_WAIT_MUTEX) {
      shell_function = scp->sc_function;
      if (scp->flags & CMD_BREAK_SWEEP) operation_requested|=OP_CONSOLE;
      // Wait execute command in sweep thread
      do {
        chThdSleepMilliseconds(10);
      } while (shell_function);
      return;
    }
    scp->sc_function(shell_nargs - 1, &shell_args[1]);
//  DEBUG_LOG(10, "ok");
    return;
  }
  if (**shell_args) // unknown command (not empty), ignore <CR>
    shell_printf("%s?" VNA_SHELL_NEWLINE_STR, shell_args[0]);
}

#ifdef __SD_CARD_LOAD__
#ifndef __USE_SD_CARD__
#error "Need enable SD card support __USE_SD_CARD__ in nanovna.h, for use ENABLE_SD_CARD_CMD"
#endif
bool sd_card_load_config(void){
  // Mount card
  if (f_mount(fs_volume, "", 1) != FR_OK)
    return FALSE;

  if (f_open(fs_file, "config.ini", FA_OPEN_EXISTING | FA_READ) != FR_OK)
    return FALSE;

  // Disable shell output
  shell_stream = NULL;
  char *buf = (char *)spi_buffer;
  UINT size = 0;

  uint16_t j = 0, i;
  while (f_read(fs_file, buf, 512, &size) == FR_OK && size > 0){
    i = 0;
    while (i < size) {
      uint8_t c = buf[i++];
      // New line (Enter)
      if (c == '\r') {
//        shell_line[j  ] = '\r';
//        shell_line[j+1] = '\n';
//        shell_line[j+2] = 0;
//        shell_printf(shell_line);
        shell_line[j] = 0; j = 0;
        const VNAShellCommand *scp = VNAShell_parceLine(shell_line);
        if (scp && (scp->flags&CMD_RUN_IN_LOAD))
          scp->sc_function(shell_nargs - 1, &shell_args[1]);
        continue;
      }
      // Others (skip)
      if (c < 0x20) continue;
      // Store
      if (j < VNA_SHELL_MAX_LENGTH - 1)
        shell_line[j++] = (char)c;
    }
  }
  f_close(fs_file);
  PREPARE_STREAM;
  return TRUE;
}
#endif

#ifdef VNA_SHELL_THREAD
static THD_WORKING_AREA(waThread2, /* cmd_* max stack size + alpha */442);
THD_FUNCTION(myshellThread, p)
{
  (void)p;
  chRegSetThreadName("shell");
  while (true) {
    shell_printf(VNA_SHELL_PROMPT_STR);
    if (VNAShell_readLine(shell_line, VNA_SHELL_MAX_LENGTH))
      VNAShell_executeLine(shell_line);
    else // Putting a delay in order to avoid an endless loop trying to read an unavailable stream.
      chThdSleepMilliseconds(100);
  }
}
#endif

/*
 * I2C bus settings
 */
#if STM32_I2C1_CLOCK == 8    // STM32_I2C1SW == STM32_I2C1SW_HSI     (HSI=8MHz)
#if   STM32_I2C_SPEED == 400 // 400kHz @ HSI 8MHz (Use 26.4.10 I2C_TIMINGR register configuration examples from STM32 RM0091 Reference manual)
 #define STM32_I2C_INIT_T   STM32_TIMINGR_PRESC(0U)  |\
                            STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(1U) |\
                            STM32_TIMINGR_SCLH(3U)   | STM32_TIMINGR_SCLL(9U)
 #define STM32_I2C_TIMINGR  STM32_TIMINGR_PRESC(0U)  |\
                            STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(1U) |\
                            STM32_TIMINGR_SCLH(3U)   | STM32_TIMINGR_SCLL(9U)
#endif
#elif  STM32_I2C1_CLOCK == 48 // STM32_I2C1SW == STM32_I2C1SW_SYSCLK  (SYSCLK = 48MHz)
 #define STM32_I2C_INIT_T   STM32_TIMINGR_PRESC(5U) |\
                            STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(3U) |\
                            STM32_TIMINGR_SCLH(3U)   | STM32_TIMINGR_SCLL(9U)

 #if   STM32_I2C_SPEED == 400 // 400kHz @ SYSCLK 48MHz (Use 26.4.10 I2C_TIMINGR register configuration examples from STM32 RM0091 Reference manual)
 #define STM32_I2C_TIMINGR  STM32_TIMINGR_PRESC(5U)  |\
                            STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(3U) |\
                            STM32_TIMINGR_SCLH(3U)   | STM32_TIMINGR_SCLL(9U)
 #elif STM32_I2C_SPEED == 600 // 600kHz @ SYSCLK 48MHz, manually get values, x1.5 I2C speed
 #define STM32_I2C_TIMINGR  STM32_TIMINGR_PRESC(0U)   |\
                            STM32_TIMINGR_SCLDEL(10U) | STM32_TIMINGR_SDADEL(10U) |\
                            STM32_TIMINGR_SCLH(30U)   | STM32_TIMINGR_SCLL(50U)
 #elif STM32_I2C_SPEED == 900 // 900kHz @ SYSCLK 48MHz, manually get values, x2 I2C speed
 #define STM32_I2C_TIMINGR  STM32_TIMINGR_PRESC(0U)   |\
                            STM32_TIMINGR_SCLDEL(10U) | STM32_TIMINGR_SDADEL(10U) |\
                            STM32_TIMINGR_SCLH(23U)   | STM32_TIMINGR_SCLL(30U)
 #endif
#elif  STM32_I2C1_CLOCK == 72 // STM32_I2C1SW == STM32_I2C1SW_SYSCLK  (SYSCLK = 72MHz)
 #define STM32_I2C_INIT_T   STM32_TIMINGR_PRESC(0U)   |\
                            STM32_TIMINGR_SCLDEL(20U) | STM32_TIMINGR_SDADEL(20U) |\
                            STM32_TIMINGR_SCLH(80U)   | STM32_TIMINGR_SCLL(100U)

 #if   STM32_I2C_SPEED == 400 // ~400kHz @ SYSCLK 72MHz (Use 26.4.10 I2C_TIMINGR register configuration examples from STM32 RM0091 Reference manual)
 #define STM32_I2C_TIMINGR  STM32_TIMINGR_PRESC(0U)   |\
                            STM32_TIMINGR_SCLDEL(10U) | STM32_TIMINGR_SDADEL(10U) |\
                            STM32_TIMINGR_SCLH(80U)   | STM32_TIMINGR_SCLL(100U)
 #elif STM32_I2C_SPEED == 600 // ~600kHz @ SYSCLK 72MHz, manually get values, x1.5 I2C speed
 #define STM32_I2C_TIMINGR  STM32_TIMINGR_PRESC(0U)   |\
                            STM32_TIMINGR_SCLDEL(10U) | STM32_TIMINGR_SDADEL(10U) |\
                            STM32_TIMINGR_SCLH(40U)   | STM32_TIMINGR_SCLL(80U)
 #elif STM32_I2C_SPEED == 900 // ~900kHz @ SYSCLK 72MHz, manually get values, x2 I2C speed
 #define STM32_I2C_TIMINGR  STM32_TIMINGR_PRESC(0U)   |\
                            STM32_TIMINGR_SCLDEL(10U) | STM32_TIMINGR_SDADEL(10U) |\
                            STM32_TIMINGR_SCLH(30U)   | STM32_TIMINGR_SCLL(40U)
 #endif
#endif

#ifndef STM32_I2C_TIMINGR
#error "Need define I2C bus TIMINGR settings"
#endif

// I2C clock bus setting: depend from STM32_I2C1SW in mcuconf.h
static const I2CConfig i2ccfg = {
  .timingr = STM32_I2C_INIT_T,  // TIMINGR register initialization. (use I2C timing configuration tool for STM32F3xx and STM32F0xx microcontrollers (AN4235))
  .cr1 = 0,                      // CR1 register initialization.
  .cr2 = 0                       // CR2 register initialization.
};

// Main thread stack size defined in makefile USE_PROCESS_STACKSIZE = 0x200
// Profile stack usage (enable threads command by def ENABLE_THREADS_COMMAND) show:
// Stack maximum usage = 472 bytes (need test more and run all commands), free stack = 40 bytes
//
int main(void)
{
/*
 * Initialize ChibiOS systems
 */
  halInit();
  chSysInit();

/*
 * SPI bus and LCD Initialize
 */
  lcd_init();

/*
 * Restore config
 */
  config_recall();

/*
 * restore frequencies and calibration 0 slot / backup id properties from flash memory
 */
  load_start_properties();

/*
 * Set frequency offset
 */
#ifdef USE_VARIABLE_OFFSET
  si5351_set_frequency_offset(IF_OFFSET);
#endif
/*
 * Init Shell console connection data
 */
  shell_init_connection();

/*
 * I2C bus
 */
  i2cStart(&I2CD1, &i2ccfg);

/*
 * Start si5351
 */
  si5351_init();

/*
 * Initialize RTC library (not used ChibiOS RTC module)
 */
#ifdef __USE_RTC__
  rtc_init();
#endif

/*
 * tlv320aic Initialize (audio codec)
 */
  tlv320aic3204_init();
//  chThdSleepMilliseconds(100);

/*
 * I2S Initialize
 */
  i2sInit();
  i2sObjectInit(&I2SD2);
  i2sStart(&I2SD2, &i2sconfig);
  i2sStartExchange(&I2SD2);

/*
 * SD Card init (if inserted) allow fix issues
 * Some card after insert work in SDIO mode and can corrupt SPI exchange (need switch it to SPI)
 */
#ifdef __USE_SD_CARD__
  disk_initialize(0);
#endif

/*
 * Starting DAC1 driver, setting up the output pin as analog as suggested by the Reference Manual.
 */
#ifdef  __VNA_ENABLE_DAC__
  dac_init();
#endif

/*
 * I2C bus run on work speed
 */
  set_I2C_timings(STM32_I2C_TIMINGR);

/*
 * Startup sweep thread
 */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO-1, Thread1, NULL);

  while (1) {
    if (shell_check_connect()) {
      shell_printf(VNA_SHELL_NEWLINE_STR"NanoVNA Shell"VNA_SHELL_NEWLINE_STR);
#ifdef VNA_SHELL_THREAD
#if CH_CFG_USE_WAITEXIT == FALSE
#error "VNA_SHELL_THREAD use chThdWait, need enable CH_CFG_USE_WAITEXIT in chconf.h"
#endif
      thread_t *shelltp = chThdCreateStatic(waThread2, sizeof(waThread2),
                                            NORMALPRIO + 1,
                                            myshellThread, NULL);
      chThdWait(shelltp);
#else
      do {
        shell_printf(VNA_SHELL_PROMPT_STR);
        if (VNAShell_readLine(shell_line, VNA_SHELL_MAX_LENGTH))
          VNAShell_executeLine(shell_line);
        else
          chThdSleepMilliseconds(200);
      } while (shell_check_connect());
#endif
    }
    chThdSleepMilliseconds(1000);
  }
}

/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
void HardFault_Handler(void);

void hard_fault_handler_c(uint32_t *sp) __attribute__((naked));

void HardFault_Handler(void)
{
  uint32_t *sp;
  //__asm volatile ("mrs %0, msp \n\t": "=r" (sp) );
  __asm volatile("mrs %0, psp \n\t" : "=r"(sp));
  hard_fault_handler_c(sp);
}

void hard_fault_handler_c(uint32_t *sp) 
{
#ifdef ENABLE_HARD_FAULT_HANDLER_DEBUG
  uint32_t r0  = sp[0];
  uint32_t r1  = sp[1];
  uint32_t r2  = sp[2];
  uint32_t r3  = sp[3];
  register uint32_t  r4 __asm("r4");
  register uint32_t  r5 __asm("r5");
  register uint32_t  r6 __asm("r6");
  register uint32_t  r7 __asm("r7");
  register uint32_t  r8 __asm("r8");
  register uint32_t  r9 __asm("r9");
  register uint32_t r10 __asm("r10");
  register uint32_t r11 __asm("r11");
  uint32_t r12 = sp[4];
  uint32_t lr  = sp[5];
  uint32_t pc  = sp[6];
  uint32_t psr = sp[7];
  int y = 0;
  int x = 20;
  lcd_set_background(LCD_BG_COLOR);
  lcd_set_foreground(LCD_FG_COLOR);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "SP  0x%08x",  (uint32_t)sp);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R0  0x%08x",  r0);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R1  0x%08x",  r1);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R2  0x%08x",  r2);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R3  0x%08x",  r3);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R4  0x%08x",  r4);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R5  0x%08x",  r5);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R6  0x%08x",  r6);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R7  0x%08x",  r7);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R8  0x%08x",  r8);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R9  0x%08x",  r9);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R10 0x%08x", r10);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R11 0x%08x", r11);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "R12 0x%08x", r12);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "LR  0x%08x",  lr);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "PC  0x%08x",  pc);
  lcd_printf(x, y+=FONT_STR_HEIGHT, "PSR 0x%08x", psr);

  shell_printf("===================================\r\n");
#else
  (void)sp;
#endif
  while (true) {
  }
}
// For new compilers
//void _exit(int x){(void)x;}
//void _kill(void){}
//int  _write (int file, char *data, int len) {(void)file; (void)data; return len;}
//void _getpid(void){}
