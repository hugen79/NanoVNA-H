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

#include "ch.h"
#include "hal.h"
#include "usbcfg.h"
#include "si5351.h"
#include "nanovna.h"
#include "fft.h"

#include <chprintf.h>
#include <string.h>
#include <math.h>

/*
 *  Shell settings
 */
// If need run shell as thread (use more amount of memory fore stack), after
// enable this need reduce spi_buffer size, by default shell run in main thread
// #define VNA_SHELL_THREAD

static BaseSequentialStream *shell_stream;

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

//#define ENABLED_DUMP_COMMAND
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
// Enable si5351 timing command, used for debug
//#define ENABLE_SI5351_TIMINGS
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
// Enable SD card console command
//#define ENABLE_SD_CARD_CMD

static void apply_CH0_error_term_at(int i);
static void apply_CH1_error_term_at(int i);
static void apply_edelay(void);

static uint16_t get_sweep_mask(void);
static void cal_interpolate(void);
static void update_frequencies(bool interpolate);
static int  set_frequency(uint32_t freq);
static void set_frequencies(uint32_t start, uint32_t stop, uint16_t points);
static bool sweep(bool break_on_operation, uint16_t ch_mask);
static void transform_domain(void);
extern void show_version(bool mode);

uint8_t sweep_mode = SWEEP_ENABLE;
uint8_t redraw_request = 0; // contains REDRAW_XXX flags

// sweep operation variables
volatile uint16_t wait_count = 0;
// current sweep point (used for continue sweep if user break)
static uint16_t p_sweep = 0;
// ChibiOS i2s buffer must be 2x size (for process one while next buffer filled by DMA)
static int16_t rx_buffer[AUDIO_BUFFER_LEN * 2];
// Sweep measured data
float measured[2][POINTS_COUNT][2];
uint32_t frequencies[POINTS_COUNT];

#undef VERSION
#define VERSION "1.0.45"

// Version text, displayed in Config->Version menu, also send by info command
const char *info_about[]={
  "Board: " BOARD_NAME,
  "2019-2021 Copyright @hugen     NANOVNA.COM",
  "Based on @DiSlord @edy555 source, licensed under GPL.",
  "Version: " VERSION " ["\
  "p:"define_to_STR(POINTS_COUNT)", "\
  "IF:"define_to_STR(FREQUENCY_IF_K)"k, "\
  "ADC:"define_to_STR(AUDIO_ADC_FREQ_K)"k, "\
  "Lcd:"define_to_STR(LCD_WIDTH)"x"define_to_STR(LCD_HEIGHT)\
  "]",  "Build Time: " __DATE__ " - " __TIME__,
  "Kernel: " CH_KERNEL_VERSION,
  "Compiler: " PORT_COMPILER_NAME,
  "Architecture: " PORT_ARCHITECTURE_NAME " Core Variant: " PORT_CORE_VARIANT_NAME,
  "Port Info: " PORT_INFO,
  "Platform: " PLATFORM_NAME,
  0 // sentinel
};

// Allow draw some debug on LCD
#ifdef DEBUG_CONSOLE_SHOW
void my_debug_log(int offs, char *log){
  static uint16_t shell_line_y = 0;
  ili9341_set_foreground(LCD_FG_COLOR);
  ili9341_set_background(LCD_BG_COLOR);
  ili9341_fill(FREQUENCIES_XPOS1, shell_line_y, LCD_WIDTH-FREQUENCIES_XPOS1, 2 * FONT_GET_HEIGHT);
  ili9341_drawstring(log, FREQUENCIES_XPOS1 + offs, shell_line_y);
  shell_line_y+=FONT_STR_HEIGHT;
  if (shell_line_y >= LCD_HEIGHT - FONT_STR_HEIGHT*4) shell_line_y=0;
}
#define DEBUG_LOG(offs, text)    my_debug_log(offs, text);
#else
#define DEBUG_LOG(offs, text)
#endif

static THD_WORKING_AREA(waThread1, 768);
static THD_FUNCTION(Thread1, arg)
{
  (void)arg;
  chRegSetThreadName("sweep");

  while (1) {
    bool completed = false;
    if (sweep_mode&(SWEEP_ENABLE|SWEEP_ONCE)) {
      completed = sweep(true, get_sweep_mask());
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
    ui_process();
    // Process collected data, calculate trace coordinates and plot only if scan completed
    if ((sweep_mode & SWEEP_ENABLE) && completed) {
      if (electrical_delay != 0) apply_edelay();
      if ((domain_mode & DOMAIN_MODE) == DOMAIN_TIME) transform_domain();

      // Prepare draw graphics, cache all lines, mark screen cells for redraw
      plot_into_index(measured);
      redraw_request |= REDRAW_CELLS | REDRAW_BATTERY;
    }
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
  return bessel0(beta * sqrt(1 - r * r)) / bessel0(beta);
}

static void
transform_domain(void)
{
  // use spi_buffer as temporary buffer and calculate ifft for time domain
  // Need 2 * sizeof(float) * FFT_SIZE bytes for work
#if 2*4*FFT_SIZE > (SPI_BUFFER_SIZE * LCD_PIXEL_SIZE)
#error "Need increase spi_buffer or use less FFT_SIZE value"
#endif
  float* tmp = (float*)spi_buffer;

  uint16_t window_size = sweep_points, offset = 0;
  uint8_t is_lowpass = FALSE;
  uint8_t td_func = domain_mode & TD_FUNC;
  switch (td_func) {
    case TD_FUNC_BANDPASS:
      offset = 0;
      window_size = sweep_points;
      break;
    case TD_FUNC_LOWPASS_IMPULSE:
    case TD_FUNC_LOWPASS_STEP:
      is_lowpass = TRUE;
      offset = sweep_points;
      window_size = sweep_points * 2;
      break;
  }

  float beta = 0.0f;
  switch (domain_mode & TD_WINDOW) {
    case TD_WINDOW_MINIMUM:
//    beta = 0.0f;  // this is rectangular
      break;
    case TD_WINDOW_NORMAL:
      beta = 6.0f;
      break;
    case TD_WINDOW_MAXIMUM:
      beta = 13.0f;
      break;
  }

#if 1
  // recalculate the scale factor if any window details are changed.
  // the scale factor is to compensate for windowing.
  static float window_scale = 1.0f;
  static uint16_t td_cache = 0;
  uint16_t td_check = (domain_mode & (TD_WINDOW|TD_FUNC))|(sweep_points<<5);
  if (td_cache!=td_check){
    td_cache=td_check;
    if (td_func == TD_FUNC_LOWPASS_STEP)
      window_scale = 1.0f;
    else {
      window_scale = 0.0f;
      for (int i = 0; i < sweep_points; i++)
        window_scale += kaiser_window(i + offset, window_size, beta);
      window_scale = (FFT_SIZE/2) / window_scale;
      if (td_func == TD_FUNC_BANDPASS)
        window_scale *= 2;
    }
  }
#else
  // Disable compensation
  #define window_scale 1
#endif

  uint16_t ch_mask = get_sweep_mask();
  for (int ch = 0; ch < 2; ch++,ch_mask>>=1) {
    if ((ch_mask&1)==0) continue;
    memcpy(tmp, measured[ch], sizeof(measured[0]));
    for (int i = 0; i < sweep_points; i++) {
      float w = kaiser_window(i + offset, window_size, beta) * window_scale;
      tmp[i * 2 + 0] *= w;
      tmp[i * 2 + 1] *= w;
    }
    for (int i = sweep_points; i < FFT_SIZE; i++) {
      tmp[i * 2 + 0] = 0.0;
      tmp[i * 2 + 1] = 0.0;
    }
    if (is_lowpass) {
      for (int i = 1; i < sweep_points; i++) {
        tmp[(FFT_SIZE - i) * 2 + 0] = tmp[i * 2 + 0];
        tmp[(FFT_SIZE - i) * 2 + 1] = -tmp[i * 2 + 1];
      }
    }

    fft_inverse((float(*)[2])tmp);
    memcpy(measured[ch], tmp, sizeof(measured[0]));
    for (int i = 0; i < sweep_points; i++) {
      measured[ch][i][0] /= (float)FFT_SIZE;
      if (is_lowpass) {
        measured[ch][i][1] = 0.0;
      } else {
        measured[ch][i][1] /= (float)FFT_SIZE;
      }
    }
    if ((domain_mode & TD_FUNC) == TD_FUNC_LOWPASS_STEP) {
      for (int i = 1; i < sweep_points; i++) {
        measured[ch][i][0] += measured[ch][i - 1][0];
      }
    }
  }
}

// Shell commands output
static int shell_printf(const char *fmt, ...)
{
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
  update_frequencies(cal_status & CALSTAT_APPLY);
  resume_sweep();
}

VNA_SHELL_FUNCTION(cmd_reset)
{
  (void)argc;
  (void)argv;

  if (argc == 1) {
    if (strcmp(argv[0], "dfu") == 0) {
      shell_printf("Performing reset to DFU mode\r\n");
      enter_dfu();
      return;
    }
  }
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

double
my_atof(const char *p)
{
  int neg = FALSE;
  if (*p == '-')
    neg = TRUE;
  if (*p == '-' || *p == '+')
    p++;
  double x = my_atoi(p);
  while (_isdigit((int)*p))
    p++;
  if (*p == '.') {
    double d = 1.0f;
    p++;
    while (_isdigit((int)*p)) {
      d /= 10;
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

//
// Function used for search substring v in list
// Example need search parameter "center" in "start|stop|center|span|cw" getStringIndex return 2
// If not found return -1
// Used for easy parse command arguments
static int get_str_index(char *v, const char *list)
{
  int i = 0;
  while (1) {
    char *p = v;
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

#ifdef USE_VARIABLE_OFFSET
VNA_SHELL_FUNCTION(cmd_offset)
{
  if (argc != 1) {
    shell_printf("usage: offset {frequency offset(Hz)}\r\n");
    return;
  }
  int32_t offset = my_atoi(argv[0]);
  generate_DSP_Table(offset);
  si5351_set_frequency_offset(offset);
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
  if (value > SI5351_CLK_DRIVE_STRENGTH_8MA) value = SI5351_CLK_DRIVE_STRENGTH_AUTO;
  if (current_props._power == value) return;
  current_props._power = value;
  // Update power if pause
  if (!(sweep_mode&SWEEP_ENABLE)) si5351_set_power(value);
  redraw_request|=REDRAW_CAL_STATUS;
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
  shell_printf("20%02X/%02X/%02X %02X:%02X:%02X\r\n"\
               "usage: time {[%s] 0-99} or {b 0xYYMMDD 0xHHMMSS}\r\n", time[6], time[5], time[4], time[2], time[1], time[0], time_cmd);
}
#endif

#ifdef __VNA_ENABLE_DAC__
// Check DAC enabled in ChibiOS
#if HAL_USE_DAC == FALSE
#error "Need set HAL_USE_DAC in halconf.h for use DAC"
#endif

static const DACConfig dac1cfg1 = {
  //init:         1922U,
  init:         0,
  datamode:     DAC_DHRM_12BIT_RIGHT
};

VNA_SHELL_FUNCTION(cmd_dac)
{
  int value;
  if (argc != 1) {
    shell_printf("usage: dac {value(0-4095)}\r\n"\
                 "current value: %d\r\n", config.dac_value);
    return;
  }
  value = my_atoui(argv[0]);
  config.dac_value = value;
  dacPutChannelX(&DACD2, 0, value);
}
#endif

VNA_SHELL_FUNCTION(cmd_threshold)
{
  uint32_t value;
  if (argc != 1) {
    shell_printf("usage: threshold {frequency in harmonic mode}\r\n"\
                 "current: %d\r\n", config.harmonic_freq_threshold);
    return;
  }
  value = my_atoui(argv[0]);
  config.harmonic_freq_threshold = value;
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

  if (strcmp(argv[0], "1234") != 0) {
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

#ifdef ENABLED_DUMP_COMMAND
VNA_SHELL_FUNCTION(cmd_dump)
{
  int i, j;
  int len;

  if (argc == 1)
    dump_selection = my_atoi(argv[0]);

  dsp_start(3);
  dsp_wait();

  len = AUDIO_BUFFER_LEN;
  if (dump_selection == 1 || dump_selection == 2)
    len /= 2;
  for (i = 0; i < len; ) {
    for (j = 0; j < 16; j++, i++) {
      shell_printf("%04x ", 0xffff & (int)dump_buffer[i]);
    }
    shell_printf("\r\n");
  }
}
#endif

VNA_SHELL_FUNCTION(cmd_capture)
{
// read pixel count at one time (PART*2 bytes required for read buffer)
  (void)argc;
  (void)argv;
  int y;
#if (SPI_BUFFER_SIZE*LCD_PIXEL_SIZE) < (2*LCD_WIDTH*2)
#error "Low size of spi_buffer for cmd_capture"
#endif
  // read 2 row pixel time (read buffer limit by 2/3 + 1 from spi_buffer size)
  for (y = 0; y < LCD_HEIGHT; y += 2) {
    // use uint16_t spi_buffer[2048] (defined in ili9341) for read buffer
    ili9341_read_memory(0, y, LCD_WIDTH, 2, (uint16_t *)spi_buffer);
    streamWrite(shell_stream, (void*)spi_buffer, 2 * LCD_WIDTH * sizeof(uint16_t));
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
  .dac_value   = 1922,
  .lcd_palette = LCD_DEFAULT_PALETTE,
  .touch_cal   = DEFAULT_TOUCH_CONFIG,
  ._mode       = VNA_MODE_START_STOP,
  .harmonic_freq_threshold = FREQUENCY_THRESHOLD,
  ._serial_speed = SERIAL_DEFAULT_BITRATE,
  .vbat_offset = 320,
  ._brightness = DEFAULT_BRIGHTNESS,
  .bandwidth = BANDWIDTH_1000
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
  { 1, 0, 30, 0 }, { 0, 0, 40, 0 }, { 0, 0, 60, 0 }, { 0, 0, 80, 0 }
};

// Load propeties default settings
void load_default_properties(void)
{
//Magic add on caldata_save
//current_props.magic = CONFIG_MAGIC;
  current_props._frequency0   =     50000;    // start =  50kHz
  current_props._frequency1   = 900000000;    // end   = 900MHz
  current_props._sweep_points = POINTS_COUNT_DEFAULT; // Set default points count
  current_props._cal_status   = 0;
//This data not loaded by default
//current_props._cal_data[5][POINTS_COUNT][2];
//=============================================
  current_props._electrical_delay = 0.0;
  memcpy(current_props._trace, def_trace, sizeof(def_trace));
  memcpy(current_props._markers, def_markers, sizeof(def_markers));
  current_props._velocity_factor = 0.7;
  current_props._active_marker   = 0;
  current_props._domain_mode     = 0;
  current_props._marker_smith_format = MS_RLC;
  current_props._power = SI5351_CLK_DRIVE_STRENGTH_AUTO;
//Checksum add on caldata_save
//current_props.checksum = 0;
}

int load_properties(uint32_t id){
  int r = caldata_recall(id);
  update_frequencies(false);
  return r;
}

#ifdef ENABLED_DUMP_COMMAND
int16_t dump_buffer[AUDIO_BUFFER_LEN];
int16_t dump_selection = 0;
static void
duplicate_buffer_to_dump(int16_t *p)
{
  if (dump_selection == 1)
    p = samp_buf;
  else if (dump_selection == 2)
    p = ref_buf;
  memcpy(dump_buffer, p, sizeof dump_buffer);
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
  if (wait_count == config.bandwidth+2)      // At this moment in buffer exist noise data, reset and wait next clean buffer
    reset_dsp_accumerator();
  else if (wait_count <= config.bandwidth+1) // Clean data ready, process it
    dsp_process(p, n);
#ifdef ENABLED_DUMP_COMMAND
  duplicate_buffer_to_dump(p);
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
#define DELAY_CHANNEL_CHANGE  timings[6]
#define DELAY_SWEEP_START     timings[7]

#else
// Use x 100us settings
#define DELAY_CHANNEL_CHANGE   3    // Delay for switch ADC channel
#define DELAY_SWEEP_START     50    // Sweep start delay, allow remove noise at 1 point
#endif

#define DSP_START(delay) {ready_time = chVTGetSystemTimeX() + delay; wait_count = config.bandwidth+2;}
#define DSP_WAIT         while (wait_count) {__WFI();}
#define RESET_SWEEP      {p_sweep = 0;}

#define SWEEP_CH0_MEASURE   1
#define SWEEP_CH1_MEASURE   2

static uint16_t get_sweep_mask(void){
  uint16_t ch_mask = 0;
  int t;
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    if (trace[t].channel == 0) ch_mask|=SWEEP_CH0_MEASURE;
    if (trace[t].channel == 1) ch_mask|=SWEEP_CH1_MEASURE;
  }
  return ch_mask;
}

// main loop for measurement
static bool sweep(bool break_on_operation, uint16_t ch_mask)
{
  int delay;
  if (p_sweep>=sweep_points || break_on_operation == false) RESET_SWEEP;
  if (break_on_operation && ch_mask == 0)
    return false;
  // Blink LED while scanning
  palClearPad(GPIOC, GPIOC_LED);
//  START_PROFILE;
  ili9341_set_background(LCD_SWEEP_LINE_COLOR);
  // Wait some time for stable power
  int st_delay = DELAY_SWEEP_START;
  for (; p_sweep < sweep_points; p_sweep++) {
    delay = set_frequency(frequencies[p_sweep]);
    // CH0:REFLECTION, reset and begin measure
    if (ch_mask & SWEEP_CH0_MEASURE){
      tlv320aic3204_select(0);
      DSP_START(delay+st_delay);
      delay = DELAY_CHANNEL_CHANGE;
      //================================================
      // Place some code thats need execute while delay
      //================================================
      DSP_WAIT;
      (*sample_func)(measured[0][p_sweep]);      // calculate reflection coefficient
      if (APPLY_CALIBRATION_AFTER_SWEEP == 0 && (cal_status & CALSTAT_APPLY))
        apply_CH0_error_term_at(p_sweep);
    }
    // CH1:TRANSMISSION, reset and begin measure
    if (ch_mask & SWEEP_CH1_MEASURE){
      tlv320aic3204_select(1);
      DSP_START(delay+st_delay);
      //================================================
      // Place some code thats need execute while delay
      //================================================
      DSP_WAIT;
      (*sample_func)(measured[1][p_sweep]);      // Measure transmission coefficient
      if (APPLY_CALIBRATION_AFTER_SWEEP == 0 && (cal_status & CALSTAT_APPLY))
        apply_CH1_error_term_at(p_sweep);
    }
    if (operation_requested && break_on_operation) break;
    st_delay = 0;
// Display SPI made noise on measurement (can see in CW mode)
    if (config.bandwidth >= BANDWIDTH_100)
      ili9341_fill(OFFSETX+CELLOFFSETX, OFFSETY, (p_sweep * WIDTH)/(sweep_points-1), 1);
  }
  ili9341_set_background(LCD_GRID_COLOR);
  if (config.bandwidth >= BANDWIDTH_100)
    ili9341_fill(OFFSETX+CELLOFFSETX, OFFSETY, WIDTH, 1);
  // Apply calibration at end if need
  if (APPLY_CALIBRATION_AFTER_SWEEP && (cal_status & CALSTAT_APPLY) && p_sweep == sweep_points){
    uint16_t start_sweep;
    for (start_sweep = 0; start_sweep < p_sweep; start_sweep++){
      if (ch_mask & SWEEP_CH0_MEASURE) apply_CH0_error_term_at(start_sweep);
      if (ch_mask & SWEEP_CH1_MEASURE) apply_CH1_error_term_at(start_sweep);
    }
  }
//  STOP_PROFILE;
  // blink LED while scanning
  palSetPad(GPIOC, GPIOC_LED);
  return p_sweep == sweep_points;
}

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

static int set_frequency(uint32_t freq)
{
  return si5351_set_frequency(freq, current_props._power);
}

void set_bandwidth(uint16_t bw_count){
  config.bandwidth = bw_count&0x1FF;
  redraw_request|=REDRAW_FREQUENCY;
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
  shell_printf("bandwidth %d (%uHz)\r\n", config.bandwidth, get_bandwidth_frequency(config.bandwidth));
}

void set_sweep_points(uint16_t points){
  if (points == sweep_points || points > POINTS_COUNT)
    return;

  sweep_points = points;
  update_frequencies(cal_status & CALSTAT_APPLY);

}

#define SCAN_MASK_OUT_FREQ       0b00000001
#define SCAN_MASK_OUT_DATA0      0b00000010
#define SCAN_MASK_OUT_DATA1      0b00000100
#define SCAN_MASK_NO_CALIBRATION 0b00001000
#define SCAN_MASK_BINARY         0b10000000

VNA_SHELL_FUNCTION(cmd_scan)
{
  uint32_t start, stop;
  uint16_t points = sweep_points;
  int i;
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

  uint32_t old_cal_status = cal_status;
  if (mask&SCAN_MASK_NO_CALIBRATION) cal_status&=~CALSTAT_APPLY;
  // Rebuild frequency table if need
  if (frequencies[0]!=start || frequencies[points-1]!=stop){
    set_frequencies(start, stop, points);
    if (cal_status & CALSTAT_APPLY)
      cal_interpolate();
  }

  if (sweep_ch & (SWEEP_CH0_MEASURE|SWEEP_CH1_MEASURE))
    sweep(false, sweep_ch);

  cal_status = old_cal_status; // restore

  pause_sweep();
  // Output data after if set (faster data receive)
  if (mask) {
    if (mask&SCAN_MASK_BINARY){
      streamWrite(shell_stream, (void *)&mask, sizeof(uint16_t));
      streamWrite(shell_stream, (void *)&points, sizeof(uint16_t));
      for (i = 0; i < points; i++) {
        if (mask & SCAN_MASK_OUT_FREQ ) streamWrite(shell_stream, (void *)&frequencies[i],    sizeof(uint32_t));  // 4 bytes .. frequency
        if (mask & SCAN_MASK_OUT_DATA0) streamWrite(shell_stream, (void *)&measured[0][i][0], sizeof(float)* 2);  // 4+4 bytes .. S11 real/imag
        if (mask & SCAN_MASK_OUT_DATA1) streamWrite(shell_stream, (void *)&measured[1][i][0], sizeof(float)* 2);  // 4+4 bytes .. S21 real/imag
      }
    }
    else{
      for (i = 0; i < points; i++) {
        if (mask & SCAN_MASK_OUT_FREQ ) shell_printf("%u ", frequencies[i]);
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

void set_marker_index(int m, int idx)
{
  if (m == MARKER_INVALID || idx < 0 || idx >= sweep_points) return;
  markers[m].index = idx;
  markers[m].frequency = frequencies[idx];
}

static void
update_marker_index(void)
{
  int m, idx;
  uint32_t fstart = get_sweep_frequency(ST_START);
  uint32_t fstop  = get_sweep_frequency(ST_STOP);
  for (m = 0; m < MARKERS_MAX; m++) {
    if (!markers[m].enabled)
      continue;
    uint32_t f = markers[m].frequency;
    if (f == 0) idx = markers[m].index; // Not need update index in no freq
    else if (f < fstart) idx = 0;
    else if (f >= fstop) idx = sweep_points-1;
    else { // Search frequency index for marker frequency
#if 1
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
set_frequencies(uint32_t start, uint32_t stop, uint16_t points)
{
  uint32_t i;
  uint32_t step = (points - 1);
  uint32_t span = stop - start;
  uint32_t delta = span / step;
  uint32_t error = span % step;
  uint32_t f = start, df = step>>1;
  for (i = 0; i <= step; i++, f+=delta) {
    frequencies[i] = f;
    df+=error;
    if (df >=step) {
      f++;
      df -= step;
    }
  }
  // disable at out of sweep range
  for (; i < POINTS_COUNT; i++)
    frequencies[i] = 0;
}

static void
update_frequencies(bool interpolate)
{
  uint32_t start, stop;
  start = get_sweep_frequency(ST_START);
  stop  = get_sweep_frequency(ST_STOP);

  set_frequencies(start, stop, sweep_points);
  // operation_requested|= OP_FREQCHANGE;
  update_marker_index();
  // set grid layout
  update_grid();
  if (interpolate)
    cal_interpolate();
  RESET_SWEEP;
}

void
set_sweep_frequency(int type, uint32_t freq)
{
  int cal_applied = cal_status & CALSTAT_APPLY;

  // Check frequency for out of bounds (minimum SPAN can be any value)
  if (type != ST_SPAN && freq < START_MIN)
    freq = START_MIN;
  if (freq > STOP_MAX)
    freq = STOP_MAX;
  uint32_t center, span;
  switch (type) {
    case ST_START:
      config._mode &= ~VNA_MODE_CENTER_SPAN;
      frequency0 = freq;
      // if start > stop then make start = stop
      if (frequency1 < freq) frequency1 = freq;
      break;
    case ST_STOP:
      config._mode &= ~VNA_MODE_CENTER_SPAN;
      frequency1 = freq;
        // if start > stop then make start = stop
      if (frequency0 > freq) frequency0 = freq;
      break;
    case ST_CENTER:
      config._mode |= VNA_MODE_CENTER_SPAN;
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
      config._mode |= VNA_MODE_CENTER_SPAN;
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
      config._mode |= VNA_MODE_CENTER_SPAN;
      frequency0 = freq;
      frequency1 = freq;
      break;
  }
  update_frequencies(cal_applied);
}

uint32_t
get_sweep_frequency(int type)
{
  // Obsolete, ensure correct start/stop, start always must be < stop
  if (frequency0 > frequency1) {
    uint32_t t = frequency0;
    frequency0 = frequency1;
    frequency1 = t;
  }
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
  uint32_t value0 = 0;
  uint32_t value1 = 0;
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
    float c = 50e-15;
    //float c = 1.707e-12;
    float z0 = 50;
    float z = 2 * VNA_PI * frequencies[i] * c * z0;
    float sq = 1 + z*z;
    float s11aor = (1 - z*z) / sq;
    float s11aoi = 2*z / sq;

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
    sq = denomr*denomr+denomi*denomi;
    cal_data[ETERM_ES][i][0] = (numr*denomr + numi*denomi)/sq;
    cal_data[ETERM_ES][i][1] = (numi*denomr - numr*denomi)/sq;
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
    float invr = etr / sq;
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

static void apply_CH0_error_term_at(int i)
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
}

static void apply_CH1_error_term_at(int i)
{
    // CAUTION: Et is inversed for efficiency
    // S21a = (S21m - Ex) * Et
    float s21mr = measured[1][i][0] - cal_data[ETERM_EX][i][0];
    float s21mi = measured[1][i][1] - cal_data[ETERM_EX][i][1];
    // Not made CH1 correction by CH0 data
    float s21ar = s21mr * cal_data[ETERM_ET][i][0] - s21mi * cal_data[ETERM_ET][i][1];
    float s21ai = s21mi * cal_data[ETERM_ET][i][0] + s21mr * cal_data[ETERM_ET][i][1];
    measured[1][i][0] = s21ar;
    measured[1][i][1] = s21ai;
}

static void apply_edelay(void)
{
  int i;
  float real, imag;
  float s, c;
  uint16_t ch_mask = get_sweep_mask();
  for (i=0;i<sweep_points;i++){
    vna_sin_cos(electrical_delay * frequencies[i] * 1E-12, &s, &c);
    if (ch_mask & SWEEP_CH0_MEASURE){
      real = measured[0][i][0];
      imag = measured[0][i][1];
      measured[0][i][0] = real * c - imag * s;
      measured[0][i][1] = imag * c + real * s;
    }
    if (ch_mask & SWEEP_CH1_MEASURE){
      real = measured[1][i][0];
      imag = measured[1][i][1];
      measured[1][i][0] = real * c - imag * s;
      measured[1][i][1] = imag * c + real * s;
    }
  }
}

void
cal_collect(uint16_t type)
{
  uint16_t dst, src;
#if 1
  static const struct {
    uint16_t set_flag;
    uint16_t clr_flag;
    uint8_t dst;
    uint8_t src;
 } calibration_set[]={
//    type       set data flag       reset flag              destination source
    [CAL_LOAD] = {CALSTAT_LOAD,  ~(           CALSTAT_APPLY), CAL_LOAD,  0},
    [CAL_OPEN] = {CALSTAT_OPEN,  ~(CALSTAT_ES|CALSTAT_APPLY), CAL_OPEN,  0},
    [CAL_SHORT]= {CALSTAT_SHORT, ~(CALSTAT_ER|CALSTAT_APPLY), CAL_SHORT, 0},
    [CAL_THRU] = {CALSTAT_THRU,  ~(CALSTAT_ET|CALSTAT_APPLY), CAL_THRU,  1},
    [CAL_ISOLN]= {CALSTAT_ISOLN, ~(           CALSTAT_APPLY), CAL_ISOLN, 1},
  };
  if (type >= ARRAY_COUNT(calibration_set)) return;
  cal_status|=calibration_set[type].set_flag;
  cal_status&=calibration_set[type].clr_flag;
  dst = calibration_set[type].dst;
  src = calibration_set[type].src;
#else
  switch (type) {
//       type            set data flag            destination    source     reset flag
    case CAL_LOAD:  cal_status|= CALSTAT_LOAD;  dst = CAL_LOAD;  src = 0; break;
    case CAL_OPEN:  cal_status|= CALSTAT_OPEN;  dst = CAL_OPEN;  src = 0; cal_status&= ~(CALSTAT_ES); break;
    case CAL_SHORT: cal_status|= CALSTAT_SHORT; dst = CAL_SHORT; src = 0; cal_status&= ~(CALSTAT_ER); break;
    case CAL_THRU:  cal_status|= CALSTAT_THRU;  dst = CAL_THRU;  src = 1; cal_status&= ~(CALSTAT_ET); break;
    case CAL_ISOLN: cal_status|= CALSTAT_ISOLN; dst = CAL_ISOLN; src = 1; break;
    default:
      return;
  }
  // Disable calibration apply
  cal_status&= ~(CALSTAT_APPLY);
#endif
  // Run sweep for collect data (use minimum BANDWIDTH_30, or bigger if set)
  uint8_t bw = config.bandwidth;  // store current setting
  if (bw < BANDWIDTH_100)
    config.bandwidth = BANDWIDTH_100;

  // Set MAX settings for sweep_points on calibrate
//  if (sweep_points != POINTS_COUNT)
//    set_sweep_points(POINTS_COUNT);
  sweep(false, (src == 0) ? SWEEP_CH0_MEASURE : SWEEP_CH1_MEASURE);
  config.bandwidth = bw;          // restore

  // Copy calibration data
  memcpy(cal_data[dst], measured[src], sizeof measured[0]);
  redraw_request |= REDRAW_CAL_STATUS;
}

void
cal_done(void)
{
  if (!(cal_status & CALSTAT_LOAD))
    eterm_set(ETERM_ED, 0.0, 0.0);
  //adjust_ed();
  if ((cal_status & CALSTAT_SHORT) && (cal_status & CALSTAT_OPEN)) {
    eterm_calc_es();
    eterm_calc_er(-1);
  } else if (cal_status & CALSTAT_OPEN) {
    eterm_copy(CAL_SHORT, CAL_OPEN);
    eterm_set(ETERM_ES, 0.0, 0.0);
    eterm_calc_er(1);
  } else if (cal_status & CALSTAT_SHORT) {
    eterm_set(ETERM_ES, 0.0, 0.0);
    cal_status &= ~CALSTAT_SHORT;
    eterm_calc_er(-1);
  } else if (!(cal_status & CALSTAT_ER)){
    eterm_set(ETERM_ER, 1.0, 0.0);
  } else if (!(cal_status & CALSTAT_ES)) {
    eterm_set(ETERM_ES, 0.0, 0.0);
  }
    
  if (!(cal_status & CALSTAT_ISOLN))
    eterm_set(ETERM_EX, 0.0, 0.0);
  if (cal_status & CALSTAT_THRU) {
    eterm_calc_et();
  } else if (!(cal_status & CALSTAT_ET)) {
    eterm_set(ETERM_ET, 1.0, 0.0);
  }

  cal_status |= CALSTAT_APPLY;
  redraw_request |= REDRAW_CAL_STATUS;
}

static void
cal_interpolate(void)
{
  const properties_t *src = caldata_reference();
  uint32_t i, j;
  int eterm;
  if (src == NULL)
    return;

  // Upload not interpolated if some
  if (frequencies[0] == src->_frequency0 && frequencies[src->_sweep_points-1] == src->_frequency1){
    memcpy(current_props._cal_data, src->_cal_data, sizeof(src->_cal_data));
    cal_status = src->_cal_status;
    redraw_request |= REDRAW_CAL_STATUS;
    return;
  }
  uint32_t src_f = src->_frequency0;
  // lower than start freq of src range
  for (i = 0; i < sweep_points; i++) {
    if (frequencies[i] >= src_f)
      break;

    // fill cal_data at head of src range
    for (eterm = 0; eterm < 5; eterm++) {
      cal_data[eterm][i][0] = src->_cal_data[eterm][0][0];
      cal_data[eterm][i][1] = src->_cal_data[eterm][0][1];
    }
  }

  // ReBuild src freq list
  uint32_t src_points = (src->_sweep_points - 1);
  uint32_t span = src->_frequency1 - src->_frequency0;
  uint32_t delta = span / src_points;
  uint32_t error = span % src_points;
  uint32_t df = src_points>>1;
  j = 0;
  for (; i < sweep_points; i++) {
    uint32_t f = frequencies[i];
    if (f == 0) goto interpolate_finish;
    for (; j < src_points; j++) {
      if (src_f <= f && f < src_f + delta) {
        // found f between freqs at j and j+1
        float k1 = (delta == 0) ? 0.0 : (float)(f - src_f) / delta;
        // avoid glitch between freqs in different harmonics mode
        uint32_t idx = j;
        if (si5351_get_harmonic_lvl(src_f) != si5351_get_harmonic_lvl(src_f+delta)) {
          // f in prev harmonic, need extrapolate from prev 2 points
          if (si5351_get_harmonic_lvl(f) == si5351_get_harmonic_lvl(src_f)){
            if (idx >= 1){
              idx--; k1+=1.0;
            }
            else // point limit
              k1 = 0.0;
          }
          // f in next harmonic, need extrapolate from next 2 points
          else {
            if (idx < src_points){
              idx++; k1-=1.0;
            }
            else // point limit
              k1 = 1.0;
          }
        }
        float k0 = 1.0 - k1;
        for (eterm = 0; eterm < 5; eterm++) {
          cal_data[eterm][i][0] = src->_cal_data[eterm][idx][0] * k0 + src->_cal_data[eterm][idx+1][0] * k1;
          cal_data[eterm][i][1] = src->_cal_data[eterm][idx][1] * k0 + src->_cal_data[eterm][idx+1][1] * k1;
        }
        break;
      }
      df+=error;if (df >=src_points) {src_f++;df -= src_points;}
      src_f+=delta;
    }
    if (j == src_points)
      break;
  }

  // upper than end freq of src range
  for (; i < sweep_points; i++) {
    // fill cal_data at tail of src
    for (eterm = 0; eterm < 5; eterm++) {
      cal_data[eterm][i][0] = src->_cal_data[eterm][src_points][0];
      cal_data[eterm][i][1] = src->_cal_data[eterm][src_points][1];
    }
  }
interpolate_finish:
  cal_status = src->_cal_status | CALSTAT_INTERPOLATED;
  redraw_request |= REDRAW_CAL_STATUS;
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
  redraw_request|=REDRAW_CAL_STATUS;
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
  redraw_request |= REDRAW_CAL_STATUS;
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
  redraw_request |= REDRAW_CAL_STATUS;
  return;
 usage:
  shell_printf("recall {id}\r\n");
}

static const struct {
  const char *name;
  uint16_t refpos;
  float scale_unit;
} trace_info[MAX_TRACE_TYPE-1] = {
  { "LOGMAG", NGRIDY-1,  10.0 },
  { "PHASE",  NGRIDY/2,  90.0 },
  { "DELAY",  NGRIDY/2,  1e-9 },
  { "SMITH",         0,  1.00 },
  { "POLAR",         0,  1.00 },
  { "LINEAR",        0,  0.125},
  { "SWR",           0,  0.25 },
  { "REAL",   NGRIDY/2,  0.25 },
  { "IMAG",   NGRIDY/2,  0.25 },
  { "R",      NGRIDY/2, 100.0 },
  { "X",      NGRIDY/2, 100.0 },
  { "Q",             0,  10.0 }
};

static const char * const trc_channel_name[] = {
  "CH0", "CH1"
};

const char *get_trace_typename(int t)
{
  return trace_info[trace[t].type].name;
}

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
    trace[t].refpos = trace_info[type].refpos;
    // Set default trace scale
    trace[t].scale  = trace_info[type].scale_unit;
    force = TRUE;
  }
  if (force) {
    plot_into_index(measured);
    request_to_redraw_grid();
  }
}

void set_trace_channel(int t, int channel)
{
  if (trace[t].channel != channel) {
    trace[t].channel = channel;
    plot_into_index(measured);
    request_to_redraw_grid();
  }
}

void set_trace_scale(int t, float scale)
{
  if (trace[t].scale != scale) {
    trace[t].scale = scale;
    request_to_redraw_grid();
  }
}

void set_trace_refpos(int t, float refpos)
{
  if (trace[t].refpos != refpos) {
    trace[t].refpos = refpos;
    request_to_redraw_grid();
  }
}

void set_electrical_delay(float picoseconds)
{
  if (electrical_delay != picoseconds) {
    electrical_delay = picoseconds;
    request_to_redraw_grid();
  }
}

VNA_SHELL_FUNCTION(cmd_trace)
{
  int t;
  if (argc == 0) {
    for (t = 0; t < TRACES_MAX; t++) {
      if (trace[t].enabled) {
        const char *type = get_trace_typename(t);
        const char *channel = get_trace_chname(t);
        float scale = get_trace_scale(t);
        float refpos = get_trace_refpos(t);
        shell_printf("%d %s %s %f %f\r\n", t, type, channel, scale, refpos);
      }
    }
    return;
  }

  if (strcmp(argv[0], "all") == 0 &&
      argc > 1 && strcmp(argv[1], "off") == 0) {
    for (t = 0; t < TRACES_MAX; t++)
      set_trace_type(t, TRC_OFF);
    return;
  }

  t = my_atoi(argv[0]);
  if (t < 0 || t >= TRACES_MAX)
    goto usage;
  if (argc == 1) {
    const char *type = get_trace_typename(t);
    const char *channel = get_trace_chname(t);
    shell_printf("%d %s %s\r\n", t, type, channel);
    return;
  }
#if MAX_TRACE_TYPE != 13
#error "Trace type enum possibly changed, check cmd_trace function"
#endif
  // enum TRC_LOGMAG, TRC_PHASE, TRC_DELAY, TRC_SMITH, TRC_POLAR, TRC_LINEAR, TRC_SWR, TRC_REAL, TRC_IMAG, TRC_R, TRC_X, TRC_Q, TRC_OFF
  static const char cmd_type_list[] = "logmag|phase|delay|smith|polar|linear|swr|real|imag|r|x|q|off";
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
  static const char cmd_marker_smith[] = "lin|log|ri|rx|rlc";
  int t;
  if (argc == 0) {
    for (t = 0; t < MARKERS_MAX; t++) {
      if (markers[t].enabled) {
        shell_printf("%d %d %u\r\n", t+1, markers[t].index, markers[t].frequency);
      }
    }
    return;
  }
  redraw_request |= REDRAW_MARKER;
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

VNA_SHELL_FUNCTION(cmd_touchcal)
{
  (void)argc;
  (void)argv;
  //extern int16_t touch_cal[4];
  int i;

  shell_printf("first touch upper left, then lower right...");
  touch_cal_exec();
  shell_printf("done\r\n");

  shell_printf("touch cal params: ");
  for (i = 0; i < 4; i++) {
    shell_printf("%d ", config.touch_cal[i]);
  }
  shell_printf("\r\n");
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
    if (frequencies[i] == 0) break;
    shell_printf("%u\r\n", frequencies[i]);
  }
}

#ifdef ENABLE_TRANSFORM_COMMAND
static void
set_domain_mode(int mode) // accept DOMAIN_FREQ or DOMAIN_TIME
{
  if (mode != (domain_mode & DOMAIN_MODE)) {
    domain_mode = (domain_mode & ~DOMAIN_MODE) | (mode & DOMAIN_MODE);
    redraw_request |= REDRAW_FREQUENCY | REDRAW_MARKER;
    uistat.lever_mode = LM_MARKER;
  }
}

static void
set_timedomain_func(int func) // accept TD_FUNC_LOWPASS_IMPULSE, TD_FUNC_LOWPASS_STEP or TD_FUNC_BANDPASS
{
  domain_mode = (domain_mode & ~TD_FUNC) | (func & TD_FUNC);
}

static void
set_timedomain_window(int func) // accept TD_WINDOW_MINIMUM/TD_WINDOW_NORMAL/TD_WINDOW_MAXIMUM
{
  domain_mode = (domain_mode & ~TD_WINDOW) | (func & TD_WINDOW);
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
    stat.rms[0] = sqrtf(acc0 / count);
    stat.rms[1] = sqrtf(acc1 / count);
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
    shell_printf("%d\r\n", config.vbat_offset);
    return;
  }
  config.vbat_offset = (int16_t)my_atoi(argv[0]);
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

#ifdef ENABLE_I2C_TIMINGS
VNA_SHELL_FUNCTION(cmd_i2ctime)
{
  (void)argc;
  uint32_t tim =  STM32_TIMINGR_PRESC(0U)  |
                  STM32_TIMINGR_SCLDEL(my_atoui(argv[0])) | STM32_TIMINGR_SDADEL(my_atoui(argv[1])) |
                  STM32_TIMINGR_SCLH(my_atoui(argv[2])) | STM32_TIMINGR_SCLL(my_atoui(argv[3]));
  I2CD1.i2c->CR1 &=~I2C_CR1_PE;
  I2CD1.i2c->TIMINGR = tim;
  I2CD1.i2c->CR1 |= I2C_CR1_PE;

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
  int i;
  if (argc != 2) {
    shell_printf("usage: color {id} {rgb24}\r\n");
    for (i=0; i < MAX_PALETTE; i++) {
      color = GET_PALTETTE_COLOR(i);
      color = HEXRGB(color);
      shell_printf(" %2d: 0x%06x\r\n", i, color);
    }
    return;
  }
  i = my_atoi(argv[0]);
  if (i >= MAX_PALETTE)
    return;
  color = RGBHEX(my_atoui(argv[1]));
  config.lcd_palette[i] = color;
  // Redraw all
  redraw_request|= REDRAW_AREA|REDRAW_CAL_STATUS|REDRAW_FREQUENCY;
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
  if (argc == 0 || argc > 2 || (config._mode & VNA_MODE_SERIAL)) return;
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

#ifdef ENABLE_SD_CARD_CMD
#ifndef __USE_SD_CARD__
#error "Need enable SD card support __USE_SD_CARD__ in nanovna.h, for use ENABLE_SD_CARD_CMD"
#endif
// Fat file system work area (at the end of spi_buffer)
static FATFS *fs_volume   = (FATFS *)(((uint8_t*)(&spi_buffer[SPI_BUFFER_SIZE])) - sizeof(FATFS));
// FatFS file object (at the end of spi_buffer)
static FIL   *fs_file     = (   FIL*)(((uint8_t*)(&spi_buffer[SPI_BUFFER_SIZE])) - sizeof(FATFS) - sizeof(FIL));

static FRESULT cmd_sd_card_mount(void){
  const FRESULT res = f_mount(fs_volume, "", 1);
  if (res != FR_OK)
    shell_printf("error: card not mounted\r\n");
  return res;
}

VNA_SHELL_FUNCTION(cmd_sd_list)
{
  (void)argc;
  (void)argv;

  DIR dj;
  FILINFO fno;
  FRESULT res;
  shell_printf("sd_list:\r\n");
  res = cmd_sd_card_mount();
  if (res != FR_OK)
    return;
  res = f_findfirst(&dj, &fno, "", "*.*");
  while (res == FR_OK && fno.fname[0])
  {
    shell_printf("%s %u\r\n", fno.fname, fno.fsize);
    res = f_findnext(&dj, &fno);
  }
  f_closedir(&dj);
}

VNA_SHELL_FUNCTION(cmd_sd_readfile)
{
  FRESULT res;
  char *buf = (char *)spi_buffer;
  if (argc < 1)
  {
     shell_printf("usage: sd_readfile {filename}\r\n");
     return;
  }
  const char *filename = argv[0];
  shell_printf("sd_readfile: %s\r\n", filename);
  res = cmd_sd_card_mount();
  if (res != FR_OK)
    return;

  res = f_open(fs_file, filename, FA_OPEN_EXISTING | FA_READ);
  if (res != FR_OK)
  {
    shell_printf("error: %s not opened\r\n", filename);
    return;
  }

  // number of bytes to follow (file size)
  const uint32_t filesize = f_size(fs_file);
  streamWrite(shell_stream, (void *)&filesize, 4);

  // file data (send all data from file)
  while (1)
  {
    UINT size = 0;
    res = f_read(fs_file, buf, 512, &size);
    if (res != FR_OK || size == 0)
      break;
    streamWrite(shell_stream, (void *)buf, size);
  }
  res = f_close(fs_file);
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
#define CMD_BREAK_SWEEP 2

static const VNAShellCommand commands[] =
{
    {"scan"        , cmd_scan        , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
#ifdef ENABLE_SCANBIN_COMMAND
    {"scan_bin"    , cmd_scan_bin    , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
#endif
    {"data"        , cmd_data        , 0},
    {"frequencies" , cmd_frequencies , 0},
    {"freq"        , cmd_freq        , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
    {"sweep"       , cmd_sweep       , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
    {"power"       , cmd_power       , 0},
#ifdef USE_VARIABLE_OFFSET
    {"offset"      , cmd_offset      , CMD_WAIT_MUTEX},
#endif
    {"bandwidth"   , cmd_bandwidth   , 0},
#ifdef __USE_RTC__
    {"time"        , cmd_time        , 0},
#endif
#ifdef ENABLE_SD_CARD_CMD
    {"sd_list"       , cmd_sd_list     , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
    {"sd_readfile"   , cmd_sd_readfile , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
#endif
#ifdef __VNA_ENABLE_DAC__
    {"dac"         , cmd_dac         , 0},
#endif
    {"saveconfig"  , cmd_saveconfig  , 0},
    {"clearconfig" , cmd_clearconfig , 0},
#ifdef ENABLED_DUMP_COMMAND
    {"dump"        , cmd_dump        , 0},
#endif
#ifdef ENABLE_PORT_COMMAND
    {"port"        , cmd_port        , 0},
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
    {"pause"       , cmd_pause       , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
    {"resume"      , cmd_resume      , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
    {"cal"         , cmd_cal         , CMD_WAIT_MUTEX},
    {"save"        , cmd_save        , 0},
    {"recall"      , cmd_recall      , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
    {"trace"       , cmd_trace       , 0},
    {"marker"      , cmd_marker      , 0},
    {"edelay"      , cmd_edelay      , 0},
    {"capture"     , cmd_capture     , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
    {"vbat"        , cmd_vbat        , 0},
    {"reset"       , cmd_reset       , 0},
#ifdef __USE_SERIAL_CONSOLE__
#ifdef ENABLE_USART_COMMAND
    {"usart_cfg"   , cmd_usart_cfg   , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
    {"usart"       , cmd_usart       , CMD_WAIT_MUTEX|CMD_BREAK_SWEEP},
#endif
#endif
#ifdef ENABLE_VBAT_OFFSET_COMMAND
    {"vbat_offset" , cmd_vbat_offset , 0},
#endif
#ifdef ENABLE_TRANSFORM_COMMAND
    {"transform"   , cmd_transform   , 0},
#endif
    {"threshold"   , cmd_threshold   , 0},
    {"help"        , cmd_help        , 0},
#ifdef ENABLE_INFO_COMMAND
    {"info"        , cmd_info        , 0},
#endif
    {"version"     , cmd_version     , 0},
#ifdef ENABLE_COLOR_COMMAND
    {"color"       , cmd_color       , 0},
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
#define PREPARE_STREAM shell_stream = (config._mode&VNA_MODE_SERIAL) ? (BaseSequentialStream *)&SD1 : (BaseSequentialStream *)&SDU1;

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
    if (config._mode & VNA_MODE_SERIAL)
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
  if (config._mode & VNA_MODE_SERIAL)
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
#define PREPARE_STREAM

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
  shell_stream = (BaseSequentialStream *)&SDU1;
}
#endif

//
// Read command line from shell_stream
//
static int VNAShell_readLine(char *line, int max_size)
{
  // Read line from input stream
  uint8_t c;
  // Prepare I/O for shell_stream
  PREPARE_STREAM;
  char *ptr = line;
  while (1) {
    // Return 0 only if stream not active
    if (streamRead(shell_stream, &c, 1) == 0)
      return 0;
    // Backspace or Delete
    if (c == 8 || c == 0x7f) {
      if (ptr != line) {
        static const char backspace[] = {0x08, 0x20, 0x08, 0x00};
        shell_printf(backspace);
        ptr--;
      }
      continue;
    }
    // New line (Enter)
    if (c == '\r') {
      shell_printf(VNA_SHELL_NEWLINE_STR);
      *ptr = 0;
      return 1;
    }
    // Others (skip)
    if (c < 0x20)
      continue;
    // Store
    if (ptr < line + max_size - 1) {
      streamPut(shell_stream, c); // Echo
      *ptr++ = (char)c;
    }
  }
  return 0;
}

//
// Parse and run command line
//
static void VNAShell_executeLine(char *line)
{
  // Parse and execute line
  char *lp = line, *ep;
  shell_nargs = 0;

//  DEBUG_LOG(0, lp); // debug console log
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
      shell_printf("too many arguments, max " define_to_STR(
          VNA_SHELL_MAX_ARGUMENTS) "" VNA_SHELL_NEWLINE_STR);
      return;
    }
    // Set zero at the end of string and continue check
    *lp++ = 0;
  }
  if (shell_nargs == 0) return;
  // Execute line
  const VNAShellCommand *scp;
  for (scp = commands; scp->sc_name != NULL; scp++) {
    if (strcmp(scp->sc_name, shell_args[0]) == 0) {
      if (scp->flags & CMD_WAIT_MUTEX) {
        shell_function = scp->sc_function;
        if (scp->flags & CMD_BREAK_SWEEP) operation_requested|=OP_CONSOLE;
        // Wait execute command in sweep thread
        do {
          chThdSleepMilliseconds(100);
        } while (shell_function);
      } else {
        scp->sc_function(shell_nargs - 1, &shell_args[1]);
      }
//      DEBUG_LOG(10, "ok");
      return;
    }
  }
  shell_printf("%s?" VNA_SHELL_NEWLINE_STR, shell_args[0]);
}

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
// Define i2c bus speed, add predefined for 400k, 600k, 900k
#define STM32_I2C_SPEED                     600

#if STM32_I2C1_CLOCK == 8    // STM32_I2C1SW == STM32_I2C1SW_HSI     (HSI=8MHz)
#if   STM32_I2C_SPEED == 400 // 400kHz @ HSI 8MHz (Use 26.4.10 I2C_TIMINGR register configuration examples from STM32 RM0091 Reference manual)
 #define STM32_I2C_TIMINGR  STM32_TIMINGR_PRESC(0U)  |\
                            STM32_TIMINGR_SCLDEL(3U) | STM32_TIMINGR_SDADEL(1U) |\
                            STM32_TIMINGR_SCLH(3U)   | STM32_TIMINGR_SCLL(9U)
#endif
#elif  STM32_I2C1_CLOCK == 48 // STM32_I2C1SW == STM32_I2C1SW_SYSCLK  (SYSCLK = 48MHz)
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
  .timingr = STM32_I2C_TIMINGR,  // TIMINGR register initialization. (use I2C timing configuration tool for STM32F3xx and STM32F0xx microcontrollers (AN4235))
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

#ifdef USE_VARIABLE_OFFSET
  generate_DSP_Table(FREQUENCY_OFFSET);
#endif

/*
 * SPI bus and LCD Initialize
 */
  ili9341_init();

  show_version(0);

/*
 * Restore config
 */
  config_recall();

/*
 * restore frequencies and calibration 0 slot properties from flash memory
 */
  load_properties(0);

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
 * UI (menu, touch, buttons) and plot initialize
 */
  ui_init();
  //Initialize graph plotting
  plot_init();
  redraw_frame();

/*
 * Starting DAC1 driver, setting up the output pin as analog as suggested by the Reference Manual.
 */
#ifdef  __VNA_ENABLE_DAC__
  dacStart(&DACD2, &dac1cfg1);
  dacPutChannelX(&DACD2, 0, config.dac_value);  // Set config DAC value
#endif

/*
 * Set LCD display brightness
 */
#ifdef  __LCD_BRIGHTNESS__
  lcd_setBrightness(config._brightness);
#endif

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
  char buf[16];
  ili9341_set_background(LCD_BG_COLOR);
  ili9341_set_foreground(LCD_FG_COLOR);
  plot_printf(buf, sizeof(buf), "SP  0x%08x",  (uint32_t)sp);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R0  0x%08x",  r0);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R1  0x%08x",  r1);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R2  0x%08x",  r2);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R3  0x%08x",  r3);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R4  0x%08x",  r4);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R5  0x%08x",  r5);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R6  0x%08x",  r6);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R7  0x%08x",  r7);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R8  0x%08x",  r8);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R9  0x%08x",  r9);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R10 0x%08x", r10);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R11 0x%08x", r11);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "R12 0x%08x", r12);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "LR  0x%08x",  lr);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "PC  0x%08x",  pc);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);
  plot_printf(buf, sizeof(buf), "PSR 0x%08x", psr);ili9341_drawstring(buf, x, y+=FONT_STR_HEIGHT);

  shell_printf("===================================\r\n");
#else
  (void)sp;
#endif
  while (true) {
  }
}
// For new compilers
//void _exit(int){}
//void _kill(void){}
//void _getpid(void){}
