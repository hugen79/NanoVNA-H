/*
 * Copyright (c) 2019-2023, Dmitry (DiSlord) dislordlive@gmail.com
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
#include "chprintf.h"
#include <string.h>
#include "nanovna.h"
#include "si5351.h"

// Use size optimization (UI not need fast speed, better have smallest size)
#pragma GCC optimize ("Os")

// Hardware UI buttons/touch data / functions
#define NO_EVENT                    0
#define EVT_BUTTON_SINGLE_CLICK     0x01
#define EVT_BUTTON_DOUBLE_CLICK     0x02
#define EVT_BUTTON_DOWN_LONG        0x04
#define EVT_UP                      0x10
#define EVT_DOWN                    0x20
#define EVT_REPEAT                  0x40

#define BUTTON_DOWN_LONG_TICKS      MS2ST(500)   // 500ms
#define BUTTON_DOUBLE_TICKS         MS2ST(250)   // 250ms
#define BUTTON_REPEAT_TICKS         MS2ST( 30)   //  30ms
#define BUTTON_DEBOUNCE_TICKS       MS2ST( 20)   //  20ms

/* lever switch assignment */
#define BUTTON_DOWN                 (1<<GPIOA_LEVER1)
#define BUTTON_PUSH                 (1<<GPIOA_PUSH)
#define BUTTON_UP                   (1<<GPIOA_LEVER2)
#define READ_BUTTONS()              (palReadPort(GPIOA) & (BUTTON_DOWN | BUTTON_PUSH | BUTTON_UP))

static uint16_t  last_button = 0b0000;
static systime_t last_button_down_ticks;
static systime_t last_button_repeat_ticks;

// Touch screen
#define EVT_TOUCH_NONE              0
#define EVT_TOUCH_DOWN              1
#define EVT_TOUCH_PRESSED           2
#define EVT_TOUCH_RELEASED          3

#define TOUCH_INTERRUPT_ENABLED     1
static uint8_t touch_status_flag = 0;
static uint8_t last_touch_status = EVT_TOUCH_NONE;
static int16_t last_touch_x;
static int16_t last_touch_y;
uint8_t operation_requested = OP_NONE;

//==============================================
static uint16_t menu_button_height = MENU_BUTTON_HEIGHT(MENU_BUTTON_MIN);

enum {
  UI_NORMAL, UI_MENU, UI_KEYPAD,
#ifdef __SD_FILE_BROWSER__
  UI_BROWSER,
#endif
};

typedef struct {
  uint8_t bg;
  uint8_t fg;
  uint8_t border;
  int8_t  icon;
  union {
    int32_t  i;
    uint32_t u;
    float    f;
    const char *text;
  } p1;        // void data for label printf
  char label[32];
} button_t;

// Keypad structures
// Enum for keypads_list
enum {
  KM_START = 0, KM_STOP, KM_CENTER, KM_SPAN, KM_CW, KM_STEP, KM_VAR, // frequency input
  KM_POINTS,
  KM_TOP, KM_nTOP,
  KM_BOTTOM, KM_nBOTTOM,
  KM_SCALE, KM_nSCALE,
  KM_REFPOS, KM_EDELAY, KM_VAR_DELAY, KM_S21OFFSET, KM_VELOCITY_FACTOR,
#ifdef __S11_CABLE_MEASURE__
  KM_ACTUAL_CABLE_LEN,
#endif
  KM_XTAL, KM_THRESHOLD, KM_VBAT,
#ifdef __S21_MEASURE__
  KM_MEASURE_R,
#endif
#ifdef __VNA_Z_RENORMALIZATION__
  KM_Z_PORT,
#endif
#ifdef __USE_RTC__
  KM_RTC_DATE,
  KM_RTC_TIME,
#endif
#ifdef __USE_SD_CARD__
  KM_S1P_NAME,
  KM_S2P_NAME,
  KM_BMP_NAME,
#ifdef __SD_CARD_DUMP_TIFF__
  KM_TIF_NAME,
#endif
  KM_CAL_NAME,
#ifdef __SD_CARD_DUMP_FIRMWARE__
  KM_BIN_NAME,
#endif
#endif
  KM_NONE
};

typedef struct {
  uint16_t x_offs;
  uint16_t y_offs;
  uint16_t width;
  uint16_t height;
} keypad_pos_t;

typedef struct {
  uint8_t pos;
  int8_t  c;
} keypads_t;

typedef void (*keyboard_cb_t)(uint16_t data, button_t *b);
#define UI_KEYBOARD_CALLBACK(ui_kb_function_name) void ui_kb_function_name(uint16_t data, button_t *b)

#pragma pack(push, 2)
typedef struct {
  uint8_t keypad_type;
  uint8_t data;
  const char *name;
  const keyboard_cb_t cb;
} keypads_list;
#pragma pack(pop)

// Max keyboard input length
#define NUMINPUT_LEN 12
#if defined(FF_USE_LFN)
#define TXTINPUT_LEN (FF_MAX_LFN - 4)
#else
#define TXTINPUT_LEN (8)
#endif

#if NUMINPUT_LEN + 2 > TXTINPUT_LEN + 1
static char    kp_buf[NUMINPUT_LEN+2];  // !!!!!! WARNING size must be + 2 from NUMINPUT_LEN or TXTINPUT_LEN + 1
#else
static char    kp_buf[TXTINPUT_LEN+1];  // !!!!!! WARNING size must be + 2 from NUMINPUT_LEN or TXTINPUT_LEN + 1
#endif
static uint8_t ui_mode = UI_NORMAL;
static const keypads_t *keypads;
static uint8_t keypad_mode;
//static uint8_t keyboard_temp;           // Use for custom keyboard processing
static uint8_t menu_current_level = 0;
static int8_t  selection = -1;

// UI menu structure
// Type of menu item:
enum {
  MT_NEXT = 0,       // reference is next menu or 0 if end
  MT_SUBMENU,        // reference is submenu button
  MT_CALLBACK,       // reference is pointer to: void ui_function_name(uint16_t data)
  MT_ADV_CALLBACK    // reference is pointer to: void ui_function_name(uint16_t data, button_t *b)
};

// Button definition (used in MT_ADV_CALLBACK for custom)
#define BUTTON_ICON_NONE            -1
#define BUTTON_ICON_NOCHECK          0
#define BUTTON_ICON_CHECK            1
#define BUTTON_ICON_GROUP            2
#define BUTTON_ICON_GROUP_CHECKED    3
#define BUTTON_ICON_CHECK_AUTO       4
#define BUTTON_ICON_CHECK_MANUAL     5

#define BUTTON_BORDER_WIDTH_MASK     0x07

// Define mask for draw border (if 1 use light color, if 0 dark)
#define BUTTON_BORDER_NO_FILL        0x08
#define BUTTON_BORDER_TOP            0x10
#define BUTTON_BORDER_BOTTOM         0x20
#define BUTTON_BORDER_LEFT           0x40
#define BUTTON_BORDER_RIGHT          0x80

#define BUTTON_BORDER_FLAT           0x00
#define BUTTON_BORDER_RISE           (BUTTON_BORDER_TOP|BUTTON_BORDER_RIGHT)
#define BUTTON_BORDER_FALLING        (BUTTON_BORDER_BOTTOM|BUTTON_BORDER_LEFT)

// Call back functions for MT_CALLBACK type
typedef void (*menuaction_cb_t)(uint16_t data);
#define UI_FUNCTION_CALLBACK(ui_function_name) void ui_function_name(uint16_t data)

typedef void (*menuaction_acb_t)(uint16_t data, button_t *b);
#define UI_FUNCTION_ADV_CALLBACK(ui_function_name) void ui_function_name(uint16_t data, button_t *b)

// Set structure align as WORD (save flash memory)
#pragma pack(push, 2)
typedef struct {
  uint8_t type;
  uint8_t data;
  char *label;
  const void *reference;
} menuitem_t;
#pragma pack(pop)

#define KP_CONTINUE        0
#define KP_DONE            1
#define KP_CANCEL          2

static void ui_mode_normal(void);
static void ui_mode_menu(void);
static void draw_menu(uint32_t mask);
static void draw_button(uint16_t x, uint16_t y, uint16_t w, uint16_t h, button_t *b);
static void ui_mode_keypad(int _keypad_mode);
static void touch_position(int *x, int *y);
static void menu_move_back(bool leave_ui);
static void menu_push_submenu(const menuitem_t *submenu);
static void menu_set_submenu(const menuitem_t *submenu);
// Icons for UI
#include "icons_menu.c"

static uint16_t get_buttons(void) {
  uint16_t cur_button = READ_BUTTONS();
#ifdef __FLIP_DISPLAY__
  // swap bits in byte (swap leveler left and right bits for rotated display)
  if (VNA_MODE(VNA_MODE_FLIP_DISPLAY) && (((cur_button>>GPIOA_LEVER1)^(cur_button>>GPIOA_LEVER2))&1)) cur_button^= (1<<GPIOA_LEVER1)|(1<<GPIOA_LEVER2);
#endif
  return cur_button;
}

static uint16_t btn_check(void)
{
  systime_t ticks;
  // Debounce input
  while(TRUE){
    ticks = chVTGetSystemTimeX();
    if(ticks - last_button_down_ticks > BUTTON_DEBOUNCE_TICKS)
      break;
    chThdSleepMilliseconds(2);
  }
  uint16_t status = 0;
  uint16_t cur_button = get_buttons();
  // Detect only changed and pressed buttons
  uint16_t button_set = (last_button ^ cur_button) & cur_button;
  last_button_down_ticks = ticks;
  last_button = cur_button;

  if (button_set & BUTTON_PUSH)
    status |= EVT_BUTTON_SINGLE_CLICK;
  if (button_set & BUTTON_UP)
    status |= EVT_UP;
  if (button_set & BUTTON_DOWN)
    status |= EVT_DOWN;
  return status;
}

static uint16_t btn_wait_release(void)
{
  while (TRUE) {
    systime_t ticks = chVTGetSystemTimeX();
    systime_t dt = ticks - last_button_down_ticks;
    // Debounce input
//    if (dt < BUTTON_DEBOUNCE_TICKS){
//      chThdSleepMilliseconds(10);
//      continue;
//    }
    chThdSleepMilliseconds(10);
    uint16_t cur_button = get_buttons();
    uint16_t changed = last_button ^ cur_button;
    if (dt >= BUTTON_DOWN_LONG_TICKS && (cur_button & BUTTON_PUSH))
      return EVT_BUTTON_DOWN_LONG;
    if (changed & BUTTON_PUSH) // release
      return EVT_BUTTON_SINGLE_CLICK;

    if (changed) {
      // finished
      last_button = cur_button;
      last_button_down_ticks = ticks;
      return 0;
    }

    if (dt > BUTTON_DOWN_LONG_TICKS &&
        ticks > last_button_repeat_ticks) {
      uint16_t status = 0;
      if (cur_button & BUTTON_DOWN)
        status |= EVT_DOWN | EVT_REPEAT;
      if (cur_button & BUTTON_UP)
        status |= EVT_UP | EVT_REPEAT;
      last_button_repeat_ticks = ticks + BUTTON_REPEAT_TICKS;
      return status;
    }
  }
}

#if 0
static void btn_wait(void) {
  while (READ_PORT()) chThdSleepMilliseconds(10);
}
#endif

#if 0
static void bubbleSort(uint16_t *v, int n) {
  bool swapped = true;
  int i = 0, j;
  while (i < n - 1 && swapped) { // keep going while we swap in the unordered part
    swapped = false;
    for (j = n - 1; j > i; j--) { // unordered part
      if (v[j] < v[j - 1]) {
        SWAP(uint16_t, v[j], v[j - 1]);
        swapped = true;
      }
    }
    i++;
  }
}
#endif

#define SOFTWARE_TOUCH
//*******************************************************************************
// Software Touch module
//*******************************************************************************
#ifdef SOFTWARE_TOUCH
static int
touch_measure_y(void)
{
  // drive low to high on X line (At this state after touch_prepare_sense)
//  palSetPadMode(GPIOB, GPIOB_XN, PAL_MODE_OUTPUT_PUSHPULL); //
//  palSetPadMode(GPIOA, GPIOA_XP, PAL_MODE_OUTPUT_PUSHPULL); //
  // drive low to high on X line (coordinates from top to bottom)
  palClearPad(GPIOB, GPIOB_XN);
//  palSetPad(GPIOA, GPIOA_XP);

  // open Y line (At this state after touch_prepare_sense)
//  palSetPadMode(GPIOB, GPIOB_YN, PAL_MODE_INPUT);        // Hi-z mode
  palSetPadMode(GPIOA, GPIOA_YP, PAL_MODE_INPUT_ANALOG);   // <- ADC_TOUCH_Y channel

//  chThdSleepMilliseconds(3);
  return adc_single_read(ADC_TOUCH_Y);
}

static int
touch_measure_x(void)
{
  // drive high to low on Y line (coordinates from left to right)
  palSetPad(GPIOB, GPIOB_YN);
  palClearPad(GPIOA, GPIOA_YP);
  // Set Y line as output
  palSetPadMode(GPIOB, GPIOB_YN, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, GPIOA_YP, PAL_MODE_OUTPUT_PUSHPULL);
  // Set X line as input
  palSetPadMode(GPIOB, GPIOB_XN, PAL_MODE_INPUT);        // Hi-z mode
  palSetPadMode(GPIOA, GPIOA_XP, PAL_MODE_INPUT_ANALOG); // <- ADC_TOUCH_X channel
//  chThdSleepMilliseconds(3);
  return adc_single_read(ADC_TOUCH_X);
}
// Manually measure touch event
static inline int
touch_status(void)
{
  return adc_single_read(ADC_TOUCH_Y) > TOUCH_THRESHOLD;
}

static void
touch_prepare_sense(void)
{
  // Set Y line as input
  palSetPadMode(GPIOB, GPIOB_YN, PAL_MODE_INPUT);          // Hi-z mode
  palSetPadMode(GPIOA, GPIOA_YP, PAL_MODE_INPUT_PULLDOWN); // Use pull
  // drive high on X line (for touch sense on Y)
  palSetPad(GPIOB, GPIOB_XN);
  palSetPad(GPIOA, GPIOA_XP);
  // force high X line
  palSetPadMode(GPIOB, GPIOB_XN, PAL_MODE_OUTPUT_PUSHPULL);
  palSetPadMode(GPIOA, GPIOA_XP, PAL_MODE_OUTPUT_PUSHPULL);
//  chThdSleepMilliseconds(10); // Wait 10ms for denounce touch
}

#ifdef __REMOTE_DESKTOP__
static uint8_t touch_remote = REMOTE_NONE;
void remote_touch_set(uint16_t state, int16_t x, int16_t y) {
  touch_remote = state;
  if (x!=-1) last_touch_x = x;
  if (y!=-1) last_touch_y = y;
  handle_touch_interrupt();
}
#endif

static void
touch_start_watchdog(void)
{
  if (touch_status_flag&TOUCH_INTERRUPT_ENABLED) return;
  touch_status_flag^=TOUCH_INTERRUPT_ENABLED;
  adc_start_analog_watchdog();
#ifdef __REMOTE_DESKTOP__
  touch_remote = REMOTE_NONE;
#endif
}

static void
touch_stop_watchdog(void)
{
  if (!(touch_status_flag&TOUCH_INTERRUPT_ENABLED)) return;
  touch_status_flag^=TOUCH_INTERRUPT_ENABLED;
  adc_stop_analog_watchdog();
}

// Touch panel timer check (check press frequency 20Hz)
#if HAL_USE_GPT == TRUE
static const GPTConfig gpt3cfg = {
  1000,   // 1kHz timer clock.
  NULL,   // Timer callback.
  0x0020, // CR2:MMS=02 to output TRGO
  0
};
static void init_Timers(void) {
  gptStart(&GPTD3, &gpt3cfg);         // Init timer 3
  gptStartContinuous(&GPTD3, 10);     // Start timer 10ms period (use 1kHz clock)
}
#else
static void init_Timers(void) {
  initTimers();
  startTimer(TIM3, 10);               // Start timer 10ms period (use 1kHz clock)
}
#endif

//
// Touch init function init timer 3 trigger adc for check touch interrupt, and run measure
//
static void touch_init(void){
  // Prepare pin for measure touch event
  touch_prepare_sense();
  // Start touch interrupt, used timer_3 ADC check threshold:
  init_Timers();
  touch_start_watchdog();             // Start ADC watchdog (measure by timer 3 interval and trigger interrupt if touch pressed)
}

// Main software touch function, should:
// set last_touch_x and last_touch_x
// return touch status
static int
touch_check(void)
{
  touch_stop_watchdog();

  int stat = touch_status();
  if (stat) {
    int y = touch_measure_y();
    int x = touch_measure_x();
    touch_prepare_sense();
    if (touch_status())
    {
      last_touch_x = x;
      last_touch_y = y;
    }
#ifdef __REMOTE_DESKTOP__
    touch_remote = REMOTE_NONE;
  } else {
    stat = touch_remote == REMOTE_PRESS;
#endif
  }

  if (stat != last_touch_status) {
    last_touch_status = stat;
    return stat ? EVT_TOUCH_PRESSED : EVT_TOUCH_RELEASED;
  }
  return stat ? EVT_TOUCH_DOWN : EVT_TOUCH_NONE;
}
//*******************************************************************************
// End Software Touch module
//*******************************************************************************
#endif // end SOFTWARE_TOUCH

static inline void
touch_wait_release(void)
{
  while (touch_check() != EVT_TOUCH_RELEASED)
    ;
}

static inline void
touch_wait_pressed(void)
{
  while (touch_check() != EVT_TOUCH_PRESSED)
    ;
}

static void getTouchPoint(uint16_t x, uint16_t y, const char *name, int16_t *data) {
  // Clear screen and ask for press
  lcd_set_colors(LCD_FG_COLOR, LCD_BG_COLOR);
  lcd_clear_screen();
  lcd_blitBitmap(x, y, TOUCH_MARK_W, TOUCH_MARK_H, touch_bitmap);
  lcd_printf((LCD_WIDTH-18*FONT_WIDTH)/2, (LCD_HEIGHT-FONT_GET_HEIGHT)/2, "TOUCH %s *", name);
  // Wait release, and fill data
  touch_wait_release();
  data[0] = last_touch_x;
  data[1] = last_touch_y;
}

void
touch_cal_exec(void)
{
  const uint16_t x1 = CALIBRATION_OFFSET - TOUCH_MARK_X;
  const uint16_t y1 = CALIBRATION_OFFSET - TOUCH_MARK_Y;
  const uint16_t x2 = LCD_WIDTH  - 1 - CALIBRATION_OFFSET - TOUCH_MARK_X;
  const uint16_t y2 = LCD_HEIGHT - 1 - CALIBRATION_OFFSET - TOUCH_MARK_Y;
  uint16_t p1 = 0, p2 = 2;
#ifdef __FLIP_DISPLAY__
  if (VNA_MODE(VNA_MODE_FLIP_DISPLAY)) {p1 = 2, p2 = 0;}
#endif
  getTouchPoint(x1, y1, "UPPER LEFT", &config._touch_cal[p1]);
  getTouchPoint(x2, y2, "LOWER RIGHT", &config._touch_cal[p2]);
}

void
touch_draw_test(void)
{
  int x0, y0;
  int x1, y1;
  lcd_set_colors(LCD_FG_COLOR, LCD_BG_COLOR);
  lcd_clear_screen();
  lcd_drawstring(OFFSETX, LCD_HEIGHT - FONT_GET_HEIGHT, "TOUCH TEST: DRAG PANEL, PRESS BUTTON TO FINISH");

  while (1) {
    if (btn_check() & EVT_BUTTON_SINGLE_CLICK) break;
    if (touch_check() == EVT_TOUCH_PRESSED){
      touch_position(&x0, &y0);
      do {
        lcd_printf(10, 30, "%3d %3d ", x0, y0);
        chThdSleepMilliseconds(50);
        touch_position(&x1, &y1);
        lcd_line(x0, y0, x1, y1);
        x0 = x1;
        y0 = y1;
      } while (touch_check() != EVT_TOUCH_RELEASED);
    }
  }
}

static void
touch_position(int *x, int *y)
{
#ifdef __REMOTE_DESKTOP__
  if (touch_remote != REMOTE_NONE) {
    *x = last_touch_x;
    *y = last_touch_y;
    return;
  }
#endif
  int tx = ((LCD_WIDTH-1-CALIBRATION_OFFSET)*(last_touch_x - config._touch_cal[0]) + CALIBRATION_OFFSET * (config._touch_cal[2] - last_touch_x)) / (config._touch_cal[2] - config._touch_cal[0]);
  if (tx<0) tx = 0; else if (tx>=LCD_WIDTH ) tx = LCD_WIDTH -1;
  int ty = ((LCD_HEIGHT-1-CALIBRATION_OFFSET)*(last_touch_y - config._touch_cal[1]) + CALIBRATION_OFFSET * (config._touch_cal[3] - last_touch_y)) / (config._touch_cal[3] - config._touch_cal[1]);
  if (ty<0) ty = 0; else if (ty>=LCD_HEIGHT) ty = LCD_HEIGHT-1;
#ifdef __FLIP_DISPLAY__
  if (VNA_MODE(VNA_MODE_FLIP_DISPLAY)) {
    tx = LCD_WIDTH - 1 - tx;
    ty = LCD_HEIGHT - 1 - ty;
  }
#endif
  *x = tx;
  *y = ty;
}

static void
show_version(void)
{
  int x = 5, y = 5, i = 1;
  int str_height = FONT_STR_HEIGHT + 2;
  lcd_set_colors(LCD_FG_COLOR, LCD_BG_COLOR);

  lcd_clear_screen();
  uint16_t shift = 0b00010010000;
  lcd_set_colors(LCD_TRACE_2_COLOR, LCD_BG_COLOR);
  lcd_drawstring_size(BOARD_NAME, x , y, 3);
  y+=FONT_GET_HEIGHT*3+3-5;
  lcd_set_colors(LCD_FG_COLOR, LCD_BG_COLOR);
  while (info_about[i]) {
    do {shift>>=1; y+=5;} while (shift&1);
    lcd_drawstring(x, y+=str_height-5, info_about[i++]);
  }
  lcd_printf(x, y+= str_height, "TCXO = %q" S_Hz, config._xtal_freq);

  y+=str_height*2;
  // Update battery and time
  uint16_t cnt = 0;
  while (true) {
    if (touch_check() == EVT_TOUCH_PRESSED)
      break;
    if (btn_check() & EVT_BUTTON_SINGLE_CLICK)
      break;
    chThdSleepMilliseconds(40);
    if ((cnt++)&0x07) continue; // Not update time so fast

#ifdef __USE_RTC__
    uint32_t tr = rtc_get_tr_bin(); // TR read first
    uint32_t dr = rtc_get_dr_bin(); // DR read second
    lcd_printf(x, y, "Time: 20%02d/%02d/%02d %02d:%02d:%02d" " (LS%c)",
      RTC_DR_YEAR(dr),
      RTC_DR_MONTH(dr),
      RTC_DR_DAY(dr),
      RTC_TR_HOUR(dr),
      RTC_TR_MIN(dr),
      RTC_TR_SEC(dr),
      (RCC->BDCR & STM32_RTCSEL_MASK) == STM32_RTCSEL_LSE ? 'E' : 'I');
#endif
#if 1
    uint32_t vbat=adc_vbat_read();
    lcd_printf(x, y + str_height, "Batt: %d.%03d" S_VOLT, vbat/1000, vbat%1000);
#endif
  }
}

#ifdef __DFU_SOFTWARE_MODE__
void
enter_dfu(void)
{
  touch_stop_watchdog();
  int x = 5, y = 20;
  lcd_set_colors(LCD_FG_COLOR, LCD_BG_COLOR);
  // leave a last message 
  lcd_clear_screen();
  lcd_drawstring(x, y, "DFU: Device Firmware Update Mode\n"
                       "To exit DFU mode, please reset device yourself.");
  boardDFUEnter();
}
#endif

static bool
select_lever_mode(int mode)
{
  if (lever_mode == mode) return false;
  lever_mode = mode;
  request_to_redraw(REDRAW_BACKUP | REDRAW_FREQUENCY | REDRAW_MARKER);
  return true;
}

static UI_FUNCTION_ADV_CALLBACK(menu_calop_acb)
{
  static const struct {uint8_t mask, next;} c_list[5]={
     [CAL_LOAD] = {CALSTAT_LOAD,  3},
     [CAL_OPEN] = {CALSTAT_OPEN,  1},
     [CAL_SHORT]= {CALSTAT_SHORT, 2},
     [CAL_THRU] = {CALSTAT_THRU,  5}, // to cal done
     [CAL_ISOLN]= {CALSTAT_ISOLN, 4},
  };
  if (b){
    if (cal_status & c_list[data].mask) b->icon = BUTTON_ICON_CHECK;
    return;
  }
  // TODO: Hack! reset button state
  last_button = 0;
  cal_collect(data);
  selection = c_list[data].next;
}

extern const menuitem_t menu_save[];
static UI_FUNCTION_CALLBACK(menu_caldone_cb)
{
  cal_done();
  menu_move_back(false);
  if (data == 0)
    menu_push_submenu(menu_save);
}

static UI_FUNCTION_CALLBACK(menu_cal_reset_cb)
{
  (void)data;
  // RESET
  cal_status = 0;
  lastsaveid = NO_SAVE_SLOT;
  set_power(SI5351_CLK_DRIVE_STRENGTH_AUTO);
}

static UI_FUNCTION_ADV_CALLBACK(menu_cal_range_acb) {
  (void)data;
  bool calibrated = cal_status & (CALSTAT_ES|CALSTAT_ER|CALSTAT_ET|CALSTAT_ED|CALSTAT_EX|CALSTAT_OPEN|CALSTAT_SHORT|CALSTAT_THRU);
  if (!calibrated) return;
  if (b){
      b->bg = (cal_status&CALSTAT_INTERPOLATED) ? LCD_INTERP_CAL_COLOR : LCD_MENU_COLOR;
      plot_printf(b->label, sizeof(b->label), "CAL: %dp\n %.6F" S_Hz "\n %.6F" S_Hz, cal_sweep_points, (float)cal_frequency0, (float)cal_frequency1);
    return;
  }
  // Reset range to calibration
  if (cal_status & CALSTAT_INTERPOLATED){
    reset_sweep_frequency();
    set_power(cal_power);
  }
}

static UI_FUNCTION_ADV_CALLBACK(menu_cal_apply_acb) {
  (void)data;
  if (b){
    b->icon = (cal_status&CALSTAT_APPLY) ? BUTTON_ICON_CHECK : BUTTON_ICON_NOCHECK;
    return;
  }
  // toggle applying correction
  cal_status ^= CALSTAT_APPLY;
  request_to_redraw(REDRAW_CAL_STATUS);
}

static UI_FUNCTION_ADV_CALLBACK(menu_recall_acb)
{
  if (b){
    const properties_t *p = get_properties(data);
    if (p)
      plot_printf(b->label, sizeof(b->label), "%.6F" S_Hz "\n%.6F" S_Hz, (float)p->_frequency0, (float)p->_frequency1);
    else
      b->p1.u = data;
    if (lastsaveid == data) b->icon = BUTTON_ICON_CHECK;
    return;
  }
  load_properties(data);
}

#define MENU_CONFIG_TOUCH_CAL   0
#define MENU_CONFIG_TOUCH_TEST  1
#define MENU_CONFIG_VERSION     2
#define MENU_CONFIG_RESET       3
#define MENU_CONFIG_LOAD        4
static UI_FUNCTION_CALLBACK(menu_config_cb)
{
  switch (data) {
  case MENU_CONFIG_TOUCH_CAL:
      touch_cal_exec();
      break;
  case MENU_CONFIG_TOUCH_TEST:
      touch_draw_test();
      break;
  case MENU_CONFIG_VERSION:
      show_version();
      break;
  case MENU_CONFIG_RESET:
      clear_all_config_prop_data();
      NVIC_SystemReset();
      break;
#if defined(__SD_CARD_LOAD__) && !defined(__SD_FILE_BROWSER__)
  case MENU_CONFIG_LOAD:
      if (!sd_card_load_config())
        drawMessageBox("Error", "No config.ini", 2000);
      break;
#endif
  }
  ui_mode_normal();
  request_to_redraw(REDRAW_CLRSCR | REDRAW_AREA | REDRAW_BATTERY | REDRAW_CAL_STATUS | REDRAW_FREQUENCY);
}

static UI_FUNCTION_CALLBACK(menu_config_save_cb)
{
  (void)data;
  config_save();
  menu_move_back(true);
}

#ifdef __DFU_SOFTWARE_MODE__
static UI_FUNCTION_CALLBACK(menu_dfu_cb)
{
  (void)data;
  enter_dfu();
}
#endif

static UI_FUNCTION_ADV_CALLBACK(menu_save_acb)
{
  if (b){
    const properties_t *p = get_properties(data);
    if (p)
      plot_printf(b->label, sizeof(b->label), "%.6F" S_Hz "\n%.6F" S_Hz, (float)p->_frequency0, (float)p->_frequency1);
    else
      b->p1.u = data;
    return;
  }
  if (caldata_save(data) == 0) {
    menu_move_back(true);
    request_to_redraw(REDRAW_BACKUP | REDRAW_CAL_STATUS);
  }
}

static UI_FUNCTION_ADV_CALLBACK(menu_trace_acb)
{
  if (b){
    if (trace[data].enabled){
      b->bg = LCD_TRACE_1_COLOR + data;
      if (data == selection) b->bg = LCD_MENU_ACTIVE_COLOR;
      if (current_trace == data)
        b->icon = BUTTON_ICON_CHECK;
    }
    b->p1.u = data;
    return;
  }

  if (trace[data].enabled && data != current_trace) // for enabled trace and not current trace
    set_active_trace(data);                         // make active
  else                                              //
    set_trace_enable(data, !trace[data].enabled);   // toggle trace enable
}

extern const menuitem_t menu_marker_s11smith[];
extern const menuitem_t menu_marker_s21smith[];
static uint8_t get_smith_format(void) {return (current_trace != TRACE_INVALID) ? trace[current_trace].smith_format : 0;}

static UI_FUNCTION_ADV_CALLBACK(menu_marker_smith_acb)
{
  if (b) {
    b->icon = get_smith_format() == data ? BUTTON_ICON_GROUP_CHECKED : BUTTON_ICON_GROUP;
    b->p1.text = get_smith_format_names(data);
    return;
  }
  if (current_trace == TRACE_INVALID) return;
  trace[current_trace].smith_format = data;
  request_to_redraw(REDRAW_AREA | REDRAW_MARKER);
}

#define F_S11     0x00
#define F_S21     0x80
static UI_FUNCTION_ADV_CALLBACK(menu_format_acb)
{
  if (current_trace == TRACE_INVALID) return; // Not apply any for invalid traces
  uint16_t format = data & (~F_S21);
  uint16_t channel = data & F_S21 ? 1 : 0;
  if (b) {
    if (trace[current_trace].type == format && trace[current_trace].channel == channel)
      b->icon = BUTTON_ICON_CHECK;
    if (format == TRC_SMITH) {
      uint8_t marker_smith_format = get_smith_format();
      if ((channel == 0 && !S11_SMITH_VALUE(marker_smith_format)) ||
          (channel == 1 && !S21_SMITH_VALUE(marker_smith_format))) return;
      plot_printf(b->label, sizeof(b->label), "%s\n" R_LINK_COLOR "%s", get_trace_typename(TRC_SMITH, marker_smith_format), get_smith_format_names(marker_smith_format));
    }
    else
      b->p1.text = get_trace_typename(format, -1);
    return;
  }

  if (format == TRC_SMITH && trace[current_trace].type == TRC_SMITH && trace[current_trace].channel == channel)
    menu_push_submenu(channel == 0 ? menu_marker_s11smith : menu_marker_s21smith);
  else
    set_trace_type(current_trace, format, channel);
}

static UI_FUNCTION_ADV_CALLBACK(menu_channel_acb)
{
  (void)data;
  if (current_trace == TRACE_INVALID) {if (b) b->p1.text = ""; return;}
  int ch = trace[current_trace].channel;
  if (b){
    b->p1.text = ch == 0 ? "S11 (REFL)" : "S21 (THRU)";
    return;
  }
  set_trace_channel(current_trace, ch^1);
}

static UI_FUNCTION_ADV_CALLBACK(menu_transform_window_acb)
{
  char *text = "";
  switch(props_mode & TD_WINDOW){
    case TD_WINDOW_MINIMUM: text = "MINIMUM"; data = TD_WINDOW_NORMAL;  break;
    case TD_WINDOW_NORMAL:  text = "NORMAL";  data = TD_WINDOW_MAXIMUM; break;
    case TD_WINDOW_MAXIMUM: text = "MAXIMUM"; data = TD_WINDOW_MINIMUM; break;
  }
  if(b){
    b->p1.text = text;
    return;
  }
  props_mode = (props_mode & ~TD_WINDOW) | data;
}

static UI_FUNCTION_ADV_CALLBACK(menu_transform_acb)
{
  (void)data;
  if(b){
    if (props_mode & DOMAIN_TIME) b->icon = BUTTON_ICON_CHECK;
    b->p1.text = (props_mode&DOMAIN_TIME) ? "ON" : "OFF";
    return;
  }
  props_mode ^= DOMAIN_TIME;
  select_lever_mode(LM_MARKER);
  request_to_redraw(REDRAW_FREQUENCY | REDRAW_AREA);
}

static UI_FUNCTION_ADV_CALLBACK(menu_transform_filter_acb)
{
  if(b){
    b->icon = (props_mode & TD_FUNC) == data ? BUTTON_ICON_GROUP_CHECKED : BUTTON_ICON_GROUP;
    return;
  }
  props_mode = (props_mode & ~TD_FUNC) | data;
}

const menuitem_t menu_bandwidth[];
static UI_FUNCTION_ADV_CALLBACK(menu_bandwidth_sel_acb)
{
  (void)data;
  if (b){
    b->p1.u = get_bandwidth_frequency(config._bandwidth);
    return;
  }
  menu_push_submenu(menu_bandwidth);
}

static UI_FUNCTION_ADV_CALLBACK(menu_bandwidth_acb)
{
  if (b){
    b->icon = config._bandwidth == data ? BUTTON_ICON_GROUP_CHECKED : BUTTON_ICON_GROUP;
    b->p1.u = get_bandwidth_frequency(data);
    return;
  }
  set_bandwidth(data);
}

#pragma pack(push, 2)
typedef struct {
  const char* text;
  uint16_t update_flag;
} vna_mode_data_t;
#pragma pack(pop)

const vna_mode_data_t vna_mode_data[] = {
//                        text (if 0 use checkbox) Redraw flags on change
  [VNA_MODE_AUTO_NAME]   = {0,                     REDRAW_BACKUP},
#ifdef __USE_SMOOTH__
  [VNA_MODE_SMOOTH]      = {"Geom\0Arith",         REDRAW_BACKUP},
#endif
#ifdef __USE_SERIAL_CONSOLE__
  [VNA_MODE_CONNECTION]  = {"USB\0SERIAL",         REDRAW_BACKUP},
#endif
  [VNA_MODE_SEARCH]      = {"MAXIMUM\0MINIMUM",    REDRAW_BACKUP},
  [VNA_MODE_SHOW_GRID]   = {0,                     REDRAW_BACKUP | REDRAW_AREA},
  [VNA_MODE_DOT_GRID]    = {0,                     REDRAW_BACKUP | REDRAW_AREA},
#ifdef __USE_BACKUP__
  [VNA_MODE_BACKUP]      = {0,                     REDRAW_BACKUP},
#endif
#ifdef __FLIP_DISPLAY__
  [VNA_MODE_FLIP_DISPLAY]= {0,                     REDRAW_BACKUP | REDRAW_CLRSCR | REDRAW_AREA | REDRAW_BATTERY | REDRAW_CAL_STATUS | REDRAW_FREQUENCY},
#endif
#ifdef __DIGIT_SEPARATOR__
  [VNA_MODE_SEPARATOR]   = {"DOT '.'\0COMMA ','",  REDRAW_BACKUP | REDRAW_MARKER | REDRAW_FREQUENCY},
#endif
#ifdef __SD_CARD_DUMP_TIFF__
  [VNA_MODE_TIFF]        = {"BMP\0TIFF",           REDRAW_BACKUP},
#endif
};

void apply_VNA_mode(uint16_t idx, uint16_t value) {
  uint16_t m = 1<<idx;
  uint16_t old = config._vna_mode;
       if (value == VNA_MODE_CLR) config._vna_mode&=~m; // clear
  else if (value == VNA_MODE_SET) config._vna_mode|= m; // set
  else                            config._vna_mode^= m; // toggle
  if (old == config._vna_mode) return;
  request_to_redraw(vna_mode_data[idx].update_flag);
  // Custom processing after apply
  switch(idx) {
#ifdef __USE_SERIAL_CONSOLE__
    case VNA_MODE_CONNECTION: shell_reset_console(); break;
#endif
    case VNA_MODE_SEARCH:
      marker_search();
#ifdef UI_USE_LEVELER_SEARCH_MODE
      select_lever_mode(LM_SEARCH);
#endif
    break;
#ifdef __FLIP_DISPLAY__
    case VNA_MODE_FLIP_DISPLAY:
      lcd_set_flip(VNA_MODE(VNA_MODE_FLIP_DISPLAY));
      draw_all();
    break;
#endif
  }
}

static UI_FUNCTION_ADV_CALLBACK(menu_vna_mode_acb)
{
  if (b) {
    const char *t = vna_mode_data[data].text;
    if (t == 0)
      b->icon = VNA_MODE(data) ? BUTTON_ICON_CHECK : BUTTON_ICON_NOCHECK;
    else
      b->p1.text = VNA_MODE(data) ? t + strlen(t) + 1 : t;
    return;
  }
  apply_VNA_mode(data, VNA_MODE_TOGGLE);
}

#ifdef __USE_SMOOTH__
static UI_FUNCTION_ADV_CALLBACK(menu_smooth_acb)
{
  if (b){
    b->icon = get_smooth_factor() == data ? BUTTON_ICON_GROUP_CHECKED : BUTTON_ICON_GROUP;
    b->p1.u = data;
    return;
  }
  set_smooth_factor(data);
}
#endif

const menuitem_t menu_sweep_points[];
static UI_FUNCTION_ADV_CALLBACK(menu_points_sel_acb)
{
  (void)data;
  if (b){
    b->p1.u = sweep_points;
    return;
  }
  menu_push_submenu(menu_sweep_points);
}

static const uint16_t point_counts_set[POINTS_SET_COUNT] = POINTS_SET;
static UI_FUNCTION_ADV_CALLBACK(menu_points_acb)
{
  uint16_t p_count = point_counts_set[data];
  if (b){
    b->icon = sweep_points == p_count ? BUTTON_ICON_GROUP_CHECKED : BUTTON_ICON_GROUP;
    b->p1.u = p_count;
    return;
  }
  set_sweep_points(p_count);
}

const menuitem_t menu_power[];
static UI_FUNCTION_ADV_CALLBACK(menu_power_sel_acb)
{
  (void)data;
  if (b){
    if (current_props._power != SI5351_CLK_DRIVE_STRENGTH_AUTO)
      plot_printf(b->label, sizeof(b->label), "POWER" R_LINK_COLOR "  %um" S_AMPER, 2+current_props._power*2);
    return;
  }
  menu_push_submenu(menu_power);
}

static UI_FUNCTION_ADV_CALLBACK(menu_power_acb)
{
  if (b){
    b->icon = current_props._power == data ? BUTTON_ICON_GROUP_CHECKED : BUTTON_ICON_GROUP;
    b->p1.u = 2+data*2;
    return;
  }
  set_power(data);
}

// Process keyboard button callback, and run keyboard function
extern const keypads_list keypads_mode_tbl[];
static UI_FUNCTION_ADV_CALLBACK(menu_keyboard_acb)
{
  if (data == KM_VAR && lever_mode == LM_EDELAY) // JOG STEP button auto set (e-delay or frequency step)
    data = KM_VAR_DELAY;
  if (b) {
    const keyboard_cb_t cb = keypads_mode_tbl[data].cb;
    if (cb) cb(keypads_mode_tbl[data].data, b);
    return;
  }
  ui_mode_keypad(data);
}

// Custom keyboard menu button callback (depend from current trace)
static UI_FUNCTION_ADV_CALLBACK(menu_scale_keyboard_acb)
{
  // Not apply amplitude / scale / ref for invalid or polar graph
  if (current_trace == TRACE_INVALID) return;
  uint32_t type_mask = 1<<trace[current_trace].type;
  if (type_mask & ROUND_GRID_MASK) return;
  // Nano scale values
  if (type_mask & NANO_TYPE_MASK) data++;
  menu_keyboard_acb(data, b);
}

static UI_FUNCTION_ADV_CALLBACK(menu_pause_acb)
{
  (void)data;
  if (b){
    b->p1.text = (sweep_mode & SWEEP_ENABLE) ? "PAUSE" : "RESUME";
    b->icon = (sweep_mode & SWEEP_ENABLE) ? BUTTON_ICON_NOCHECK : BUTTON_ICON_CHECK;
    return;
  }
  toggle_sweep();
}

#define UI_MARKER_EDELAY 6
static UI_FUNCTION_CALLBACK(menu_marker_op_cb)
{
  freq_t freq = get_marker_frequency(active_marker);
  if (freq == 0)
    return; // no active marker

  switch (data) {
  case ST_START:
  case ST_STOP:
  case ST_CENTER:
    set_sweep_frequency(data, freq);
    break;
  case ST_SPAN:
    if (previous_marker == MARKER_INVALID || active_marker == previous_marker) {
      // if only 1 marker is active, keep center freq and make span the marker comes to the edge
      freq_t center = get_sweep_frequency(ST_CENTER);
      freq_t span = center > freq ? center - freq : freq - center;
      set_sweep_frequency(ST_SPAN, span * 2);
    } else {
      // if 2 or more marker active, set start and stop freq to each marker
      freq_t freq2 = get_marker_frequency(previous_marker);
      if (freq2 == 0)
        return;
      if (freq > freq2) SWAP(freq_t, freq2, freq);
      set_sweep_frequency(ST_START, freq);
      set_sweep_frequency(ST_STOP, freq2);
    }
    break;
  case UI_MARKER_EDELAY:
    {
      if (current_trace == TRACE_INVALID)
        break;
      float (*array)[2] = measured[trace[current_trace].channel];
      int index = markers[active_marker].index;
      float v = groupdelay_from_array(index, array[index]);
      set_electrical_delay(electrical_delay + v);
    }
    break;
  }
  ui_mode_normal();
}

static UI_FUNCTION_CALLBACK(menu_marker_search_dir_cb)
{
  marker_search_dir(markers[active_marker].index, data == MK_SEARCH_RIGHT ? MK_SEARCH_RIGHT : MK_SEARCH_LEFT);
  props_mode&=~TD_MARKER_TRACK;
#ifdef UI_USE_LEVELER_SEARCH_MODE
  select_lever_mode(LM_SEARCH);
#endif
}

static UI_FUNCTION_ADV_CALLBACK(menu_marker_tracking_acb)
{
  (void)data;
  if (b){
    b->icon = (props_mode & TD_MARKER_TRACK) ? BUTTON_ICON_CHECK : BUTTON_ICON_NOCHECK;
    return;
  }
  props_mode^= TD_MARKER_TRACK;
}

#ifdef __VNA_MEASURE_MODULE__
extern const menuitem_t *menu_measure_list[];
static UI_FUNCTION_ADV_CALLBACK(menu_measure_acb)
{
  (void)data;
  if (b){
    b->icon = current_props._measure == data ? BUTTON_ICON_GROUP_CHECKED : BUTTON_ICON_GROUP;
    return;
  }
  plot_set_measure_mode(data);
  menu_set_submenu(menu_measure_list[current_props._measure]);
}

static UI_FUNCTION_CALLBACK(menu_measure_cb) {
  (void)data;
  menu_push_submenu(menu_measure_list[current_props._measure]);
}
#endif

static void
active_marker_check(void)
{
  int i;
  // Auto select active marker if disabled
  if (active_marker == MARKER_INVALID)
    for (i = 0; i < MARKERS_MAX; i++)
      if (markers[i].enabled) active_marker = i;
  // Auto select previous marker if disabled
  if (previous_marker == active_marker) previous_marker = MARKER_INVALID;
  if (previous_marker == MARKER_INVALID){
    for (i = 0; i < MARKERS_MAX; i++)
      if (markers[i].enabled && i != active_marker) previous_marker = i;
  }
}

static UI_FUNCTION_ADV_CALLBACK(menu_marker_sel_acb)
{
  //if (data >= MARKERS_MAX) return;
  int mk = data;
  if (b){
         if (mk == active_marker) b->icon = BUTTON_ICON_CHECK_AUTO;
    else if (markers[mk].enabled) b->icon = BUTTON_ICON_CHECK;
    b->p1.u = mk + 1;
    return;
  }
  // Marker select click
  if (markers[mk].enabled) {            // Marker enabled
    if (mk == active_marker) {          // If active marker:
      markers[mk].enabled = FALSE;      //  disable it
      mk = previous_marker;             //  set select from previous marker
      active_marker = MARKER_INVALID;   //  invalidate active
      request_to_redraw(REDRAW_AREA);
    }
  } else {
    markers[mk].enabled = TRUE;         // Enable marker
  }
  previous_marker = active_marker;      // set previous marker as current active
  active_marker = mk;                   // set new active marker
  active_marker_check();
  request_to_redraw(REDRAW_MARKER);
}

static UI_FUNCTION_CALLBACK(menu_marker_disable_all_cb)
{
  (void)data;
  int i;
  for (i = 0; i < MARKERS_MAX; i++)
    markers[i].enabled = FALSE;     // all off
  previous_marker = MARKER_INVALID;
  active_marker = MARKER_INVALID;
  request_to_redraw(REDRAW_AREA);
}

static UI_FUNCTION_ADV_CALLBACK(menu_marker_delta_acb)
{
  (void)data;
  if (b){
    b->icon = props_mode & TD_MARKER_DELTA ? BUTTON_ICON_CHECK : BUTTON_ICON_NOCHECK;
    return;
  }
  props_mode^= TD_MARKER_DELTA;
  request_to_redraw(REDRAW_MARKER);
}

#ifdef __USE_SERIAL_CONSOLE__
static UI_FUNCTION_ADV_CALLBACK(menu_serial_speed_acb)
{
  static const uint32_t usart_speed[] = {19200, 38400, 57600, 115200, 230400, 460800, 921600, 1843200, 2000000, 3000000};
  uint32_t speed = usart_speed[data];
  if (b){
    b->icon = config._serial_speed == speed ? BUTTON_ICON_GROUP_CHECKED : BUTTON_ICON_GROUP;
    b->p1.u = speed;
    return;
  }
  shell_update_speed(speed);
}

extern const menuitem_t menu_serial_speed[];
static UI_FUNCTION_ADV_CALLBACK(menu_serial_speed_sel_acb)
{
  (void)data;
  if (b){
    b->p1.u = config._serial_speed;
    return;
  }
  menu_push_submenu(menu_serial_speed);
}
#endif

#ifdef USE_VARIABLE_OFFSET_MENU
static UI_FUNCTION_ADV_CALLBACK(menu_offset_acb)
{
  int32_t offset = (data+1) * FREQUENCY_OFFSET_STEP;
  if (b){
    b->icon = IF_OFFSET == offset ? BUTTON_ICON_GROUP_CHECKED : BUTTON_ICON_GROUP;
    b->p1.u = offset;
    return;
  }
  si5351_set_frequency_offset(offset);
}

const menuitem_t menu_offset[];
static UI_FUNCTION_ADV_CALLBACK(menu_offset_sel_acb)
{
  (void)data;
  if (b){
    b->p1.i = IF_OFFSET;
    return;
  }
  menu_push_submenu(menu_offset);
}
#endif

#ifdef __LCD_BRIGHTNESS__
// Brightness control range 0 - 100
static void lcd_setBrightness(uint16_t b){
  dac_setvalue_ch2(700 + b*(4090-700)/100);
}

static UI_FUNCTION_ADV_CALLBACK(menu_brightness_acb)
{
  (void)data;
  if (b){
    b->p1.u = config._brightness;
    return;
  }
  int16_t value = config._brightness;
  lcd_set_colors(LCD_MENU_TEXT_COLOR, LCD_MENU_COLOR);
  lcd_fill(LCD_WIDTH/2-12*FONT_WIDTH, LCD_HEIGHT/2-20, 23*FONT_WIDTH, 40);
  lcd_printf(LCD_WIDTH/2-8*FONT_WIDTH, LCD_HEIGHT/2-13, "BRIGHTNESS %3d%% ", value);
  lcd_printf(LCD_WIDTH/2-11*FONT_WIDTH, LCD_HEIGHT/2+2, S_LARROW " USE LEVELER BUTTON " S_RARROW);
  while (TRUE) {
    uint16_t status = btn_check();
    if (status & (EVT_UP|EVT_DOWN)) {
      do {
        if (status & EVT_UP  ) value+=5;
        if (status & EVT_DOWN) value-=5;
        if (value <   0) value =   0;
        if (value > 100) value = 100;
        lcd_printf(LCD_WIDTH/2-8*FONT_WIDTH, LCD_HEIGHT/2-13, "BRIGHTNESS %3d%% ", value);
        lcd_setBrightness(value);
        chThdSleepMilliseconds(200);
      } while ((status = btn_wait_release()) != 0);
    }
    if (status == EVT_BUTTON_SINGLE_CLICK)
      break;
  }
  config._brightness = (uint8_t)value;
  request_to_redraw(REDRAW_BACKUP | REDRAW_AREA);
  ui_mode_normal();
}
#endif

#ifdef __USE_SD_CARD__
// Save/Load format enum
enum {
  FMT_S1P_FILE=0, FMT_S2P_FILE, FMT_BMP_FILE,
#ifdef __SD_CARD_DUMP_TIFF__
  FMT_TIF_FILE,
#endif
  FMT_CAL_FILE,
#ifdef __SD_CARD_DUMP_FIRMWARE__
  FMT_BIN_FILE,
#endif
#ifdef __SD_CARD_LOAD__
  FMT_CMD_FILE,
#endif
};

// Save/Load file extension
static const char *file_ext[] = {
  [FMT_S1P_FILE] = "s1p",
  [FMT_S2P_FILE] = "s2p",
  [FMT_BMP_FILE] = "bmp",
#ifdef __SD_CARD_DUMP_TIFF__
  [FMT_TIF_FILE] = "tif",
#endif
  [FMT_CAL_FILE] = "cal",
#ifdef __SD_CARD_DUMP_FIRMWARE__
  [FMT_BIN_FILE] = "bin",
#endif
#ifdef __SD_CARD_LOAD__
  [FMT_CMD_FILE] = "cmd",
#endif
};

//*******************************************************************************************
// S1P and S2P file headers, and data structures
//*******************************************************************************************
static const char s1_file_header[] =
  "!File created by NanoVNA\r\n"\
  "# Hz S RI R 50\r\n";

static const char s1_file_param[] =
  "%u % f % f\r\n";

static const char s2_file_header[] =
  "!File created by NanoVNA\r\n"\
  "# Hz S RI R 50\r\n";

static const char s2_file_param[] =
  "%u % f % f % f % f 0 0 0 0\r\n";

//*******************************************************************************************
// Bitmap file header for LCD_WIDTH x LCD_HEIGHT image 16bpp (v4 format allow set RGB mask)
//*******************************************************************************************
#define BMP_UINT32(val)  ((val)>>0)&0xFF, ((val)>>8)&0xFF, ((val)>>16)&0xFF, ((val)>>24)&0xFF
#define BMP_UINT16(val)  ((val)>>0)&0xFF, ((val)>>8)&0xFF
#define BMP_H1_SIZE      (14)                        // BMP header 14 bytes
#define BMP_V4_SIZE      (108)                       // v4  header 108 bytes
#define BMP_HEAD_SIZE    (BMP_H1_SIZE + BMP_V4_SIZE) // Size of all headers
#define BMP_SIZE         (2*LCD_WIDTH*LCD_HEIGHT)    // Bitmap size = 2*w*h
#define BMP_FILE_SIZE    (BMP_SIZE + BMP_HEAD_SIZE)  // File size = headers + bitmap
static const uint8_t bmp_header_v4[BMP_H1_SIZE + BMP_V4_SIZE] = {
// BITMAPFILEHEADER (14 byte size)
  0x42, 0x4D,                // BM signature
  BMP_UINT32(BMP_FILE_SIZE), // File size (h + v4 + bitmap)
  BMP_UINT16(0),             // reserved
  BMP_UINT16(0),             // reserved
  BMP_UINT32(BMP_HEAD_SIZE), // Size of all headers (h + v4)
// BITMAPINFOv4 (108 byte size)
  BMP_UINT32(BMP_V4_SIZE),   // Data offset after this point (v4 size)
  BMP_UINT32(LCD_WIDTH),     // Width
  BMP_UINT32(LCD_HEIGHT),    // Height
  BMP_UINT16(1),             // Planes
  BMP_UINT16(16),            // 16bpp
  BMP_UINT32(3),             // Compression (BI_BITFIELDS)
  BMP_UINT32(BMP_SIZE),      // Bitmap size (w*h*2)
  BMP_UINT32(0x0EC4),        // x Resolution (96 DPI = 96 * 39.3701 inches per meter = 0x0EC4)
  BMP_UINT32(0x0EC4),        // y Resolution (96 DPI = 96 * 39.3701 inches per meter = 0x0EC4)
  BMP_UINT32(0),             // Palette size
  BMP_UINT32(0),             // Palette used
// Extend v4 header data (color mask for RGB565)
  BMP_UINT32(0b1111100000000000),// R mask = 0b11111000 00000000
  BMP_UINT32(0b0000011111100000),// G mask = 0b00000111 11100000
  BMP_UINT32(0b0000000000011111),// B mask = 0b00000000 00011111
  BMP_UINT32(0b0000000000000000),// A mask = 0b00000000 00000000
  'B','G','R','s',           // CSType = 'sRGB'
  BMP_UINT32(0),             // ciexyzRed.ciexyzX    Endpoints
  BMP_UINT32(0),             // ciexyzRed.ciexyzY
  BMP_UINT32(0),             // ciexyzRed.ciexyzZ
  BMP_UINT32(0),             // ciexyzGreen.ciexyzX
  BMP_UINT32(0),             // ciexyzGreen.ciexyzY
  BMP_UINT32(0),             // ciexyzGreen.ciexyzZ
  BMP_UINT32(0),             // ciexyzBlue.ciexyzX
  BMP_UINT32(0),             // ciexyzBlue.ciexyzY
  BMP_UINT32(0),             // ciexyzBlue.ciexyzZ
  BMP_UINT32(0),             // GammaRed
  BMP_UINT32(0),             // GammaGreen
  BMP_UINT32(0),             // GammaBlue
};

#ifdef __SD_CARD_DUMP_TIFF__
//*******************************************************************************************
// TIFF header for LCD_WIDTH x LCD_HEIGHT image 24bpp and RLE compression (packbits)
//*******************************************************************************************
#define IFD_ENTRY(type, val_t, count, value) \
           BMP_UINT16(type), \
           BMP_UINT16(val_t), \
           BMP_UINT32(count), \
           BMP_UINT32(value)

#define IFD_BYTE     1  // 8-bit unsigned integer.
#define IFD_ASCII    2  // 8-bit byte that contains a 7-bit ASCII code; the last byte must be NUL (binary zero).
#define IFD_SHORT    3  // 16-bit (2-byte) unsigned integer.
#define IFD_LONG     4  // 32-bit (4-byte) unsigned integer.
#define IFD_RATIONAL 5  // Two LONGs: the first represents the numerator of a fraction; the second, the denominator.

// TIFF Compression
#define TIFF_UNCOMPRESSED  1
#define TIFF_CCITT_1D      2
#define TIFF_CCITT_Group3  3
#define TIFF_CCITT_Group4  4
#define TIFF_LZW           5
#define TIFF_JPEG          6
#define TIFF_UNCOMPR       0x8003
#define TIFF_PACKBITS      0x8005

#define TIFF_PHOTOMETRIC_MINISWHITE  0
#define TIFF_PHOTOMETRIC_MINISBLACK  1
#define TIFF_PHOTOMETRIC_RGB         2
#define TIFF_PHOTOMETRIC_PALETTE     3
#define TIFF_PHOTOMETRIC_MASK        4
#define TIFF_PHOTOMETRIC_SEPARATED   5
#define TIFF_PHOTOMETRIC_YCBCR       6
#define TIFF_PHOTOMETRIC_CIELAB      8
#define TIFF_PHOTOMETRIC_ICCLAB      9
#define TIFF_PHOTOMETRIC_ITULAB     10
#define TIFF_PHOTOMETRIC_LOGL    32844
#define TIFF_PHOTOMETRIC_LOGLUV  32845

#define TIFF_RESUNIT_NONE           1
#define TIFF_RESUNIT_INCH           2
#define TIFF_RESUNIT_CENTIMETER     3

// TIFF file header data
#define IFD_ENTRIES_COUNT    7
#define IFD_DATA_OFFSET      (10 + 12 * IFD_ENTRIES_COUNT + 4)

#define IFD_BPS_OFFSET       IFD_DATA_OFFSET
//#define IFD_XR_OFFSET        IFD_DATA_OFFSET + 6
//#define IFD_YR_OFFSET        IFD_DATA_OFFSET + 6 + 8
#define IFD_STRIP_OFFSET     IFD_DATA_OFFSET + 6 // + 8 + 8

static const uint8_t tif_header[] = {
  0x49, 0x49,                                             // Byte order 'II' (0x4949) - little indian or 'MM' (0x4D4D) - big indian
  BMP_UINT16(0x002A),                                     // TIFF version number (always 2Ah)
  BMP_UINT32(0x0008),                                     // IFD offset
  BMP_UINT16(IFD_ENTRIES_COUNT),                          // IFD entries NUM
  IFD_ENTRY(0x0100, IFD_SHORT,   1, LCD_WIDTH),           // Image Width
  IFD_ENTRY(0x0101, IFD_SHORT,   1, LCD_HEIGHT),          // Image Height
  IFD_ENTRY(0x0102, IFD_SHORT,   3, IFD_BPS_OFFSET),      // BitsPerSample  = 0x0008 0x0008 0x0008
  IFD_ENTRY(0x0103, IFD_SHORT,   1, TIFF_PACKBITS),       // Compression
  IFD_ENTRY(0x0106, IFD_SHORT,   1, TIFF_PHOTOMETRIC_RGB),// PhotometricInterpretation = RGB
  IFD_ENTRY(0x0111, IFD_LONG,    1, IFD_STRIP_OFFSET),    // StripOffsets    = Offset to image data
  IFD_ENTRY(0x0115, IFD_SHORT,   1, 0x03),                // SamplesPerPixel = 3
//IFD_ENTRY(0x0116, IFD_SHORT,   1, LCD_HEIGHT),          // RowsPerStrip    = LCD_HEIGHT
//IFD_ENTRY(0x0117, IFD_LONG,    1, 0x00000000),          // StripByteCounts = Image Width * Image Height * SamplesPerPixel (if set to 0, possible open any size image)
//IFD_ENTRY(0x011A, IFD_RATIONAL,1, IFD_XR_OFFSET),       // XResolution
//IFD_ENTRY(0x011B, IFD_RATIONAL,1, IFD_YR_OFFSET),       // YResolution
//IFD_ENTRY(0x0128, IFD_SHORT,   1, TIFF_RESUNIT_INCH),   // ResolutionUnit = Inch
  0x00, 0x00, 0x00, 0x00,
                                                          // IDF data
  BMP_UINT16(8),                                          //   BitsPerSample
  BMP_UINT16(8),
  BMP_UINT16(8),
//BMP_UINT32(72), BMP_UINT32(1),                          //   XResolution 72 / 1
//BMP_UINT32(72), BMP_UINT32(1),                          //   YResolution 72 / 1
                                                          //   After Image data
};

// RLE packbits algorithm
static int packbits(char *source, char *dest, int size) {
  int i = 0, rle, l, pk = 0, sz = 0;
  while ((l = size - i) > 0) {
    if (l > 128) l = 128;                              // Limit search RLE block size to 128
    char c = source[i++];                              // Get next byte and write to block
    for (rle = 0; c == source[i + rle] && --l; rle++); // Calculate this byte RLE sequence size = rle + 1
    if (sz && rle < 2) rle = 0;                        // Ignore (rle + 1) < 3 sequence on run non RLE input
    else if (sz == 0 || rle > 0) sz = pk++;            // Reset state or RLE sequence found -> start new block
    dest[pk++] = c;                                    // Write char to block
    if (rle > 0) {i+= rle; dest[sz] = -rle;}           // Write RLE sequence size and go to new block
    else if ((dest[sz] = pk - sz - 2) < 127)           // Continue write non RLE data while 1 + (non_rle + 1) < 127
      continue;
    sz = 0;                                            // Block complete
  }
  return pk;
}
#endif

static void swap_bytes(uint16_t *buf, int size) {
  for (int i = 0; i < size; i++)
    buf[i] = __REVSH(buf[i]); // swap byte order (example 0x10FF to 0xFF10)
}

// Create file name from current time
static FRESULT vna_create_file(char *fs_filename)
{
//  shell_printf("S file\r\n");
  FRESULT res = f_mount(fs_volume, "", 1);
//  shell_printf("Mount = %d\r\n", res);
  if (res != FR_OK)
    return res;
  res = f_open(fs_file, fs_filename, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
//  shell_printf("Open %s, = %d\r\n", fs_filename, res);
  return res;
}

static void vna_save_file(char *name, uint8_t format)
{
  char *buf_8;
  uint16_t *buf_16;
  int i, y;
  UINT size;
  char fs_filename[FF_LFN_BUF];
  // For screenshot need back to normal mode and redraw screen before capture!!
  // Redraw use spi_buffer so need do it before any file ops
  if (ui_mode != UI_NORMAL && (format == FMT_BMP_FILE
#ifdef __SD_CARD_DUMP_TIFF__
    || format == FMT_TIF_FILE
#endif
      )) {
    ui_mode_normal();
    draw_all();
  }

  // Prepare filename and open for write
  if (name == NULL) {   // Auto name, use date / time
#if FF_USE_LFN >= 1
    uint32_t tr = rtc_get_tr_bcd(); // TR read first
    uint32_t dr = rtc_get_dr_bcd(); // DR read second
    plot_printf(fs_filename, FF_LFN_BUF, "VNA_%06x_%06x.%s", dr, tr, file_ext[format]);
#else
    plot_printf(fs_filename, FF_LFN_BUF, "%08x.%s", rtc_get_FAT(), file_ext[format]);
#endif
  }
  else
    plot_printf(fs_filename, FF_LFN_BUF, "%s.%s", name, file_ext[format]);

//  UINT total_size = 0;
//  systime_t time = chVTGetSystemTimeX();
  // Prepare filename = .s1p / .s2p / .bmp .... and open for write
  FRESULT res = vna_create_file(fs_filename);
  if (res == FR_OK) {
    const char *s_file_format;
    switch(format) {
      /*
       *  Save touchstone file for VNA (use rev 1.1 format)
       *  https://en.wikipedia.org/wiki/Touchstone_file
       */
      case FMT_S1P_FILE:
      case FMT_S2P_FILE:
      buf_8  = (char *)spi_buffer;
      // Write SxP file
      if (format == FMT_S1P_FILE){
        s_file_format = s1_file_param;
        // write sxp header (not write NULL terminate at end)
        res = f_write(fs_file, s1_file_header, sizeof(s1_file_header)-1, &size);
//        total_size+=size;
      }
      else {
        s_file_format = s2_file_param;
        // Write s2p header (not write NULL terminate at end)
        res = f_write(fs_file, s2_file_header, sizeof(s2_file_header)-1, &size);
//        total_size+=size;
      }
      // Write all points data
      for (i = 0; i < sweep_points && res == FR_OK; i++) {
        size = plot_printf(buf_8, 128, s_file_format, getFrequency(i), measured[0][i][0], measured[0][i][1], measured[1][i][0], measured[1][i][1]);
//        total_size+=size;
        res = f_write(fs_file, buf_8, size, &size);
      }
      break;
      /*
       *  Save bitmap file (use v4 format allow set RGB mask)
       */
      case FMT_BMP_FILE:
      buf_16 = spi_buffer;
      res = f_write(fs_file, bmp_header_v4, sizeof(bmp_header_v4), &size); // Write header struct
//      total_size+=size;
      lcd_set_background(LCD_SWEEP_LINE_COLOR);
      for (y = LCD_HEIGHT-1; y >= 0 && res == FR_OK; y--) {
        lcd_read_memory(0, y, LCD_WIDTH, 1, buf_16);
        swap_bytes(buf_16, LCD_WIDTH);
        res = f_write(fs_file, buf_16, LCD_WIDTH*sizeof(uint16_t), &size);
//        total_size+=size;
        lcd_fill(LCD_WIDTH-1, y, 1, 1);
      }
      break;
#ifdef __SD_CARD_DUMP_TIFF__
      case FMT_TIF_FILE:
          buf_16 = spi_buffer;
          lcd_set_background(LCD_SWEEP_LINE_COLOR);
          res = f_write(fs_file, tif_header, sizeof(tif_header), &size); // Write header struct
          for (y = 0; y < LCD_HEIGHT && res == FR_OK; y++) {
            // Use LCD_WIDTH + 128 bytes offset for RGB888 data
            // Use 0 offset for compressed RLE (maximum need WIDTH * 4 + 128 bytes in spi_buffer)
            buf_8 = (char *)spi_buffer + 128;
            // Read LCD line in RGB565 format (swapped bytes)
            lcd_read_memory(0, y, LCD_WIDTH, 1, buf_16);
            // Convert to RGB888
            for (int x = LCD_WIDTH - 1; x >= 0; x--) {
              uint16_t color = (buf_16[x] << 8) | (buf_16[x] >> 8);
              buf_8[3*x + 0] = (color>>8) & 0xF8;// if (buf_8[3*x + 0] < 0) buf_8[3*x + 0]+= 7;
              buf_8[3*x + 1] = (color>>3) & 0xFC;// if (buf_8[3*x + 1] < 0) buf_8[3*x + 1]+= 3;
              buf_8[3*x + 2] = (color<<3) & 0xF8;// if (buf_8[3*x + 2] < 0) buf_8[3*x + 2]+= 7;
            }
            size = packbits(buf_8, (char *)spi_buffer, LCD_WIDTH * 3);
            res = f_write(fs_file, spi_buffer, size, &size);
            lcd_fill(LCD_WIDTH-1, y, 1, 1);
          }
      break;
#endif
      /*
       *  Save calibration
       */
      case FMT_CAL_FILE:
      {
        const char *src = (char*)&current_props;
        const uint32_t total = sizeof(current_props);
        res = f_write(fs_file, src, total, &size);
      }
      break;
#ifdef __SD_CARD_DUMP_FIRMWARE__
      /*
       * Dump firmware to SD card as bin file image
       */
      case FMT_BIN_FILE:
      {
        const char *src = (const char*)FLASH_START_ADDRESS;
        const uint32_t total = FLASH_TOTAL_SIZE;
        res = f_write(fs_file, src, total, &size);
      }
      break;
#endif
    }
    f_close(fs_file);
//    shell_printf("Close = %d\r\n", res);
//    testLog();
//    time = chVTGetSystemTimeX() - time;
//    shell_printf("Total time: %dms (write %d byte/sec)\r\n", time/10, total_size*10000/time);
  }

  drawMessageBox("SD CARD SAVE", res == FR_OK ? fs_filename : "  Fail write  ", 2000);
  request_to_redraw(REDRAW_AREA|REDRAW_FREQUENCY);
  ui_mode_normal();
}

static uint16_t fixScreenshotFormat(uint16_t data) {
#ifdef __SD_CARD_DUMP_TIFF__
  if (data == FMT_BMP_FILE && VNA_MODE(VNA_MODE_TIFF)) return FMT_TIF_FILE;
#endif
  return data;
}

static UI_FUNCTION_CALLBACK(menu_sdcard_cb)
{
  data = fixScreenshotFormat(data);
  if (VNA_MODE(VNA_MODE_AUTO_NAME))
    vna_save_file(NULL, data);
  else
    ui_mode_keypad(data + KM_S1P_NAME); // If no auto name, call text keyboard input
}

#ifdef __SD_FILE_BROWSER__
#include "vna_modules/vna_browser.c"
#endif
#endif // __USE_SD_CARD__

static UI_FUNCTION_ADV_CALLBACK(menu_band_sel_acb)
{
  (void)data;
  if (b){
    b->p1.text = config._band_mode == 0 ? "Si5351" : "MS5351";
    return;
  }
  config._band_mode = config._band_mode == 0 ? 1 : 0;
  si5351_set_band_mode(config._band_mode);
}

#if STORED_TRACES > 0
static UI_FUNCTION_ADV_CALLBACK(menu_stored_trace_acb)
{
  if (b){
    b->p1.text = getStoredTraces() & (1<<data) ? "CLEAR" : "STORE";
    return;
  }
  toogleStoredTrace(data);
}
#endif

static UI_FUNCTION_CALLBACK(menu_back_cb)
{
  (void)data;
  menu_move_back(false);
}

// Back button submenu list
static const menuitem_t menu_back[] = {
  { MT_CALLBACK,   0, S_LARROW " BACK", menu_back_cb },
  { MT_NEXT,       0, NULL,             NULL } // sentinel
};

#ifdef __USE_SD_CARD__
#ifdef __SD_FILE_BROWSER__
static const menuitem_t menu_sdcard_browse[] = {
  { MT_CALLBACK, FMT_BMP_FILE, "LOAD\nSCREENSHOT", menu_sdcard_browse_cb },
  { MT_CALLBACK, FMT_S1P_FILE, "LOAD S1P", menu_sdcard_browse_cb },
  { MT_CALLBACK, FMT_S2P_FILE, "LOAD S2P", menu_sdcard_browse_cb },
  { MT_CALLBACK, FMT_CAL_FILE, "LOAD CAL", menu_sdcard_browse_cb },
  { MT_NEXT,     0, NULL, menu_back } // next-> menu_back
};
#endif

static const menuitem_t menu_sdcard[] = {
#ifdef __SD_FILE_BROWSER__
  { MT_SUBMENU,              0, "LOAD",       menu_sdcard_browse },
#endif
  { MT_CALLBACK, FMT_S1P_FILE, "SAVE S1P",   menu_sdcard_cb },
  { MT_CALLBACK, FMT_S2P_FILE, "SAVE S2P",   menu_sdcard_cb },
  { MT_CALLBACK, FMT_BMP_FILE, "SCREENSHOT", menu_sdcard_cb },
  { MT_CALLBACK, FMT_CAL_FILE, "SAVE\nCALIBRATION", menu_sdcard_cb },
  { MT_ADV_CALLBACK, VNA_MODE_AUTO_NAME, "AUTO NAME", menu_vna_mode_acb},
#ifdef __SD_CARD_DUMP_TIFF__
  { MT_ADV_CALLBACK, VNA_MODE_TIFF, "IMAGE FORMAT\n " R_LINK_COLOR "%s", menu_vna_mode_acb },
#endif
  { MT_NEXT,     0, NULL, menu_back } // next-> menu_back
};
#endif

static const menuitem_t menu_calop[] = {
  { MT_ADV_CALLBACK, CAL_OPEN,  "OPEN",  menu_calop_acb },
  { MT_ADV_CALLBACK, CAL_SHORT, "SHORT", menu_calop_acb },
  { MT_ADV_CALLBACK, CAL_LOAD,  "LOAD",  menu_calop_acb },
  { MT_ADV_CALLBACK, CAL_ISOLN, "ISOLN", menu_calop_acb },
  { MT_ADV_CALLBACK, CAL_THRU,  "THRU",  menu_calop_acb },
  { MT_CALLBACK, 0,             "DONE",  menu_caldone_cb },
  { MT_CALLBACK, 1,             "DONE IN RAM",  menu_caldone_cb },
  { MT_ADV_CALLBACK, KM_EDELAY, "E-DELAY\n " R_LINK_COLOR "%b.7F" S_SECOND, menu_keyboard_acb },
  { MT_NEXT,     0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_save[] = {
  { MT_ADV_CALLBACK, 0, "Empty %d", menu_save_acb },//87632
  { MT_ADV_CALLBACK, 1, "Empty %d", menu_save_acb },
  { MT_ADV_CALLBACK, 2, "Empty %d", menu_save_acb },
#if SAVEAREA_MAX > 3
  { MT_ADV_CALLBACK, 3, "Empty %d", menu_save_acb },
#endif
#if SAVEAREA_MAX > 4
  { MT_ADV_CALLBACK, 4, "Empty %d", menu_save_acb },
#endif
#if SAVEAREA_MAX > 5
  { MT_ADV_CALLBACK, 5, "Empty %d", menu_save_acb },
#endif
#if SAVEAREA_MAX > 6
  { MT_ADV_CALLBACK, 6, "Empty %d", menu_save_acb },
#endif
#ifdef __SD_FILE_BROWSER__
  { MT_CALLBACK, FMT_CAL_FILE, "SAVE TO\n SD CARD", menu_sdcard_cb },
#endif
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_recall[] = {
  { MT_ADV_CALLBACK, 0, "Empty %d", menu_recall_acb },
  { MT_ADV_CALLBACK, 1, "Empty %d", menu_recall_acb },
  { MT_ADV_CALLBACK, 2, "Empty %d", menu_recall_acb },
#if SAVEAREA_MAX > 3
  { MT_ADV_CALLBACK, 3, "Empty %d", menu_recall_acb },
#endif
#if SAVEAREA_MAX > 4
  { MT_ADV_CALLBACK, 4, "Empty %d", menu_recall_acb },
#endif
#if SAVEAREA_MAX > 5
  { MT_ADV_CALLBACK, 5, "Empty %d", menu_recall_acb },
#endif
#if SAVEAREA_MAX > 6
  { MT_ADV_CALLBACK, 6, "Empty %d", menu_recall_acb },
#endif
#ifdef __SD_FILE_BROWSER__
  { MT_CALLBACK, FMT_CAL_FILE, "LOAD FROM\n SD CARD", menu_sdcard_browse_cb },
#endif
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_power[] = {
  { MT_ADV_CALLBACK, SI5351_CLK_DRIVE_STRENGTH_AUTO, "AUTO",  menu_power_acb },
  { MT_ADV_CALLBACK, SI5351_CLK_DRIVE_STRENGTH_2MA, "%u m" S_AMPER, menu_power_acb },
  { MT_ADV_CALLBACK, SI5351_CLK_DRIVE_STRENGTH_4MA, "%u m" S_AMPER, menu_power_acb },
  { MT_ADV_CALLBACK, SI5351_CLK_DRIVE_STRENGTH_6MA, "%u m" S_AMPER, menu_power_acb },
  { MT_ADV_CALLBACK, SI5351_CLK_DRIVE_STRENGTH_8MA, "%u m" S_AMPER, menu_power_acb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_cal[] = {
  { MT_SUBMENU,      0, "CALIBRATE",     menu_calop },
  { MT_ADV_CALLBACK, 0, "POWER  AUTO",   menu_power_sel_acb },
  { MT_SUBMENU,      0, "SAVE",          menu_save },
  { MT_ADV_CALLBACK, 0, "RANGE",         menu_cal_range_acb },
  { MT_CALLBACK,     0, "RESET",         menu_cal_reset_cb },
  { MT_ADV_CALLBACK, 0, "APPLY",         menu_cal_apply_acb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_trace[] = {
  { MT_ADV_CALLBACK, 0, "TRACE %d", menu_trace_acb },
  { MT_ADV_CALLBACK, 1, "TRACE %d", menu_trace_acb },
  { MT_ADV_CALLBACK, 2, "TRACE %d", menu_trace_acb },
  { MT_ADV_CALLBACK, 3, "TRACE %d", menu_trace_acb },
#if STORED_TRACES == 1
  { MT_ADV_CALLBACK, 0, "%s TRACE", menu_stored_trace_acb},
#elif STORED_TRACES > 1
  { MT_ADV_CALLBACK, 0, "%s TRACE A", menu_stored_trace_acb},
  { MT_ADV_CALLBACK, 1, "%s TRACE B", menu_stored_trace_acb},
#if STORED_TRACES > 2
  { MT_ADV_CALLBACK, 2, "%s TRACE C", menu_stored_trace_acb},
#endif
#endif
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_format4[] = {
  { MT_ADV_CALLBACK, F_S21|TRC_Rser,   "SERIES R",   menu_format_acb },
  { MT_ADV_CALLBACK, F_S21|TRC_Xser,   "SERIES X",   menu_format_acb },
  { MT_ADV_CALLBACK, F_S21|TRC_Zser,   "SERIES |Z|", menu_format_acb },
  { MT_ADV_CALLBACK, F_S21|TRC_Rsh,    "SHUNT R",    menu_format_acb },
  { MT_ADV_CALLBACK, F_S21|TRC_Xsh,    "SHUNT X",    menu_format_acb },
  { MT_ADV_CALLBACK, F_S21|TRC_Zsh,    "SHUNT |Z|",  menu_format_acb },
  { MT_ADV_CALLBACK, F_S21|TRC_Qs21,   "Q FACTOR",   menu_format_acb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_formatS21[] = {
  { MT_ADV_CALLBACK, F_S21|TRC_LOGMAG, "LOGMAG",      menu_format_acb },
  { MT_ADV_CALLBACK, F_S21|TRC_PHASE,  "PHASE",       menu_format_acb },
  { MT_ADV_CALLBACK, F_S21|TRC_DELAY,  "DELAY",       menu_format_acb },
  { MT_ADV_CALLBACK, F_S21|TRC_SMITH,  "SMITH",       menu_format_acb },
  { MT_ADV_CALLBACK, F_S21|TRC_POLAR,  "POLAR",       menu_format_acb },
  { MT_ADV_CALLBACK, F_S21|TRC_LINEAR, "LINEAR",      menu_format_acb },
  { MT_ADV_CALLBACK, F_S21|TRC_REAL,   "REAL",        menu_format_acb },
  { MT_ADV_CALLBACK, F_S21|TRC_IMAG,   "IMAG",        menu_format_acb },
  { MT_SUBMENU,          0, S_RARROW " MORE",     menu_format4 },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_format3[] = {
  { MT_ADV_CALLBACK, F_S11|TRC_ZPHASE, "Z PHASE",    menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_sC,     "SERIES C",   menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_sL,     "SERIES L",   menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_Rp,     "PARALLEL R", menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_Xp,     "PARALLEL X", menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_pC,     "PARALLEL C", menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_pL,     "PARALLEL L", menu_format_acb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_format2[] = {
  { MT_ADV_CALLBACK, F_S11|TRC_POLAR,  "POLAR",       menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_LINEAR, "LINEAR",      menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_REAL,   "REAL",        menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_IMAG,   "IMAG",        menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_Q,      "Q FACTOR",    menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_G,      "CONDUCTANCE", menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_B,      "SUSCEPTANCE", menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_Y,      "|Y|",         menu_format_acb },
  { MT_SUBMENU,         0, S_RARROW " MORE",     menu_format3 },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_formatS11[] = {
  { MT_ADV_CALLBACK, F_S11|TRC_LOGMAG, "LOGMAG",       menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_PHASE,  "PHASE",        menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_DELAY,  "DELAY",        menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_SMITH,  "SMITH",        menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_SWR,    "SWR",          menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_R,      "RESISTANCE",   menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_X,      "REACTANCE",    menu_format_acb },
  { MT_ADV_CALLBACK, F_S11|TRC_Z,      "|Z|",          menu_format_acb },
  { MT_SUBMENU,          0, S_RARROW " MORE",     menu_format2 },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_scale[] = {
  { MT_ADV_CALLBACK, KM_TOP,       "TOP",                 menu_scale_keyboard_acb },
  { MT_ADV_CALLBACK, KM_BOTTOM,    "BOTTOM",              menu_scale_keyboard_acb },
  { MT_ADV_CALLBACK, KM_SCALE,     "SCALE/DIV",           menu_scale_keyboard_acb },
  { MT_ADV_CALLBACK, KM_REFPOS,    "REFERENCE\nPOSITION", menu_scale_keyboard_acb },
  { MT_ADV_CALLBACK, KM_EDELAY, "E-DELAY\n " R_LINK_COLOR "%b.7F" S_SECOND, menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_S21OFFSET, "S21 OFFSET\n " R_LINK_COLOR "%b.3F" S_dB, menu_keyboard_acb },
#ifdef __USE_GRID_VALUES__
  { MT_ADV_CALLBACK, VNA_MODE_SHOW_GRID, "SHOW GRID\nVALUES", menu_vna_mode_acb },
  { MT_ADV_CALLBACK, VNA_MODE_DOT_GRID , "DOT GRID",          menu_vna_mode_acb },
#endif
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_transform[] = {
  { MT_ADV_CALLBACK, 0,                       "TRANSFORM\n%s",      menu_transform_acb },
  { MT_ADV_CALLBACK, TD_FUNC_LOWPASS_IMPULSE, "LOW PASS\nIMPULSE",  menu_transform_filter_acb },
  { MT_ADV_CALLBACK, TD_FUNC_LOWPASS_STEP,    "LOW PASS\nSTEP",     menu_transform_filter_acb },
  { MT_ADV_CALLBACK, TD_FUNC_BANDPASS,        "BANDPASS",           menu_transform_filter_acb },
  { MT_ADV_CALLBACK, 0,                       "WINDOW\n " R_LINK_COLOR "%s", menu_transform_window_acb },
  { MT_ADV_CALLBACK, KM_VELOCITY_FACTOR,      "VELOCITY F.\n " R_LINK_COLOR "%d%%%%", menu_keyboard_acb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_bandwidth[] = {
#ifdef BANDWIDTH_8000
  { MT_ADV_CALLBACK, BANDWIDTH_8000, "%u " S_Hz, menu_bandwidth_acb },
#endif
#ifdef BANDWIDTH_4000
  { MT_ADV_CALLBACK, BANDWIDTH_4000, "%u " S_Hz, menu_bandwidth_acb },
#endif
#ifdef BANDWIDTH_2000
  { MT_ADV_CALLBACK, BANDWIDTH_2000, "%u " S_Hz, menu_bandwidth_acb },
#endif
#ifdef BANDWIDTH_1000
  { MT_ADV_CALLBACK, BANDWIDTH_1000, "%u " S_Hz, menu_bandwidth_acb },
#endif
#ifdef BANDWIDTH_333
  { MT_ADV_CALLBACK, BANDWIDTH_333,  "%u " S_Hz, menu_bandwidth_acb },
#endif
#ifdef BANDWIDTH_100
  { MT_ADV_CALLBACK, BANDWIDTH_100,  "%u " S_Hz, menu_bandwidth_acb },
#endif
#ifdef BANDWIDTH_30
  { MT_ADV_CALLBACK, BANDWIDTH_30,   "%u " S_Hz, menu_bandwidth_acb },
#endif
#ifdef BANDWIDTH_10
  { MT_ADV_CALLBACK, BANDWIDTH_10,   "%u " S_Hz, menu_bandwidth_acb },
#endif
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

#ifdef __USE_SMOOTH__
const menuitem_t menu_smooth_count[] = {
  { MT_ADV_CALLBACK, VNA_MODE_SMOOTH, "SMOOTH\n " R_LINK_COLOR "%s avg",menu_vna_mode_acb },
  { MT_ADV_CALLBACK, 0, "SMOOTH\nOFF",menu_smooth_acb },
  { MT_ADV_CALLBACK, 1, "x%d", menu_smooth_acb },
  { MT_ADV_CALLBACK, 2, "x%d", menu_smooth_acb },
  { MT_ADV_CALLBACK, 4, "x%d", menu_smooth_acb },
  { MT_ADV_CALLBACK, 5, "x%d", menu_smooth_acb },
  { MT_ADV_CALLBACK, 6, "x%d", menu_smooth_acb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};
#endif

const menuitem_t menu_display[] = {
  { MT_SUBMENU,      0, "TRACE",                               menu_trace },
  { MT_SUBMENU,      0, "FORMAT\n S11 (REFL)",                 menu_formatS11 },
  { MT_SUBMENU,      0, "FORMAT\n S21 (THRU)",                 menu_formatS21 },
  { MT_ADV_CALLBACK, 0, "CHANNEL\n " R_LINK_COLOR "%s",        menu_channel_acb },
  { MT_SUBMENU,      0, "SCALE",                               menu_scale },
  { MT_SUBMENU,      0, "TRANSFORM",                           menu_transform },
  { MT_ADV_CALLBACK, 0, "IF BANDWIDTH\n " R_LINK_COLOR "%u" S_Hz, menu_bandwidth_sel_acb },
#ifdef __USE_SMOOTH__
  { MT_SUBMENU,      0, "DATA SMOOTH",                         menu_smooth_count },
#endif
#ifdef __VNA_Z_RENORMALIZATION__
  { MT_ADV_CALLBACK, KM_Z_PORT, "PORT-Z\n " R_LINK_COLOR "50 " S_RARROW "%bF" S_OHM, menu_keyboard_acb},
#endif
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_sweep_points[] = {
  { MT_ADV_CALLBACK, KM_POINTS, "SET POINTS\n " R_LINK_COLOR "%d", (const void *)menu_keyboard_acb },
  { MT_ADV_CALLBACK, 0, "%d point", menu_points_acb },
#if POINTS_SET_COUNT > 1
  { MT_ADV_CALLBACK, 1, "%d point", menu_points_acb },
#endif
#if POINTS_SET_COUNT > 2
  { MT_ADV_CALLBACK, 2, "%d point", menu_points_acb },
#endif
#if POINTS_SET_COUNT > 3
  { MT_ADV_CALLBACK, 3, "%d point", menu_points_acb },
#endif
#if POINTS_SET_COUNT > 4
  { MT_ADV_CALLBACK, 4, "%d point", menu_points_acb },
#endif
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_stimulus[] = {
  { MT_ADV_CALLBACK, KM_START,  "START",         menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_STOP,   "STOP",          menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_CENTER, "CENTER",        menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_SPAN,   "SPAN",          menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_CW,     "CW FREQ",       menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_STEP,   "FREQ STEP\n " R_LINK_COLOR "%bF" S_Hz, menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_VAR,    "JOG STEP\n " R_LINK_COLOR "AUTO",      menu_keyboard_acb },
  { MT_ADV_CALLBACK,      0,    "SWEEP POINTS\n " R_LINK_COLOR "%u",  menu_points_sel_acb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_marker_sel[] = {
  { MT_ADV_CALLBACK, 0, "MARKER %d", menu_marker_sel_acb },
#if MARKERS_MAX >=2
  { MT_ADV_CALLBACK, 1, "MARKER %d", menu_marker_sel_acb },
#endif
#if MARKERS_MAX >=3
  { MT_ADV_CALLBACK, 2, "MARKER %d", menu_marker_sel_acb },
#endif
#if MARKERS_MAX >=4
  { MT_ADV_CALLBACK, 3, "MARKER %d", menu_marker_sel_acb },
#endif
#if MARKERS_MAX >=5
  { MT_ADV_CALLBACK, 4, "MARKER %d", menu_marker_sel_acb },
#endif
#if MARKERS_MAX >=6
  { MT_ADV_CALLBACK, 5, "MARKER %d", menu_marker_sel_acb },
#endif
#if MARKERS_MAX >=7
  { MT_ADV_CALLBACK, 6, "MARKER %d", menu_marker_sel_acb },
#endif
#if MARKERS_MAX >=8
  { MT_ADV_CALLBACK, 7, "MARKER %d", menu_marker_sel_acb },
#endif
  { MT_CALLBACK, 0,     "ALL OFF", menu_marker_disable_all_cb },
  { MT_ADV_CALLBACK, 0,   "DELTA", menu_marker_delta_acb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_marker_ops[] = {
  { MT_CALLBACK, ST_START,         S_RARROW" START",   menu_marker_op_cb },
  { MT_CALLBACK, ST_STOP,          S_RARROW" STOP",    menu_marker_op_cb },
  { MT_CALLBACK, ST_CENTER,        S_RARROW" CENTER",  menu_marker_op_cb },
  { MT_CALLBACK, ST_SPAN,          S_RARROW" SPAN",    menu_marker_op_cb },
  { MT_CALLBACK, UI_MARKER_EDELAY, S_RARROW" E-DELAY", menu_marker_op_cb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_marker_s21smith[] = {
  { MT_ADV_CALLBACK, MS_LIN,      "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_LOG,      "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_REIM,     "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_SHUNT_RX, "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_SHUNT_RLC, "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_SERIES_RX,"%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_SERIES_RLC,"%s", menu_marker_smith_acb },
  { MT_NEXT, 0, NULL, (const void *)menu_back } // next-> menu_back
};

const menuitem_t menu_marker_s11smith[] = {
  { MT_ADV_CALLBACK, MS_LIN, "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_LOG, "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_REIM,"%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_RX,  "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_RLC, "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_GB,  "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_GLC, "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_RpXp,"%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_RpLC,"%s", menu_marker_smith_acb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

#ifdef __VNA_MEASURE_MODULE__
// Select menu depend from measure mode
#ifdef __USE_LC_MATCHING__
const menuitem_t menu_measure_lc[] = {
  { MT_ADV_CALLBACK, MEASURE_NONE,        "OFF",                menu_measure_acb },
  { MT_ADV_CALLBACK, MEASURE_LC_MATH,     "L/C MATCH",          menu_measure_acb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};
#endif

#ifdef __S11_CABLE_MEASURE__
const menuitem_t menu_measure_cable[] = {
  { MT_ADV_CALLBACK, MEASURE_NONE,        "OFF",                menu_measure_acb },
  { MT_ADV_CALLBACK, MEASURE_S11_CABLE,   "CABLE\n (S11)",      menu_measure_acb },
  { MT_ADV_CALLBACK, KM_VELOCITY_FACTOR,  "VELOCITY F.\n " R_LINK_COLOR "%d%%%%", menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_ACTUAL_CABLE_LEN, "CABLE LENGTH",       menu_keyboard_acb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};
#endif

#ifdef __S11_RESONANCE_MEASURE__
const menuitem_t menu_measure_resonance[] = {
  { MT_ADV_CALLBACK, MEASURE_NONE,        "OFF",                menu_measure_acb },
  { MT_ADV_CALLBACK, MEASURE_S11_RESONANCE,"RESONANCE\n (S11)", menu_measure_acb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};
#endif

#ifdef __S21_MEASURE__
const menuitem_t menu_measure_s21[] = {
  { MT_ADV_CALLBACK, MEASURE_NONE,        "OFF",                menu_measure_acb },
  { MT_ADV_CALLBACK, MEASURE_SHUNT_LC,    "SHUNT LC\n (S21)",   menu_measure_acb },
  { MT_ADV_CALLBACK, MEASURE_SERIES_LC,   "SERIES LC\n (S21)",  menu_measure_acb },
  { MT_ADV_CALLBACK, MEASURE_SERIES_XTAL, "SERIES\nXTAL (S21)", menu_measure_acb },
  { MT_ADV_CALLBACK, KM_MEASURE_R,        " Rl = " R_LINK_COLOR "%b.4F" S_OHM, menu_keyboard_acb},
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};
#endif

const menuitem_t menu_measure[] = {
  { MT_ADV_CALLBACK, MEASURE_NONE,        "OFF",                menu_measure_acb },
#ifdef __USE_LC_MATCHING__
  { MT_ADV_CALLBACK, MEASURE_LC_MATH,     "L/C MATCH",          menu_measure_acb },
#endif
#ifdef __S11_CABLE_MEASURE__
  { MT_ADV_CALLBACK, MEASURE_S11_CABLE,   "CABLE\n (S11)",      menu_measure_acb },
#endif
#ifdef __S11_RESONANCE_MEASURE__
  { MT_ADV_CALLBACK, MEASURE_S11_RESONANCE,"RESONANCE\n (S11)", menu_measure_acb },
#endif
#ifdef __S21_MEASURE__
  { MT_ADV_CALLBACK, MEASURE_SHUNT_LC,    "SHUNT LC\n (S21)",   menu_measure_acb },
  { MT_ADV_CALLBACK, MEASURE_SERIES_LC,   "SERIES LC\n (S21)",  menu_measure_acb },
  { MT_ADV_CALLBACK, MEASURE_SERIES_XTAL, "SERIES\nXTAL (S21)", menu_measure_acb },
#endif
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

// Dynamic menu selector depend from measure mode
const menuitem_t *menu_measure_list[] = {
  [MEASURE_NONE] = menu_measure,
#ifdef __USE_LC_MATCHING__
  [MEASURE_LC_MATH] = menu_measure_lc,
#endif
#ifdef __S21_MEASURE__
  [MEASURE_SHUNT_LC] = menu_measure_s21,
  [MEASURE_SERIES_LC] = menu_measure_s21,
  [MEASURE_SERIES_XTAL] = menu_measure_s21,
#endif
#ifdef __S11_CABLE_MEASURE__
  [MEASURE_S11_CABLE] = menu_measure_cable,
#endif
#ifdef __S11_RESONANCE_MEASURE__
  [MEASURE_S11_RESONANCE] = menu_measure_resonance,
#endif
};
#endif

const menuitem_t menu_marker[] = {
  { MT_SUBMENU,                0,   "SELECT\nMARKER",              menu_marker_sel    },
  { MT_ADV_CALLBACK,VNA_MODE_SEARCH,"SEARCH\n " R_LINK_COLOR "%s", menu_vna_mode_acb },
  { MT_CALLBACK, MK_SEARCH_LEFT,    "SEARCH\n " S_LARROW "LEFT",   menu_marker_search_dir_cb },
  { MT_CALLBACK, MK_SEARCH_RIGHT,   "SEARCH\n " S_RARROW "RIGHT",  menu_marker_search_dir_cb },
  { MT_SUBMENU,                0,   "OPERATIONS",                  menu_marker_ops    },
  { MT_ADV_CALLBACK,           0,   "TRACKING",                    menu_marker_tracking_acb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

#ifdef __DFU_SOFTWARE_MODE__
const menuitem_t menu_dfu[] = {
  { MT_CALLBACK, 0, "RESET AND\nENTER DFU", menu_dfu_cb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};
#endif

#ifdef __USE_SERIAL_CONSOLE__
const menuitem_t menu_serial_speed[] = {
  { MT_ADV_CALLBACK, 0, "%u", menu_serial_speed_acb },
  { MT_ADV_CALLBACK, 1, "%u", menu_serial_speed_acb },
  { MT_ADV_CALLBACK, 2, "%u", menu_serial_speed_acb },
  { MT_ADV_CALLBACK, 3, "%u", menu_serial_speed_acb },
  { MT_ADV_CALLBACK, 4, "%u", menu_serial_speed_acb },
  { MT_ADV_CALLBACK, 5, "%u", menu_serial_speed_acb },
  { MT_ADV_CALLBACK, 6, "%u", menu_serial_speed_acb },
  { MT_ADV_CALLBACK, 7, "%u", menu_serial_speed_acb },
  { MT_ADV_CALLBACK, 8, "%u", menu_serial_speed_acb },
  { MT_ADV_CALLBACK, 9, "%u", menu_serial_speed_acb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_connection[] = {
  { MT_ADV_CALLBACK, VNA_MODE_CONNECTION, "CONNECTION\n " R_LINK_COLOR "%s", menu_vna_mode_acb },
  { MT_ADV_CALLBACK, 0, "SERIAL SPEED\n " R_LINK_COLOR "%u", menu_serial_speed_sel_acb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};
#endif

const menuitem_t menu_clear[] = {
  { MT_CALLBACK, MENU_CONFIG_RESET, "CLEAR ALL\nAND RESET", menu_config_cb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

#ifdef USE_VARIABLE_OFFSET_MENU
const menuitem_t menu_offset[] = {
  { MT_ADV_CALLBACK, 0, "%d" S_Hz, menu_offset_acb },
  { MT_ADV_CALLBACK, 1, "%d" S_Hz, menu_offset_acb },
  { MT_ADV_CALLBACK, 2, "%d" S_Hz, menu_offset_acb },
  { MT_ADV_CALLBACK, 3, "%d" S_Hz, menu_offset_acb },
  { MT_ADV_CALLBACK, 4, "%d" S_Hz, menu_offset_acb },
  { MT_ADV_CALLBACK, 5, "%d" S_Hz, menu_offset_acb },
  { MT_ADV_CALLBACK, 6, "%d" S_Hz, menu_offset_acb },
  { MT_ADV_CALLBACK, 7, "%d" S_Hz, menu_offset_acb },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};
#endif

const menuitem_t menu_device1[] = {
  { MT_ADV_CALLBACK, 0,                  "MODE\n " R_LINK_COLOR "%s",      menu_band_sel_acb },
#ifdef __DIGIT_SEPARATOR__
  { MT_ADV_CALLBACK, VNA_MODE_SEPARATOR, "SEPARATOR\n " R_LINK_COLOR "%s", menu_vna_mode_acb },
#endif
#ifdef __SD_CARD_DUMP_FIRMWARE__
  { MT_CALLBACK, FMT_BIN_FILE,           "DUMP\nFIRMWARE",                 menu_sdcard_cb },
#endif
#ifdef __SD_CARD_LOAD__
#ifdef __SD_FILE_BROWSER__
  { MT_CALLBACK, FMT_CMD_FILE,           "LOAD COMMAND\n SCRIPT",          menu_sdcard_browse_cb },
#else
  { MT_CALLBACK, MENU_CONFIG_LOAD,       "LOAD\nCONFIG.INI",               menu_config_cb },
#endif
#endif
  { MT_SUBMENU, 0,                       "CLEAR CONFIG",                   menu_clear },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_device[] = {
  { MT_ADV_CALLBACK, KM_THRESHOLD, "THRESHOLD\n " R_LINK_COLOR "%.6q",         menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_XTAL,      "TCXO\n " R_LINK_COLOR "%.6q",              menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_VBAT,      "VBAT OFFSET\n " R_LINK_COLOR "%um" S_VOLT, menu_keyboard_acb },
#ifdef USE_VARIABLE_OFFSET_MENU
  { MT_ADV_CALLBACK, 0,            "IF OFFSET\n " R_LINK_COLOR "%d" S_Hz,      menu_offset_sel_acb },
#endif
#ifdef __USE_BACKUP__
  { MT_ADV_CALLBACK, VNA_MODE_BACKUP,"REMEMBER\nSTATE",                        menu_vna_mode_acb},
#endif
#ifdef __FLIP_DISPLAY__
  { MT_ADV_CALLBACK, VNA_MODE_FLIP_DISPLAY, "FLIP\nDISPLAY",                   menu_vna_mode_acb },
#endif
#ifdef __USE_RTC__
  { MT_ADV_CALLBACK, KM_RTC_DATE,  "SET DATE",                                 menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_RTC_TIME,  "SET TIME",                                 menu_keyboard_acb },
#endif
  { MT_SUBMENU, 0, S_RARROW" MORE", menu_device1 },
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_config[] = {
  { MT_CALLBACK,  MENU_CONFIG_TOUCH_CAL, "TOUCH CAL",     menu_config_cb },
  { MT_CALLBACK, MENU_CONFIG_TOUCH_TEST, "TOUCH TEST",    menu_config_cb },
  { MT_SUBMENU,                       0, "EXPERT\nSETTINGS", menu_device },
  { MT_CALLBACK,                      0, "SAVE CONFIG",   menu_config_save_cb },
#ifdef __USE_SERIAL_CONSOLE__
  { MT_SUBMENU,                       0, "CONNECTION",    menu_connection },
#endif
  { MT_CALLBACK,    MENU_CONFIG_VERSION, "VERSION",       menu_config_cb },
#ifdef __LCD_BRIGHTNESS__
  { MT_ADV_CALLBACK,                  0, "BRIGHTNESS\n " R_LINK_COLOR "%d%%%%", menu_brightness_acb },
#endif
#ifdef __DFU_SOFTWARE_MODE__
  { MT_SUBMENU,                       0, S_RARROW "DFU",   menu_dfu },
#endif
  { MT_NEXT, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_top[] = {
  { MT_SUBMENU, 0, "DISPLAY",   menu_display },
  { MT_SUBMENU, 0, "MARKER",    menu_marker },
  { MT_SUBMENU, 0, "STIMULUS",  menu_stimulus },
  { MT_SUBMENU, 0, "CALIBRATE", menu_cal },
  { MT_SUBMENU, 0, "RECALL",    menu_recall },
#ifdef __VNA_MEASURE_MODULE__
  { MT_CALLBACK,0, "MEASURE",   menu_measure_cb },
#endif
#ifdef __USE_SD_CARD__
  { MT_SUBMENU, 0, "SD CARD",   menu_sdcard },
#endif
  { MT_SUBMENU, 0, "CONFIG",    menu_config },
  { MT_ADV_CALLBACK, 0, "%s\nSWEEP", menu_pause_acb },
  { MT_NEXT, 0, NULL, NULL } // sentinel
};

#define MENU_STACK_DEPTH_MAX 5
const menuitem_t *menu_stack[MENU_STACK_DEPTH_MAX] = {
  menu_top, NULL, NULL, NULL, NULL
};

static const menuitem_t *menu_next_item(const menuitem_t *m){
  if (m == NULL) return NULL;
  m++; // Next item
  return m->type == MT_NEXT ? (menuitem_t *)m->reference : m;
}

static const menuitem_t *current_menu_item(int i) {
  const menuitem_t *m = menu_stack[menu_current_level];
  while (i--) m = menu_next_item(m);
  return m;
}

static int current_menu_get_count(void) {
  int i = 0;
  const menuitem_t *m = menu_stack[menu_current_level];
  while (m){m = menu_next_item(m); i++;}
  return i;
}

static int get_lines_count(const char *label) {
  int n = 1;
  while (*label)
    if (*label++ == '\n')
      n++;
  return n;
}

static void
ensure_selection(void)
{
  int i = current_menu_get_count();
  if (selection < 0)
    selection = -1;
  else if (selection >= i)
    selection = i-1;
  if      (i < MENU_BUTTON_MIN) i = MENU_BUTTON_MIN;
  else if (i >=MENU_BUTTON_MAX) i = MENU_BUTTON_MAX;
  menu_button_height = MENU_BUTTON_HEIGHT(i);
}

static void
menu_move_back(bool leave_ui)
{
  if (menu_current_level == 0)
    return;
  menu_current_level--;
  ensure_selection();
  if (leave_ui)
    ui_mode_normal();
}

static void
menu_set_submenu(const menuitem_t *submenu) {
  menu_stack[menu_current_level] = submenu;
  ensure_selection();
}

static void
menu_push_submenu(const menuitem_t *submenu) {
  if (menu_current_level < MENU_STACK_DEPTH_MAX-1)
    menu_current_level++;
  menu_set_submenu(submenu);
}

/*
static void
menu_move_top(void)
{
  if (menu_current_level == 0)
    return;
  menu_current_level = 0;
  ensure_selection();
}
*/

static void
menu_invoke(int item)
{
  const menuitem_t *menu = current_menu_item(item);
  if (menu == NULL) return;
  switch (menu->type) {
  case MT_CALLBACK:
    if (menu->reference) ((menuaction_cb_t)menu->reference)(menu->data);
    break;

  case MT_ADV_CALLBACK:
    if (menu->reference) ((menuaction_acb_t)menu->reference)(menu->data, NULL);
    break;

  case MT_SUBMENU:
    menu_push_submenu((const menuitem_t*)menu->reference);
    break;
  }
  // Redraw menu after if UI in menu mode
  if (ui_mode == UI_MENU)
    draw_menu(-1);
}

// Draw button function
static void
draw_button(uint16_t x, uint16_t y, uint16_t w, uint16_t h, button_t *b) {
  uint16_t type = b->border;
  uint16_t bw = type & BUTTON_BORDER_WIDTH_MASK;
  // Draw border if width > 0
  if (bw) {
    uint16_t br = LCD_RISE_EDGE_COLOR;
    uint16_t bd = LCD_FALLEN_EDGE_COLOR;
    lcd_set_background(type&BUTTON_BORDER_TOP    ? br : bd);lcd_fill(x,          y,           w, bw); // top
    lcd_set_background(type&BUTTON_BORDER_LEFT   ? br : bd);lcd_fill(x,          y,          bw,  h); // left
    lcd_set_background(type&BUTTON_BORDER_RIGHT  ? br : bd);lcd_fill(x + w - bw, y,          bw,  h); // right
    lcd_set_background(type&BUTTON_BORDER_BOTTOM ? br : bd);lcd_fill(x,          y + h - bw,  w, bw); // bottom
  }
  // Set colors for button and text
  lcd_set_colors(b->fg, b->bg);
  if (type & BUTTON_BORDER_NO_FILL) return;
  lcd_fill(x + bw, y + bw, w - (bw * 2), h - (bw * 2));
}

// Draw message box function
void drawMessageBox(const char *header, const char *text, uint32_t delay) {
  button_t b;
  int x , y;
  b.bg = LCD_MENU_COLOR;
  b.fg = LCD_MENU_TEXT_COLOR;
  b.border = BUTTON_BORDER_FLAT|1;
  if (header) {// Draw header
    draw_button((LCD_WIDTH-MESSAGE_BOX_WIDTH)/2, LCD_HEIGHT/2-40, MESSAGE_BOX_WIDTH, 60, &b);
    x = (LCD_WIDTH-MESSAGE_BOX_WIDTH)/2 + 10;
    y = LCD_HEIGHT/2-40 + 5;
    lcd_drawstring(x, y, header);
    request_to_redraw(REDRAW_AREA);
  }
  if (text) {  // Draw window
    lcd_set_colors(LCD_MENU_TEXT_COLOR, LCD_FG_COLOR);
    lcd_fill((LCD_WIDTH-MESSAGE_BOX_WIDTH)/2+3, LCD_HEIGHT/2-40+FONT_STR_HEIGHT+8, MESSAGE_BOX_WIDTH-6, 60-FONT_STR_HEIGHT-8-3);
    x = (LCD_WIDTH-MESSAGE_BOX_WIDTH)/2 + 20;
    y = LCD_HEIGHT/2-40 + FONT_STR_HEIGHT + 8 + 14;
    lcd_drawstring(x, y, text);
    request_to_redraw(REDRAW_AREA);
  }

  do {
    chThdSleepMilliseconds(delay == 0 ? 50 : delay);
  } while (delay == 0 && btn_check() != EVT_BUTTON_SINGLE_CLICK && touch_check() != EVT_TOUCH_PRESSED);
}

//
// KEYBOARD functions
//
enum {NUM_KEYBOARD, TXT_KEYBOARD};

// Keyboard size and position data
static const keypad_pos_t key_pos[] = {
  [NUM_KEYBOARD] = {KP_X_OFFSET, KP_Y_OFFSET, KP_WIDTH, KP_HEIGHT},
  [TXT_KEYBOARD] = {KPF_X_OFFSET, KPF_Y_OFFSET, KPF_WIDTH, KPF_HEIGHT}
};

static const keypads_t keypads_freq[] = {
  { 16, NUM_KEYBOARD },     // 16 buttons NUM keyboard (4x4 size)
  { 0x13, KP_PERIOD },
  { 0x03, KP_0 },           // 7 8 9 G
  { 0x02, KP_1 },           // 4 5 6 M
  { 0x12, KP_2 },           // 1 2 3 k
  { 0x22, KP_3 },           // 0 . < x
  { 0x01, KP_4 },
  { 0x11, KP_5 },
  { 0x21, KP_6 },
  { 0x00, KP_7 },
  { 0x10, KP_8 },
  { 0x20, KP_9 },
  { 0x30, KP_G },
  { 0x31, KP_M },
  { 0x32, KP_k },
  { 0x33, KP_X1 },
  { 0x23, KP_BS }
};

static const keypads_t keypads_ufloat[] = { //
  { 13, NUM_KEYBOARD },     // 13 buttons NUM keyboard (4x4 size)
  { 0x13, KP_PERIOD },
  { 0x03, KP_0 },           // 7 8 9
  { 0x02, KP_1 },           // 4 5 6
  { 0x12, KP_2 },           // 1 2 3
  { 0x22, KP_3 },           // 0 . < x
  { 0x01, KP_4 },
  { 0x11, KP_5 },
  { 0x21, KP_6 },
  { 0x00, KP_7 },
  { 0x10, KP_8 },
  { 0x20, KP_9 },
  { 0x33, KP_ENTER },
  { 0x23, KP_BS }
};

static const keypads_t keypads_percent[] = { //
  { 13, NUM_KEYBOARD },     // 13 buttons NUM keyboard (4x4 size)
  { 0x13, KP_PERIOD },
  { 0x03, KP_0 },           // 7 8 9
  { 0x02, KP_1 },           // 4 5 6
  { 0x12, KP_2 },           // 1 2 3
  { 0x22, KP_3 },           // 0 . < %
  { 0x01, KP_4 },
  { 0x11, KP_5 },
  { 0x21, KP_6 },
  { 0x00, KP_7 },
  { 0x10, KP_8 },
  { 0x20, KP_9 },
  { 0x33, KP_PERCENT },
  { 0x23, KP_BS }
};

static const keypads_t keypads_float[] = {
  { 14, NUM_KEYBOARD },     // 14 buttons NUM keyboard (4x4 size)
  { 0x13, KP_PERIOD },
  { 0x03, KP_0 },           // 7 8 9
  { 0x02, KP_1 },           // 4 5 6
  { 0x12, KP_2 },           // 1 2 3 -
  { 0x22, KP_3 },           // 0 . < x
  { 0x01, KP_4 },
  { 0x11, KP_5 },
  { 0x21, KP_6 },
  { 0x00, KP_7 },
  { 0x10, KP_8 },
  { 0x20, KP_9 },
  { 0x32, KP_MINUS },
  { 0x33, KP_ENTER },
  { 0x23, KP_BS }
};

static const keypads_t keypads_mfloat[] = {
  { 16, NUM_KEYBOARD },     // 16 buttons NUM keyboard (4x4 size)
  { 0x13, KP_PERIOD },
  { 0x03, KP_0 },           // 7 8 9 u
  { 0x02, KP_1 },           // 4 5 6 m
  { 0x12, KP_2 },           // 1 2 3 -
  { 0x22, KP_3 },           // 0 . < x
  { 0x01, KP_4 },
  { 0x11, KP_5 },
  { 0x21, KP_6 },
  { 0x00, KP_7 },
  { 0x10, KP_8 },
  { 0x20, KP_9 },
  { 0x30, KP_u },
  { 0x31, KP_m },
  { 0x32, KP_MINUS },
  { 0x33, KP_ENTER },
  { 0x23, KP_BS }
};

static const keypads_t keypads_mkufloat[] = {
  { 15, NUM_KEYBOARD },     // 15 buttons NUM keyboard (4x4 size)
  { 0x13, KP_PERIOD },
  { 0x03, KP_0 },           // 7 8 9
  { 0x02, KP_1 },           // 4 5 6 m
  { 0x12, KP_2 },           // 1 2 3 k
  { 0x22, KP_3 },           // 0 . < x
  { 0x01, KP_4 },
  { 0x11, KP_5 },
  { 0x21, KP_6 },
  { 0x00, KP_7 },
  { 0x10, KP_8 },
  { 0x20, KP_9 },
  { 0x31, KP_m },
  { 0x32, KP_k },
  { 0x33, KP_ENTER },
  { 0x23, KP_BS }
};

static const keypads_t keypads_nfloat[] = {
  { 16, NUM_KEYBOARD },     // 16 buttons NUM keyboard (4x4 size)
  { 0x13, KP_PERIOD },
  { 0x03, KP_0 },           // 7 8 9 u
  { 0x02, KP_1 },           // 4 5 6 n
  { 0x12, KP_2 },           // 1 2 3 p
  { 0x22, KP_3 },           // 0 . < -
  { 0x01, KP_4 },
  { 0x11, KP_5 },
  { 0x21, KP_6 },
  { 0x00, KP_7 },
  { 0x10, KP_8 },
  { 0x20, KP_9 },
  { 0x30, KP_u },
  { 0x31, KP_n },
  { 0x32, KP_p },
  { 0x33, KP_MINUS },
  { 0x23, KP_BS }
};

#if 0
//  ABCD keyboard
static const keypads_t keypads_text[] = {
  {40, TXT_KEYBOARD },   // 40 buttons TXT keyboard (10x4 size)
  {0x00, '0'}, {0x10, '1'}, {0x20, '2'}, {0x30, '3'}, {0x40, '4'}, {0x50, '5'}, {0x60, '6'}, {0x70, '7'}, {0x80, '8'}, {0x90, '9'},
  {0x01, 'A'}, {0x11, 'B'}, {0x21, 'C'}, {0x31, 'D'}, {0x41, 'E'}, {0x51, 'F'}, {0x61, 'G'}, {0x71, 'H'}, {0x81, 'I'}, {0x91, 'J'},
  {0x02, 'K'}, {0x12, 'L'}, {0x22, 'M'}, {0x32, 'N'}, {0x42, 'O'}, {0x52, 'P'}, {0x62, 'Q'}, {0x72, 'R'}, {0x82, 'S'}, {0x92, 'T'},
  {0x03, 'U'}, {0x13, 'V'}, {0x23, 'W'}, {0x33, 'X'}, {0x43, 'Y'}, {0x53, 'Z'}, {0x63, '_'}, {0x73, '-'}, {0x83, S_LARROW[0]}, {0x93, S_ENTER[0]},
};
#else
// QWERTY keyboard
static const keypads_t keypads_text[] = {
  {40, TXT_KEYBOARD },   // 40 buttons TXT keyboard (10x4 size)
  {0x00, '1'}, {0x10, '2'}, {0x20, '3'}, {0x30, '4'}, {0x40, '5'}, {0x50, '6'}, {0x60, '7'}, {0x70, '8'}, {0x80, '9'}, {0x90, '0'},
  {0x01, 'Q'}, {0x11, 'W'}, {0x21, 'E'}, {0x31, 'R'}, {0x41, 'T'}, {0x51, 'Y'}, {0x61, 'U'}, {0x71, 'I'}, {0x81, 'O'}, {0x91, 'P'},
  {0x02, 'A'}, {0x12, 'S'}, {0x22, 'D'}, {0x32, 'F'}, {0x42, 'G'}, {0x52, 'H'}, {0x62, 'J'}, {0x72, 'K'}, {0x82, 'L'}, {0x92, '_'},
  {0x03, '-'}, {0x13, 'Z'}, {0x23, 'X'}, {0x33, 'C'}, {0x43, 'V'}, {0x53, 'B'}, {0x63, 'N'}, {0x73, 'M'}, {0x83, S_LARROW[0]}, {0x93, S_ENTER[0]},
};
#endif

enum {KEYPAD_FREQ, KEYPAD_UFLOAT, KEYPAD_PERCENT, KEYPAD_FLOAT, KEYPAD_MFLOAT, KEYPAD_MKUFLOAT, KEYPAD_NFLOAT, KEYPAD_TEXT};
static const keypads_t *keypad_type_list[] = {
  [KEYPAD_FREQ]   = keypads_freq,   // frequency input
  [KEYPAD_UFLOAT] = keypads_ufloat, // unsigned float input
  [KEYPAD_PERCENT]= keypads_percent,// unsigned float input in percent
  [KEYPAD_FLOAT]  = keypads_float,  // signed float input
  [KEYPAD_MFLOAT]   = keypads_mfloat,  //   signed milli/micro float input
  [KEYPAD_MKUFLOAT] = keypads_mkufloat,// unsigned milli/kilo float input
  [KEYPAD_NFLOAT] = keypads_nfloat, // signed micro/nano/pico float input
  [KEYPAD_TEXT]   = keypads_text    // text input
};

// Get value from keyboard functions
float keyboard_get_float(void)   {return my_atof(kp_buf);}
freq_t keyboard_get_freq(void)   {return my_atoui(kp_buf);}
uint32_t keyboard_get_uint(void) {return my_atoui(kp_buf);}

// Keyboard call back functions, allow get value for Keyboard menu button (see menu_keyboard_acb) and apply on finish input
UI_KEYBOARD_CALLBACK(input_freq) {
  if (b) {
    if (data == ST_VAR && var_freq)
      plot_printf(b->label, sizeof(b->label), "JOG STEP\n " R_LINK_COLOR "%.3q" S_Hz, var_freq);
    if (data == ST_STEP) b->p1.f = (float)get_sweep_frequency(ST_SPAN) / (sweep_points - 1);
    return;
  }
  set_sweep_frequency(data, keyboard_get_freq());
}

UI_KEYBOARD_CALLBACK(input_var_delay) {
  (void)data;
  if (b) {
    if (current_props._var_delay)
      plot_printf(b->label, sizeof(b->label), "JOG STEP\n " R_LINK_COLOR "%F" S_SECOND, current_props._var_delay);
    return;
  }
  current_props._var_delay = keyboard_get_float();
}

// Call back functions for MT_CALLBACK type
UI_KEYBOARD_CALLBACK(input_points) {
  (void)data;
  if (b) {b->p1.u = sweep_points; return;}
  set_sweep_points(keyboard_get_uint());
}

UI_KEYBOARD_CALLBACK(input_amplitude) {
  int type = trace[current_trace].type;
  float scale = get_trace_scale(current_trace);
  float ref = get_trace_refpos(current_trace);
  float bot =  (0      - ref) * scale;
  float top =  (NGRIDY - ref) * scale;

  if (b) {
    float val = data == 0 ? top : bot;
    if (type == TRC_SWR) val+= 1.0f;
    plot_printf(b->label, sizeof(b->label), "%s\n " R_LINK_COLOR "%.4F%s", data == 0 ? "TOP" : "BOTTOM", val, trace_info_list[type].symbol);
    return;
  }
  float value = keyboard_get_float();
  if (type == TRC_SWR) value-= 1.0f; // Hack for SWR trace!
  if (data == 0) top = value;        // top value input
  else           bot = value;        // bottom value input
  scale = (top - bot) / NGRIDY;
  ref = (top == bot) ? -value : -bot / scale;
  set_trace_scale(current_trace, scale);
  set_trace_refpos(current_trace, ref);
}

UI_KEYBOARD_CALLBACK(input_scale) {
  (void)data;
  if (b) {b->p1.f = get_trace_scale(current_trace); return;}
  set_trace_scale(current_trace, keyboard_get_float());
}

UI_KEYBOARD_CALLBACK(input_ref) {
  (void)data;
  if (b) {b->p1.f = get_trace_refpos(current_trace); return;}
  set_trace_refpos(current_trace, keyboard_get_float());
}

UI_KEYBOARD_CALLBACK(input_edelay) {
  (void)data;
  if (b) {b->p1.f = electrical_delay; return;}
  set_electrical_delay(keyboard_get_float());
}

UI_KEYBOARD_CALLBACK(input_s21_offset) {
  (void)data;
  if (b) {b->p1.f = s21_offset; return;}
  set_s21_offset(keyboard_get_float());
}

UI_KEYBOARD_CALLBACK(input_velocity) {
  (void)data;
  if (b) {b->p1.u = velocity_factor; return;}
  velocity_factor = keyboard_get_uint();
}

#ifdef __S11_CABLE_MEASURE__
extern float real_cable_len;
UI_KEYBOARD_CALLBACK(input_cable_len) {
  (void)data;
  if (b) {
    if (real_cable_len == 0.0f) return;
    plot_printf(b->label, sizeof(b->label), "%s\n " R_LINK_COLOR "%.4F%s", "CABLE LENGTH", real_cable_len, S_METRE);
    return;
  }
  real_cable_len = keyboard_get_float();
}
#endif

UI_KEYBOARD_CALLBACK(input_xtal) {
  (void)data;
  if (b) {b->p1.u = config._xtal_freq; return;}
  si5351_set_tcxo(keyboard_get_uint());
}

UI_KEYBOARD_CALLBACK(input_harmonic) {
  (void)data;
  if (b) {b->p1.u = config._harmonic_freq_threshold; return;}
  config._harmonic_freq_threshold = keyboard_get_uint();
}

UI_KEYBOARD_CALLBACK(input_vbat) {
  (void)data;
  if (b) {b->p1.u = config._vbat_offset; return;}
  config._vbat_offset = keyboard_get_uint();
}

#ifdef __S21_MEASURE__
UI_KEYBOARD_CALLBACK(input_measure_r) {
  (void)data;
  if (b) {b->p1.f = config._measure_r; return;}
  config._measure_r = keyboard_get_float();
}
#endif

#ifdef __VNA_Z_RENORMALIZATION__
UI_KEYBOARD_CALLBACK(input_portz) {
  (void)data;
  if (b) {b->p1.f = current_props._portz; return;}
  current_props._portz = keyboard_get_float();
}
#endif

#ifdef __USE_RTC__
UI_KEYBOARD_CALLBACK(input_date_time) {
  if (b) return;
  int i = 0;
  uint32_t  dt_buf[2];
  dt_buf[0] = rtc_get_tr_bcd(); // TR should be read first for sync
  dt_buf[1] = rtc_get_dr_bcd(); // DR should be read second
  //            0    1   2       4      5     6
  // time[] ={sec, min, hr, 0, day, month, year, 0}
  uint8_t   *time = (uint8_t*)dt_buf;
  for (; i < 6 && kp_buf[i]!=0; i++) kp_buf[i]-= '0';
  for (; i < 6                ; i++) kp_buf[i] =   0;
  for (i = 0; i < 3; i++) kp_buf[i] = (kp_buf[2*i]<<4) | kp_buf[2*i+1]; // BCD format
  if (data == KM_RTC_DATE) {
    // Month limit 1 - 12 (in BCD)
         if (kp_buf[1] <    1) kp_buf[1] =    1;
    else if (kp_buf[1] > 0x12) kp_buf[1] = 0x12;
    // Day limit (depend from month):
    uint8_t day_max = 28 + ((0b11101100000000000010111110111011001100>>(kp_buf[1]<<1))&3);
    day_max = ((day_max/10)<<4)|(day_max%10); // to BCD
         if (kp_buf[2] <  1)      kp_buf[2] = 1;
    else if (kp_buf[2] > day_max) kp_buf[2] = day_max;
    time[6] = kp_buf[0]; // year
    time[5] = kp_buf[1]; // month
    time[4] = kp_buf[2]; // day
  }
  else {
    // Hour limit 0 - 23, min limit 0 - 59, sec limit 0 - 59 (in BCD)
    if (kp_buf[0] > 0x23) kp_buf[0] = 0x23;
    if (kp_buf[1] > 0x59) kp_buf[1] = 0x59;
    if (kp_buf[2] > 0x59) kp_buf[2] = 0x59;
    time[2] = kp_buf[0]; // hour
    time[1] = kp_buf[1]; // min
    time[0] = kp_buf[2]; // sec
  }
  rtc_set_time(dt_buf[1], dt_buf[0]);
}
#endif

#ifdef __USE_SD_CARD__
UI_KEYBOARD_CALLBACK(input_filename) {
  if (b) return;
  vna_save_file(kp_buf, data);
}
#endif

const keypads_list keypads_mode_tbl[KM_NONE] = {
//                      key format     data for cb    text at bottom        callback function
[KM_START]           = {KEYPAD_FREQ,   ST_START,      "START",              input_freq     }, // start
[KM_STOP]            = {KEYPAD_FREQ,   ST_STOP,       "STOP",               input_freq     }, // stop
[KM_CENTER]          = {KEYPAD_FREQ,   ST_CENTER,     "CENTER",             input_freq     }, // center
[KM_SPAN]            = {KEYPAD_FREQ,   ST_SPAN,       "SPAN",               input_freq     }, // span
[KM_CW]              = {KEYPAD_FREQ,   ST_CW,         "CW FREQ",            input_freq     }, // cw freq
[KM_STEP]            = {KEYPAD_FREQ,   ST_STEP,       "FREQ STEP",          input_freq     }, // freq as point step
[KM_VAR]             = {KEYPAD_FREQ,   ST_VAR,        "JOG STEP",           input_freq     }, // VAR freq step
[KM_POINTS]          = {KEYPAD_UFLOAT, 0,             "POINTS",             input_points   }, // Points num
[KM_TOP]             = {KEYPAD_MFLOAT, 0,             "TOP",                input_amplitude}, // top graph value
[KM_nTOP]            = {KEYPAD_NFLOAT, 0,             "TOP",                input_amplitude}, // top graph value
[KM_BOTTOM]          = {KEYPAD_MFLOAT, 1,             "BOTTOM",             input_amplitude}, // bottom graph value
[KM_nBOTTOM]         = {KEYPAD_NFLOAT, 1,             "BOTTOM",             input_amplitude}, // bottom graph value
[KM_SCALE]           = {KEYPAD_UFLOAT, KM_SCALE,      "SCALE",              input_scale    }, // scale
[KM_nSCALE]          = {KEYPAD_NFLOAT, KM_nSCALE,     "SCALE",              input_scale    }, // nano / pico scale value
[KM_REFPOS]          = {KEYPAD_FLOAT,  0,             "REFPOS",             input_ref      }, // refpos
[KM_EDELAY]          = {KEYPAD_NFLOAT, 0,             "E-DELAY",            input_edelay   }, // electrical delay
[KM_VAR_DELAY]       = {KEYPAD_NFLOAT, 0,             "JOG STEP",           input_var_delay}, // VAR electrical delay
[KM_S21OFFSET]       = {KEYPAD_FLOAT,  0,             "S21 OFFSET",         input_s21_offset},// S21 level offset
[KM_VELOCITY_FACTOR] = {KEYPAD_PERCENT,0,             "VELOCITY%%",         input_velocity }, // velocity factor
#ifdef __S11_CABLE_MEASURE__
[KM_ACTUAL_CABLE_LEN]= {KEYPAD_MKUFLOAT,0,            "CABLE LENGTH",       input_cable_len}, // real cable length input for VF calculation
#endif
[KM_XTAL]            = {KEYPAD_FREQ,   0,             "TCXO 26M" S_Hz,      input_xtal     }, // XTAL frequency
[KM_THRESHOLD]       = {KEYPAD_FREQ,   0,             "THRESHOLD",          input_harmonic }, // Harmonic threshold frequency
[KM_VBAT]            = {KEYPAD_UFLOAT, 0,             "BAT OFFSET",         input_vbat     }, // Vbat offset input in mV
#ifdef __S21_MEASURE__
[KM_MEASURE_R]       = {KEYPAD_UFLOAT, 0,             "MEASURE Rl",         input_measure_r}, // CH0 port impedance in Om
#endif
#ifdef __VNA_Z_RENORMALIZATION__
[KM_Z_PORT]          = {KEYPAD_UFLOAT, 0,             "PORT Z 50" S_RARROW, input_portz    }, // Port Z renormalization impedance
#endif
#ifdef __USE_RTC__
[KM_RTC_DATE]        = {KEYPAD_UFLOAT, KM_RTC_DATE,   "SET DATE\nYY MM DD", input_date_time}, // Date
[KM_RTC_TIME]        = {KEYPAD_UFLOAT, KM_RTC_TIME,   "SET TIME\nHH MM SS", input_date_time}, // Time
#endif
#ifdef __USE_SD_CARD__
[KM_S1P_NAME]        = {KEYPAD_TEXT,   FMT_S1P_FILE,  "S1P",                input_filename }, // s1p filename
[KM_S2P_NAME]        = {KEYPAD_TEXT,   FMT_S2P_FILE,  "S2P",                input_filename }, // s2p filename
[KM_BMP_NAME]        = {KEYPAD_TEXT,   FMT_BMP_FILE,  "BMP",                input_filename }, // bmp filename
#ifdef __SD_CARD_DUMP_TIFF__
[KM_TIF_NAME]        = {KEYPAD_TEXT,   FMT_TIF_FILE,  "TIF",                input_filename }, // tif filename
#endif
[KM_CAL_NAME]        = {KEYPAD_TEXT,   FMT_CAL_FILE,  "CAL",                input_filename }, // cal filename
#ifdef __SD_CARD_DUMP_FIRMWARE__
[KM_BIN_NAME]        = {KEYPAD_TEXT,   FMT_BIN_FILE,  "BIN",                input_filename }, // bin filename
#endif
#endif
};

static void
keypad_set_value(void) {
  const keyboard_cb_t cb = keypads_mode_tbl[keypad_mode].cb;
  if (cb) cb(keypads_mode_tbl[keypad_mode].data, NULL);
}

static void
draw_keypad_button(int id) {
  if (id < 0) return;
  button_t button;
  button.fg = LCD_MENU_TEXT_COLOR;

  if (id == selection) {
    button.bg = LCD_MENU_ACTIVE_COLOR;
    button.border = KEYBOARD_BUTTON_BORDER|BUTTON_BORDER_FALLING;
  } else{
    button.bg = LCD_MENU_COLOR;
    button.border = KEYBOARD_BUTTON_BORDER|BUTTON_BORDER_RISE;
  }

  const keypad_pos_t *p = &key_pos[keypads[0].c];
  int x = p->x_offs + (keypads[id+1].pos>> 4) * p->width;
  int y = p->y_offs + (keypads[id+1].pos&0xF) * p->height;
  draw_button(x, y, p->width, p->height, &button);
  if (keypads[0].c == NUM_KEYBOARD) {
    lcd_drawfont(keypads[id+1].c,
                     x + (KP_WIDTH - NUM_FONT_GET_WIDTH) / 2,
                     y + (KP_HEIGHT - NUM_FONT_GET_HEIGHT) / 2);
  } else {
#if 0
    lcd_drawchar(keypads[id+1].c,
                     x + (KPF_WIDTH - FONT_WIDTH) / 2,
                     y + (KPF_HEIGHT - FONT_GET_HEIGHT) / 2);
#else
    lcd_drawchar_size(keypads[id+1].c,
                     x + KPF_WIDTH/2 - FONT_WIDTH + 1,
                     y + KPF_HEIGHT/2 - FONT_GET_HEIGHT, 2);
#endif
  }
}

static void
draw_keypad(void)
{
  int i;
  for(i = 0; i < keypads[0].pos; i++)
    draw_keypad_button(i);
}

static int period_pos(void) {int j; for (j = 0; kp_buf[j] && kp_buf[j] != '.'; j++); return j;}

static void draw_numeric_area_frame(void) {
  lcd_set_colors(LCD_INPUT_TEXT_COLOR, LCD_INPUT_BG_COLOR);
  lcd_fill(0, LCD_HEIGHT-NUM_INPUT_HEIGHT, LCD_WIDTH, NUM_INPUT_HEIGHT);
  const char *label = keypads_mode_tbl[keypad_mode].name;
  int lines = get_lines_count(label);
  lcd_drawstring(10, LCD_HEIGHT-(FONT_STR_HEIGHT * lines + NUM_INPUT_HEIGHT)/2, label);
}

static void
draw_numeric_input(const char *buf)
{
  uint16_t x = 14 + 10 * FONT_WIDTH;
  uint16_t y = LCD_HEIGHT-(NUM_FONT_GET_HEIGHT+NUM_INPUT_HEIGHT)/2;
  uint16_t xsim;
#ifdef __USE_RTC__
  if ((1<<keypad_mode)&((1<<KM_RTC_DATE)|(1<<KM_RTC_TIME)))
    xsim = 0b01010100;
  else
#endif
    xsim = (0b00100100100100100 >>(2-(period_pos()%3)))&(~1);
  lcd_set_colors(LCD_INPUT_TEXT_COLOR, LCD_INPUT_BG_COLOR);
  while(*buf) {
    int c = *buf++;
         if (c == '.'){c = KP_PERIOD;xsim<<=4;}
    else if (c == '-'){c = KP_MINUS; xsim&=~3;}
    else if (c >= '0' && c <= '9') c-= '0';
    else continue;
    // Add space before char
    uint16_t space = xsim&1 ? 2 + 10 : 2;
    xsim>>=1;
    lcd_fill(x, y, space, NUM_FONT_GET_HEIGHT);
    x+=space;
    lcd_drawfont(c, x, y);
    x+=NUM_FONT_GET_WIDTH;
  }
  lcd_fill(x, y, NUM_FONT_GET_WIDTH+2+10, NUM_FONT_GET_HEIGHT);
}

static void
draw_text_input(const char *buf)
{
  lcd_set_colors(LCD_INPUT_TEXT_COLOR, LCD_INPUT_BG_COLOR);
#if 0
  uint16_t x = 14 + 5 * FONT_WIDTH;
  uint16_t y = LCD_HEIGHT-(FONT_GET_HEIGHT + NUM_INPUT_HEIGHT)/2;
  lcd_fill(x, y, FONT_WIDTH * 20, FONT_GET_HEIGHT);
  lcd_printf(x, y, buf);
#else
  int n = 2;
  uint16_t x = 14 + 5 * FONT_WIDTH;
  uint16_t y = LCD_HEIGHT-(FONT_GET_HEIGHT*n + NUM_INPUT_HEIGHT)/2;
  lcd_fill(x, y, FONT_WIDTH * 20 * n, FONT_GET_HEIGHT*n);
  lcd_drawstring_size(buf, x, y, n);
#endif
}

/*
 * Keyboard UI processing
 */
static int
num_keypad_click(int c, int kp_index) {
  if (c >= KP_k && c <= KP_PERCENT) {
    if (kp_index == 0)
      return KP_CANCEL;
    if (c >= KP_k && c <= KP_G) {        // Apply k, M, G input (add zeroes and shift . right)
      uint16_t scale = c - KP_k + 1;
      scale+= (scale<<1);
      int i = period_pos(); if (scale + i > NUMINPUT_LEN) scale = NUMINPUT_LEN - i;
      do {
        char v = kp_buf[i+1]; if (v == 0 || kp_buf[i] == 0) {v = '0'; kp_buf[i+2] = 0;}
        kp_buf[i+1] = kp_buf[i];
        kp_buf[i++] = v;
      } while (--scale);
    } else if (c >= KP_m && c <= KP_p) { // Apply m, u, n, p input (add format at end for atof function)
      const char prefix[] = {'m', 'u', 'n', 'p'};
      kp_buf[kp_index  ] = prefix[c - KP_m];
      kp_buf[kp_index+1] = 0;
    }
    return KP_DONE;
  }
#ifdef __USE_RTC__
  int maxlength = (1<<keypad_mode)&((1<<KM_RTC_DATE)|(1<<KM_RTC_TIME)) ? 6 : NUMINPUT_LEN;
#else
  int maxlength = NUMINPUT_LEN;
#endif
  if (c == KP_BS) {
    if (kp_index == 0) return KP_CANCEL;
      --kp_index;
  } else if (c == KP_MINUS) {
    int i;
    if (kp_buf[0] == '-') {for (i = 0; i < NUMINPUT_LEN; i++) kp_buf[i] = kp_buf[i+1]; --kp_index;}
    else                  {for (i = NUMINPUT_LEN; i > 0; i--) kp_buf[i] = kp_buf[i-1]; kp_buf[0] = '-'; if (kp_index < maxlength) ++kp_index;}
//    if (kp_index == 0)
//      kp_buf[kp_index++] = '-';
  } else if (kp_index < maxlength) {
    if (c <= KP_9)
      kp_buf[kp_index++] = '0' + c;
    else if (c == KP_PERIOD && kp_index == period_pos() && maxlength == NUMINPUT_LEN) // append period if there are no period and for num input (skip for date/time)
      kp_buf[kp_index++] = '.';
  }
  kp_buf[kp_index] = '\0';
  draw_numeric_input(kp_buf);
  return KP_CONTINUE;
}

static int
txt_keypad_click(int c, int kp_index)
{
  if (c == S_ENTER[0]) {  // Enter
    return kp_index == 0 ? KP_CANCEL : KP_DONE;
  }
  if (c == S_LARROW[0]) { // Backspace
    if (kp_index == 0)
      return KP_CANCEL;
    --kp_index;
  } else if (kp_index < TXTINPUT_LEN) { // any other text input
    kp_buf[kp_index++] = c;
  }
  kp_buf[kp_index] = '\0';
  draw_text_input(kp_buf);
  return KP_CONTINUE;
}

static void
ui_mode_keypad(int _keypad_mode)
{
  if (ui_mode == UI_KEYPAD)
    return;
  set_area_size(0, 0);
  // keypads array
  keypad_mode = _keypad_mode;
  keypads = keypad_type_list[keypads_mode_tbl[keypad_mode].keypad_type];
  selection = -1;
  kp_buf[0] = 0;
  ui_mode = UI_KEYPAD;
  draw_menu(-1);
  draw_keypad();
  draw_numeric_area_frame();
}

static void
keypad_click(int key) {
  int c = keypads[key+1].c;  // !!! Use key + 1 (zero key index used or size define)
  int index = strlen(kp_buf);
  int result = keypads[0].c == NUM_KEYBOARD ? num_keypad_click(c, index) : txt_keypad_click(c, index);
  if (result == KP_DONE) keypad_set_value(); // apply input done
  // Exit loop on done or cancel
  if (result != KP_CONTINUE)
    ui_mode_normal();
}

static void
ui_keypad_touch(int touch_x, int touch_y)
{
  const keypad_pos_t *p = &key_pos[keypads[0].c];
  if (touch_x < p->x_offs || touch_y < p->y_offs) return;
  // Calculate key position from touch x and y
  touch_x-= p->x_offs; touch_x/= p->width;
  touch_y-= p->y_offs; touch_y/= p->height;
  uint8_t pos = (touch_y & 0x0F) | (touch_x<<4);
  for (int i = 0; i < keypads[0].pos; i++) {
    if (keypads[i+1].pos != pos) continue;
    int old = selection;
    draw_keypad_button(selection = i);  // draw new focus
    draw_keypad_button(old);            // Erase old focus
    touch_wait_release();
    selection = -1;
    draw_keypad_button(i);              // erase new focus
    keypad_click(i);                    // Process input
    return;
  }
  return;
}

static void
ui_keypad_lever(uint16_t status)
{
  if (status == EVT_BUTTON_SINGLE_CLICK) {
    if (selection >= 0) // Process input
      keypad_click(selection);
    return;
  }
  int keypads_last_index = keypads[0].pos - 1;
  do {
    int old = selection;
    if ((status & EVT_DOWN) && --selection < 0) selection = keypads_last_index;
    if ((status & EVT_UP)   && ++selection > keypads_last_index) selection = 0;
    draw_keypad_button(old);
    draw_keypad_button(selection);
    chThdSleepMilliseconds(100);
  } while ((status = btn_wait_release()) != 0);
}
//========================== end keyboard input =======================

//
// UI Menu functions
//
static void
draw_menu_buttons(const menuitem_t *m, uint32_t mask)
{
  int i;
  int y = MENU_BUTTON_Y_OFFSET;
  for (i = 0; i < MENU_BUTTON_MAX && m; i++, m = menu_next_item(m), y+=menu_button_height) {
    if ((mask&(1<<i)) == 0) continue;
    button_t button;
    button.fg = LCD_MENU_TEXT_COLOR;
    button.icon = BUTTON_ICON_NONE;
    // focus only in MENU mode but not in KEYPAD mode
    if (ui_mode == UI_MENU && i == selection){
      button.bg = LCD_MENU_ACTIVE_COLOR;
      button.border = MENU_BUTTON_BORDER|BUTTON_BORDER_FALLING;
    } else{
      button.bg = LCD_MENU_COLOR;
      button.border = MENU_BUTTON_BORDER|BUTTON_BORDER_RISE;
    }
    // Custom button, apply custom settings/label from callback
    char *text;
    uint16_t text_offs;
    if (m->type == MT_ADV_CALLBACK) {
      button.label[0] = 0;
      if (m->reference) ((menuaction_acb_t)m->reference)(m->data, &button);
      // Apply custom text, from button label and
      if (button.label[0] == 0)
        plot_printf(button.label, sizeof(button.label), m->label, button.p1.u);
      text = button.label;
    }
    else
      text = m->label;
    // Draw button
    draw_button(LCD_WIDTH-MENU_BUTTON_WIDTH, y, MENU_BUTTON_WIDTH, menu_button_height, &button);
    // Draw icon if need (and add extra shift for text)
    if (button.icon >= 0) {
      lcd_blitBitmap(LCD_WIDTH-MENU_BUTTON_WIDTH+MENU_BUTTON_BORDER + MENU_ICON_OFFSET, y+(menu_button_height-ICON_HEIGHT)/2, ICON_WIDTH, ICON_HEIGHT, ICON_GET_DATA(button.icon));
      text_offs = LCD_WIDTH-MENU_BUTTON_WIDTH+MENU_BUTTON_BORDER + MENU_ICON_OFFSET + ICON_SIZE;
    } else
      text_offs = LCD_WIDTH-MENU_BUTTON_WIDTH+MENU_BUTTON_BORDER + MENU_TEXT_OFFSET;
    // Draw button text
    int lines = get_lines_count(text);
#if _USE_FONT_ != _USE_SMALL_FONT_
    if (menu_button_height < lines * FONT_GET_HEIGHT + 2) {
      lcd_set_font(FONT_SMALL);
      lcd_drawstring(text_offs, y+(menu_button_height - lines * sFONT_GET_HEIGHT - 1)/2, text);
    }
    else {
      lcd_set_font(FONT_NORMAL);
      lcd_drawstring(text_offs, y+(menu_button_height - lines * FONT_GET_HEIGHT)/2, text);
    }
#else
    lcd_drawstring(text_offs, y+(menu_button_height - lines * FONT_GET_HEIGHT)/2, text);
#endif
  }
  // Erase empty buttons
  if (AREA_HEIGHT_NORMAL + OFFSETY > y) {
    lcd_set_background(LCD_BG_COLOR);
    lcd_fill(LCD_WIDTH-MENU_BUTTON_WIDTH, y, MENU_BUTTON_WIDTH, AREA_HEIGHT_NORMAL + OFFSETY - y);
  }
  lcd_set_font(FONT_NORMAL);
}

static void
draw_menu(uint32_t mask)
{
  draw_menu_buttons(menu_stack[menu_current_level], mask);
}

#if 0
static void
erase_menu_buttons(void)
{
  lcd_set_background(LCD_BG_COLOR);
  lcd_fill(LCD_WIDTH-MENU_BUTTON_WIDTH, 0, MENU_BUTTON_WIDTH, MENU_BUTTON_HEIGHT*MENU_BUTTON_MAX);
}
#endif

//  Menu mode processing
static void
ui_mode_menu(void)
{
  if (ui_mode == UI_MENU)
    return;

  ui_mode = UI_MENU;
  // narrowen plotting area
  set_area_size(AREA_WIDTH_NORMAL - MENU_BUTTON_WIDTH, AREA_HEIGHT_NORMAL);
  ensure_selection();
  draw_menu(-1);
}

static void
ui_menu_lever(uint16_t status)
{
  uint16_t count = current_menu_get_count();
  if (status & EVT_BUTTON_SINGLE_CLICK) {
    if ((uint16_t)selection >= count)
      ui_mode_normal();
    else
      menu_invoke(selection);
    return;
  }

  do {
    uint32_t mask = 1<<selection;
    if (status & EVT_UP  ) selection++;
    if (status & EVT_DOWN) selection--;
    // close menu if no menu item
    if ((uint16_t)selection >= count){
      ui_mode_normal();
      return;
    }
    draw_menu(mask|(1<<selection));
    chThdSleepMilliseconds(100);
  } while ((status = btn_wait_release()) != 0);
}

static void
ui_menu_touch(int touch_x, int touch_y)
{
  if (LCD_WIDTH-MENU_BUTTON_WIDTH < touch_x) {
    int16_t i = (touch_y - MENU_BUTTON_Y_OFFSET) / menu_button_height;
    if ((uint16_t)i < (uint16_t)current_menu_get_count()) {
      uint32_t mask = (1<<i)|(1<<selection);
      selection = i;
      draw_menu(mask);
      touch_wait_release();
      selection = -1;
      menu_invoke(i);
      return;
    }
  }

  touch_wait_release();
  ui_mode_normal();
}
//================== end menu processing =================================

/*
 * Normal plot processing
 */
static void
ui_mode_normal(void)
{
  if (ui_mode == UI_NORMAL)
    return;
  set_area_size(AREA_WIDTH_NORMAL, AREA_HEIGHT_NORMAL);
  if (ui_mode == UI_MENU)
    request_to_draw_cells_behind_menu();
#ifdef __SD_FILE_BROWSER__
  if (ui_mode == UI_KEYPAD || ui_mode == UI_BROWSER)
    request_to_redraw(REDRAW_CLRSCR | REDRAW_AREA | REDRAW_BATTERY | REDRAW_CAL_STATUS | REDRAW_FREQUENCY);
#else
  if (ui_mode == UI_KEYPAD)
    request_to_redraw(REDRAW_CLRSCR | REDRAW_AREA | REDRAW_BATTERY | REDRAW_CAL_STATUS | REDRAW_FREQUENCY);
#endif
  ui_mode = UI_NORMAL;
}

#define MARKER_SPEEDUP  3
static void
lever_move_marker(uint16_t status)
{
  if (active_marker == MARKER_INVALID || !markers[active_marker].enabled) return;
  uint16_t step = 1<<MARKER_SPEEDUP;
  do {
    int idx = (int)markers[active_marker].index;
    if ((status & EVT_DOWN) && (idx-= step>>MARKER_SPEEDUP) <                0) idx = 0;
    if ((status & EVT_UP  ) && (idx+= step>>MARKER_SPEEDUP) > sweep_points - 1) idx = sweep_points-1;
    set_marker_index(active_marker, idx);
    redraw_marker(active_marker);
    step++;
  } while ((status = btn_wait_release()) != 0);
}

#ifdef UI_USE_LEVELER_SEARCH_MODE
static void
lever_search_marker(int status)
{
  if (active_marker == active_marker) return;
  if (status & EVT_DOWN)
    marker_search_dir(markers[active_marker].index, MK_SEARCH_LEFT);
  else if (status & EVT_UP)
    marker_search_dir(markers[active_marker].index, MK_SEARCH_RIGHT);
}
#endif

// ex. 10942 -> 10000
//      6791 ->  5000
//       341 ->   200
static freq_t
step_round(freq_t v) {
  // decade step
  freq_t nx, x = 1;
  while((nx = x*10) < v) x = nx;
  // 1-2-5 step
  if (x * 2 > v) return x;
  if (x * 5 > v) return x * 2;
  return x * 5;
}

static void
lever_frequency(uint16_t status) {
  uint16_t mode;
  freq_t freq;
  if (lever_mode == LM_FREQ_0) {
    if (FREQ_IS_STARTSTOP()) {mode = ST_START; freq = get_sweep_frequency(ST_START);}
    else                     {mode = ST_CENTER;freq = get_sweep_frequency(ST_CENTER);}
  } else {
    if (FREQ_IS_STARTSTOP()) {mode = ST_STOP;  freq = get_sweep_frequency(ST_STOP);}
    else                     {mode = ST_SPAN;  freq = get_sweep_frequency(ST_SPAN);}
  }
  if (mode == ST_SPAN && !var_freq) {
    if (status & EVT_UP  ) freq = step_round(freq*4 + 1);
    if (status & EVT_DOWN) freq = step_round(freq   - 1);
  } else {
    freq_t step = var_freq ? var_freq : step_round(get_sweep_frequency(ST_SPAN) / 4);
    if (status & EVT_UP  ) freq+= step;
    if (status & EVT_DOWN) freq-= step;
  }
  while (btn_wait_release() != 0);
  if (freq > FREQUENCY_MAX || freq < FREQUENCY_MIN) return;
  set_sweep_frequency(mode, freq);
}

#define STEPRATIO 0.2f
static void
lever_edelay(uint16_t status)
{
  float value = electrical_delay;
  if (current_props._var_delay == 0.0f) {
    float ratio = value > 0 ?  STEPRATIO : -STEPRATIO;
    if (status & EVT_UP  ) value*= (1.0f + ratio);
    if (status & EVT_DOWN) value*= (1.0f - ratio);
  } else {
    if (status & EVT_UP  ) value+= current_props._var_delay;
    if (status & EVT_DOWN) value-= current_props._var_delay;
  }
  set_electrical_delay(value);
  while (btn_wait_release() != 0);
}

static bool
touch_pickup_marker(int touch_x, int touch_y)
{
  touch_x -= OFFSETX;
  touch_y -= OFFSETY;
  int i = MARKER_INVALID, mt, m, t;
  int min_dist = MARKER_PICKUP_DISTANCE * MARKER_PICKUP_DISTANCE;
  // Search closest marker to touch position
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    for (m = 0; m < MARKERS_MAX; m++) {
      if (!markers[m].enabled)
        continue;
      // Get distance to marker from touch point
      int dist = distance_to_index(t, markers[m].index, touch_x, touch_y);
      if (dist < min_dist) {
        min_dist = dist;
        i  = m;
        mt = t;
      }
    }
  }
  // Marker not found
  if (i == MARKER_INVALID)
    return FALSE;
  // Marker found, set as active and start drag it
  if (active_marker != i) {
    previous_marker = active_marker;
    active_marker = i;
  }
  // Disable tracking
  props_mode&= ~TD_MARKER_TRACK;
  // Leveler mode = marker move
  select_lever_mode(LM_MARKER);
  // select trace
  set_active_trace(mt);
  // drag marker until release
  do {
    touch_position(&touch_x, &touch_y);
    int index = search_nearest_index(touch_x - OFFSETX, touch_y - OFFSETY, current_trace);
    if (index >= 0 && markers[active_marker].index != index) {
      set_marker_index(active_marker, index);
      redraw_marker(active_marker);
    } else
      chThdSleepMilliseconds(50); // Not check new position too fast
  } while (touch_check()!= EVT_TOUCH_RELEASED);
  return TRUE;
}

static bool
touch_lever_mode_select(int touch_x, int touch_y)
{
  int mode = -1;
  if (touch_y > HEIGHT && (props_mode & DOMAIN_MODE) == DOMAIN_FREQ) // Only for frequency domain
    mode = touch_x < FREQUENCIES_XPOS2 ? LM_FREQ_0 : LM_FREQ_1;
  if (touch_y < UI_MARKER_Y0)
    mode = (touch_x < (LCD_WIDTH / 2) && electrical_delay != 0.0) ? LM_EDELAY : LM_MARKER;
  if (mode == -1) return FALSE;

  touch_wait_release();
  // Check already selected
  if (select_lever_mode(mode)) return TRUE;
  // Call keyboard for enter
  switch(mode) {
    case LM_FREQ_0: ui_mode_keypad(FREQ_IS_CENTERSPAN() ? KM_CENTER : KM_START); break;
    case LM_FREQ_1: ui_mode_keypad(FREQ_IS_CENTERSPAN() ? KM_SPAN   : KM_STOP ); break;
    case LM_EDELAY: ui_mode_keypad(KM_EDELAY); break;
  }
  return TRUE;
}

static void
ui_normal_lever(uint16_t status)
{
  if (status & EVT_BUTTON_SINGLE_CLICK) {
    ui_mode_menu();
    return;
  }
  switch (lever_mode) {
    case LM_MARKER: lever_move_marker(status);   break;
#ifdef UI_USE_LEVELER_SEARCH_MODE
    case LM_SEARCH: lever_search_marker(status); break;
#endif
    case LM_FREQ_0:
    case LM_FREQ_1: lever_frequency(status); break;
    case LM_EDELAY: lever_edelay(status); break;
  }
}

static bool
touch_apply_ref_scale(int touch_x, int touch_y) {
  int t = current_trace;
  // do not scale invalid or smith chart
  if (t == TRACE_INVALID || trace[t].type == TRC_SMITH) return FALSE;
  if (touch_x < UI_SCALE_REF_X0 || touch_x > UI_SCALE_REF_X1 ||
      touch_y < OFFSETY     || touch_y > AREA_HEIGHT_NORMAL) return FALSE;
  float ref   = get_trace_refpos(t);
  float scale = get_trace_scale(t);

       if (touch_y < GRIDY*1*NGRIDY/4) ref+=0.5f;
  else if (touch_y < GRIDY*2*NGRIDY/4) {scale*=2.0f;ref=ref/2.0f - NGRIDY/4 + NGRIDY/2;}
  else if (touch_y < GRIDY*3*NGRIDY/4) {scale/=2.0f;ref=ref*2.0f - NGRIDY   + NGRIDY/2;}
  else                                 ref-=0.5f;

  set_trace_scale(t, scale);
  set_trace_refpos(t, ref);
  chThdSleepMilliseconds(200);
  return TRUE;
}

#ifdef __USE_SD_CARD__
static bool
touch_made_screenshot(int touch_x, int touch_y) {
  if (touch_y < HEIGHT || touch_x < FREQUENCIES_XPOS3 || touch_x > FREQUENCIES_XPOS2)
    return FALSE;
  touch_wait_release();
  menu_sdcard_cb(FMT_BMP_FILE);
  return TRUE;
}
#endif

static void
ui_normal_touch(int touch_x, int touch_y) {
  // Try drag marker
  if (touch_pickup_marker(touch_x, touch_y))
    return;
#ifdef __USE_SD_CARD__
  // Try made screenshot
  if (touch_made_screenshot(touch_x, touch_y))
    return;
#endif
  if (touch_apply_ref_scale(touch_x, touch_y))
    return;
  // Try select lever mode (top and bottom screen)
  if (touch_lever_mode_select(touch_x, touch_y))
    return;
  // default: switch menu mode after release
  touch_wait_release();
  ui_mode_menu();
}
//========================== end normal plot input =======================
static const struct {
  void (*button)(uint16_t status);
  void (*touch)(int touch_x, int touch_y);
} ui_handler[] = {
  [UI_NORMAL ] = {ui_normal_lever , ui_normal_touch},
  [UI_MENU   ] = {ui_menu_lever   , ui_menu_touch},
  [UI_KEYPAD ] = {ui_keypad_lever , ui_keypad_touch},
#ifdef __SD_FILE_BROWSER__
  [UI_BROWSER] = {ui_browser_lever, ui_browser_touch},
#endif
};

static void
ui_process_lever(void) {
  uint16_t status = btn_check();
  if (status)
    ui_handler[ui_mode].button(status);
}

static
void ui_process_touch(void)
{
  int touch_x, touch_y;
  int status = touch_check();
  if (status == EVT_TOUCH_PRESSED || status == EVT_TOUCH_DOWN) {
    touch_position(&touch_x, &touch_y);
    ui_handler[ui_mode].touch(touch_x, touch_y);
  }
}

void
ui_process(void)
{
//if (ui_mode >= UI_END) return; // for safe
  if (operation_requested&OP_LEVER)
    ui_process_lever();
  if (operation_requested&OP_TOUCH)
    ui_process_touch();

  touch_start_watchdog();
  operation_requested = OP_NONE;
}

void handle_button_interrupt(uint16_t channel) {
  (void)channel;
  operation_requested|= OP_LEVER;
  //cur_button = READ_PORT() & BUTTON_MASK;
}

//static systime_t t_time = 0;
// Triggered touch interrupt call
void handle_touch_interrupt(void)
{
  operation_requested|= OP_TOUCH;
//  systime_t n_time = chVTGetSystemTimeX();
//  shell_printf("%d\r\n", n_time - t_time);
//  t_time = n_time;
}

#if HAL_USE_EXT == TRUE // Use ChibiOS EXT code (need lot of flash ~1.5k)
static void handle_button_ext(EXTDriver *extp, expchannel_t channel)
{
  (void)extp;
  handle_button_interrupt((uint16_t)channel);
}

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, handle_button_ext}, // EXT1
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, handle_button_ext}, // EXT2
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, handle_button_ext}, // EXT3
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_DISABLED, NULL}
  }
};

static void init_EXT(void) {
  extStart(&EXTD1, &extcfg);
}
#else // Use custom EXT lib, allow save flash (but need fix for different CPU)
static void init_EXT(void) {
  // Activates the EXT driver 1.
  extStart();
  ext_channel_enable(1, EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOA);
  ext_channel_enable(2, EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOA);
  ext_channel_enable(3, EXT_CH_MODE_RISING_EDGE | EXT_MODE_GPIOA);
}
#endif

void
ui_init()
{
  adc_init();
  // Activates the EXT driver 1.
  init_EXT();
  // Init touch subsystem
  touch_init();
  // Set LCD display brightness
#ifdef  __LCD_BRIGHTNESS__
  lcd_setBrightness(config._brightness);
#endif
}
