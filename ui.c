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
#include "chprintf.h"
#include "nanovna.h"
#include "si5351.h"

#define NO_EVENT                    0
#define EVT_BUTTON_SINGLE_CLICK     0x01
#define EVT_BUTTON_DOUBLE_CLICK     0x02
#define EVT_BUTTON_DOWN_LONG        0x04
#define EVT_UP                  0x10
#define EVT_DOWN                0x20
#define EVT_REPEAT              0x40

#define BUTTON_DOWN_LONG_TICKS      MS2ST(500)   // 500ms
#define BUTTON_DOUBLE_TICKS         MS2ST(250)   // 250ms
#define BUTTON_REPEAT_TICKS         MS2ST( 30)   //  30ms
#define BUTTON_DEBOUNCE_TICKS       MS2ST( 20)   //  20ms

/* lever switch assignment */
#define BIT_UP1     3
#define BIT_PUSH    2
#define BIT_DOWN1   1

#define READ_PORT() palReadPort(GPIOA)
#define BUTTON_MASK 0b1111

static uint16_t last_button = 0b0000;
static systime_t last_button_down_ticks;
static systime_t last_button_repeat_ticks;

uint8_t operation_requested = OP_NONE;

static uint16_t menu_button_height = MENU_BUTTON_HEIGHT(MENU_BUTTON_MIN);

enum {
  UI_NORMAL, UI_MENU, UI_NUMERIC, UI_KEYPAD
};

// Keypad structures
// Enum for keypads_list
enum {
  KM_START = 0, KM_STOP, KM_CENTER, KM_SPAN, KM_CW, KM_VAR,
  KM_SCALE, KM_REFPOS, KM_EDELAY, KM_VELOCITY_FACTOR, KM_SCALEDELAY,
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
  KM_NONE
};

typedef struct {
  uint8_t x:4;
  uint8_t y:4;
  uint8_t c;
} keypads_t;

typedef struct {
  const keypads_t *keypad_type;
  const char *name;
} keypads_list;
// Max keyboard input length
#define NUMINPUT_LEN 12

static uint8_t ui_mode = UI_NORMAL;
static const keypads_t *keypads;
static uint8_t keypad_mode;
static char    kp_buf[NUMINPUT_LEN+2];
static int8_t  kp_index = 0;
static uint8_t menu_current_level = 0;
static int8_t  selection = -1;

// UI menu structure
// Type of menu item:
#define MT_NONE            0x00
#define MT_SUBMENU         0x01
#define MT_CALLBACK        0x02
#define MT_ADV_CALLBACK    0x03

// Set for custom label
#define MT_CUSTOM_LABEL    0

// Button definition (used in MT_ADV_CALLBACK for custom)
#define BUTTON_ICON_NONE            -1
#define BUTTON_ICON_NOCHECK          0
#define BUTTON_ICON_CHECK            1
#define BUTTON_ICON_GROUP            2
#define BUTTON_ICON_GROUP_CHECKED    3
#define BUTTON_ICON_CHECK_AUTO       4
#define BUTTON_ICON_CHECK_MANUAL     5

#define BUTTON_BORDER_NONE           0x00
#define BUTTON_BORDER_WIDTH_MASK     0x0F

// Define mask for draw border (if 1 use light color, if 0 dark)
#define BUTTON_BORDER_TYPE_MASK      0xF0
#define BUTTON_BORDER_TOP            0x10
#define BUTTON_BORDER_BOTTOM         0x20
#define BUTTON_BORDER_LEFT           0x40
#define BUTTON_BORDER_RIGHT          0x80

#define BUTTON_BORDER_FLAT           0x00
#define BUTTON_BORDER_RISE           (BUTTON_BORDER_TOP|BUTTON_BORDER_RIGHT)
#define BUTTON_BORDER_FALLING        (BUTTON_BORDER_BOTTOM|BUTTON_BORDER_LEFT)

typedef struct {
  uint16_t bg;
  uint16_t fg;
  uint8_t  border;
  int8_t   icon;
  union {
    int32_t  i;
    uint32_t u;
    float    f;
    const char *text;
  } p1;        // void data for label printf
  char label[32];
} button_t;

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

// Touch screen
#define EVT_TOUCH_NONE     0
#define EVT_TOUCH_DOWN     1
#define EVT_TOUCH_PRESSED  2
#define EVT_TOUCH_RELEASED 3

#define TOUCH_INTERRUPT_ENABLED   1
static uint8_t touch_status_flag = 0;
static uint8_t last_touch_status = EVT_TOUCH_NONE;
static int16_t last_touch_x;
static int16_t last_touch_y;

#define KP_CONTINUE        0
#define KP_DONE            1
#define KP_CANCEL          2

static void ui_mode_normal(void);
static void ui_mode_menu(void);
static void draw_menu(uint32_t mask);
static void ui_mode_keypad(int _keypad_mode);
static void touch_position(int *x, int *y);
static void menu_move_back(bool leave_ui);
static void menu_push_submenu(const menuitem_t *submenu);

void drawMessageBox(char *header, char *text, uint32_t delay);

#ifdef UI_USE_NUMERIC_INPUT
static void ui_mode_numeric(int _keypad_mode);
#endif

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
  uint16_t cur_button = READ_PORT() & BUTTON_MASK;
  // Detect only changed and pressed buttons
  uint16_t button_set = (last_button ^ cur_button) & cur_button;
  last_button_down_ticks = ticks;
  last_button = cur_button;

  if (button_set & (1<<BIT_PUSH))
    status |= EVT_BUTTON_SINGLE_CLICK;
  if (button_set & (1<<BIT_UP1))
    status |= EVT_UP;
  if (button_set & (1<<BIT_DOWN1))
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
    uint16_t cur_button = READ_PORT() & BUTTON_MASK;
    uint16_t changed = last_button ^ cur_button;
    if (dt >= BUTTON_DOWN_LONG_TICKS && (cur_button & (1<<BIT_PUSH)))
      return EVT_BUTTON_DOWN_LONG;
    if (changed & (1<<BIT_PUSH)) // release
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
      if (cur_button & (1<<BIT_DOWN1))
        status |= EVT_DOWN | EVT_REPEAT;
      if (cur_button & (1<<BIT_UP1))
        status |= EVT_UP | EVT_REPEAT;
      last_button_repeat_ticks = ticks + BUTTON_REPEAT_TICKS;
      return status;
    }
  }
}

#if 0
static void btn_wait(void){
  while (READ_PORT() & BUTTON_MASK) chThdSleepMilliseconds(10);
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
// ADC read count for measure X and Y (2^N count)
#define TOUCH_X_N 3
#define TOUCH_Y_N 3
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

//  chThdSleepMilliseconds(20);
  uint32_t v = 0, cnt = 1<<TOUCH_Y_N;
  do{v+=adc_single_read(ADC_TOUCH_Y);}while(--cnt);
  return v>>TOUCH_Y_N;
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

  uint32_t v = 0, cnt = 1<<TOUCH_X_N;
  do{v+=adc_single_read(ADC_TOUCH_X);}while(--cnt);
  return v>>TOUCH_X_N;
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
static const GPTConfig gpt3cfg = {
  20,     // 200Hz timer clock. 200/10 = 20Hz touch check
  NULL,   // Timer callback.
  0x0020, // CR2:MMS=02 to output TRGO
  0
};

//
// Touch init function init timer 3 trigger adc for check touch interrupt, and run measure
//
static void touch_init(void){
  // Prepare pin for measure touch event
  touch_prepare_sense();
  // Start touch interrupt, used timer_3 ADC check threshold:
  gptStart(&GPTD3, &gpt3cfg);         // Init timer 3
  gptStartContinuous(&GPTD3, 10);     // Start timer 3 vs timer 10 interval
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

#define CALIBRATION_OFFSET 16
#define TOUCH_MARK_W        9
#define TOUCH_MARK_H        9
#define TOUCH_MARK_X        4
#define TOUCH_MARK_Y        4
static const uint8_t touch_bitmap[]={
  _BMP16(0b0000100000000000),
  _BMP16(0b0100100100000000),
  _BMP16(0b0010101000000000),
  _BMP16(0b0000100000000000),
  _BMP16(0b1111011110000000),
  _BMP16(0b0000100000000000),
  _BMP16(0b0010101000000000),
  _BMP16(0b0100100100000000),
  _BMP16(0b0000100000000000),
};

static void getTouchPoint(uint16_t x, uint16_t y, const char *name, int16_t *data) {
  // Clear screen and ask for press
  lcd_set_foreground(LCD_FG_COLOR);
  lcd_set_background(LCD_BG_COLOR);
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
  getTouchPoint(x1, y1, "UPPER LEFT", &config._touch_cal[0]);
  getTouchPoint(x2, y2, "LOWER RIGHT", &config._touch_cal[2]);
}

void
touch_draw_test(void)
{
  int x0, y0;
  int x1, y1;
  lcd_set_foreground(LCD_FG_COLOR);
  lcd_set_background(LCD_BG_COLOR);
  lcd_clear_screen();
  lcd_drawstring(OFFSETX, LCD_HEIGHT - FONT_GET_HEIGHT, "TOUCH TEST: DRAG PANEL, PRESS BUTTON TO FINISH");

  while (1) {
    if (btn_check() & EVT_BUTTON_SINGLE_CLICK) break;
    if (touch_check() == EVT_TOUCH_PRESSED){
      touch_position(&x0, &y0);
      do {
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
  *x = tx;
  *y = ty;
}

static void
show_version(void)
{
  int x = 5, y = 5, i = 1;
  int str_height = FONT_STR_HEIGHT + 2;
  lcd_set_foreground(LCD_TRACE_2_COLOR);
  lcd_set_background(LCD_BG_COLOR);

  lcd_clear_screen();
  uint16_t shift = 0b00010010000;
  lcd_drawstring_size(BOARD_NAME, x , y, 3);
  lcd_set_foreground(LCD_FG_COLOR);
  y+=FONT_GET_HEIGHT*3+3-5;
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
  lcd_set_foreground(LCD_FG_COLOR);
  lcd_set_background(LCD_BG_COLOR);
  // leave a last message 
  lcd_clear_screen();
  lcd_drawstring(x, y, "DFU: Device Firmware Update Mode\n"
                       "To exit DFU mode, please reset device yourself.");
  // see __early_init in ./NANOVNA_STM32_F072/board.c
  *((unsigned long *)BOOT_FROM_SYTEM_MEMORY_MAGIC_ADDRESS) = BOOT_FROM_SYTEM_MEMORY_MAGIC;
  NVIC_SystemReset();
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
     [CAL_THRU] = {CALSTAT_THRU,  5},//Calibration screen does not show electrical delay
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

static UI_FUNCTION_ADV_CALLBACK(menu_cal_range_acb){
  (void)data;
  bool calibrated = cal_status & (CALSTAT_ES|CALSTAT_ER|CALSTAT_ET|CALSTAT_ED|CALSTAT_EX|CALSTAT_OPEN|CALSTAT_SHORT|CALSTAT_THRU);
  if (b){
    if (calibrated){
      b->bg = (cal_status&CALSTAT_INTERPOLATED) ? LCD_INTERP_CAL_COLOR : LCD_MENU_COLOR;
     plot_printf(b->label, sizeof(b->label), "CAL: %dp\n %.6F" S_Hz "\n %.6F" S_Hz, cal_sweep_points, (float)cal_frequency0, (float)cal_frequency1);
     }
  else
      plot_printf(b->label, sizeof(b->label), "RESET\nCAL RANGE");
    return;
  }
  // Reset range to calibration
  if (calibrated && (cal_status&CALSTAT_INTERPOLATED)){
    reset_sweep_frequency();
    set_power(cal_power);
  }
}

static UI_FUNCTION_ADV_CALLBACK(menu_cal_apply_acb)
{
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
      plot_printf(b->label, sizeof(b->label), "%.6F" S_Hz "\n%.6F" S_Hz, (float)p->_frequency0, (float)p->_frequency1, data);
    else
      plot_printf(b->label, sizeof(b->label), "Empty %d", data);
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
#ifdef __SD_CARD_LOAD__
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
    b->p1.u = data;
    return;
  }
  if (caldata_save(data) == 0) {
    menu_move_back(true);
    request_to_redraw(REDRAW_CAL_STATUS);
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

  if (trace[data].enabled) {
    if (data == current_trace) {
      trace[data].enabled = FALSE;          // disable if active trace is selected
      current_trace = TRACE_INVALID;        // invalidate current
      for (int i = 0; i < TRACES_MAX; i++)  // set first enabled as current trace
        if (trace[i].enabled) {current_trace = i; break;}
    } else {
      // make active selected trace
      current_trace = data;
    }
  } else {
    trace[data].enabled = TRUE;
    current_trace = data;
  }
  request_to_redraw(REDRAW_AREA);
}

extern const menuitem_t menu_marker_smith[];
static UI_FUNCTION_ADV_CALLBACK(menu_marker_smith_acb)
{
  if (b){
    b->icon = marker_smith_format == data ? BUTTON_ICON_GROUP_CHECKED : BUTTON_ICON_GROUP;
    b->p1.text = get_smith_format_names(data);
    return;
  }
  marker_smith_format = data;
  request_to_redraw(REDRAW_AREA | REDRAW_MARKER);
}

static UI_FUNCTION_ADV_CALLBACK(menu_format_acb)
{
  if (current_trace == TRACE_INVALID) return;
  if (b){
    if (trace[current_trace].type == data)
      b->icon = BUTTON_ICON_CHECK;
    const char *name = get_trace_typename(data);
    if (data == TRC_SMITH)
      plot_printf(b->label, sizeof(b->label), "%s\n" R_LINK_COLOR " %s", name, get_smith_format_names(marker_smith_format));
    else
      b->p1.text = name;
    return;
  }
  if (trace[current_trace].type == data && data == TRC_SMITH)
    menu_push_submenu(menu_marker_smith);
  else
    set_trace_type(current_trace, data);
//  ui_mode_normal();
}

#if 0
static UI_FUNCTION_ADV_CALLBACK(menu_channel_acb)
{
  if (current_trace == TRACE_INVALID) return;
  if (b){
    if (trace[current_trace].channel == data)
      b->icon = BUTTON_ICON_CHECK;
    return;
  }
  set_trace_channel(current_trace, data);
  menu_move_back(true);
}
#endif

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
}

static UI_FUNCTION_ADV_CALLBACK(menu_transform_filter_acb)
{
  if(b){
    b->icon = (props_mode & TD_FUNC) == data ? BUTTON_ICON_GROUP_CHECKED : BUTTON_ICON_GROUP;
    return;
  }
  props_mode = (props_mode & ~TD_FUNC) | data;
//  ui_mode_normal();
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

#ifdef __USE_SMOOTH__
static UI_FUNCTION_ADV_CALLBACK(menu_smooth_func_acb)
{
  (void)data;
  if (b){
    b->p1.text = (VNA_mode & VNA_SMOOTH_FUNCTION) ? "Arith" : "Geom";
    return;
  }
  VNA_mode^= VNA_SMOOTH_FUNCTION;
}

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

#ifdef __USE_BACKUP__
static UI_FUNCTION_ADV_CALLBACK(menu_backup_acb)
{
  (void)data;
  if (b){
    b->icon = (VNA_mode & VNA_MODE_BACKUP) ? BUTTON_ICON_CHECK : BUTTON_ICON_NOCHECK;
    return;
  }
  VNA_mode^= VNA_MODE_BACKUP;
  request_to_redraw(REDRAW_BACKUP);
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
    if (current_props._power == SI5351_CLK_DRIVE_STRENGTH_AUTO)
      plot_printf(b->label, sizeof(b->label), "POWER  AUTO");
    else
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

static UI_FUNCTION_ADV_CALLBACK(menu_keyboard_acb)
{
  if (data == KM_SCALE && current_trace != TRACE_INVALID) {
    if ((1<<trace[current_trace].type) & ((1<<TRC_DELAY)|(1<<TRC_sC)|(1<<TRC_sL)|(1<<TRC_pC)|(1<<TRC_pL)))
      data = KM_SCALEDELAY;
  }
  if (b){
    switch(data){
//    case KM_SCALE:           b->p1.f = current_trace != TRACE_INVALID ? get_trace_scale(current_trace) : 0; break;
      case KM_VELOCITY_FACTOR: b->p1.u = velocity_factor; break;
      case KM_VAR:             plot_printf(b->label, sizeof(b->label), var_freq ? "JOG STEP\n" R_LINK_COLOR " %.3q" S_Hz : "JOG STEP\n AUTO", var_freq); break;
      case KM_XTAL:            b->p1.u = config._xtal_freq; break;
      case KM_THRESHOLD:       b->p1.u = config._harmonic_freq_threshold; break;
      case KM_VBAT:            b->p1.u = config._vbat_offset; break;
      case KM_EDELAY:          b->p1.f = electrical_delay * 1E-12; break;
#ifdef __S21_MEASURE__
      case KM_MEASURE_R:       b->p1.f = config._measure_r; break;
#endif
#ifdef __VNA_Z_RENORMALIZATION__
      case KM_Z_PORT:          b->p1.f = current_props._portz; break;
#endif
    }
    return;
  }
#ifdef UI_USE_NUMERIC_INPUT
  if (btn_wait_release() & EVT_BUTTON_DOWN_LONG) {
    ui_mode_numeric(data);
    return;
  }
#endif
  ui_mode_keypad(data);
}

#ifdef __USE_GRID_VALUES__
static UI_FUNCTION_ADV_CALLBACK(menu_grid_acb)
{
  if (b){
    b->icon = VNA_mode & data ? BUTTON_ICON_CHECK : BUTTON_ICON_NOCHECK;
    return;
  }
  VNA_mode^=data;
  request_to_redraw(REDRAW_AREA);
}
#endif

static UI_FUNCTION_ADV_CALLBACK(menu_pause_acb)
{
  (void)data;
  if (b){
    b->p1.text = sweep_mode&SWEEP_ENABLE ? "PAUSE" : "RESUME";
    b->icon = sweep_mode&SWEEP_ENABLE ? BUTTON_ICON_NOCHECK : BUTTON_ICON_CHECK;
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
    {
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
    }
    break;
  case UI_MARKER_EDELAY:
    { 
      if (current_trace == TRACE_INVALID)
        break;
      float (*array)[2] = measured[trace[current_trace].channel];
      int index = markers[active_marker].index;
      float v = groupdelay_from_array(index, array[index]);
      set_electrical_delay(electrical_delay + (v / 1e-12));
    }
    break;
  }
  ui_mode_normal();
}

static UI_FUNCTION_ADV_CALLBACK(menu_marker_search_mode_acb)
{
  (void)data;
  if (b){
    b->p1.text = (VNA_mode & VNA_MODE_SEARCH_MIN) ? "MINIMUM" : "MAXIMUM";
    return;
  }
  VNA_mode^= VNA_MODE_SEARCH_MIN;
  marker_search(true);
#ifdef UI_USE_LEVELER_SEARCH_MODE
  select_lever_mode(LM_SEARCH);
#endif
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
static UI_FUNCTION_ADV_CALLBACK(menu_measure_acb)
{
  (void)data;
  if (b){
    b->icon = current_props._measure == data ? BUTTON_ICON_GROUP_CHECKED : BUTTON_ICON_GROUP;
    return;
  }
  plot_set_measure_mode(data);
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
  if (b){
    if (data < MARKERS_MAX){
           if (data == active_marker) b->icon = BUTTON_ICON_CHECK_AUTO;
      else if (markers[data].enabled) b->icon = BUTTON_ICON_CHECK;
      b->p1.u = data + 1;
    }
    return;
  }
  // Marker select click
  if (data < MARKERS_MAX) {
    int mk = data;
    if (markers[mk].enabled) {            // Marker enabled
      if (mk == active_marker) {          // If active marker:
        markers[mk].enabled = FALSE;      //  disable it
        mk = previous_marker;             //  set select from previous marker
        active_marker = MARKER_INVALID;   //  invalidate active
      }
    } else {
      markers[mk].enabled = TRUE;         // Enable marker

    }
    previous_marker = active_marker;      // set previous marker as current active
    active_marker = mk;                   // set new active marker
    active_marker_check();
  }
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
  request_to_redraw(REDRAW_MARKER);
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
  config._serial_speed = speed;
  shell_update_speed();
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


static UI_FUNCTION_ADV_CALLBACK(menu_connection_acb)
{
  if (b){
    b->icon = (VNA_mode & VNA_MODE_CONNECTION_MASK) == data ? BUTTON_ICON_GROUP_CHECKED : BUTTON_ICON_GROUP;
    return;
  }
  VNA_mode = (VNA_mode & ~VNA_MODE_CONNECTION_MASK) | data;
  shell_reset_console();
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
static UI_FUNCTION_ADV_CALLBACK(menu_brightness_acb)
{
  (void)data;
  if (b){
    b->p1.u = config._brightness;
    return;
  }
  int16_t value = config._brightness;
  lcd_set_foreground(LCD_MENU_TEXT_COLOR);
  lcd_set_background(LCD_MENU_COLOR);
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
      } while ((status = btn_wait_release()) != 0);
    }
    if (status == EVT_BUTTON_SINGLE_CLICK)
      break;
  }
  config._brightness = (uint8_t)value;
  lcd_setBrightness(value);
  request_to_redraw(REDRAW_AREA);
  ui_mode_normal();
}
#endif

#ifdef __USE_SD_CARD__
// Save format enum
enum {
  SAVE_S1P_FILE=0, SAVE_S2P_FILE, SAVE_BMP_FILE,
#ifdef __SD_CARD_DUMP_FIRMWARE__
  SAVE_BIN_FILE
#endif
};

// Save file extension
static const char *file_ext[] = {
  [SAVE_S1P_FILE] = "s1p",
  [SAVE_S2P_FILE] = "s2p",
  [SAVE_BMP_FILE] = "bmp",
#ifdef __SD_CARD_DUMP_FIRMWARE__
  [SAVE_BIN_FILE] = "bin",
#endif
};

//*******************************************************************************************
// S1P and S2P file headers, and data structures
//*******************************************************************************************
static const char s1_file_header[] =
  "!File created by NanoVNA\r\n"\
  "# Hz S RI R 50\r\n";

static const char s1_file_param[] =
  "%10u % f % f\r\n";

static const char s2_file_header[] =
  "!File created by NanoVNA\r\n"\
  "# Hz S RI R 50\r\n";

static const char s2_file_param[] =
  "%10u % f % f % f % f 0 0 0 0\r\n";

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

// Create file name from current time
static FRESULT vna_create_file(char *fs_filename, const char *ext)
{
//  shell_printf("S file\r\n");
  FRESULT res = f_mount(fs_volume, "", 1);
//  shell_printf("Mount = %d\r\n", res);
  if (res != FR_OK)
    return res;
  // Prepare filename and open for write
#if FF_USE_LFN >= 1
  uint32_t tr = rtc_get_tr_bcd(); // TR read first
  uint32_t dr = rtc_get_dr_bcd(); // DR read second
  plot_printf(fs_filename, FF_LFN_BUF, "VNA_%06x_%06x.%s", dr, tr, ext);
#else
  plot_printf(fs_filename, FF_LFN_BUF, "%08x.%s", rtc_get_FAT(), ext);
#endif
  res = f_open(fs_file, fs_filename, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
  //  shell_printf("Open %s, = %d\r\n", fs_filename, res);
  return res;
}

static UI_FUNCTION_CALLBACK(menu_sdcard_cb)
{
  char *buf_8;
  uint16_t *buf_16;
  char filename[32];
  int i, y;
  UINT size;
  // For screenshot need back to normal mode and redraw screen before capture!!
  // Redraw use spi_buffer so need do it before any file ops
  if (data == SAVE_BMP_FILE && ui_mode != UI_NORMAL){
    ui_mode_normal();
    draw_all(true);
  }
//  UINT total_size = 0;
//  systime_t time = chVTGetSystemTimeX();
  // Prepare filename = .s1p / .s2p / .bmp and open for write
  FRESULT res = vna_create_file(filename, file_ext[data]);
  if (res == FR_OK){
    const char *s_file_format;
    switch(data) {
      /*
       *  Save touchstone file for VNA (use rev 1.1 format)
       *  https://en.wikipedia.org/wiki/Touchstone_file
       */
      case SAVE_S1P_FILE:
      case SAVE_S2P_FILE:
      buf_8  = (char *)spi_buffer;
      // Write SxP file
      if (data == SAVE_S1P_FILE){
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
      case SAVE_BMP_FILE:
      buf_16 = spi_buffer;
      res = f_write(fs_file, bmp_header_v4, BMP_HEAD_SIZE, &size); // Write header struct
//      total_size+=size;
      for (y = LCD_HEIGHT-1; y >= 0 && res == FR_OK; y--) {
        lcd_read_memory(0, y, LCD_WIDTH, 1, buf_16);
        for (i = 0; i < LCD_WIDTH; i++)
          buf_16[i] = __REVSH(buf_16[i]); // swap byte order (example 0x10FF to 0xFF10)
        res = f_write(fs_file, buf_16, LCD_WIDTH*sizeof(uint16_t), &size);
//        total_size+=size;
      }
      break;
#ifdef __SD_CARD_DUMP_FIRMWARE__
      /*
       * Dump firmware to SD card as bin file image
       */
      case SAVE_BIN_FILE:
      {
        const char *src = (const char*)FLASH_START_ADDRESS;
        for (i = 0; i < FLASH_TOTAL_SIZE && res == FR_OK; i+=512)
          res = f_write(fs_file, &src[i], 512, &size);
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

  drawMessageBox("SD CARD SAVE", res == FR_OK ? filename : "  Fail write  ", 2000);
  request_to_redraw(REDRAW_AREA);
  ui_mode_normal();
}
#endif

#ifdef __BAND_MODE__
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

#endif

#ifdef __DIGIT_SEPARATOR__
static UI_FUNCTION_ADV_CALLBACK(menu_separator_acb)
{
  (void)data;
  if (b){
    b->p1.text = DIGIT_SEPARATOR == '.' ? " DOT '.'" : " COMMA ','";
    return;
  }
  DIGIT_SEPARATOR = DIGIT_SEPARATOR == '.' ? ',' : '.';
}
#endif

#if STORED_TRACES > 0
static UI_FUNCTION_CALLBACK(menu_stored_trace_cb)
{
  if (data & 1)
    disableStoredTrace(data>>1);
  else
    storeCurrentTrace(data>>1);
}
#endif

static UI_FUNCTION_CALLBACK(menu_back_cb)
{
  (void)data;
  menu_move_back(false);
}

// Back button submenu list
static const menuitem_t menu_back[] = {
  { MT_CALLBACK,   0, S_LARROW" BACK", menu_back_cb },
  { MT_NONE,     0, NULL, NULL } // sentinel
};

#ifdef __USE_SD_CARD__
static const menuitem_t menu_sdcard[] = {
  { MT_CALLBACK, SAVE_S1P_FILE, "SAVE S1P",   menu_sdcard_cb },
  { MT_CALLBACK, SAVE_S2P_FILE, "SAVE S2P",   menu_sdcard_cb },
  { MT_CALLBACK, SAVE_BMP_FILE, "SCREENSHOT", menu_sdcard_cb },
  { MT_NONE,     0, NULL, menu_back } // next-> menu_back
};
#endif

static const menuitem_t menu_calop[] = {
  { MT_ADV_CALLBACK, CAL_OPEN,  "OPEN",  menu_calop_acb },
  { MT_ADV_CALLBACK, CAL_SHORT, "SHORT", menu_calop_acb },
  { MT_ADV_CALLBACK, CAL_LOAD,  "LOAD",  menu_calop_acb },
  { MT_ADV_CALLBACK, CAL_ISOLN, "ISOLN", menu_calop_acb },
  { MT_ADV_CALLBACK, CAL_THRU,  "THRU",  menu_calop_acb },
  // { MT_ADV_CALLBACK, KM_EDELAY, "E-DELAY\n" R_LINK_COLOR " %b.7F" S_SECOND, menu_keyboard_acb },
  { MT_CALLBACK, 0,             "DONE",  menu_caldone_cb },
  { MT_CALLBACK, 1,             "DONE IN RAM",  menu_caldone_cb },
  { MT_NONE,     0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_save[] = {
  { MT_ADV_CALLBACK, 0, "SAVE %d", menu_save_acb },
  { MT_ADV_CALLBACK, 1, "SAVE %d", menu_save_acb },
  { MT_ADV_CALLBACK, 2, "SAVE %d", menu_save_acb },
#if SAVEAREA_MAX > 3
  { MT_ADV_CALLBACK, 3, "SAVE %d", menu_save_acb },
#endif
#if SAVEAREA_MAX > 4
  { MT_ADV_CALLBACK, 4, "SAVE %d", menu_save_acb },
#endif
#if SAVEAREA_MAX > 5
  { MT_ADV_CALLBACK, 5, "SAVE %d", menu_save_acb },
#endif
#if SAVEAREA_MAX > 6
  { MT_ADV_CALLBACK, 6, "SAVE %d", menu_save_acb },
#endif
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_recall[] = {
  { MT_ADV_CALLBACK, 0, MT_CUSTOM_LABEL, menu_recall_acb },
  { MT_ADV_CALLBACK, 1, MT_CUSTOM_LABEL, menu_recall_acb },
  { MT_ADV_CALLBACK, 2, MT_CUSTOM_LABEL, menu_recall_acb },
#if SAVEAREA_MAX > 3
  { MT_ADV_CALLBACK, 3, MT_CUSTOM_LABEL, menu_recall_acb },
#endif
#if SAVEAREA_MAX > 4
  { MT_ADV_CALLBACK, 4, MT_CUSTOM_LABEL, menu_recall_acb },
#endif
#if SAVEAREA_MAX > 5
  { MT_ADV_CALLBACK, 5, MT_CUSTOM_LABEL, menu_recall_acb },
#endif
#if SAVEAREA_MAX > 6
  { MT_ADV_CALLBACK, 6, MT_CUSTOM_LABEL, menu_recall_acb },
#endif
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_power[] = {
  { MT_ADV_CALLBACK, SI5351_CLK_DRIVE_STRENGTH_AUTO, "AUTO",  menu_power_acb },
  { MT_ADV_CALLBACK, SI5351_CLK_DRIVE_STRENGTH_2MA, "%u m" S_AMPER, menu_power_acb },
  { MT_ADV_CALLBACK, SI5351_CLK_DRIVE_STRENGTH_4MA, "%u m" S_AMPER, menu_power_acb },
  { MT_ADV_CALLBACK, SI5351_CLK_DRIVE_STRENGTH_6MA, "%u m" S_AMPER, menu_power_acb },
  { MT_ADV_CALLBACK, SI5351_CLK_DRIVE_STRENGTH_8MA, "%u m" S_AMPER, menu_power_acb },
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_cal[] = {
  { MT_SUBMENU,      0, "CALIBRATE",     menu_calop },
  { MT_ADV_CALLBACK, 0, MT_CUSTOM_LABEL, menu_power_sel_acb },
  { MT_SUBMENU,      0, "SAVE",          menu_save },
  { MT_ADV_CALLBACK, 0, MT_CUSTOM_LABEL, menu_cal_range_acb },
  { MT_CALLBACK,     0, "RESET",         menu_cal_reset_cb },
  { MT_ADV_CALLBACK, 0, "APPLY",         menu_cal_apply_acb },
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_trace[] = {
  { MT_ADV_CALLBACK, 0, "TRACE %d", menu_trace_acb },
  { MT_ADV_CALLBACK, 1, "TRACE %d", menu_trace_acb },
  { MT_ADV_CALLBACK, 2, "TRACE %d", menu_trace_acb },
  { MT_ADV_CALLBACK, 3, "TRACE %d", menu_trace_acb },
#if STORED_TRACES > 0
  { MT_CALLBACK,     0, "STORE TRACE", menu_stored_trace_cb},
  { MT_CALLBACK,     1, "CLEAN STORE", menu_stored_trace_cb},
#endif
#if STORED_TRACES > 1
  { MT_CALLBACK,     2, "STORE TRACE 1", menu_stored_trace_cb},
  { MT_CALLBACK,     3, "CLEAN STORE 1", menu_stored_trace_cb},
#endif
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_format3[] = {
{ MT_ADV_CALLBACK, TRC_sC,     "SERIES C",   menu_format_acb },
{ MT_ADV_CALLBACK, TRC_sL,     "SERIES L",   menu_format_acb },
{ MT_ADV_CALLBACK, TRC_Rp,     "PARALLEL R", menu_format_acb },
{ MT_ADV_CALLBACK, TRC_Xp,     "PARALLEL X", menu_format_acb },
{ MT_ADV_CALLBACK, TRC_pC,     "PARALLEL C", menu_format_acb },
{ MT_ADV_CALLBACK, TRC_pL,     "PARALLEL L", menu_format_acb },
{ MT_NONE, 0, NULL, menu_back } // next-> menu_back
};
const menuitem_t menu_format2[] = {
  { MT_ADV_CALLBACK, TRC_POLAR,  "POLAR", menu_format_acb },
  { MT_ADV_CALLBACK, TRC_LINEAR, "LINEAR", menu_format_acb },
  { MT_ADV_CALLBACK, TRC_REAL,   "REAL", menu_format_acb },
  { MT_ADV_CALLBACK, TRC_IMAG,   "IMAG", menu_format_acb },
  { MT_ADV_CALLBACK, TRC_Q,      "Q FACTOR", menu_format_acb },
  { MT_ADV_CALLBACK, TRC_G,      "CONDUCT",  menu_format_acb },
  { MT_ADV_CALLBACK, TRC_B,      "SUSCEPT",  menu_format_acb },
  { MT_ADV_CALLBACK, TRC_Y,      "|Y|",      menu_format_acb },
  { MT_SUBMENU,         0, S_RARROW " MORE", menu_format3 },
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_format[] = {
  { MT_ADV_CALLBACK, TRC_LOGMAG, "LOGMAG",        menu_format_acb },
  { MT_ADV_CALLBACK, TRC_PHASE,  "PHASE",         menu_format_acb },
  { MT_ADV_CALLBACK, TRC_DELAY,  "DELAY",         menu_format_acb },
  { MT_ADV_CALLBACK, TRC_SMITH,  MT_CUSTOM_LABEL, menu_format_acb },
  { MT_ADV_CALLBACK, TRC_SWR,    "SWR",           menu_format_acb },
  { MT_ADV_CALLBACK, TRC_R,      "RESISTANCE",    menu_format_acb },
  { MT_ADV_CALLBACK, TRC_X,      "REACTANCE",     menu_format_acb },
  { MT_ADV_CALLBACK, TRC_Z,      "|Z|",           menu_format_acb },
  { MT_SUBMENU,          0, S_RARROW " MORE",     menu_format2 },
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_scale[] = {
  { MT_ADV_CALLBACK, KM_SCALE,  "SCALE/DIV",           menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_REFPOS, "REFERENCE\nPOSITION", menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_EDELAY, "E-DELAY\n" R_LINK_COLOR " %b.7F" S_SECOND, menu_keyboard_acb },
#ifdef __USE_GRID_VALUES__
  { MT_ADV_CALLBACK, VNA_MODE_SHOW_GRID, "SHOW GRID\nVALUES", menu_grid_acb },
  { MT_ADV_CALLBACK, VNA_MODE_DOT_GRID , "DOT GRID",          menu_grid_acb },
#endif
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};

#if 0
const menuitem_t menu_channel[] = {
  { MT_ADV_CALLBACK, 0, "S11\nREFLECT", menu_channel_acb },
  { MT_ADV_CALLBACK, 1, "S21\nTHROUGH", menu_channel_acb },
  { MT_CANCEL, 0, S_LARROW" BACK", NULL },
  { MT_NONE, 0, NULL, NULL } // sentinel
};
#endif

const menuitem_t menu_transform[] = {
  { MT_ADV_CALLBACK, 0,                       "TRANSFORM\n%s",      menu_transform_acb },
  { MT_ADV_CALLBACK, TD_FUNC_LOWPASS_IMPULSE, "LOW PASS\nIMPULSE",  menu_transform_filter_acb },
  { MT_ADV_CALLBACK, TD_FUNC_LOWPASS_STEP,    "LOW PASS\nSTEP",     menu_transform_filter_acb },
  { MT_ADV_CALLBACK, TD_FUNC_BANDPASS,        "BANDPASS",           menu_transform_filter_acb },
  { MT_ADV_CALLBACK, 0,                       "WINDOW\n" R_LINK_COLOR " %s", menu_transform_window_acb },
  { MT_ADV_CALLBACK, KM_VELOCITY_FACTOR, "VELOCITY\nFACTOR" R_LINK_COLOR " %d%%%%", menu_keyboard_acb },
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
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
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};

#ifdef __USE_SMOOTH__
const menuitem_t menu_smooth_count[] = {
  { MT_ADV_CALLBACK, 0, "SMOOTH\n" R_LINK_COLOR "%s avg",menu_smooth_func_acb },
  { MT_ADV_CALLBACK, 0, "SMOOTH\nOFF",menu_smooth_acb },
  { MT_ADV_CALLBACK, 1, "x%d", menu_smooth_acb },
  { MT_ADV_CALLBACK, 2, "x%d", menu_smooth_acb },
  { MT_ADV_CALLBACK, 4, "x%d", menu_smooth_acb },
  { MT_ADV_CALLBACK, 5, "x%d", menu_smooth_acb },
  { MT_ADV_CALLBACK, 6, "x%d", menu_smooth_acb },
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};
#endif

const menuitem_t menu_display[] = {
  { MT_SUBMENU,      0, "TRACE",                            menu_trace },
  { MT_SUBMENU,      0, "FORMAT",                           menu_format },
  { MT_SUBMENU,      0, "SCALE",                            menu_scale },
  { MT_ADV_CALLBACK, 0, "CHANNEL\n" R_LINK_COLOR " %s",     menu_channel_acb },
  { MT_SUBMENU,      0, "TRANSFORM",                        menu_transform },
  { MT_ADV_CALLBACK, 0, "BANDWIDTH\n" R_LINK_COLOR " %u" S_Hz, menu_bandwidth_sel_acb },
#ifdef __USE_SMOOTH__
  { MT_SUBMENU,      0, "DATA SMOOTH",                      menu_smooth_count },
#endif
#ifdef __VNA_Z_RENORMALIZATION__
  { MT_ADV_CALLBACK, KM_Z_PORT, "PORT-Z\n" R_LINK_COLOR " 50 " S_RARROW " %bF" S_OHM, menu_keyboard_acb},
#endif
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_sweep_points[] = {
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
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_stimulus[] = {
  { MT_ADV_CALLBACK, KM_START,  "START",         menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_STOP,   "STOP",          menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_CENTER, "CENTER",        menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_SPAN,   "SPAN",          menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_CW,     "CW FREQ",       menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_VAR,    MT_CUSTOM_LABEL, menu_keyboard_acb },
  { MT_ADV_CALLBACK,      0,    "SWEEP POINTS\n" R_LINK_COLOR " %u",  menu_points_sel_acb },
  { MT_ADV_CALLBACK, 0, "%s\nSWEEP", menu_pause_acb },
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
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
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_marker_ops[] = {
  { MT_CALLBACK, ST_START,         S_RARROW" START",   menu_marker_op_cb },
  { MT_CALLBACK, ST_STOP,          S_RARROW" STOP",    menu_marker_op_cb },
  { MT_CALLBACK, ST_CENTER,        S_RARROW" CENTER",  menu_marker_op_cb },
  { MT_CALLBACK, ST_SPAN,          S_RARROW" SPAN",    menu_marker_op_cb },
  { MT_CALLBACK, UI_MARKER_EDELAY, S_RARROW" E-DELAY", menu_marker_op_cb },
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};


const menuitem_t menu_marker_smith[] = {
  { MT_ADV_CALLBACK, MS_LIN, "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_LOG, "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_REIM,"%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_RX,  "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_RLC, "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_GB,  "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_GLC, "%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_RpXp,"%s", menu_marker_smith_acb },
  { MT_ADV_CALLBACK, MS_RpLC,"%s", menu_marker_smith_acb },
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};

#ifdef __VNA_MEASURE_MODULE__
const menuitem_t menu_marker_measure[] = {
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
  { MT_ADV_CALLBACK, KM_MEASURE_R,        "MEASURE\n Rl =" R_LINK_COLOR " %b.4F"S_OHM, menu_keyboard_acb},
#endif
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};
#endif

const menuitem_t menu_marker[] = {
  { MT_SUBMENU,                0, "SELECT\nMARKER",              menu_marker_sel    },
  { MT_ADV_CALLBACK,           0, "SEARCH\n" R_LINK_COLOR " %s", menu_marker_search_mode_acb },
  { MT_CALLBACK, MK_SEARCH_LEFT,  "SEARCH\n " S_LARROW" LEFT",   menu_marker_search_dir_cb },
  { MT_CALLBACK, MK_SEARCH_RIGHT, "SEARCH\n " S_RARROW" RIGHT",  menu_marker_search_dir_cb },
  { MT_SUBMENU,                0, "OPERATIONS",                  menu_marker_ops    },
  { MT_ADV_CALLBACK,           0, "TRACKING",                    menu_marker_tracking_acb },
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};

#ifdef __DFU_SOFTWARE_MODE__
const menuitem_t menu_dfu[] = {
  { MT_CALLBACK, 0, "RESET AND\nENTER DFU", menu_dfu_cb },
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
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
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_connection[] = {
  { MT_ADV_CALLBACK, VNA_MODE_USB,    "USB",          menu_connection_acb },
  { MT_ADV_CALLBACK, VNA_MODE_SERIAL, "SERIAL",       menu_connection_acb },
  { MT_ADV_CALLBACK,               0, "SERIAL SPEED\n " R_LINK_COLOR "%u", menu_serial_speed_sel_acb },
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};
#endif

const menuitem_t menu_clear[] = {
  { MT_CALLBACK, MENU_CONFIG_RESET, "CLEAR ALL\nAND RESET", menu_config_cb },
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
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
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};
#endif

const menuitem_t menu_device1[] = {
  { MT_ADV_CALLBACK, 0,            "MODE\n" R_LINK_COLOR " %s",          menu_band_sel_acb },
#ifdef __DIGIT_SEPARATOR__
  { MT_ADV_CALLBACK, 0,            "SEPARATOR\n" R_LINK_COLOR "%s",      menu_separator_acb },
#endif
#ifdef __SD_CARD_DUMP_FIRMWARE__
  { MT_CALLBACK, SAVE_BIN_FILE,    "DUMP\nFIRMWARE",     menu_sdcard_cb },
#endif
#ifdef __SD_CARD_LOAD__
  { MT_CALLBACK, MENU_CONFIG_LOAD, "LOAD\nCONFIG.INI",   menu_config_cb },
#endif
  { MT_SUBMENU, 0,                 "CLEAR CONFIG",       menu_clear },
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};
const menuitem_t menu_device[] = {
  { MT_ADV_CALLBACK, KM_THRESHOLD, "THRESHOLD\n" R_LINK_COLOR " %.6q",         menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_XTAL,      "TCXO\n" R_LINK_COLOR " %.6q",              menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_VBAT,      "VBAT OFFSET\n" R_LINK_COLOR " %um" S_VOLT, menu_keyboard_acb },
#ifdef USE_VARIABLE_OFFSET_MENU
  { MT_ADV_CALLBACK, 0,            "IF OFFSET\n" R_LINK_COLOR " %d" S_Hz,      menu_offset_sel_acb },
#endif
#ifdef __USE_BACKUP__
  { MT_ADV_CALLBACK, 0,            "REMEMBER\nSTATE",                          menu_backup_acb },
#endif
#ifdef __USE_RTC__
  { MT_ADV_CALLBACK, KM_RTC_DATE,  "SET DATE",                                 menu_keyboard_acb },
  { MT_ADV_CALLBACK, KM_RTC_TIME,  "SET TIME",                                 menu_keyboard_acb },
#endif
  { MT_SUBMENU, 0, S_RARROW" MORE", menu_device1 },
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
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
  { MT_ADV_CALLBACK,                  0, "BRIGHTNESS\n" R_LINK_COLOR " %d%%%%", menu_brightness_acb },
#endif
#ifdef __DFU_SOFTWARE_MODE__
  { MT_SUBMENU,                       0, S_RARROW"DFU",   menu_dfu },
#endif
  { MT_NONE, 0, NULL, menu_back } // next-> menu_back
};

const menuitem_t menu_top[] = {
  { MT_SUBMENU, 0, "DISPLAY",  menu_display },
  { MT_SUBMENU, 0, "MARKER",    menu_marker },
  { MT_SUBMENU, 0, "STIMULUS",  menu_stimulus },
  { MT_SUBMENU, 0, "CALIBRATE", menu_cal },
  { MT_SUBMENU, 0, "RECALL",    menu_recall },
#ifdef __VNA_MEASURE_MODULE__
  { MT_SUBMENU, 0, "MEASURE",   menu_marker_measure },
#endif
#ifdef __USE_SD_CARD__
  { MT_SUBMENU, 0, "SD CARD", menu_sdcard },
#endif
  { MT_SUBMENU, 0, "CONFIG", menu_config },
  { MT_NONE, 0, NULL, NULL } // sentinel
};

#define MENU_STACK_DEPTH_MAX 5
const menuitem_t *menu_stack[MENU_STACK_DEPTH_MAX] = {
  menu_top, NULL, NULL, NULL, NULL
};

static const menuitem_t *menu_next_item(const menuitem_t *m){
  if (m == NULL) return NULL;
  m++; // Next item
  return m->type == MT_NONE ? (menuitem_t *)m->reference : m;
}

static const menuitem_t *current_menu_item(int i){
  const menuitem_t *m = menu_stack[menu_current_level];
  while (i--) m = menu_next_item(m);
  return m;
}

static int current_menu_get_count(void){
  int i = 0;
  const menuitem_t *m = menu_stack[menu_current_level];
  while (m){m = menu_next_item(m); i++;}
  return i;
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
menu_push_submenu(const menuitem_t *submenu)
{
  if (menu_current_level < MENU_STACK_DEPTH_MAX-1)
    menu_current_level++;
  menu_stack[menu_current_level] = submenu;
  ensure_selection();
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

// Key names (use numfont16x22.c glyph)
#define KP_0          0
#define KP_1          1
#define KP_2          2
#define KP_3          3
#define KP_4          4
#define KP_5          5
#define KP_6          6
#define KP_7          7
#define KP_8          8
#define KP_9          9
#define KP_PERIOD    10
#define KP_MINUS     11
#define KP_X1        12
#define KP_K         13
#define KP_M         14
#define KP_G         15
#define KP_BS        16
#define KP_INF       17
#define KP_DB        18
#define KP_PLUSMINUS 19
#define KP_KEYPAD    20
#define KP_N         21
#define KP_P         22
#define KP_ENTER     23
// Stop
#define KP_NONE      255

static const keypads_t keypads_freq[] = {
  { 1, 3, KP_PERIOD },
  { 0, 3, KP_0 },
  { 0, 2, KP_1 },
  { 1, 2, KP_2 },
  { 2, 2, KP_3 },
  { 0, 1, KP_4 },
  { 1, 1, KP_5 },
  { 2, 1, KP_6 },
  { 0, 0, KP_7 },
  { 1, 0, KP_8 },
  { 2, 0, KP_9 },
  { 3, 0, KP_G },
  { 3, 1, KP_M },
  { 3, 2, KP_K },
  { 3, 3, KP_X1 },
  { 2, 3, KP_BS },
  { 0, 0, KP_NONE}
};

static const keypads_t keypads_scale[] = {
  { 1, 3, KP_PERIOD },
  { 0, 3, KP_0 },
  { 0, 2, KP_1 },
  { 1, 2, KP_2 },
  { 2, 2, KP_3 },
  { 0, 1, KP_4 },
  { 1, 1, KP_5 },
  { 2, 1, KP_6 },
  { 0, 0, KP_7 },
  { 1, 0, KP_8 },
  { 2, 0, KP_9 },
  { 3, 3, KP_ENTER },
  { 2, 3, KP_BS },
  { 0, 0, KP_NONE }
};

static const keypads_t keypads_ref[] = {
  { 1, 3, KP_PERIOD },
  { 0, 3, KP_0 },
  { 0, 2, KP_1 },
  { 1, 2, KP_2 },
  { 2, 2, KP_3 },
  { 0, 1, KP_4 },
  { 1, 1, KP_5 },
  { 2, 1, KP_6 },
  { 0, 0, KP_7 },
  { 1, 0, KP_8 },
  { 2, 0, KP_9 },
  { 3, 2, KP_MINUS },
  { 3, 3, KP_ENTER },
  { 2, 3, KP_BS },
  { 0, 0, KP_NONE }
};

static const keypads_t keypads_time[] = {
  { 1, 3, KP_PERIOD },
  { 0, 3, KP_0 },
  { 0, 2, KP_1 },
  { 1, 2, KP_2 },
  { 2, 2, KP_3 },
  { 0, 1, KP_4 },
  { 1, 1, KP_5 },
  { 2, 1, KP_6 },
  { 0, 0, KP_7 },
  { 1, 0, KP_8 },
  { 2, 0, KP_9 },
  { 3, 1, KP_N },
  { 3, 2, KP_P },
  { 3, 3, KP_MINUS },
  { 2, 3, KP_BS },
  { 0, 0, KP_NONE }
};

static const keypads_list keypads_mode_tbl[KM_NONE] = {
[KM_START]           = {keypads_freq , "START"         }, // start
[KM_STOP]            = {keypads_freq , "STOP"          }, // stop
[KM_CENTER]          = {keypads_freq , "CENTER"        }, // center
[KM_SPAN]            = {keypads_freq , "SPAN"          }, // span
[KM_CW]              = {keypads_freq , "CW FREQ"       }, // cw freq
[KM_VAR]             = {keypads_freq , "JOG STEP"      }, // VAR freq step
[KM_SCALE]           = {keypads_scale, "SCALE"         }, // scale
[KM_REFPOS]          = {keypads_ref,   "REFPOS"        }, // refpos
[KM_EDELAY]          = {keypads_time , "E-DELAY"       }, // electrical delay
[KM_VELOCITY_FACTOR] = {keypads_scale, "VELOCITY%%"    }, // velocity factor
[KM_SCALEDELAY]      = {keypads_time , "DELAY"         }, // scale of delay
[KM_XTAL]            = {keypads_freq , "TCXO 26M" S_Hz }, // XTAL frequency
[KM_THRESHOLD]       = {keypads_freq , "THRESHOLD"     }, // Harmonic threshold frequency
[KM_VBAT]            = {keypads_scale, "BAT OFFSET"    }, // Vbat offset input in mV
#ifdef __S21_MEASURE__
[KM_MEASURE_R]       = {keypads_scale, "MEASURE Rl" }, // CH0 port impedance in Om
#endif
#ifdef __VNA_Z_RENORMALIZATION__
[KM_Z_PORT]          = {keypads_scale, "PORT Z 50" S_RARROW }, // Port Z renormalization impedance
#endif
#ifdef __USE_RTC__
[KM_RTC_DATE]        = {keypads_scale, "SET DATE\n YYMMDD"}, // Date
[KM_RTC_TIME]        = {keypads_scale, "SET TIME\n HHMMSS"}, // Time
#endif
};

static void
set_numeric_value(void)
{
  float  f_val = my_atof(kp_buf);
  freq_t u_val = my_atoui(kp_buf);
  switch (keypad_mode) {
    case KM_START:    set_sweep_frequency(ST_START,  u_val); break;
    case KM_STOP:     set_sweep_frequency(ST_STOP,   u_val); break;
    case KM_CENTER:   set_sweep_frequency(ST_CENTER, u_val); break;
    case KM_SPAN:     set_sweep_frequency(ST_SPAN,   u_val); break;
    case KM_CW:       set_sweep_frequency(ST_CW,     u_val); break;
    case KM_VAR:      set_sweep_frequency(ST_VAR,    u_val); break;
    case KM_SCALE:    set_trace_scale(current_trace, f_val); break;
    case KM_REFPOS:   set_trace_refpos(current_trace, f_val);break;
    case KM_EDELAY:   set_electrical_delay(f_val);           break; // pico seconds
    case KM_VELOCITY_FACTOR: velocity_factor = u_val;        break;
    case KM_SCALEDELAY: set_trace_scale(current_trace, f_val * 1e-12); break;// pico second
    case KM_XTAL:     si5351_set_tcxo(u_val);                break;
    case KM_THRESHOLD:config._harmonic_freq_threshold= u_val;break;
    case KM_VBAT:     config._vbat_offset = u_val;           break;
#ifdef __S21_MEASURE__
    case KM_MEASURE_R:config._measure_r = f_val;             break;
#endif
#ifdef __VNA_Z_RENORMALIZATION__
    case KM_Z_PORT:   current_props._portz = f_val;          break;
#endif
#ifdef __USE_RTC__
    case KM_RTC_DATE:
    case KM_RTC_TIME:
    {
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
      if (keypad_mode == KM_RTC_DATE) {
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
    break;
#endif
  }
}

static void
draw_button(uint16_t x, uint16_t y, uint16_t w, uint16_t h, button_t *b)
{
  uint16_t bw = b->border&BUTTON_BORDER_WIDTH_MASK;
  lcd_set_foreground(b->fg);
  lcd_set_background(b->bg);lcd_fill(x + bw, y + bw, w - (bw * 2), h - (bw * 2));
  if (bw==0) return;
  uint16_t br = LCD_RISE_EDGE_COLOR;
  uint16_t bd = LCD_FALLEN_EDGE_COLOR;
  uint16_t type = b->border;
  lcd_set_background(type&BUTTON_BORDER_TOP    ? br : bd);lcd_fill(x,          y,           w, bw); // top
  lcd_set_background(type&BUTTON_BORDER_RIGHT  ? br : bd);lcd_fill(x + w - bw, y,          bw,  h); // right
  lcd_set_background(type&BUTTON_BORDER_LEFT   ? br : bd);lcd_fill(x,          y,          bw,  h); // left
  lcd_set_background(type&BUTTON_BORDER_BOTTOM ? br : bd);lcd_fill(x,          y + h - bw,  w, bw); // bottom
  // Set colors for button text after
  lcd_set_background(b->bg);
}

void drawMessageBox(char *header, char *text, uint32_t delay){
  button_t b;
  int x , y;
  b.bg = LCD_MENU_COLOR;
  b.fg = LCD_MENU_TEXT_COLOR;
  b.border = BUTTON_BORDER_FLAT|1;
  // Draw header
  draw_button((LCD_WIDTH-MESSAGE_BOX_WIDTH)/2, LCD_HEIGHT/2-40, MESSAGE_BOX_WIDTH, 60, &b);
  x = (LCD_WIDTH-MESSAGE_BOX_WIDTH)/2 + 10;
  y = LCD_HEIGHT/2-40 + 5;
  lcd_drawstring(x, y, header);
  // Draw window
  lcd_set_background(LCD_FG_COLOR);
  lcd_fill((LCD_WIDTH-MESSAGE_BOX_WIDTH)/2+3, LCD_HEIGHT/2-40+FONT_STR_HEIGHT+8, MESSAGE_BOX_WIDTH-6, 60-FONT_STR_HEIGHT-8-3);
  x = (LCD_WIDTH-MESSAGE_BOX_WIDTH)/2 + 20;
  y = LCD_HEIGHT/2-40 + FONT_STR_HEIGHT + 8 + 14;
  lcd_drawstring(x, y, text);
  chThdSleepMilliseconds(delay);
}

static void
draw_keypad(uint32_t mask)
{
  int i;
  button_t button;
  button.fg = LCD_MENU_TEXT_COLOR;
  for(i = 0; keypads[i].c != KP_NONE; i++) {
    if ((mask&(1<<i)) == 0) continue;
    if (i == selection){
      button.bg = LCD_MENU_ACTIVE_COLOR;
      button.border = KEYBOARD_BUTTON_BORDER|BUTTON_BORDER_FALLING;
    }
    else{
      button.bg = LCD_MENU_COLOR;
      button.border = KEYBOARD_BUTTON_BORDER|BUTTON_BORDER_RISE;
    }
    int x = KP_GET_X(keypads[i].x);
    int y = KP_GET_Y(keypads[i].y);
    draw_button(x, y, KP_WIDTH, KP_HEIGHT, &button);
    lcd_drawfont(keypads[i].c,
                     x + (KP_WIDTH - NUM_FONT_GET_WIDTH) / 2,
                     y + (KP_HEIGHT - NUM_FONT_GET_HEIGHT) / 2);
  }
}

static int period_pos(void) {int j; for (j = 0; j < kp_index && kp_buf[j] != '.'; j++); return j;}

static void
draw_numeric_area_frame(void)
{
  lcd_set_foreground(LCD_INPUT_TEXT_COLOR);
  lcd_set_background(LCD_INPUT_BG_COLOR);
  lcd_fill(0, LCD_HEIGHT-NUM_INPUT_HEIGHT, LCD_WIDTH, NUM_INPUT_HEIGHT);
  lcd_drawstring(10, LCD_HEIGHT-(FONT_GET_HEIGHT+NUM_INPUT_HEIGHT)/2, keypads_mode_tbl[keypad_mode].name);
  //lcd_drawfont(KP_KEYPAD, 300, 216);
}

static void
draw_numeric_input(const char *buf)
{
  bool focused = FALSE;
  uint16_t x = 14 + 10 * FONT_WIDTH;
  uint16_t y = LCD_HEIGHT-(NUM_FONT_GET_HEIGHT+NUM_INPUT_HEIGHT)/2;
  uint16_t xsim;
#ifdef __USE_RTC__
  if ((1<<keypad_mode)&((1<<KM_RTC_DATE)|(1<<KM_RTC_TIME)))
    xsim = 0b01010100;
  else
#endif
    xsim = (0b00100100100100100 >>(2-(period_pos()%3)))&(~1);
  int c;
  while(*buf) {
    c = *buf++;
         if (c == '.'){c = KP_PERIOD;xsim<<=4;}
    else if (c == '-'){c = KP_MINUS; xsim&=~3;}
    else// if (c >= '0' && c <= '9')
      c = c - '0';
    uint16_t fg = LCD_INPUT_TEXT_COLOR;
    uint16_t bg = LCD_INPUT_BG_COLOR;
#ifdef UI_USE_NUMERIC_INPUT
    if (ui_mode == UI_NUMERIC && uistat.digit == 8-i) {
      fg = LCD_SPEC_INPUT_COLOR;
        focused = true;
      if (uistat.digit_mode){
        bg = LCD_SPEC_INPUT_COLOR;
        fg = LCD_INPUT_TEXT_COLOR;
      }
    }
#endif
    lcd_set_foreground(fg);
    lcd_set_background(bg);
    if (c < 0 && focused) c = 0;
    // Add space before char
    uint16_t space = xsim&1 ? 2 + 10 : 2;
    xsim>>=1;
    lcd_fill(x, y, space, NUM_FONT_GET_HEIGHT);
    x+=space;
    if (c < 0) continue; // c is number
    lcd_drawfont(c, x, y);
    x+=NUM_FONT_GET_WIDTH;
  }
  lcd_set_background(LCD_INPUT_BG_COLOR);
  lcd_fill(x, y, NUM_FONT_GET_WIDTH+2+10, NUM_FONT_GET_HEIGHT);
}

static int
menu_is_multiline(const char *label)
{
  int n = 1;
  while (*label)
    if (*label++ == '\n')
      n++;
  return n;
}

/*
 * Button icons bitmaps
 */
#define ICON_WIDTH        16
#define ICON_HEIGHT       11
#define ICON_GET_DATA(i) (&button_icons[2*ICON_HEIGHT*(i)])
static const uint8_t button_icons[] = {
  _BMP16(0b0011111111110000),
  _BMP16(0b0010000000010000),
  _BMP16(0b0010000000010000),
  _BMP16(0b0010000000010000),
  _BMP16(0b0010000000010000),
  _BMP16(0b0010000000010000),
  _BMP16(0b0010000000010000),
  _BMP16(0b0010000000010000),
  _BMP16(0b0010000000010000),
  _BMP16(0b0010000000010000),
  _BMP16(0b0011111111110000),

  _BMP16(0b0011111111110000),
  _BMP16(0b0010000000001000),
  _BMP16(0b0010000000011000),
  _BMP16(0b0010000000110000),
  _BMP16(0b0010000001100000),
  _BMP16(0b0010100011010000),
  _BMP16(0b0010110110010000),
  _BMP16(0b0010011100010000),
  _BMP16(0b0010001000010000),
  _BMP16(0b0010000000010000),
  _BMP16(0b0011111111110000),

  _BMP16(0b0000000000000000),
  _BMP16(0b0000011110000000),
  _BMP16(0b0000100001000000),
  _BMP16(0b0001000000100000),
  _BMP16(0b0010000000010000),
  _BMP16(0b0010000000010000),
  _BMP16(0b0010000000010000),
  _BMP16(0b0010000000010000),
  _BMP16(0b0001000000100000),
  _BMP16(0b0000100001000000),
  _BMP16(0b0000011110000000),

  _BMP16(0b0000000000000000),
  _BMP16(0b0000011110000000),
  _BMP16(0b0000100001000000),
  _BMP16(0b0001001100100000),
  _BMP16(0b0010011110010000),
  _BMP16(0b0010111111010000),
  _BMP16(0b0010111111010000),
  _BMP16(0b0010011110010000),
  _BMP16(0b0001001100100000),
  _BMP16(0b0000100001000000),
  _BMP16(0b0000011110000000),

  _BMP16(0b0011111111111000),
  _BMP16(0b0010000000001000),
  _BMP16(0b0010001111101000),
  _BMP16(0b0010011001101000),
  _BMP16(0b0010110001101000),
  _BMP16(0b0010110001101000),
  _BMP16(0b0010111111101000),
  _BMP16(0b0010110001101000),
  _BMP16(0b0010110001101000),
  _BMP16(0b0010000000001000),
  _BMP16(0b0011111111111000),

  _BMP16(0b0011111111111000),
  _BMP16(0b0010000000001000),
  _BMP16(0b0010110001101000),
  _BMP16(0b0010110001101000),
  _BMP16(0b0010111011101000),
  _BMP16(0b0010111111101000),
  _BMP16(0b0010110101101000),
  _BMP16(0b0010110101101000),
  _BMP16(0b0010110001101000),
  _BMP16(0b0010000000001000),
  _BMP16(0b0011111111111000),
};

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
    }
    else{
      button.bg = LCD_MENU_COLOR;
      button.border = MENU_BUTTON_BORDER|BUTTON_BORDER_RISE;
    }
    // Custom button, apply custom settings/label from callback
    char *text;
    uint16_t text_offs;
    if (m->type == MT_ADV_CALLBACK){
      if (m->reference) ((menuaction_acb_t)m->reference)(m->data, &button);
      // Apply custom text, from button label and
      if (m->label != MT_CUSTOM_LABEL)
        plot_printf(button.label, sizeof(button.label), m->label, button.p1.u);
      text = button.label;
    }
    else
      text = m->label;
    // Draw button
    draw_button(LCD_WIDTH-MENU_BUTTON_WIDTH, y, MENU_BUTTON_WIDTH, menu_button_height, &button);
    // Draw icon if need (and add extra shift for text)
    if (button.icon >=0){
      lcd_blitBitmap(LCD_WIDTH-MENU_BUTTON_WIDTH+MENU_BUTTON_BORDER + 1, y+(menu_button_height-ICON_HEIGHT)/2, ICON_WIDTH, ICON_HEIGHT, ICON_GET_DATA(button.icon));
      text_offs = LCD_WIDTH-MENU_BUTTON_WIDTH+MENU_BUTTON_BORDER + 1 + ICON_WIDTH;
    }
    else
      text_offs = LCD_WIDTH-MENU_BUTTON_WIDTH+MENU_BUTTON_BORDER + 5;
    // Draw button text
    int lines = menu_is_multiline(text);
#if _USE_FONT_ != _USE_SMALL_FONT_
    if (menu_button_height < lines*FONT_GET_HEIGHT + 2) {
      lcd_set_font(FONT_SMALL);
      lcd_drawstring(text_offs, y+(menu_button_height-lines*sFONT_GET_HEIGHT - 1)/2, text);
    }
    else {
      lcd_set_font(FONT_NORMAL);
      lcd_drawstring(text_offs, y+(menu_button_height-lines*FONT_GET_HEIGHT)/2, text);
    }
#else
    lcd_drawstring(text_offs, y+(menu_button_height-lines*FONT_GET_HEIGHT)/2, text);
#endif
  }
  // Erase empty buttons
  if (AREA_HEIGHT_NORMAL + OFFSETY > y){
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

/*
 * Menu mode processing
 */
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
ui_process_menu_lever(uint16_t status)
{
  if (status & EVT_BUTTON_SINGLE_CLICK) {
    menu_invoke(selection);
    return;
  }
  uint16_t count = current_menu_get_count();
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
menu_apply_touch(int touch_x, int touch_y)
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

#ifdef UI_USE_NUMERIC_INPUT
static void
erase_numeric_input(void)
{
  lcd_set_background(LCD_BG_COLOR);
  lcd_fill(0, LCD_HEIGHT-NUM_INPUT_HEIGHT, LCD_WIDTH, NUM_INPUT_HEIGHT);
}

static void
fetch_numeric_target(void)
{
  switch (keypad_mode) {
  case KM_START:     uistat.value = get_sweep_frequency(ST_START ); break;
  case KM_STOP:      uistat.value = get_sweep_frequency(ST_STOP  ); break;
  case KM_CENTER:    uistat.value = get_sweep_frequency(ST_CENTER); break;
  case KM_SPAN:      uistat.value = get_sweep_frequency(ST_SPAN  ); break;
  case KM_CW:        uistat.value = get_sweep_frequency(ST_CW    ); break;
  case KM_VAR:       uistat.value = var_freq;                       break;
  case KM_SCALE:     uistat.value = get_trace_scale(current_trace) * 1000; break;
  case KM_REFPOS:    uistat.value = get_trace_refpos(current_trace) * 1000;break;
  case KM_EDELAY:    uistat.value = get_electrical_delay(); break;
  case KM_VELOCITY_FACTOR: uistat.value = velocity_factor; break;
  case KM_SCALEDELAY: uistat.value = get_trace_scale(current_trace) * 1e12; break;
  case KM_XTAL:      uistat.value = config._xtal_freq;              break;
  case KM_THRESHOLD: uistat.value = config._harmonic_freq_threshold;break;
  case KM_VBAT:      uistat.value = config._vbat_offset;            break;
  }

  uint32_t x = uistat.value;
  for (uistat.digit = 0; x >= 10 && uistat.digit < 9; uistat.digit++)
    x /= 10;
}

static void
draw_numeric_area(void)
{
  char buf[10];
  plot_printf(buf, sizeof buf, "%9d", uistat.value);
  draw_numeric_input(buf);
}

static void
ui_mode_numeric(int _keypad_mode)
{
  if (ui_mode == UI_NUMERIC)
    return;

  // keypads array
  keypad_mode = _keypad_mode;
  ui_mode = UI_NUMERIC;
  area_width = AREA_WIDTH_NORMAL;
  area_height = LCD_HEIGHT-NUM_INPUT_HEIGHT;//AREA_HEIGHT_NORMAL - 32;

  draw_numeric_area_frame();
  fetch_numeric_target();
  draw_numeric_area();
}

static void
ui_process_numeric_lever(uint16_t status)
{
  if (status == EVT_BUTTON_SINGLE_CLICK) {
    status = btn_wait_release();
    if (uistat.digit_mode) {
      if (status & (EVT_BUTTON_SINGLE_CLICK | EVT_BUTTON_DOWN_LONG)) {
        uistat.digit_mode = FALSE;
        draw_numeric_area();
      }
    } else {
      if (status & EVT_BUTTON_DOWN_LONG) {
        uistat.digit_mode = TRUE;
        draw_numeric_area();
      } else if (status & EVT_BUTTON_SINGLE_CLICK) {
        set_numeric_value();
        goto exit;
      }
    }
    return;
  }

  do {
    if (uistat.digit_mode) {
      if (status & EVT_DOWN) {
        if (uistat.digit < 8)
          uistat.digit++;
        else
          goto exit;
      }
      if (status & EVT_UP) {
        if (uistat.digit > 0)
          uistat.digit--;
        else
          goto exit;
      }
    } else {
      int32_t step = 1;
      int n;
      for (n = uistat.digit; n > 0; n--)
        step *= 10;
      if (status & EVT_DOWN)
        uistat.value += step;
      if (status & EVT_UP)
        uistat.value -= step;
    }
    draw_numeric_area();
  } while ((status = btn_wait_release()) != 0);
  return;

 exit:
  // cancel operation
  ui_mode_normal();
}

static void
numeric_apply_touch(int touch_x, int touch_y)
{
  if (touch_x < 64) {
    ui_mode_normal();
    return;
  }
  if (touch_x > 64+9*20+8+8) {
    ui_mode_keypad(keypad_mode);
    ui_process_keypad();
    return;
  }

  if (touch_y > LCD_HEIGHT-40) {
    int n = 9 - (touch_x - 64) / 20;
    uistat.digit = n;
    uistat.digit_mode = TRUE;
  } else {
    int step, n;
    if (touch_y < 100) {
      step = 1;
    } else {
      step = -1;
    }

    for (n = uistat.digit; n > 0; n--)
      step *= 10;
    uistat.value += step;
  }
  draw_numeric_area();

  touch_wait_release();
  uistat.digit_mode = FALSE;
  draw_numeric_area();

  return;
}
#endif

/*
 * Keyboard processing
 */
static void
ui_mode_keypad(int _keypad_mode)
{
  if (ui_mode == UI_KEYPAD)
    return;
  set_area_size(0, 0);
  // keypads array
  keypad_mode = _keypad_mode;
  keypads = keypads_mode_tbl[keypad_mode].keypad_type;
  selection = -1;
  kp_index = 0;
  ui_mode = UI_KEYPAD;
  draw_menu(-1);
  draw_keypad(-1);
  draw_numeric_area_frame();
}

static int
keypad_click(int key)
{
  int c = keypads[key].c;
  if (c == KP_ENTER) c = KP_X1;
  if ((c >= KP_X1 && c <= KP_G) || c == KP_N || c == KP_P) {
    if (kp_index == 0)
      return KP_CANCEL;
    uint16_t scale = 0;
    if (c > KP_X1 && c <= KP_G) scale = c - KP_X1;
    if (c == KP_N) scale = 1;
    if (scale){
      scale+= (scale<<1);
      int i = period_pos(); if (i+scale>NUMINPUT_LEN) scale = NUMINPUT_LEN - 1 - i;
      while (scale--) {
        char v = kp_buf[i+1]; if (v == 0 || kp_buf[i] == 0) {v = '0'; kp_buf[i+2] = 0;}
        kp_buf[i+1] = kp_buf[i];
        kp_buf[i  ] = v;
        i++;
      }
    }
    // numeric input done
    set_numeric_value();
    return KP_DONE;
  }

  if (c <= KP_9 && kp_index < NUMINPUT_LEN) {
    kp_buf[kp_index++] = '0' + c;
  } else if (c == KP_PERIOD && kp_index < NUMINPUT_LEN) {
    // append period if there are no period
    if (kp_index == period_pos())
      kp_buf[kp_index++] = '.';
  } else if (c == KP_MINUS) {
    if (kp_index == 0)
      kp_buf[kp_index++] = '-';
  } else if (c == KP_BS) {
    if (kp_index == 0)
      return KP_CANCEL;
    --kp_index;
  }
  kp_buf[kp_index] = '\0';
  draw_numeric_input(kp_buf);
  return KP_CONTINUE;
}

static void
keypad_apply_touch(int touch_x, int touch_y)
{
  int i = 0;
  do {
    int x = KP_GET_X(keypads[i].x);
    int y = KP_GET_Y(keypads[i].y);
    if (x < touch_x && touch_x < x+KP_WIDTH && y < touch_y && touch_y < y+KP_HEIGHT) {
      // draw focus
      uint32_t mask = (1<<i)|(1<<selection);
      selection = i;
      draw_keypad(mask);
      touch_wait_release();
      // erase focus
      selection = -1;
      draw_keypad(1<<i);
      // Exit loop on done or cancel
      if (keypad_click(i) != KP_CONTINUE)
        ui_mode_normal();
      return;
    }
  }while (keypads[++i].c != KP_NONE);
  return;
}

static void
ui_process_keypad_lever(uint16_t status)
{
  if (status == EVT_BUTTON_SINGLE_CLICK){
    // Process input
    int result = keypad_click(selection);
    // Exit loop on done or cancel
    if (result != KP_CONTINUE)
      ui_mode_normal();
    return;
  }
  int keypads_last_index;
  for (keypads_last_index = 0; keypads[keypads_last_index+1].c != KP_NONE; keypads_last_index++)
    ;
  do {
    uint32_t mask = 1<<selection;
    if ((status & EVT_DOWN) && --selection < 0)
      selection = keypads_last_index;
    if ((status & EVT_UP)   && ++selection > keypads_last_index)
        selection = 0;
    draw_keypad(mask|(1<<selection));
    chThdSleepMilliseconds(100);
  } while ((status = btn_wait_release()) != 0);
}
//========================== end keyboard input =======================

/*
 * Normal plot processing
 */
static void
ui_mode_normal(void)
{
  if (ui_mode == UI_NORMAL)
    return;
  set_area_size(AREA_WIDTH_NORMAL, AREA_HEIGHT_NORMAL);
#ifdef UI_USE_NUMERIC_INPUT
  if (ui_mode == UI_NUMERIC) {
    request_to_draw_cells_behind_numeric_input();
    erase_numeric_input();
  }
#endif
  if (ui_mode == UI_MENU)
    request_to_draw_cells_behind_menu();
  if (ui_mode == UI_KEYPAD)
    request_to_redraw(REDRAW_CLRSCR | REDRAW_AREA | REDRAW_BATTERY | REDRAW_CAL_STATUS | REDRAW_FREQUENCY);
  request_to_redraw(REDRAW_FREQUENCY);
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
    if (status & EVT_DOWN) {
      idx-= step>>MARKER_SPEEDUP;
      if (idx < 0) idx = 0;
    }
    if (status & EVT_UP) {
     idx+= step>>MARKER_SPEEDUP;
      if (idx  > sweep_points-1)
        idx = sweep_points-1 ;
    }
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
static uint32_t
step_round(uint32_t v)
{
  // decade step
  uint32_t nx, x = 1;
  while((nx = x*10) < v) x = nx;
  // 1-2-5 step
  if (x * 2 > v) return x;
  if (x * 5 > v) return x * 2;
  return x * 5;
}

static void
lever_frequency(uint16_t status, int mode)
{
  freq_t freq = get_sweep_frequency(mode);
  if (mode == ST_SPAN){
    if (status & EVT_UP  ) freq = var_freq ? (freq + var_freq) : step_round(freq*4 + 1);
    if (status & EVT_DOWN) freq = var_freq ? (freq - var_freq) : step_round(freq   - 1);
  }
  else{
    freq_t span = var_freq ? var_freq : step_round(get_sweep_frequency(ST_SPAN) / 4);
    if (status & EVT_UP  ) freq+= span;
    if (status & EVT_DOWN) freq-= span;
  }
  if (freq > STOP_MAX || freq < START_MIN) return;
  set_sweep_frequency(mode, freq);
}

#define STEPRATIO 0.2
static void
lever_edelay(uint16_t status)
{
  float value = electrical_delay;
  float ratio = value > 0 ?  STEPRATIO : -STEPRATIO;
  if (status & EVT_UP)
    value*= (1 - ratio);
  if (status & EVT_DOWN)
    value*= (1 + ratio);
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
  current_trace = mt;
  // drag marker until release
  do {
    touch_position(&touch_x, &touch_y);
    int index = search_nearest_index(touch_x - OFFSETX, touch_y - OFFSETY, current_trace);
    if (index >= 0) {
      set_marker_index(active_marker, index);
      redraw_marker(active_marker);
    }
  } while (touch_check()!= EVT_TOUCH_RELEASED);
  return TRUE;
}

static bool
touch_lever_mode_select(int touch_x, int touch_y)
{
  int mode = -1;
  if (touch_y > HEIGHT)
    mode = touch_x < FREQUENCIES_XPOS2 ? LM_FREQ_0 : LM_FREQ_1;
  if (touch_y < 25)
    mode = (touch_x < FREQUENCIES_XPOS2 && electrical_delay != 0.0) ? LM_EDELAY : LM_MARKER;
  if (mode == -1) return FALSE;

  touch_wait_release();
  // Check already selected
  if (select_lever_mode(mode)) return TRUE;
  // Call keyboard for enter
  switch(mode){
    case LM_FREQ_0: ui_mode_keypad(FREQ_IS_CENTERSPAN() ? KM_CENTER : KM_START); break;
    case LM_FREQ_1: ui_mode_keypad(FREQ_IS_CENTERSPAN() ? KM_SPAN   : KM_STOP ); break;
    case LM_EDELAY: ui_mode_keypad(KM_EDELAY); break;
  }
  return TRUE;
}

static void
ui_process_normal_lever(uint16_t status)
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
    case LM_FREQ_0: lever_frequency(status, FREQ_IS_STARTSTOP() ? ST_START : ST_CENTER); break;
    case LM_FREQ_1: lever_frequency(status, FREQ_IS_STARTSTOP() ? ST_STOP  : ST_SPAN  ); break;
    case LM_EDELAY: lever_edelay(status); break;
  }
}

static bool
normal_apply_ref_scale(int touch_x, int touch_y){
  int t = current_trace;
  // do not scale invalid or smith chart
  if (t == TRACE_INVALID || trace[t].type == TRC_SMITH) return FALSE;
  if (touch_x < OFFSETX - 5 || touch_x > OFFSETX + CELLOFFSETX + 10 ||
      touch_y < OFFSETY     || touch_y > AREA_HEIGHT_NORMAL) return FALSE;
  float ref   = trace[t].refpos;
  float scale = trace[t].scale;

       if (touch_y < GRIDY*1*NGRIDY/4) ref+=0.5f;
  else if (touch_y < GRIDY*2*NGRIDY/4) {scale*=2.0f;ref=ref/2-NGRIDY/4 + NGRIDY/2;}
  else if (touch_y < GRIDY*3*NGRIDY/4) {scale/=2.0f;ref=2*ref-NGRIDY   + NGRIDY/2;}
  else                                 ref-=0.5f;

  trace[t].scale  = scale;
  trace[t].refpos =   ref;
  plot_into_index(measured);
  request_to_redraw(REDRAW_AREA);
  chThdSleepMilliseconds(100);
  return TRUE;
}

#ifdef __USE_SD_CARD__
static bool
made_screenshot(int touch_x, int touch_y)
{
  if (touch_y < HEIGHT || touch_x < FREQUENCIES_XPOS3 || touch_x > FREQUENCIES_XPOS2)
    return FALSE;
  touch_wait_release();
  menu_sdcard_cb(SAVE_BMP_FILE);
  return TRUE;
}
#endif

static void
normal_apply_touch(int touch_x, int touch_y){
  // Try drag marker
  if (touch_pickup_marker(touch_x, touch_y))
    return;
#ifdef __USE_SD_CARD__
  // Try made screenshot
  if (made_screenshot(touch_x, touch_y))
    return;
#endif
  if (normal_apply_ref_scale(touch_x, touch_y))
    return;
  // Try select lever mode (top and bottom screen)
  if (touch_lever_mode_select(touch_x, touch_y))
    return;
  // default: switch menu mode after release
  touch_wait_release();
  ui_mode_menu();
}
//========================== end normal plot input =======================

static void
ui_process_lever(void)
{
//  last_button = 0;
  uint16_t status = btn_check();
  if (status == 0) return;
  switch (ui_mode) {
    case UI_NORMAL: ui_process_normal_lever(status);  break;
    case UI_MENU:   ui_process_menu_lever(status);    break;
#ifdef UI_USE_NUMERIC_INPUT
    case UI_NUMERIC:ui_process_numeric_lever(status); break;
#endif
    case UI_KEYPAD: ui_process_keypad_lever(status);  break;
  }
}

static
void ui_process_touch(void)
{
  int touch_x, touch_y;
  int status = touch_check();
  if (status == EVT_TOUCH_PRESSED || status == EVT_TOUCH_DOWN) {
    touch_position(&touch_x, &touch_y);
    switch (ui_mode) {
      case UI_NORMAL: normal_apply_touch(touch_x, touch_y); break;
      case UI_MENU:   menu_apply_touch(touch_x, touch_y);   break;
#ifdef UI_USE_NUMERIC_INPUT
      case UI_NUMERIC:numeric_apply_touch(touch_x, touch_y);break;
#endif
      case UI_KEYPAD: keypad_apply_touch(touch_x, touch_y); break;
    }
  }
}

void
ui_process(void)
{
  if (operation_requested&OP_LEVER)
    ui_process_lever();
  if (operation_requested&OP_TOUCH)
    ui_process_touch();

  touch_start_watchdog();
  operation_requested = OP_NONE;
}

/* Triggered when the button is pressed or released. The LED4 is set to ON.*/
static void extcb1(EXTDriver *extp, expchannel_t channel)
{
  (void)extp;
  (void)channel;
  operation_requested|=OP_LEVER;
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

static const EXTConfig extcfg = {
  {
    {EXT_CH_MODE_DISABLED, NULL},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, extcb1},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, extcb1},
    {EXT_CH_MODE_RISING_EDGE | EXT_CH_MODE_AUTOSTART | EXT_MODE_GPIOA, extcb1},
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

void
ui_init()
{
  adc_init();
  // Activates the EXT driver 1.
  extStart(&EXTD1, &extcfg);
  // Init touch subsystem
  touch_init();
}
