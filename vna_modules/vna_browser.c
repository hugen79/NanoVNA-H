/*
 * Copyright (c) 2019-2023, Dmitry (DiSlord) dislordlive@gmail.com
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
static uint16_t file_count;
static uint16_t page_count;
static uint16_t current_page;
static uint16_t browser_mode;

#define BROWSER_DELETE    1

// Buttons in browser
enum {FILE_BUTTON_LEFT = 0, FILE_BUTTON_RIGHT, FILE_BUTTON_EXIT, FILE_BUTTON_DEL, FILE_BUTTON_FILE};

#define SMALL_BUTTON_SIZE    (6 * FONT_WIDTH)
// Button position on screen
typedef struct  {
  uint16_t x;
  uint16_t y;
  uint16_t w;
  uint8_t  h;
  uint8_t  ofs;
} browser_btn_t;
static const browser_btn_t browser_btn[] = {
  [FILE_BUTTON_LEFT] = {         0  + SMALL_BUTTON_SIZE, LCD_HEIGHT - FILE_BOTTOM_HEIGHT, LCD_WIDTH/2 - 2*SMALL_BUTTON_SIZE, FILE_BOTTOM_HEIGHT, (LCD_WIDTH/2 - 2*SMALL_BUTTON_SIZE - FONT_WIDTH)/2}, // < previous
  [FILE_BUTTON_RIGHT]= {LCD_WIDTH/2 + SMALL_BUTTON_SIZE, LCD_HEIGHT - FILE_BOTTOM_HEIGHT, LCD_WIDTH/2 - 2*SMALL_BUTTON_SIZE, FILE_BOTTOM_HEIGHT, (LCD_WIDTH/2 - 2*SMALL_BUTTON_SIZE - FONT_WIDTH)/2}, // > next
  [FILE_BUTTON_EXIT] = {LCD_WIDTH   - SMALL_BUTTON_SIZE, LCD_HEIGHT - FILE_BOTTOM_HEIGHT,                 SMALL_BUTTON_SIZE, FILE_BOTTOM_HEIGHT, (                SMALL_BUTTON_SIZE - FONT_WIDTH)/2}, // X exit
  [FILE_BUTTON_DEL]  = {         0  +                 0, LCD_HEIGHT - FILE_BOTTOM_HEIGHT,                 SMALL_BUTTON_SIZE, FILE_BOTTOM_HEIGHT, (              SMALL_BUTTON_SIZE - 3*FONT_WIDTH)/2}, // DEL
  // File button, only size and start position, must be idx = FILE_BUTTON_FILE
  [FILE_BUTTON_FILE] = {                              0,                               0,           LCD_WIDTH/FILES_COLUMNS, FILE_BUTTON_HEIGHT,                                   FONT_WIDTH/2 + 3},
};

static void browser_get_button_pos(int idx, browser_btn_t *b) {
  int n = idx >= FILE_BUTTON_FILE ? FILE_BUTTON_FILE : idx;
#if 0
  memcpy(b, &browser_btn[n], sizeof(browser_btn_t));
#else
  b->x = browser_btn[n].x;
  b->y = browser_btn[n].y;
  b->w = browser_btn[n].w;
  b->h = browser_btn[n].h;
  b->ofs = browser_btn[n].ofs;
#endif
  if (idx > FILE_BUTTON_FILE) { // for file buttons use multiplier from start offset
    idx-= FILE_BUTTON_FILE;
    b->x+= b->w * (idx / FILES_ROWS);
    b->y+= b->h * (idx % FILES_ROWS);
  }
}

static void browser_draw_button(int idx, const char *txt) {
  if (idx < 0) return;
  button_t b;
  browser_btn_t btn;
  browser_get_button_pos(idx, &btn);
  // Mark DEL button in file delete mode
  b.bg = (idx == FILE_BUTTON_DEL && (browser_mode & BROWSER_DELETE)) ? LCD_LOW_BAT_COLOR : LCD_MENU_COLOR;
  b.fg = LCD_MENU_TEXT_COLOR;
  b.border = (idx == selection) ? BROWSER_BUTTON_BORDER|BUTTON_BORDER_FALLING : BROWSER_BUTTON_BORDER|BUTTON_BORDER_RISE;
  if (txt == NULL) b.border|= BUTTON_BORDER_NO_FILL;
  ui_draw_button(btn.x, btn.y, btn.w, btn.h, &b);
  if (txt) lcd_printf(btn.x + btn.ofs, btn.y + (btn.h - FONT_STR_HEIGHT) / 2, txt);
}

static bool compare_ext(const char *name, const char *ext) {
  int i = 0, j = 0;
  while (name[i]) if (name[i++] == '.') j = i;    // Get last '.' position + 1
  return j == 0 ? false : strcmpi(&name[j], ext); // Compare text after '.' and ext
}

static FRESULT sd_findnext(DIR* dp, FILINFO* fno) {
  while (f_readdir(dp, fno) == FR_OK && fno->fname[0]) {
    if (fno->fattrib & AM_DIR) continue;
    if (compare_ext(fno->fname, dp->pat)) return FR_OK;
//#if FF_USE_LFN && FF_USE_FIND == 2
//    if (compare_ext(fno->altname, dp->pat)) return FR_OK;
//#endif
  }
  return FR_NO_FILE;
}

static FRESULT sd_open_dir(DIR* dp, const TCHAR* path, const TCHAR* pattern) {
  dp->pat = pattern;
  return f_opendir(dp, path);
}

static void browser_open_file(int sel) {
  FILINFO fno;
  DIR dj;
  int cnt;
  if ((uint16_t)sel >= file_count) return;
  if (f_mount(fs_volume, "", 1) != FR_OK) return;
repeat:
  cnt = sel;
  if (sd_open_dir(&dj, "", file_opt[keypad_mode].ext) != FR_OK) return;  // open dir
  while (sd_findnext(&dj, &fno) == FR_OK && cnt != 0) cnt--;         // skip cnt files
  f_closedir(&dj);
  if (cnt != 0) return;

  // Delete file if in delete mode
  if (browser_mode & BROWSER_DELETE) {f_unlink(fno.fname); return;}

  // Load file, get load function
  file_load_cb_t load = file_opt[keypad_mode].load;
  if (load == NULL) return;
  //
  lcd_set_colors(LCD_FG_COLOR, LCD_BG_COLOR);

  if (f_open(fs_file, fno.fname, FA_READ) != FR_OK) return;
  //  START_PROFILE;
  const char *error = load(fs_file, &fno, keypad_mode);
  f_close(fs_file);
  //  STOP_PROFILE;
  // Check, need continue load next or previous file
  bool need_continue = file_opt[keypad_mode].opt & FILE_OPT_CONTINUE;
  if (error) {
    lcd_clear_screen();
    ui_message_box(error, fno.fname, need_continue ? 100 : 2000);
  }
  if (!need_continue) return;

  // Process input
  while (1) {
    uint16_t status = btn_check();
    int key = -1;
    if (status & EVT_DOWN) key = 0;
    if (status & EVT_UP  ) key = 1;
    if (status & EVT_BUTTON_SINGLE_CLICK) key = 2;

    status = touch_check();
    if (status == EVT_TOUCH_PRESSED || status == EVT_TOUCH_DOWN) {
      int touch_x, touch_y;
      touch_position(&touch_x, &touch_y);
           if (touch_x < LCD_WIDTH *1/3) key = 0;
      else if (touch_x < LCD_WIDTH *2/3) key = 2;
      else                               key = 1;
      touch_wait_release();
    }
    //chThdSleepMilliseconds(100); // Device hang after ~2min in this place, not switch thread back
    delayMilliseconds(100);
    int old_sel = sel;
         if (key == 0) {if (--sel < 0) sel = file_count - 1;}
    else if (key == 1) {if (++sel > file_count - 1) sel = 0;}
    else if (key == 2) break;
    if (old_sel != sel) goto repeat;
  }
}

static void browser_draw_buttons(void) {
  browser_draw_button(FILE_BUTTON_DEL, "DEL");
  browser_draw_button(FILE_BUTTON_LEFT,  "<");
  browser_draw_button(FILE_BUTTON_RIGHT, ">");
  browser_draw_button(FILE_BUTTON_EXIT,  "X");
}

static void browser_draw_page(int page) {
  FILINFO fno;
  DIR dj;
  // Mount SD card and open directory
  if (f_mount(fs_volume, "", 1) != FR_OK ||
      sd_open_dir(&dj, "", file_opt[keypad_mode].ext) != FR_OK) {
    ui_message_box("ERROR", "NO CARD", 2000);
    ui_mode_normal();
    return;
  }
  // Draw Browser UI
  int cnt = 0;
  uint16_t start_file = (page - 1) * FILES_PER_PAGE;
  lcd_set_background(LCD_MENU_COLOR);
  //lcd_clear_screen();
  while (sd_findnext(&dj, &fno) == FR_OK) {
    if (cnt >= start_file && cnt < (start_file + FILES_PER_PAGE)) {
      //uint16_t sec = ((fno.ftime<<1)  & 0x3F);
      //uint16_t min = ((fno.ftime>>5)  & 0x3F);
      //uint16_t h   = ((fno.ftime>>11) & 0x1F);
      //uint16_t d   = ((fno.fdate>>0)  & 0x1F);
      //uint16_t m   = ((fno.fdate>>5)  & 0x0F);
      //uint16_t year= ((fno.fdate>>9)  & 0x3F) + 1980;
      //lcd_printf(x, y, "%2d %s %u - %u/%02u/%02u %02u:%02u:%02u", cnt, fno.fname, fno.fsize, year, m, d, h, min, sec);
      browser_draw_button(cnt - start_file + FILE_BUTTON_FILE, fno.fname);
    }
    cnt++;
    if (file_count && (start_file + FILES_PER_PAGE == cnt)) break;
  }
  f_closedir(&dj);
  // Calculate page and file count on first run
  if (file_count == 0) {
    file_count = cnt;
    page_count = cnt == 0 ? 1 : (file_count + FILES_PER_PAGE - 1) / FILES_PER_PAGE;
  }
  // Erase not used button
  cnt-= start_file;
  while(cnt < FILES_PER_PAGE) {
    browser_btn_t btn;
    browser_get_button_pos(cnt + FILE_BUTTON_FILE, &btn);
    lcd_fill(btn.x, btn.y, btn.w, btn.h);
    cnt++;
  }
  lcd_fill(0, LCD_HEIGHT - FILE_BOTTOM_HEIGHT, LCD_WIDTH, FILE_BOTTOM_HEIGHT);

  browser_draw_buttons();
  lcd_printf(LCD_WIDTH / 2 - 3 * FONT_WIDTH, LCD_HEIGHT - (FILE_BOTTOM_HEIGHT + FONT_STR_HEIGHT) / 2, "- %u | %u -", page, page_count);
  return;
}

static void browser_key_press(int key) {
  int page;
  switch (key) {
    case FILE_BUTTON_LEFT:
    case FILE_BUTTON_RIGHT: // Switch page on left / right change
      page = current_page;
      if (key == FILE_BUTTON_LEFT  && --current_page < 1) current_page = page_count;
      if (key == FILE_BUTTON_RIGHT && ++current_page > page_count) current_page = 1;
      if (page != current_page)
        browser_draw_page(current_page);
    break;
    case FILE_BUTTON_EXIT:  //Exit
      ui_mode_normal();
    break;
    case FILE_BUTTON_DEL:   // Toggle delete mode
      browser_mode^= BROWSER_DELETE;
      browser_draw_buttons();
    break;
    case FILE_BUTTON_FILE:  // Open or delete file
    default:
      browser_open_file(key - FILE_BUTTON_FILE + (current_page - 1) * FILES_PER_PAGE);
      if (browser_mode & BROWSER_DELETE) {
        file_count = 0;                      // Reeset file count (recalculate on draw page)
        selection = -1;                      // Reset delection
        browser_mode&=~BROWSER_DELETE;       // Exit file delete mode
        browser_draw_page(current_page);
        return;
      }
      ui_mode_normal(); // Exit
    break;
  }
}

static int browser_get_max(void) {
  // get max buttons depend from page and file count
  int max = current_page == page_count ? (file_count % FILES_PER_PAGE) : FILES_PER_PAGE;
  if (file_count > 0 && max == 0) max = FILES_PER_PAGE;
  return max + FILE_BUTTON_FILE - 1;
}

void ui_mode_browser(int mode) {
  if (ui_mode == UI_BROWSER)
    return;
  set_area_size(0, 0);
  ui_mode = UI_BROWSER;
  keypad_mode = mode;
  current_page = 1;
  file_count = 0;
  selection = -1;
  browser_mode = 0;
  browser_draw_page(current_page);
}

// Process UI input for browser
static void ui_browser_touch(int touch_x, int touch_y) {
  browser_btn_t btn;
  int old = selection;
  int max = browser_get_max();
  for (int idx = 0; idx <= max; idx++) {
    browser_get_button_pos(idx, &btn);
    if (touch_x < btn.x || touch_x >= btn.x + btn.w ||
        touch_y < btn.y || touch_y >= btn.y + btn.h) continue;
    // Found button under touch
    browser_draw_button(selection = idx, NULL);  // draw new selection
    browser_draw_button(old, NULL);              // clear old
    touch_wait_release();
    selection = -1;
    browser_draw_button(idx, NULL);              // clear selection
    browser_key_press(idx);
    return;
  }
}

static void ui_browser_lever(uint16_t status) {
  if (status == EVT_BUTTON_SINGLE_CLICK) {
    if (selection >= 0) browser_key_press(selection); // Process click
    return;
  }
  int max = browser_get_max();
  do {
    int old = selection;
    if((status & EVT_DOWN) && --selection < 0) selection = max;
    if((status & EVT_UP)   && ++selection > max) selection = 0;
    if (old != selection) {
      browser_draw_button(old, NULL);       // clear old selection
      browser_draw_button(selection, NULL); // draw new selection
    }
    chThdSleepMilliseconds(100);
  } while ((status = btn_wait_release()) != 0);
}
