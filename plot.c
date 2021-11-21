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

#include <string.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "nanovna.h"

static void cell_draw_marker_info(int x0, int y0);
static void draw_battery_status(void);
static void draw_cal_status(void);
static void draw_frequencies(void);
static int  cell_printf(int16_t x, int16_t y, const char *fmt, ...);
static void markmap_all_markers(void);

static int16_t grid_offset;
static int16_t grid_width;

static uint8_t redraw_request = 0; // contains REDRAW_XXX flags

static uint16_t area_width  = AREA_WIDTH_NORMAL;
static uint16_t area_height = AREA_HEIGHT_NORMAL;

// Cell render use spi buffer
static pixel_t *cell_buffer;

// indicate dirty cells (not redraw if cell data not changed)
#define MAX_MARKMAP_X    ((LCD_WIDTH+CELLWIDTH-1)/CELLWIDTH)
#define MAX_MARKMAP_Y    ((LCD_HEIGHT+CELLHEIGHT-1)/CELLHEIGHT)
// Define markmap mask size
#if MAX_MARKMAP_X <= 8
typedef uint8_t map_t;
#elif MAX_MARKMAP_X <= 16
typedef uint16_t map_t;
#elif MAX_MARKMAP_X <= 32
typedef uint32_t map_t;
#endif

static map_t   markmap[2][MAX_MARKMAP_Y];
static uint8_t current_mappage = 0;

// Trace data cache, for faster redraw cells
typedef struct {
  uint16_t y;
  uint16_t x;
} index_t;
static index_t trace_index[TRACE_INDEX_COUNT][POINTS_COUNT];

#if 1
// All used in plot v > 0
#define float2int(v) ((int)((v)+0.5))
#else
static int 
float2int(float v) 
{
  if (v < 0) return v - 0.5;
  if (v > 0) return v + 0.5;
  return 0;
}
#endif

static inline int
circle_inout(int x, int y, int r)
{
  int d = x*x + y*y;
  if (d < r*r - r) return  1; // in  circle
  if (d > r*r + r) return -1; // out circle
  return 0;                   // on  circle
}

static bool
polar_grid(int x, int y)
{
  uint32_t d = x*x + y*y;

  if (d > P_RADIUS*P_RADIUS + P_RADIUS) return 0;
  if (d > P_RADIUS*P_RADIUS - P_RADIUS) return 1;

  // vertical and horizontal axis
  if (x == 0 || y == 0) return 1;

  if (d < P_RADIUS*P_RADIUS/25 - P_RADIUS/5) return 0;
  if (d < P_RADIUS*P_RADIUS/25 + P_RADIUS/5) return 1;

  if (d < P_RADIUS*P_RADIUS*4/25 - P_RADIUS*2/5) return 0;
  if (d < P_RADIUS*P_RADIUS*4/25 + P_RADIUS*2/5) return 1;

  // cross sloping lines
  if (x == y || x == -y) return 1;

  if (d < P_RADIUS*P_RADIUS*9/25 - P_RADIUS*3/5) return 0;
  if (d < P_RADIUS*P_RADIUS*9/25 + P_RADIUS*3/5) return 1;

  if (d < P_RADIUS*P_RADIUS*16/25 - P_RADIUS*4/5) return 0;
  if (d < P_RADIUS*P_RADIUS*16/25 + P_RADIUS*4/5) return 1;

  return 0;
}

static void
cell_polar_grid(int x0, int y0, int w, int h, pixel_t color)
{
  int x, y;
  // offset to center
  x0 -= P_CENTER_X;
  y0 -= P_CENTER_Y;
  for (y = 0; y < h; y++)
    for (x = 0; x < w; x++)
      if (polar_grid(x + x0, y + y0)) cell_buffer[y * CELLWIDTH + x] = color;
}

/*
 * Render Smith grid (if mirror by x possible get Admittance grid)
 */
static bool
smith_grid(int x, int y)
{
#if 0
  int d;
  // outer circle
  d = circle_inout(x, y, P_RADIUS);
  if (d < 0) return 0;
  if (d == 0) return 1;

  // horizontal axis
  if (y == 0) return 1;

  if (y < 0) y = -y; // mirror by y axis
  if (x >= 0) {                       // valid only if x >= 0
    if (x >= r/2){                    // valid only if x >= P_RADIUS/2
      // Constant Reactance Circle: 2j : R/2 = P_RADIUS/2 (mirror by y)
      if (circle_inout(x - P_RADIUS, y - P_RADIUS/2, P_RADIUS/2) == 0) return 1;

      // Constant Resistance Circle: 3 : R/4 = P_RADIUS/4
      d = circle_inout(x - 3*P_RADIUS/4, y, P_RADIUS/4);
      if (d > 0) return 0;
      if (d == 0) return 1;
    }
    // Constant Reactance Circle: 1j : R = P_RADIUS  (mirror by y)
    d = circle_inout(x - P_RADIUS, y - P_RADIUS, P_RADIUS);
    if (d == 0) return 1;

    // Constant Resistance Circle: 1 : R/2
    d = circle_inout(x - P_RADIUS/2, y, P_RADIUS/2);
    if (d > 0) return 0;
    if (d == 0) return 1;
  }
  // Constant Reactance Circle: 1/2j : R*2  (mirror by y)
  if (circle_inout(x - P_RADIUS, y - P_RADIUS*2, P_RADIUS*2) == 0) return 1;

  // Constant Resistance Circle: 1/3 : R*3/4
  if (circle_inout(x - P_RADIUS/4, y, P_RADIUS*3/4) == 0) return 1;
  return 0;
#else
  uint16_t r = P_RADIUS;
  // outer circle
  int32_t _r = x*x + y*y;
  int32_t d = _r;
  if (d > r*r + r) return 0;
  if (d > r*r - r) return 1;          // 1
  // horizontal axis
  if (y == 0) return 1;
  if (y <  0) y = -y; // mirror by y axis
  uint16_t r_y = r*y;                 // Radius limit ~256 pixels!! or need uint32
  if (x >= 0) {                       // valid only if x >= 0
    if (x >= r/2){
      // Constant Reactance Circle: 2j : R/2 = P_RADIUS/2 (mirror by y)
      d = _r - 2*r*x - r_y + r*r + r/2;
      if ((uint32_t)d <= r) return 1; // 2

      // Constant Resistance Circle: 3 : R/4 = P_RADIUS/4
      d = _r - (3*r/2)*x + r*r/2 + r/4;
      if (d <    0) return 0;
      if (d <= r/2) return 1;         // 3
    }
    // Constant Reactance Circle: 1j : R = P_RADIUS  (mirror by y)
    d = _r - 2*r*x - 2*r_y + r*r + r;
    if ((uint32_t)d<=2*r) return 1;   // 4

    // Constant Resistance Circle: 1 : R/2
    d = _r - r*x + r/2;
    if (d <  0) return 0;
    if (d <= r) return 1;             // 5
  }
  // Constant Reactance Circle: 1/2j : R*2  (mirror by y)
  d = _r - 2*r*x - 4*r_y + r*r + r*2;
  if ((uint32_t) d<= r*4) return 1;   // 6

  // Constant Resistance Circle: 1/3 : R*3/4
  d = _r - x*(r/2) - r*r/2 + r*3/4;
  if ((uint32_t)d<=r*3/2) return 1;   // 7
  return 0;
#endif
}

static void
cell_smith_grid(int x0, int y0, int w, int h, pixel_t color)
{
  int x, y;
  // offset to center
  x0 -= P_CENTER_X;
  y0 -= P_CENTER_Y;
  for (y = 0; y < h; y++)
    for (x = 0; x < w; x++)
      if (smith_grid(x + x0, y + y0)) cell_buffer[y * CELLWIDTH + x] = color;
}

#if 0
static void
cell_admit_grid(int x0, int y0, int w, int h, pixel_t color)
{
  int x, y;
  // offset to center
  x0 -= P_CENTER_X;
  y0 -= P_CENTER_Y;
  for (y = 0; y < h; y++)
    for (x = 0; x < w; x++)
      if (smith_grid(-(x + x0), y + y0)) cell_buffer[y * CELLWIDTH + x] = color;
}
#endif

void update_grid(void)
{
  freq_t gdigit = 100000000;
  freq_t fstart = get_sweep_frequency(ST_START);
  freq_t fspan  = get_sweep_frequency(ST_SPAN);
  freq_t grid;

  while (gdigit > 100) {
    grid = 5 * gdigit;
    if (fspan / grid >= 4)
      break;
    grid = 2 * gdigit;
    if (fspan / grid >= 4)
      break;
    grid = gdigit;
    if (fspan / grid >= 4)
      break;
    gdigit /= 10;
  }

  grid_offset = (WIDTH) * ((fstart % grid) / 100) / (fspan / 100);
  grid_width = (WIDTH) * (grid / 100) / (fspan / 1000);
}

static inline int
rectangular_grid_x(int x)
{
  x -= CELLOFFSETX;
  if ((uint32_t)x > WIDTH) return 0;
  if ((((x + grid_offset) * 10) % grid_width) < 10 || x == 0 || x == WIDTH)
    return 1;
  return 0;
}

static inline int
rectangular_grid_y(int y)
{
  if (y < 0 || (y % GRIDY))
    return 0;
  return 1;
}

/*
 * calculate log10(abs(gamma))
 */ 
static float
logmag(int i, const float *v)
{
  (void) i;
  float x = v[0]*v[0] + v[1]*v[1];
//  return log10f(x) *  10.0f;
//  return vna_logf(x) * (10.0f / logf(10.0f));
  return vna_log10f_x_10(x);
}

/*
 * calculate phase[-2:2] of coefficient
 */ 
static float
phase(int i, const float *v)
{
  (void) i;
  return (180.0f / VNA_PI) * vna_atan2f(v[1], v[0]);
}

/*
 * calculate groupdelay
 */
static float
groupdelay(const float *v, const float *w, uint32_t deltaf)
{
#if 1
  // atan(w)-atan(v) = atan((w-v)/(1+wv))
  float r = w[0]*v[0] + w[1]*v[1];
  float i = w[0]*v[1] - w[1]*v[0];
  return vna_atan2f(i, r) / (2 * VNA_PI * deltaf);
#else
  return (vna_atan2f(w[0], w[1]) - vna_atan2f(v[0], v[1])) / (2 * VNA_PI * deltaf);
#endif
}

/*
 * calculate abs(gamma)
 */
static float
linear(int i, const float *v)
{
  (void) i;
  return vna_sqrtf(v[0]*v[0] + v[1]*v[1]);
}

/*
 * calculate vswr; (1+gamma)/(1-gamma)
 */ 
static float
swr(int i, const float *v)
{
  (void) i;
  float x = linear(i, v);
  if (x > 0.99)
    return INFINITY;
  return (1 + x)/(1 - x);
}

#ifdef __VNA_Z_RENORMALIZATION__
#define PORT_Z current_props._portz
#else
#define PORT_Z 50.0f
#endif

static float
resistance(int i, const float *v)
{
  (void) i;
  const float z0 = PORT_Z;
  const float d = (1 - v[0])*(1 - v[0]) + v[1]*v[1];
  return z0 * (1 - v[0]*v[0] - v[1]*v[1]) / d;
}

static float
reactance(int i, const float *v)
{
  (void) i;
  const float z0 = PORT_Z;
  const float d = (1 - v[0])*(1 - v[0]) + v[1]*v[1];
  return 2 * z0 * v[1] / d;
}

static float
mod_z(int i, const float *v)
{
  (void) i;
  const float z0 = PORT_Z;
  const float d = (1 - v[0])*(1 - v[0]) + v[1]*v[1];
  return z0 * vna_sqrtf(4 * v[0] / d + 1); // always >= 0
}

static float
imag_lc(int i, const float *v)
{
  const float zi = reactance(i, v);
  float w = 2 * VNA_PI * getFrequency(i);
  if (zi < 0) return 1 / (w * zi); // Capacity
  return            zi / (w     ); // Inductive
}

static float
qualityfactor(int i, const float *v)
{
  (void) i;
  const float im = 2 * v[1];
  const float re = 1 - v[0]*v[0] - v[1]*v[1];
  return vna_fabsf(im / re);
}

static float
real(int i, const float *v)
{
  (void) i;
  return v[0];
}

static float
imag(int i, const float *v)
{
  (void) i;
  return v[1];
}

float
groupdelay_from_array(int i, const float *v)
{
  int bottom = (i ==              0) ? 0 : -1; // get prev point
  int top    = (i == sweep_points-1) ? 0 :  1; // get next point
  freq_t deltaf = get_sweep_frequency(ST_SPAN) / ((sweep_points - 1) / (top - bottom));
  return groupdelay(&v[2*bottom], &v[2*top], deltaf);
}

static inline void
cartesian_scale(const float *v, int16_t *xp, int16_t *yp, float scale)
{
  int16_t x = P_CENTER_X + float2int(v[0] * scale);
  int16_t y = P_CENTER_Y - float2int(v[1] * scale);
  if      (x <      0) x = 0;
  else if (x >  WIDTH) x = WIDTH;
  if      (y <      0) y = 0;
  else if (y > HEIGHT) y = HEIGHT;
  *xp = x;
  *yp = y;
}

#if MAX_TRACE_TYPE != 13
#error "Redefined trace_type list, need check format_list"
#endif

const trace_info_t trace_info_list[MAX_TRACE_TYPE] = {
// Type             name    format           delta format             ref  scale  get value
[TRC_LOGMAG] = {"LOGMAG", "%.2fdB",       S_DELTA"%.2fdB",       NGRIDY-1,  10.0, logmag               },
[TRC_PHASE]  = {"PHASE",  "%.1f"S_DEGREE, S_DELTA"%.2f"S_DEGREE, NGRIDY/2,  90.0, phase                },
[TRC_DELAY]  = {"DELAY",  "%.4Fs",        "%.4Fs",               NGRIDY/2,  1e-9, groupdelay_from_array},
[TRC_SMITH]  = {"SMITH",  NULL,           NULL,                         0,  1.00, NULL                 }, // Custom
[TRC_POLAR]  = {"POLAR",  "%.2f%+j.2f",   "%.2f%+j.2f",                 0,  1.00, NULL                 }, // Custom
//[TRC_ADMIT]= {"ADMIT",  "%.2f%+j.2f",   "%.2f%+j.2f",                 0,  1.00, NULL                 }, // Custom
[TRC_LINEAR] = {"LINEAR", "%.4f",         S_DELTA"%.3f",                0, 0.125, linear               },
[TRC_SWR]    = {"SWR",    "%.3f",         S_DELTA"%.3f",                0,  0.25, swr                  },
[TRC_REAL]   = {"REAL",   "%.4f",         S_DELTA"%.3f",         NGRIDY/2,  0.25, real                 },
[TRC_IMAG]   = {"IMAG",   "%.4fj",        S_DELTA"%.4fj",        NGRIDY/2,  0.25, imag                 },
[TRC_R]      = {"R",      "%.4F"S_OHM,    S_DELTA"%.4F"S_OHM,           0, 100.0, resistance           },
[TRC_X]      = {"X",      "%.4F"S_OHM,    S_DELTA"%.4F"S_OHM,    NGRIDY/2, 100.0, reactance            },
[TRC_Z]      = {"|Z|",    "%.3f",         S_DELTA"%.3f",                0,  50.0, mod_z                },
[TRC_Q]      = { "Q",     "%.3f",         S_DELTA"%.3f",                0,  10.0, qualityfactor        },
};

// Calculate and cache point coordinates for trace
static void
trace_into_index(int t, float array[POINTS_COUNT][2])
{
  uint16_t point_count = sweep_points-1;
  index_t *index = trace_index[t];
  uint32_t type    = 1<<trace[t].type;
  get_value_cb_t c = trace_info_list[trace[t].type].get_value_cb; // Get callback for value calculation
  float refpos = HEIGHT - (get_trace_refpos(t))*GRIDY + 0.5;  // 0.5 for pixel align
  float scale = get_trace_scale(t);
  if (type & RECTANGULAR_GRID_MASK) {                         // Run build for rect grid
    const float dscale = GRIDY / scale;
    if (type & (1<<TRC_SWR))  // For SWR need shift value by 1.0 down
      refpos+= dscale;
    uint16_t delta = WIDTH / point_count;
    uint16_t error = WIDTH % point_count;
    int32_t x = CELLOFFSETX, dx = (point_count>>1), y, i;
    for (i = 0; i <= point_count; i++, x+=delta) {
      float v = 0;
      if (c) v = c(i, array[i]);         // Get value
      if (v == INFINITY) {
        y = 0;
      } else {
        y = refpos - v * dscale;
             if (y <      0) y = 0;
        else if (y > HEIGHT) y = HEIGHT;
      }
      index[i].x = x;
      index[i].y = y;
      dx+=error; if (dx >=point_count) {x++; dx-= point_count;}
    }
    return;
  }
  // Smith/Polar grid
  if (type & ((1<<TRC_POLAR)|(1<<TRC_SMITH))){ // Need custom calculations
    const float rscale = P_RADIUS / scale;
    int16_t y, x, i;
    for (i = 0; i <= point_count; i++){
      cartesian_scale(array[i], &x, &y, rscale);
      index[i].x = x;
      index[i].y = y;
    }
    return;
  }
#if 0
  // Admit grid
  if (type & (1<<TRC_ADMIT)){ // Need custom calculations
    const float rscale = P_RADIUS / scale;
    int16_t y, x, i;
    for (i = 0; i <= point_count; i++){
      cartesian_scale(array[i], &x, &y, rscale);
      index[i].x = x;
      index[i].y = y;
    }
    return;
  }
#endif
}

static void
format_smith_value(int xpos, int ypos, const float *coeff, uint16_t idx)
{
  char *format;
  float zr, zi;
  switch (marker_smith_format) {
  case MS_LIN:
    zr = linear(idx, coeff);
    zi = phase(idx, coeff);
    format = "%.2f %.1f" S_DEGREE;
    break;
  case MS_LOG:
    zr = logmag(idx, coeff);
    zi = phase(idx, coeff);
    format = (zr == -INFINITY) ? "-"S_INFINITY" dB" : "%.1fdB %.1f" S_DEGREE;
    break;
  case MS_REIM:
    zr = real(idx, coeff);
    zi = imag(idx, coeff);
    format = "%F%+jF";
    break;
  case MS_RX:
    zr = resistance(idx, coeff);
    zi = reactance(idx, coeff);
    format = "%F%+jF"S_OHM;
    break;
  case MS_RLC:
    zr = resistance(idx, coeff);
    zi = imag_lc(idx, coeff);
    if (zi < 0)
      format = "%F"S_OHM" %FF"; // Capacity
    else
      format = "%F"S_OHM" %FH"; // Inductive
    zi = vna_fabsf(zi);
    break;
  default:
    return;
  }
  cell_printf(xpos, ypos, format, zr, zi);
}

static void
trace_print_value_string(int xpos, int ypos, int t, int index, int index_ref)
{
  // Check correct input
  int type = trace[t].type;
  if (type >= MAX_TRACE_TYPE) return;
  float (*array)[2] = measured[trace[t].channel];
  float v = 0.0f;
  float *coeff = array[index];
  float *coeff_ref = NULL;
  const char *format;
  // Get format data
  if (index_ref >=0){                           // Delta value
    coeff_ref = array[index_ref];
    format = trace_info_list[type].dformat;
  }
  else{                                         // No delta
    format = trace_info_list[type].format;
  }

  get_value_cb_t c = trace_info_list[type].get_value_cb;
  if (c){                                                   // Run standard get value function from table
    v = c(index, coeff);                                    // Get value
    if (coeff_ref && v != INFINITY) v-=c(index, coeff_ref); // Calculate delta value
  }
  else { // Need custom calculations
    switch (type) {
    case TRC_SMITH:
      format_smith_value(xpos, ypos, coeff, index);
      return;
    //case TRC_ADMIT:
    case TRC_POLAR:
      cell_printf(xpos, ypos, format, coeff[0], coeff[1]);
      return;
    default:
      return;
    }
  }
  cell_printf(xpos, ypos, format, v);
}

static int
trace_print_info(int xpos, int ypos, int t)
{
  float scale = get_trace_scale(t);
  char *format;
  switch (trace[t].type) {
    case TRC_LOGMAG: format = "%s %.0fdB/"; break;
    case TRC_PHASE:  format = "%s %.0f" S_DEGREE "/"; break;
    case TRC_SMITH:
    //case TRC_ADMIT:
    case TRC_POLAR:  format = (scale != 1.0) ? "%s %.1fFS" : "%s "; break;
    default:         format = "%s %F/"; break;
  }
  return cell_printf(xpos, ypos, format, get_trace_typename(t), scale);
}

static float time_of_index(int idx)
{
  freq_t span = get_sweep_frequency(ST_SPAN);
  return (idx * (sweep_points-1)) / ((float)FFT_SIZE * span);
}

static float distance_of_index(int idx)
{
  return velocity_factor * (SPEED_OF_LIGHT / 200.0f) * time_of_index(idx);
}

static inline void
swap_markmap(void)
{
  current_mappage^= 1;
}

static inline void
clear_markmap(void)
{
  memset(markmap[current_mappage], 0, sizeof markmap[current_mappage]);
}

/*
 * Force full screen update
 */
static inline void
force_set_markmap(void)
{
  memset(markmap[current_mappage], 0xff, sizeof markmap[current_mappage]);
}

/*
 * Force region of screen update
 */
static void
invalidate_rect_func(int x0, int y0, int x1, int y1)
{
  int x, y;
  if (y0 < 0            ) y0 = 0;
  if (y1 >=MAX_MARKMAP_Y) y1 = MAX_MARKMAP_Y-1;
  map_t *map = markmap[current_mappage];
  for (y = y0; y <= y1; y++)
    for (x = x0; x <= x1; x++)
      map[y]|= 1 << x;
}
#define invalidate_rect(x0, y0, x1, y1) invalidate_rect_func((x0)/CELLWIDTH, (y0)/CELLHEIGHT, (x1)/CELLWIDTH, (y1)/CELLHEIGHT)

#if STORED_TRACES > 0
static uint8_t enabled_store_trace = 0;
void storeCurrentTrace(int idx){
  if (current_trace == TRACE_INVALID) return;
  memcpy(trace_index[TRACES_MAX + idx], trace_index[current_trace], sizeof(trace_index[0]));
  enabled_store_trace|=1<<idx;
}

void disableStoredTrace(int idx){
  enabled_store_trace&=~(1<<idx);
}

static bool needProcessTrace(uint16_t idx) {
  if (idx < TRACES_MAX) {
    if (trace[idx].enabled)
      return true;
  }
  else if ((enabled_store_trace & (1<<(idx-TRACES_MAX))))
    return true;
  return false;
}
#else
#define enabled_store_trace 0
static bool needProcessTrace(uint16_t idx) {
  return trace[idx].enabled;
}
#endif

static void
mark_cells_from_index(void)
{
  int t, i, j;
  /* mark cells between each neighbor points */
  map_t *map = &markmap[current_mappage][0];
  for (t = 0; t < TRACE_INDEX_COUNT; t++) {
    if (!needProcessTrace(t))
      continue;
    index_t *index = trace_index[t];
    int m0 = index[0].x / CELLWIDTH;
    int n0 = index[0].y / CELLHEIGHT;
    map[n0] |= 1 << m0;
    for (i = 1; i < sweep_points; i++) {
      int m1 = index[i].x / CELLWIDTH;
      int n1 = index[i].y / CELLHEIGHT;
      if (m0 == m1 && n0 == n1)
        continue;
      int x0 = m0; int x1 = m1; if (x0>x1) SWAP(int, x0, x1); m0 = m1;
      int y0 = n0; int y1 = n1; if (y0>y1) SWAP(int, y0, y1); n0 = n1;
      for (; y0 <= y1; y0++)
        for (j = x0; j <= x1; j++)
          map[y0] |= 1 << j;
    }
  }
}

void set_area_size(uint16_t w, uint16_t h){
  area_width  = w;
  area_height = h;
}

static inline void
markmap_upperarea(void)
{
  // Hardcoded, Text info from upper area
#if  LCD_WIDTH  == 320 && _USE_FONT_== 1
	 invalidate_rect(0, 0, AREA_WIDTH_NORMAL, (MARKERS_MAX+ 1)*FONT_STR_HEIGHT);
#else
	invalidate_rect(0, 0, AREA_WIDTH_NORMAL, ((MARKERS_MAX+1)/2 + 1)*FONT_STR_HEIGHT);

#endif
}

//
// in most cases _compute_outcode clip calculation not give render line speedup
//
static inline void
cell_drawline(int x0, int y0, int x1, int y1, pixel_t c)
{
  if (x0 < 0 && x1 < 0) return;
  if (y0 < 0 && y1 < 0) return;
  if (x0 >= CELLWIDTH && x1 >= CELLWIDTH) return;
  if (y0 >= CELLHEIGHT && y1 >= CELLHEIGHT) return;

  // modifed Bresenham's line algorithm, see https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
  // Draw from top to bottom (most graph contain vertical lines)
  if (y1 < y0) { SWAP(int, x0, x1); SWAP(int, y0, y1); }
  int dx =-(x1 - x0), sx = 1; if (dx > 0) { dx = -dx; sx = -sx; }
  int dy = (y1 - y0);
  int err = ((dy + dx) < 0 ? -dx : -dy) / 2;
  // Fast skip points while y0 < 0
  if (y0 < 0){
    while(1){
      int e2 = err;
      if (e2 > dx) { err-= dy; x0+=sx;}
      if (e2 < dy) { err-= dx; y0++; if (y0 == 0) break;}
    }
  }
  // align y by CELLWIDTH for faster calculations
  y0*=CELLWIDTH;
  y1*=CELLWIDTH;
  while (1) {
    if ((uint32_t)x0 < CELLWIDTH)
      cell_buffer[y0 + x0]|= c;
    if (x0 == x1 && y0 == y1)
      return;
    int e2 = err;
    if (e2 > dx) { err-= dy; x0+=sx;}
    if (e2 < dy) { err-= dx; y0+=CELLWIDTH; if (y0>=CELLHEIGHT*CELLWIDTH) return;} // stop after cell bottom
  }
}

// Give a little speedup then draw rectangular plot (50 systick on all calls, all render req 700 systick)
// Write more difficult algorithm for search indexes not give speedup
static int
search_index_range_x(int x1, int x2, index_t *index, int *i0, int *i1)
{
  int i, j;
  int head = 0;
  int tail = sweep_points;
  int idx_x;

  // Search index point in cell
  while (1) {
    i = (head + tail) / 2;
    idx_x = index[i].x;
    if (idx_x >= x2) { // index after cell
      if (tail == i)
        return false;
      tail = i;
    }
    else if (idx_x < x1) {    // index before cell
      if (head == i)
        return false;
      head = i;
    }
    else  // index in cell (x =< idx_x < cell_end)
      break;
  }
  j = i;
  // Search index left from point
  do {
    if (j == 0) break;
    j--;
  } while (x1 <= index[j].x);
  *i0 = j;
  // Search index right from point
  do {
    if (i >=sweep_points-1) break;
    i++;
  } while (index[i].x < x2);
  *i1 = i;

  return TRUE;
}

static void
cell_blit_bitmap(int16_t x, int16_t y, uint16_t w, uint16_t h, const uint8_t *bmp)
{
  if (x <= -w)
    return;
  uint8_t bits = 0;
  int c = h+y, r;
  for (; y < c; y++) {
    for (r = 0; r < w; r++, bits<<=1) {
      if ((r&7)==0) bits = *bmp++;
      if ((0x80 & bits) == 0) continue;    // no pixel
      if ((uint32_t)(y+0) >= CELLHEIGHT) continue; // y   < 0 || y   >= CELLHEIGHT
      if ((uint32_t)(x+r) >= CELLWIDTH ) continue; // x+r < 0 || x+r >= CELLWIDTH
      cell_buffer[y*CELLWIDTH + x + r] = foreground_color;
    }
  }
}

typedef struct {
  const void *vmt;
  int16_t x;
  int16_t y;
} cellPrintStream;

static msg_t cellPut(void *ip, uint8_t ch) {
  cellPrintStream *ps = ip;
  if (ps->x < CELLWIDTH){
    uint16_t w = FONT_GET_WIDTH(ch);
    cell_blit_bitmap(ps->x, ps->y, w, FONT_GET_HEIGHT, FONT_GET_DATA(ch));
    ps->x+= w;
  }
  return MSG_OK;
}

// Simple print in buffer function
static int cell_printf(int16_t x, int16_t y, const char *fmt, ...) {
  static const struct lcd_printStreamVMT {
    _base_sequential_stream_methods
  } cell_vmt = {NULL, NULL, cellPut, NULL};
  // Skip print if not on cell (at top/bottom/right)
  if ((uint32_t)(y+FONT_GET_HEIGHT) >= CELLHEIGHT + FONT_GET_HEIGHT || x >= CELLWIDTH)
    return 0;
  va_list ap;
  // Init small cell print stream
  cellPrintStream ps = {&cell_vmt, x, y};
  // Performing the print operation using the common code.
  va_start(ap, fmt);
  int retval = chvprintf((BaseSequentialStream *)(void *)&ps, fmt, ap);
  va_end(ap);
  // Return number of bytes that would have been written.
  return retval;
}

#ifdef __VNA_MEASURE_MODULE__


typedef void (*measure_cell_cb_t)(int x0, int y0);
typedef void (*measure_prepare_cb_t)(uint8_t mode, uint8_t update_mask);

static measure_cell_cb_t    measure_cell_handler = NULL;
static uint8_t data_update = 0;

#define MESAURE_NONE 0
#define MESAURE_S11  1
#define MESAURE_S21  2
#define MESAURE_ALL  3

#define MEASURE_UPD_SWEEP  1
#define MEASURE_UPD_FREQ   2
#define MEASURE_UPD_ALL    3

// Include L/C match functions
#ifdef __USE_LC_MATCHING__
  #include "lc_matching.c"
#endif

static const struct {
  uint8_t option;
  uint8_t update;
  measure_cell_cb_t    measure_cell;
  measure_prepare_cb_t measure_prepare;
} measure[]={
  [MEASURE_NONE]        = {MESAURE_NONE,                0,               NULL,             NULL },
#ifdef __USE_LC_MATCHING__
  [MEASURE_LC_MATH]     = {MESAURE_NONE,  MEASURE_UPD_ALL,      draw_lc_match, prepare_lc_match },
#endif
#ifdef __S21_MEASURE__
  [MEASURE_SHUNT_LC]    = {MESAURE_S21, MEASURE_UPD_SWEEP, draw_serial_result, prepare_series   },
  [MEASURE_SERIES_LC]   = {MESAURE_S21, MEASURE_UPD_SWEEP, draw_serial_result, prepare_series   },
  [MEASURE_SERIES_XTAL] = {MESAURE_S21, MEASURE_UPD_SWEEP, draw_serial_result, prepare_series   },
#endif
#ifdef __S11_CABLE_MEASURE__
  [MEASURE_S11_CABLE]   = {MESAURE_S11, MEASURE_UPD_ALL,       draw_s11_cable, prepare_s11_cable},
#endif
};

static inline void measure_set_flag(uint8_t flag) {
  data_update|= flag;
}

void plot_set_measure_mode(uint8_t mode) {
  if (mode >= MEASURE_END) return;
  measure_cell_handler = measure[mode].measure_cell;
  current_props._measure = mode;
  data_update = 0xFF;
  request_to_redraw(REDRAW_AREA);
}

uint16_t plot_get_measure_channels(void) {
  return measure[current_props._measure].option;
}

static void measure_prepare(void) {
  if (current_props._measure == 0) return;
  measure_prepare_cb_t measure_cb = measure[current_props._measure].measure_prepare;
  // Do measure and cache data only if update flags some
  if (measure_cb && (data_update & measure[current_props._measure].update))
    measure_cb(current_props._measure, data_update);
  data_update = 0;
}

static void cell_draw_measure(int x0, int y0){
  if (measure_cell_handler == NULL) return;
  lcd_set_background(LCD_BG_COLOR);
  lcd_set_foreground(LCD_LC_MATCH_COLOR);
  measure_cell_handler(x0, y0);
}
#endif

// Reference bitmap (size and offset)
#define REFERENCE_WIDTH    6
#define REFERENCE_HEIGHT   5
#define REFERENCE_X_OFFSET 5
#define REFERENCE_Y_OFFSET 2
static const uint8_t reference_bitmap[]={
  _BMP8(0b11000000),
  _BMP8(0b11110000),
  _BMP8(0b11111100),
  _BMP8(0b11110000),
  _BMP8(0b11000000),
};

// Marker bitmaps (size and offsets)
#if _USE_BIG_MARKER_ == 0
#define MARKER_WIDTH       7
#define MARKER_HEIGHT     10
#define X_MARKER_OFFSET    3
#define Y_MARKER_OFFSET   10
#define MARKER_BITMAP(i)  (&marker_bitmap[(i)*MARKER_HEIGHT])
static const uint8_t marker_bitmap[]={
// Marker Back plate
  _BMP8(0b11111110),
  _BMP8(0b11111110),
  _BMP8(0b11111110),
  _BMP8(0b11111110),
  _BMP8(0b11111110),
  _BMP8(0b11111110),
  _BMP8(0b11111110),
  _BMP8(0b01111100),
  _BMP8(0b00111000),
  _BMP8(0b00010000),
  // Marker 1
  _BMP8(0b00000000),
  _BMP8(0b00010000),
  _BMP8(0b00110000),
  _BMP8(0b00010000),
  _BMP8(0b00010000),
  _BMP8(0b00010000),
  _BMP8(0b00111000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
#if MARKERS_MAX >=2
  // Marker 2
  _BMP8(0b00000000),
  _BMP8(0b00111000),
  _BMP8(0b01000100),
  _BMP8(0b00000100),
  _BMP8(0b00111000),
  _BMP8(0b01000000),
  _BMP8(0b01111100),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
#endif
#if MARKERS_MAX >=3
  // Marker 3
  _BMP8(0b00000000),
  _BMP8(0b00111000),
  _BMP8(0b01000100),
  _BMP8(0b00011000),
  _BMP8(0b00000100),
  _BMP8(0b01000100),
  _BMP8(0b00111000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
#endif
#if MARKERS_MAX >=4
  // Marker 4
  _BMP8(0b00000000),
  _BMP8(0b00001000),
  _BMP8(0b00011000),
  _BMP8(0b00101000),
  _BMP8(0b01001000),
  _BMP8(0b01001000),
  _BMP8(0b01111100),
  _BMP8(0b00001000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
#endif
#if MARKERS_MAX >=5
  // Marker 5
  _BMP8(0b00000000),
  _BMP8(0b01111100),
  _BMP8(0b01000000),
  _BMP8(0b01111000),
  _BMP8(0b00000100),
  _BMP8(0b01000100),
  _BMP8(0b00111000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
#endif
#if MARKERS_MAX >=6
  // Marker 6
  _BMP8(0b00000000),
  _BMP8(0b00111100),
  _BMP8(0b01000000),
  _BMP8(0b01111000),
  _BMP8(0b01000100),
  _BMP8(0b01000100),
  _BMP8(0b00111000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
#endif
#if MARKERS_MAX >=7
  // Marker 7
  _BMP8(0b00000000),
  _BMP8(0b01111100),
  _BMP8(0b01000100),
  _BMP8(0b00000100),
  _BMP8(0b00001000),
  _BMP8(0b00010000),
  _BMP8(0b00010000),
  _BMP8(0b00010000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
#endif
#if MARKERS_MAX >=8
  // Marker 8
  _BMP8(0b00000000),
  _BMP8(0b00111000),
  _BMP8(0b01000100),
  _BMP8(0b00111000),
  _BMP8(0b01000100),
  _BMP8(0b01000100),
  _BMP8(0b00111000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
  _BMP8(0b00000000),
#endif
};

#elif _USE_BIG_MARKER_ == 1
#define MARKER_WIDTH       11
#define MARKER_HEIGHT      14
#define X_MARKER_OFFSET     5
#define Y_MARKER_OFFSET    14
#define MARKER_BITMAP(i)   (&marker_bitmap[(i)*2*MARKER_HEIGHT])
static const uint8_t marker_bitmap[]={
  // Marker Back plate
  _BMP16(0b0000000000000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0111111111000000),
  _BMP16(0b0011111110000000),
  _BMP16(0b0001111100000000),
  _BMP16(0b0000111000000000),
  _BMP16(0b0000010000000000),
  // Marker 1
  _BMP16(0b1111111111100000),
  _BMP16(0b1000000000100000),
  _BMP16(0b1000011000100000),
  _BMP16(0b1000111000100000),
  _BMP16(0b1001011000100000),
  _BMP16(0b1000011000100000),
  _BMP16(0b1000011000100000),
  _BMP16(0b1000011000100000),
  _BMP16(0b1000011000100000),
  _BMP16(0b1000011000100000),
  _BMP16(0b0100111101000000),
  _BMP16(0b0010000010000000),
  _BMP16(0b0001000100000000),
  _BMP16(0b0000101000000000),
#if MARKERS_MAX >=2
  // Marker 2
  _BMP16(0b1111111111100000),
  _BMP16(0b1000000000100000),
  _BMP16(0b1000111100100000),
  _BMP16(0b1001100110100000),
  _BMP16(0b1001100110100000),
  _BMP16(0b1000000110100000),
  _BMP16(0b1000001100100000),
  _BMP16(0b1000111000100000),
  _BMP16(0b1001100000100000),
  _BMP16(0b1001100000100000),
  _BMP16(0b0101111101000000),
  _BMP16(0b0010000010000000),
  _BMP16(0b0001000100000000),
  _BMP16(0b0000101000000000),
#endif
#if MARKERS_MAX >=3
  // Marker 3
  _BMP16(0b1111111111100000),
  _BMP16(0b1000000000100000),
  _BMP16(0b1001111100100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1000000110100000),
  _BMP16(0b1000011100100000),
  _BMP16(0b1000000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b0101111101000000),
  _BMP16(0b0010000010000000),
  _BMP16(0b0001000100000000),
  _BMP16(0b0000101000000000),
#endif
#if MARKERS_MAX >=4
  // Marker 4
  _BMP16(0b1111111111100000),
  _BMP16(0b1000000000100000),
  _BMP16(0b1000001100100000),
  _BMP16(0b1000011100100000),
  _BMP16(0b1000111100100000),
  _BMP16(0b1001101100100000),
  _BMP16(0b1011001100100000),
  _BMP16(0b1011001100100000),
  _BMP16(0b1011111110100000),
  _BMP16(0b1000001100100000),
  _BMP16(0b0100001101000000),
  _BMP16(0b0010000010000000),
  _BMP16(0b0001000100000000),
  _BMP16(0b0000101000000000),
#endif
#if MARKERS_MAX >=5
  // Marker 5
  _BMP16(0b1111111111100000),
  _BMP16(0b1000000000100000),
  _BMP16(0b1011111110100000),
  _BMP16(0b1011000000100000),
  _BMP16(0b1011000000100000),
  _BMP16(0b1011111100100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1000000110100000),
  _BMP16(0b1000000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b0101111101000000),
  _BMP16(0b0010000010000000),
  _BMP16(0b0001000100000000),
  _BMP16(0b0000101000000000),
#endif
#if MARKERS_MAX >=6
  // Marker 6
  _BMP16(0b1111111111100000),
  _BMP16(0b1000000000100000),
  _BMP16(0b1001111100100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000000100000),
  _BMP16(0b1011011100100000),
  _BMP16(0b1011100110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b0101111101000000),
  _BMP16(0b0010000010000000),
  _BMP16(0b0001000100000000),
  _BMP16(0b0000101000000000),
#endif
#if MARKERS_MAX >=7
  // Marker 7
  _BMP16(0b1111111111100000),
  _BMP16(0b1000000000100000),
  _BMP16(0b1011111110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1000000110100000),
  _BMP16(0b1000001100100000),
  _BMP16(0b1000011000100000),
  _BMP16(0b1000110000100000),
  _BMP16(0b1000110000100000),
  _BMP16(0b1000110000100000),
  _BMP16(0b0100110001000000),
  _BMP16(0b0010000010000000),
  _BMP16(0b0001000100000000),
  _BMP16(0b0000101000000000),
#endif
#if MARKERS_MAX >=8
  // Marker 8
  _BMP16(0b1111111111100000),
  _BMP16(0b1000000000100000),
  _BMP16(0b1001111100100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1001111100100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b1011000110100000),
  _BMP16(0b0101111101000000),
  _BMP16(0b0010000010000000),
  _BMP16(0b0001000100000000),
  _BMP16(0b0000101000000000),
#endif
};
#endif

static void
markmap_marker(int marker)
{
  int t;
  if (!markers[marker].enabled)
    return;
  int mk_idx = markers[marker].index;
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    index_t *index = trace_index[t];
    int x = index[mk_idx].x - X_MARKER_OFFSET;
    int y = index[mk_idx].y - Y_MARKER_OFFSET;
    invalidate_rect(x, y, x+MARKER_WIDTH-1, y+MARKER_HEIGHT-1);
  }
}

static void
markmap_all_markers(void)
{
  int i;
  for (i = 0; i < MARKERS_MAX; i++) {
    if (!markers[i].enabled)
      continue;
    markmap_marker(i);
  }
  markmap_upperarea();
}

#if 0
static void
markmap_all_refpos(void)
{
  invalidate_rect(OFFSETX, OFFSETY, CELLOFFSETX+1, AREA_HEIGHT_NORMAL);
}
#endif

//
// Marker search functions
//
static bool _greater(int x, int y) { return x > y; }
static bool _lesser(int x, int y) { return x < y; }
void
marker_search(bool update)
{
  int i, value;
  int found = 0;
  if (current_trace == TRACE_INVALID || active_marker == MARKER_INVALID)
    return;
  // Select search index table
  index_t *index = trace_index[current_trace];
  // Select compare function (depend from config settings)
  bool (*compare)(int x, int y) = (VNA_mode & VNA_MODE_SEARCH_MIN) ? _lesser : _greater;
  for (i = 1, value = index[0].y; i < sweep_points; i++) {
    if ((*compare)(value, index[i].y)) {
      value = index[i].y;
      found = i;
    }
  }
  set_marker_index(active_marker, found);
  if (update)
    redraw_marker(active_marker);
}

void
marker_search_dir(int16_t from, int16_t dir)
{
  int i, value;
  int found = -1;
  if (current_trace == TRACE_INVALID || active_marker == MARKER_INVALID)
    return;
  // Select search index table
  index_t *index = trace_index[current_trace];
  // Select compare function (depend from config settings)
  bool (*compare)(int x, int y) = (VNA_mode & VNA_MODE_SEARCH_MIN) ? _lesser : _greater;
  // Search next
  for (i = from + dir,  value = index[from].y; i >= 0 && i < sweep_points; i+=dir) {
    if ((*compare)(value, index[i].y))
      break;
    value = index[i].y;
  }
  //
  for (; i >= 0 && i < sweep_points; i+=dir) {
    if ((*compare)(index[i].y, value))
      break;
    value = index[i].y;
    found = i;
  }
  if (found < 0) return;
  set_marker_index(active_marker, found);
  redraw_marker(active_marker);
}

int
distance_to_index(int8_t t, uint16_t idx, int16_t x, int16_t y)
{
  index_t *index = trace_index[t];
  x-= index[idx].x;
  y-= index[idx].y;
  return x*x + y*y;
}

int
search_nearest_index(int x, int y, int t)
{
  int min_i = -1;
  int min_d = MARKER_PICKUP_DISTANCE * MARKER_PICKUP_DISTANCE;
  int i;
  for (i = 0; i < sweep_points; i++) {
    int d = distance_to_index(t, i, x , y);
    if (d < min_d) {
      min_d = d;
      min_i = i;
    }
  }
  return min_i;
}

//
// Build graph data and cache it for output
//
void
plot_into_index(float array[2][POINTS_COUNT][2])
{
  int t;
//  START_PROFILE;
  // Cache trace data indexes
  for (t = 0; t < TRACES_MAX; t++) {
    if (trace[t].enabled)
      trace_into_index(t, array[trace[t].channel]);
  }
//  STOP_PROFILE;
  // Marker track on data update
  if (props_mode & TD_MARKER_TRACK)
    marker_search(false);
#ifdef __VNA_MEASURE_MODULE__
  // Current scan update
  measure_set_flag(MEASURE_UPD_SWEEP);
#endif

  // Build cell list for update from data indexes
  mark_cells_from_index();
  // Mark for update cells, and add markers
  request_to_redraw(REDRAW_MARKER | REDRAW_CELLS);
}

#ifdef __USE_GRID_VALUES__
static void cell_grid_line_info(int x0, int y0)
{
  // Skip not selected trace
  if (current_trace == TRACE_INVALID) return;
  // Skip for SMITH/POLAR and off trace
  uint32_t trace_type = 1 << trace[current_trace].type;
  if (trace_type & (ROUND_GRID_MASK | (1 << TRC_OFF))) return;
  // Render at right
  int16_t xpos = GRID_X_TEXT - x0;
  int16_t ypos = 0           - y0 + 2;
  // Get top value
  float scale = get_trace_scale(current_trace);
  float   ref = get_trace_refpos(current_trace);
  float     v = (NGRIDY - ref) * scale;
  if (trace_type&(1 << TRC_SWR)) v+=1.0f;  // For SWR trace, value shift by 1.0
  // Render grid values
  lcd_set_foreground(LCD_TRACE_1_COLOR + current_trace);
  do {
    cell_printf(xpos, ypos, "% 6.3F", v); v-=scale;
  }while((ypos+=GRIDY) < CELLHEIGHT);
}
#endif

static void
draw_cell(int m, int n)
{
  int x0 = m * CELLWIDTH;
  int y0 = n * CELLHEIGHT;
  int w = CELLWIDTH;
  int h = CELLHEIGHT;
  int x, y;
  int i0, i1, i;
  int t;
  pixel_t c;
  // Clip cell by area
  if (w > area_width - x0)
    w = area_width - x0;
  if (h > area_height - y0)
    h = area_height - y0;
  if (w <= 0 || h <= 0)
    return;
//  PULSE;
  cell_buffer = lcd_get_cell_buffer();
  // Clear buffer ("0 : height" lines)
#if 0
  // use memset 350 system ticks for all screen calls
  // as understand it use 8 bit set, slow down on 32 bit systems
  memset(spi_buffer,  GET_PALTETTE_COLOR(LCD_BG_COLOR), (h*CELLWIDTH)*sizeof(uint16_t));
#else
  // use direct set  35 system ticks for all screen calls
#if CELLWIDTH%8 != 0
#error "CELLWIDTH % 8 should be == 0 for speed, or need rewrite cell cleanup"
#endif
#if LCD_PIXEL_SIZE == 2
  // Set DEFAULT_BG_COLOR for 8 pixels in one cycle
  int count = h*CELLWIDTH / 8;
  uint32_t *p = (uint32_t *)cell_buffer;
  uint32_t clr = GET_PALTETTE_COLOR(LCD_BG_COLOR) | (GET_PALTETTE_COLOR(LCD_BG_COLOR) << 16);
  while (count--) {
    p[0] = clr;
    p[1] = clr;
    p[2] = clr;
    p[3] = clr;
    p += 4;
  }
#elif  LCD_PIXEL_SIZE == 1
  // Set DEFAULT_BG_COLOR for 16 pixels in one cycle
  int count = h*CELLWIDTH / 16;
  uint32_t *p = (uint32_t *)cell_buffer;
  uint32_t clr = (GET_PALTETTE_COLOR(LCD_BG_COLOR)<< 0)|(GET_PALTETTE_COLOR(LCD_BG_COLOR)<< 8) |
                 (GET_PALTETTE_COLOR(LCD_BG_COLOR)<<16)|(GET_PALTETTE_COLOR(LCD_BG_COLOR)<<24);
  while (count--) {
    p[0] = clr;
    p[1] = clr;
    p[2] = clr;
    p[3] = clr;
    p += 4;
  }
#else
#error "Write cell fill for different  LCD_PIXEL_SIZE"
#endif

#endif

// Draw grid
#if 1
  c = GET_PALTETTE_COLOR(LCD_GRID_COLOR);
  // Generate grid type list
  uint32_t trace_type = 0;
  int t_count = 0;
  for (t = 0; t < TRACES_MAX; t++) {
    if (trace[t].enabled) {
      trace_type |= (1 << trace[t].type);
      t_count++;
    }
  }
  const int step = (VNA_mode & VNA_MODE_DOT_GRID) ? 2 : 1;
  // Draw rectangular plot (40 system ticks for all screen calls)
  if (trace_type & RECTANGULAR_GRID_MASK) {
    for (x = 0; x < w; x++) {
      if (rectangular_grid_x(x + x0)) {
        for (y = 0; y < h*CELLWIDTH; y+=step*CELLWIDTH) cell_buffer[y + x] = c;
      }
    }
    for (y = 0; y < h; y++) {
      if (rectangular_grid_y(y + y0)) {
        for (x = 0; x < w; x+=step)
          if ((uint32_t)(x + x0 - CELLOFFSETX) <= WIDTH)
            cell_buffer[y * CELLWIDTH + x] = c;
      }
    }
  }
  // Smith greed line (1000 system ticks for all screen calls)
  if (trace_type & (1 << TRC_SMITH))
    cell_smith_grid(x0, y0, w, h, c);
  // Polar greed line (800 system ticks for all screen calls)
  else if (trace_type & (1 << TRC_POLAR))
    cell_polar_grid(x0, y0, w, h, c);
#if 0
  else if (trace_type & (1 << TRC_ADMIT))
    cell_admit_grid(x0, y0, w, h, c);
#endif
#endif

#ifdef __USE_GRID_VALUES__
  // Only right cells
  if ((VNA_mode & VNA_MODE_SHOW_GRID) && m >= (GRID_X_TEXT)/CELLWIDTH)
    cell_grid_line_info(x0, y0);
#endif

//  PULSE;
// Draw traces (50-600 system ticks for all screen calls, depend from lines
// count and size)
#if 1
  for (t = 0; t < TRACE_INDEX_COUNT; t++) {
    if (!needProcessTrace(t))
      continue;
    c = GET_PALTETTE_COLOR(LCD_TRACE_1_COLOR + t);
    index_t *index = trace_index[t];
    i0 = i1 = 0;
    // draw rectangular plot (search index range in cell, save 50-70 system ticks for all screen calls)
    if ((1 << trace[t].type) & RECTANGULAR_GRID_MASK && !enabled_store_trace){
      search_index_range_x(x0, x0 + w, index, &i0, &i1);
    }else{
      // draw polar plot (check all points)
      i1 = sweep_points - 1;
    }
#if 1
    // Line mode
    for (i = i0; i < i1; i++) {
      int x1 = index[i].x - x0;
      int y1 = index[i].y - y0;
      int x2 = index[i + 1].x - x0;
      int y2 = index[i + 1].y - y0;
      cell_drawline(x1, y1, x2, y2, c);
    }
#else
    // Dot mode
    for (i = i0; i < i1; i++) {
      int x = index[i].x - x0;
      int y = index[i].y - y0;
      if ((uint32_t)x < CELLWIDTH && (uint32_t)y < CELLHEIGHT)
        cell_buffer[y * CELLWIDTH + x] = c;
    }
#endif
  }
#else
  for (x = 0; x < area_width; x += 6)
    cell_drawline(x - x0, 0 - y0, area_width - x - x0, area_height - y0,
                  config.trace_color[0]);
#endif
//  PULSE;
// draw marker symbols on each trace (<10 system ticks for all screen calls)
  int m_count = 0;
#if 1
  for (i = 0; i < MARKERS_MAX; i++) {
    if (!markers[i].enabled)
      continue;
    m_count++;
    for (t = 0; t < TRACES_MAX; t++) {
      if (!trace[t].enabled)
        continue;
      index_t *index = trace_index[t];
      int16_t mk_idx = markers[i].index;
      int16_t x = index[mk_idx].x - x0 - X_MARKER_OFFSET;
      int16_t y = index[mk_idx].y - y0 - Y_MARKER_OFFSET;
      // Check marker icon on cell
      if ((uint32_t)(x+MARKER_WIDTH ) < (CELLWIDTH  + MARKER_WIDTH ) &&
          (uint32_t)(y+MARKER_HEIGHT) < (CELLHEIGHT + MARKER_HEIGHT)){
          // Draw marker plate
          lcd_set_foreground(LCD_TRACE_1_COLOR + t);
          cell_blit_bitmap(x, y, MARKER_WIDTH, MARKER_HEIGHT, MARKER_BITMAP(0));
          // Draw marker number
          lcd_set_foreground(LCD_BG_COLOR);
          cell_blit_bitmap(x, y, MARKER_WIDTH, MARKER_HEIGHT, MARKER_BITMAP(i+1));
      }
    }
  }
#endif
// Draw trace and marker info on the top (50 system ticks for all screen calls)
#if 1
  int cnt = t_count > m_count ? t_count : m_count;
  // Get max marker/trace string count add one string for edelay/marker freq
#if  LCD_WIDTH  == 320 && _USE_FONT_== 1
  if (n <= ((cnt+  1)*FONT_STR_HEIGHT)/CELLHEIGHT)
#else
  if (n <= (((cnt+1)/2 + 1)*FONT_STR_HEIGHT)/CELLHEIGHT)

#endif
    cell_draw_marker_info(x0, y0);
#endif

// Measure data output
#ifdef __VNA_MEASURE_MODULE__
  cell_draw_measure(x0, y0);
#endif
//  PULSE;
// Draw reference position (<10 system ticks for all screen calls)
  for (t = 0; t < TRACES_MAX; t++) {
    // Skip draw reference position for disabled/smith/polar traces
    if (!trace[t].enabled || ((1 << trace[t].type) & (ROUND_GRID_MASK | (1 << TRC_OFF))))
      continue;
    int x = 0 - x0 + CELLOFFSETX - REFERENCE_X_OFFSET;
    if ((uint32_t)(x + REFERENCE_WIDTH) < CELLWIDTH + REFERENCE_WIDTH) {
      int y = HEIGHT - float2int(get_trace_refpos(t) * GRIDY) - y0 - REFERENCE_Y_OFFSET;
      if ((uint32_t)(y + REFERENCE_HEIGHT) < CELLHEIGHT + REFERENCE_HEIGHT){
        lcd_set_foreground(LCD_TRACE_1_COLOR + t);
        cell_blit_bitmap(x , y, REFERENCE_WIDTH, REFERENCE_HEIGHT, reference_bitmap);
      }
    }
  }
// Need right clip cell render (25 system ticks for all screen calls)
#if 1
  if (w < CELLWIDTH) {
    pixel_t *src = cell_buffer + CELLWIDTH;
    pixel_t *dst = cell_buffer + w;
    for (y = h; --y; src += CELLWIDTH - w)
      for (x = w; x--;)
        *dst++ = *src++;
  }
#endif
  // Draw cell (500 system ticks for all screen calls)
  lcd_bulk_continue(OFFSETX + x0, OFFSETY + y0, w, h);
}

static void
draw_all_cells(bool flush_markmap)
{
  int m, n;
#ifdef __VNA_MEASURE_MODULE__
  measure_prepare();
#endif
//  START_PROFILE
  for (n = 0; n < (area_height+CELLHEIGHT-1) / CELLHEIGHT; n++){
    map_t update_map = markmap[0][n] | markmap[1][n];
    if (update_map == 0) continue;
    for (m = 0; update_map; update_map>>=1, m++)
      if (update_map & 1)
        draw_cell(m, n);
  }

#if 0
  lcd_bulk_finish();
  for (m = 0; m < (area_width+CELLWIDTH-1) / CELLWIDTH; m++)
    for (n = 0; n < (area_height+CELLHEIGHT-1) / CELLHEIGHT; n++) {
      if ((markmap[0][n] | markmap[1][n]) & (1 << m))
        lcd_set_background(LCD_LOW_BAT_COLOR);
      else
        lcd_set_background(LCD_NORMAL_BAT_COLOR);
      lcd_fill(m*CELLWIDTH+OFFSETX, n*CELLHEIGHT, 2, 2);
    }
#endif
  if (flush_markmap) {
    // keep current map for update
    swap_markmap();
    // clear map for next plotting
    clear_markmap();
  }
  // Flush LCD buffer, wait completion (need call after end use lcd_bulk_continue mode)
  lcd_bulk_finish();
//  STOP_PROFILE
}

void
draw_all(bool flush)
{
  if (area_width == 0) {redraw_request = 0; return;}
  if (redraw_request & REDRAW_CLRSCR){
    lcd_set_background(LCD_BG_COLOR);
    lcd_clear_screen();
  }
  if (redraw_request & REDRAW_AREA)
    force_set_markmap();
  else {
    if (redraw_request & REDRAW_MARKER) markmap_all_markers();
  }
  if (redraw_request & (REDRAW_CELLS | REDRAW_MARKER | REDRAW_AREA))
    draw_all_cells(flush);
  if (redraw_request & REDRAW_FREQUENCY)
    draw_frequencies();
  if (redraw_request & REDRAW_CAL_STATUS)
    draw_cal_status();
  if (redraw_request & REDRAW_BATTERY)
    draw_battery_status();
  redraw_request = 0;
}

//
// Call this function then need fast draw marker and marker info
// Used in ui.c for leveler move marker, drag marker and etc.
void
redraw_marker(int8_t marker)
{
  if (marker == MARKER_INVALID)
    return;
  // mark map on new position of marker
  markmap_marker(marker);

#ifdef __VNA_MEASURE_MODULE__
  if (marker == active_marker)
    measure_set_flag(MEASURE_UPD_FREQ);
#endif
  // mark cells on marker info
  markmap_upperarea();

  draw_all_cells(TRUE);
  // Force redraw all area after (disable artifacts after fast marker update area)
  request_to_redraw(REDRAW_AREA);
}

// Marker and trace data position
#if  LCD_WIDTH  == 320 && _USE_FONT_== 1
static const struct {uint16_t x, y;} marker_pos[]={
  {1 +             CELLOFFSETX, 1                    },
  {1 +             CELLOFFSETX, 1 +   FONT_STR_HEIGHT},
  {1 +             CELLOFFSETX, 1 + 2*FONT_STR_HEIGHT},
  {1 +             CELLOFFSETX, 1 + 3*FONT_STR_HEIGHT},
  {1 +             CELLOFFSETX, 1 + 4*FONT_STR_HEIGHT},
  {1 +             CELLOFFSETX, 1 + 5*FONT_STR_HEIGHT},
  {1 +             CELLOFFSETX, 1 + 6*FONT_STR_HEIGHT},
  {1 +             CELLOFFSETX, 1 + 7*FONT_STR_HEIGHT},
};

#else

static const struct {uint16_t x, y;} marker_pos[]={
  {1 +             CELLOFFSETX, 1                    },
  {1 + (WIDTH/2) + CELLOFFSETX, 1                    },
  {1 +             CELLOFFSETX, 1 +   FONT_STR_HEIGHT},
  {1 + (WIDTH/2) + CELLOFFSETX, 1 +   FONT_STR_HEIGHT},
  {1 +             CELLOFFSETX, 1 + 2*FONT_STR_HEIGHT},
  {1 + (WIDTH/2) + CELLOFFSETX, 1 + 2*FONT_STR_HEIGHT},
  {1 +             CELLOFFSETX, 1 + 3*FONT_STR_HEIGHT},
  {1 + (WIDTH/2) + CELLOFFSETX, 1 + 3*FONT_STR_HEIGHT},
};
#endif

#if _USE_FONT_== 0
#define MARKER_FREQ       "%.6qHz"
#define MARKER_FREQ_SIZE        67
#define PORT_Z_OFFSET            1
#else
#define MARKER_FREQ         "%qHz"
#define MARKER_FREQ_SIZE       112
#define PORT_Z_OFFSET            0
#endif

static void
cell_draw_marker_info(int x0, int y0)
{
  int t, mk, xpos, ypos;
  if (active_marker == MARKER_INVALID)
    return;
  int active_marker_idx = markers[active_marker].index;
  int j = 0;
  // Marker (for current selected trace) display mode (selected more then 1 marker, and minimum one trace)
  if (previous_marker != MARKER_INVALID && current_trace != TRACE_INVALID) {
    t = current_trace;
    for (mk = 0; mk < MARKERS_MAX; mk++) {
      if (!markers[mk].enabled)
        continue;
      xpos = marker_pos[j].x - x0;
      ypos = marker_pos[j].y - y0;
      j++;
      lcd_set_foreground(LCD_TRACE_1_COLOR + t);
      if (mk == active_marker)
        cell_printf(xpos, ypos, S_SARROW);
      xpos += FONT_WIDTH;
      cell_printf(xpos, ypos, "M%d", mk+1);
      xpos += 3*FONT_WIDTH - 2;
      int32_t  delta_index = -1;
      uint32_t mk_index = markers[mk].index;
      freq_t freq = get_marker_frequency(mk);
      if ((props_mode & TD_MARKER_DELTA) && mk != active_marker) {
        freq_t freq1 = get_marker_frequency(active_marker);
        freq_t delta = freq > freq1 ? freq - freq1 : freq1 - freq;
        delta_index = active_marker_idx;
        cell_printf(xpos, ypos, S_DELTA MARKER_FREQ, delta);
      } else {
        cell_printf(xpos, ypos, MARKER_FREQ, freq);
      }
      xpos += MARKER_FREQ_SIZE;
      lcd_set_foreground(LCD_FG_COLOR);
      trace_print_value_string(xpos, ypos, t, mk_index, delta_index);
    }
  } else /*if (active_marker != MARKER_INVALID)*/{ // Trace display mode
    for (t = 0; t < TRACES_MAX; t++) {
      if (!trace[t].enabled)
        continue;
      xpos = marker_pos[j].x - x0;
      ypos = marker_pos[j].y - y0;
      j++;
      lcd_set_foreground(LCD_TRACE_1_COLOR + t);
      if (t == current_trace)
        cell_printf(xpos, ypos, S_SARROW);
      xpos += FONT_WIDTH;
      cell_printf(xpos, ypos, get_trace_chname(t));
      xpos += 3*FONT_WIDTH + 4;

      int n = trace_print_info(xpos, ypos, t);
      xpos += n * FONT_WIDTH + 2;
      lcd_set_foreground(LCD_FG_COLOR);
      trace_print_value_string(xpos, ypos, t, active_marker_idx, -1);
    }
  }

  lcd_set_foreground(LCD_FG_COLOR);
  // Marker frequency data print
#if  LCD_WIDTH  == 320 && _USE_FONT_== 1
  xpos =  12+(WIDTH/2) + CELLOFFSETX   - x0;
    ypos =  1 + j*FONT_STR_HEIGHT - y0;
#else

    xpos = 3 + (WIDTH/2) + CELLOFFSETX   - x0;
    ypos =  1 + ((j+1)/2)*FONT_STR_HEIGHT - y0;
#endif

  if (previous_marker != MARKER_INVALID && current_trace != TRACE_INVALID) {
    // draw marker delta
    if (!(props_mode & TD_MARKER_DELTA) && active_marker != previous_marker) {
      int previous_marker_idx = markers[previous_marker].index;
      cell_printf(xpos, ypos, S_DELTA"%d-%d:", active_marker+1, previous_marker+1);
      xpos += 5*FONT_WIDTH + 2;
      if ((props_mode & DOMAIN_MODE) == DOMAIN_FREQ) {
        freq_t freq  = get_marker_frequency(active_marker);
        freq_t freq1 = get_marker_frequency(previous_marker);
        freq_t delta = freq >= freq1 ? freq - freq1 : freq1 - freq;
        cell_printf(xpos, ypos, "%c%qHz", freq >= freq1 ? '+' : '-', delta);
      } else {
        cell_printf(xpos, ypos, "%Fs (%Fm)", time_of_index(active_marker_idx) - time_of_index(previous_marker_idx),
                                             distance_of_index(active_marker_idx) - distance_of_index(previous_marker_idx));
      }
    }
  }
  else /*if (active_marker != MARKER_INVALID)*/{
    // draw marker frequency
    if (lever_mode == LM_MARKER)
      cell_printf(xpos, ypos, S_SARROW);
    xpos += FONT_WIDTH;
    cell_printf(xpos, ypos, "M%d:", active_marker+1);
    //cell_drawstring(buf, xpos, ypos);
    xpos += 3*FONT_WIDTH + 4;
    if ((props_mode & DOMAIN_MODE) == DOMAIN_FREQ)
      cell_printf(xpos, ypos, "%qHz", get_marker_frequency(active_marker));
    else
      cell_printf(xpos, ypos, "%Fs (%Fm)", time_of_index(active_marker_idx), distance_of_index(active_marker_idx));
  }

  if (electrical_delay != 0.0f) {
    // draw electrical delay
#if  LCD_WIDTH  == 320 && _USE_FONT_== 1
	    xpos = 1      - x0;
	    ypos = 1 + (j)*FONT_STR_HEIGHT - y0;
#else
    xpos = 1 + 18 + CELLOFFSETX          - x0;
    ypos = 1 + ((j+1)/2)*FONT_STR_HEIGHT - y0;
#endif

    if (lever_mode == LM_EDELAY)
      cell_printf(xpos, ypos, S_SARROW);
    xpos += 5;

    float edelay = electrical_delay * 1e-12; // to seconds
    cell_printf(xpos, ypos, "Edelay: %Fs (%Fm)", edelay, edelay * (SPEED_OF_LIGHT / 100.0f) * velocity_factor);
  }
#ifdef __VNA_Z_RENORMALIZATION__
  if (current_props._portz != 50.0f) {
    xpos = 1 + 18 + CELLOFFSETX          - x0;
    ypos = PORT_Z_OFFSET + ((j+1)/2 + 1)*FONT_STR_HEIGHT - y0;
    cell_printf(xpos, ypos, "PORT-Z: 50 " S_RARROW " %F" S_OHM, current_props._portz);
  }
#endif
}

static void
draw_frequencies(void)
{
  char lm0 = lever_mode == LM_FREQ_0 ? S_SARROW[0] : ' ';
  char lm1 = lever_mode == LM_FREQ_1 ? S_SARROW[0] : ' ';
  // Draw frequency string
  lcd_set_foreground(LCD_FG_COLOR);
  lcd_set_background(LCD_BG_COLOR);
  lcd_fill(0, FREQUENCIES_YPOS, LCD_WIDTH, FONT_GET_HEIGHT);
  // Prepare text for frequency string
  if ((props_mode & DOMAIN_MODE) == DOMAIN_FREQ) {
    if (FREQ_IS_CW()) {
      lcd_printf(FREQUENCIES_XPOS1, FREQUENCIES_YPOS, "%c%s %15qHz", lm0, "CW", get_sweep_frequency(ST_CW));
    } else if (FREQ_IS_STARTSTOP()) {
      lcd_printf(FREQUENCIES_XPOS1, FREQUENCIES_YPOS, "%c%s %15qHz", lm0, "START", get_sweep_frequency(ST_START));
      lcd_printf(FREQUENCIES_XPOS2, FREQUENCIES_YPOS, "%c%s %15qHz", lm1,  "STOP", get_sweep_frequency(ST_STOP));
    } else if (FREQ_IS_CENTERSPAN()) {
      lcd_printf(FREQUENCIES_XPOS1, FREQUENCIES_YPOS, "%c%s %15qHz", lm0,"CENTER", get_sweep_frequency(ST_CENTER));
      lcd_printf(FREQUENCIES_XPOS2, FREQUENCIES_YPOS, "%c%s %15qHz", lm1,  "SPAN", get_sweep_frequency(ST_SPAN));
    }
  } else {
    lcd_printf(FREQUENCIES_XPOS1, FREQUENCIES_YPOS, "%c%s 0s",        lm0, "START");
    lcd_printf(FREQUENCIES_XPOS2, FREQUENCIES_YPOS, "%c%s %Fs (%Fm)", lm1, "STOP", time_of_index(sweep_points-1), distance_of_index(sweep_points-1));
  }
  // Draw bandwidth and point count
#if  LCD_WIDTH  == 480 || _USE_FONT_== 0
  lcd_set_foreground(LCD_BW_TEXT_COLOR);
  lcd_printf(FREQUENCIES_XPOS3, FREQUENCIES_YPOS,"bw:%uHz %up", get_bandwidth_frequency(config._bandwidth), sweep_points);
#elif  LCD_WIDTH  == 320 && _USE_FONT_== 3
  lcd_set_foreground(LCD_BW_TEXT_COLOR);
  lcd_printf(FREQUENCIES_XPOS3, FREQUENCIES_YPOS,"bw:%uHz", get_bandwidth_frequency(config._bandwidth));
#endif
}

/*
 * Draw/update calibration status panel
 */
static void
draw_cal_status(void)
{
  uint32_t i;
  int x = CALIBRATION_INFO_POSX;
  int y = CALIBRATION_INFO_POSY;
  lcd_set_background(LCD_BG_COLOR);
  lcd_set_foreground(LCD_DISABLE_CAL_COLOR);
  lcd_fill(x, y, OFFSETX - x, 10*(FONT_STR_HEIGHT));

  if (cal_status & CALSTAT_APPLY) {
    // Set 'C' string for slot status
    char c[4] = {'C', '0' + lastsaveid, 0, 0};
    if (lastsaveid == NO_SAVE_SLOT) c[1] = '*';
    if (cal_status & CALSTAT_INTERPOLATED){lcd_set_foreground(LCD_INTERP_CAL_COLOR); c[0] = 'c';}
    else  lcd_set_foreground(LCD_FG_COLOR);
    lcd_drawstring(x, y, c);
    lcd_set_foreground(LCD_FG_COLOR);
  }

  static const struct {char text, zero; uint16_t mask;} calibration_text[]={
    {'O', 0, CALSTAT_OPEN},
    {'S', 0, CALSTAT_SHORT},
    {'D', 0, CALSTAT_ED},
    {'R', 0, CALSTAT_ER},
    {'S', 0, CALSTAT_ES},
    {'T', 0, CALSTAT_ET},
    {'t', 0, CALSTAT_THRU},
    {'X', 0, CALSTAT_EX}
  };
  for (i = 0; i < ARRAY_COUNT(calibration_text); i++)
    if (cal_status & calibration_text[i].mask)
      lcd_drawstring(x, y+=FONT_STR_HEIGHT, &calibration_text[i].text);

  if ((cal_status & CALSTAT_APPLY) && cal_power != current_props._power)
    lcd_set_foreground(LCD_DISABLE_CAL_COLOR);

  // 2,4,6,8 mA power or auto
  lcd_printf(x, y+=FONT_STR_HEIGHT, "P%c", current_props._power > 3 ? ('a') : (current_props._power * 2 + '2'));
#ifdef __USE_SMOOTH__
  y+=FONT_STR_HEIGHT;
  uint8_t smooth = get_smooth_factor();
  if (smooth > 0){
    lcd_set_foreground(LCD_FG_COLOR);
    lcd_printf(x, y+=FONT_STR_HEIGHT, "s%d", smooth);
  }
#endif
}

/*
 * Draw battery level
 */
#define BATTERY_TOP_LEVEL       4100
#define BATTERY_BOTTOM_LEVEL    3200
#define BATTERY_WARNING_LEVEL   3300
static void draw_battery_status(void)
{
  int16_t vbat = adc_vbat_read();
  if (vbat <= 0)
    return;
  uint8_t string_buf[24];
  // Set battery color
  lcd_set_foreground(vbat < BATTERY_WARNING_LEVEL ? LCD_LOW_BAT_COLOR : LCD_NORMAL_BAT_COLOR);
  lcd_set_background(LCD_BG_COLOR);
//  plot_printf(string_buf, sizeof string_buf, "V:%d", vbat);
//  lcd_drawstringV(string_buf, 1, 60);
  // Prepare battery bitmap image
  // Battery top
  int x = 0;
  string_buf[x++] = 0b00000000;
  string_buf[x++] = 0b00111100;
  string_buf[x++] = 0b00111100;
  string_buf[x++] = 0b11111111;
  // Fill battery status
  for (int power=BATTERY_TOP_LEVEL; power > BATTERY_BOTTOM_LEVEL; ){
    if ((x&3) == 0) {string_buf[x++] = 0b10000001; continue;}
    string_buf[x++] = (power > vbat) ? 0b10000001 : // Empty line
                                       0b10111101;  // Full line
    power-=100;
  }
  // Battery bottom
  string_buf[x++] = 0b10000001;
  string_buf[x++] = 0b11111111;
  // Draw battery
  lcd_blitBitmap(BATTERY_ICON_POSX, BATTERY_ICON_POSY, 8, x, string_buf);
}

/*
 * Update cells behind menu
 */
void
request_to_draw_cells_behind_menu(void)
{
  // Values Hardcoded from ui.c
  invalidate_rect(LCD_WIDTH-MENU_BUTTON_WIDTH-OFFSETX, 0, LCD_WIDTH-OFFSETX, LCD_HEIGHT-1);
  request_to_redraw(REDRAW_CELLS);
}

/*
 * Update cells behind numeric input
 */
void
request_to_draw_cells_behind_numeric_input(void)
{
  // Values Hardcoded from ui.c
  invalidate_rect(0, LCD_HEIGHT-NUM_INPUT_HEIGHT, LCD_WIDTH-1, LCD_HEIGHT-1);
  request_to_redraw(REDRAW_CELLS);
}

/*
 * Set update mask for next screen update
 */
void
request_to_redraw(uint8_t mask)
{
  redraw_request|= mask;
}

void
plot_init(void)
{
  request_to_redraw(REDRAW_AREA | REDRAW_BATTERY | REDRAW_CAL_STATUS | REDRAW_FREQUENCY);
  draw_all(true);
}
