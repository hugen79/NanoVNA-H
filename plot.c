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

#include <math.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "nanovna.h"

static void cell_draw_marker_info(int x0, int y0);
static void draw_battery_status(void);

static int16_t grid_offset;
static int16_t grid_width;

int16_t area_width  = AREA_WIDTH_NORMAL;
int16_t area_height = AREA_HEIGHT_NORMAL;

// Counter for sweep
uint16_t sweep_count = 0;

// Cell render use spi buffer
static pixel_t *cell_buffer;
// Check buffer size
#if CELLWIDTH*CELLHEIGHT > SPI_BUFFER_SIZE
#error "Too small spi_buffer size SPI_BUFFER_SIZE < CELLWIDTH*CELLHEIGH"
#endif

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
//   CELL_X[16:31] x position
//   CELL_Y[ 0:15] y position
typedef uint32_t index_t;
static index_t trace_index[TRACES_MAX][POINTS_COUNT];

#define INDEX(x, y) ((((index_t)(x))<<16)|(((index_t)(y))))
#define CELL_X(i)  ((int)(((i)>>16)))
#define CELL_Y(i)  ((int)(((i)&0xFFFF)))

//#define float2int(v) ((int)(v))
static int 
float2int(float v) 
{
  if (v < 0) return v - 0.5;
  if (v > 0) return v + 0.5;
  return 0;
}

void update_grid(void)
{
  uint32_t gdigit = 100000000;
  uint32_t fstart = get_sweep_frequency(ST_START);
  uint32_t fspan  = get_sweep_frequency(ST_SPAN);
  uint32_t grid;

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

  redraw_request |= REDRAW_FREQUENCY|REDRAW_AREA;
}

static inline int
circle_inout(int x, int y, int r)
{
  int d = x*x + y*y - r*r;
  if (d < -r)
    return 1;
  if (d >  r)
    return -1;
  return 0;
}

static int
polar_grid(int x, int y)
{
  // offset to center
  x -= P_CENTER_X;
  y -= P_CENTER_Y;

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

/*
 * Constant Resistance circle: (u - r/(r+1))^2 + v^2 = 1/(r+1)^2
 * Constant Reactance circle:  (u - 1)^2 + (v-1/x)^2 = 1/x^2
 */
static int
smith_grid(int x, int y)
{
  int d;

  // offset to center
  x -= P_CENTER_X;
  y -= P_CENTER_Y;

  // outer circle
  d = circle_inout(x, y, P_RADIUS);
  if (d < 0) return 0;
  if (d == 0) return 1;

  // horizontal axis
  if (y == 0) return 1;

  // shift circle center to right origin
  x -= P_RADIUS;

  // Constant Reactance Circle: 2j : R/2 = P_RADIUS/2
  if (circle_inout(x, y + P_RADIUS / 2, P_RADIUS / 2) == 0) return 1;
  if (circle_inout(x, y - P_RADIUS / 2, P_RADIUS / 2) == 0) return 1;

  // Constant Resistance Circle: 3 : R/4 = P_RADIUS/4
  d = circle_inout(x + P_RADIUS / 4, y, P_RADIUS / 4);
  if (d > 0) return 0;
  if (d == 0) return 1;

  // Constant Reactance Circle: 1j : R = P_RADIUS
  if (circle_inout(x, y + P_RADIUS, P_RADIUS) == 0) return 1;
  if (circle_inout(x, y - P_RADIUS, P_RADIUS) == 0) return 1;

  // Constant Resistance Circle: 1 : R/2
  d = circle_inout(x + P_RADIUS / 2, y, P_RADIUS / 2);
  if (d > 0) return 0;
  if (d == 0) return 1;

  // Constant Reactance Circle: 1/2j : R*2
  if (circle_inout(x, y + P_RADIUS * 2, P_RADIUS * 2) == 0) return 1;
  if (circle_inout(x, y - P_RADIUS * 2, P_RADIUS * 2) == 0) return 1;

  // Constant Resistance Circle: 1/3 : R*3/4
  if (circle_inout(x + P_RADIUS * 3 / 4, y, P_RADIUS * 3 / 4) == 0) return 1;
  return 0;
}

#if 0
static int
smith_grid2(int x, int y, float scale)
{
  int d;

  // offset to center
  x -= P_CENTER_X;
  y -= P_CENTER_Y;

  // outer circle
  d = circle_inout(x, y, P_RADIUS);
  if (d < 0)
    return 0;
  if (d == 0)
    return 1;

  // shift circle center to right origin
  x -= P_RADIUS * scale;

  // Constant Reactance Circle: 2j : R/2 = 58
  if (circle_inout(x, y+58*scale, 58*scale) == 0)
    return 1;
  if (circle_inout(x, y-58*scale, 58*scale) == 0)
    return 1;
#if 0
  // Constant Resistance Circle: 3 : R/4 = 29
  d = circle_inout(x+29*scale, y, 29*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
  d = circle_inout(x-29*scale, y, 29*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
#endif

  // Constant Reactance Circle: 1j : R = 116
  if (circle_inout(x, y+116*scale, 116*scale) == 0)
    return 1;
  if (circle_inout(x, y-116*scale, 116*scale) == 0)
    return 1;

  // Constant Resistance Circle: 1 : R/2 = 58
  d = circle_inout(x+58*scale, y, 58*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
  d = circle_inout(x-58*scale, y, 58*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;

  // Constant Reactance Circle: 1/2j : R*2 = 232
  if (circle_inout(x, y+232*scale, 232*scale) == 0)
    return 1;
  if (circle_inout(x, y-232*scale, 232*scale) == 0)
    return 1;

#if 0
  // Constant Resistance Circle: 1/3 : R*3/4 = 87
  d = circle_inout(x+87*scale, y, 87*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
  d = circle_inout(x+87*scale, y, 87*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
#endif

  // Constant Resistance Circle: 0 : R
  d = circle_inout(x+P_RADIUS*scale, y, P_RADIUS*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
  d = circle_inout(x-P_RADIUS*scale, y, P_RADIUS*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;

  // Constant Resistance Circle: -1/3 : R*3/2 = 174
  d = circle_inout(x+174*scale, y, 174*scale);
  if (d > 0) return 0;
  if (d == 0) return 1;
  d = circle_inout(x-174*scale, y, 174*scale);
  //if (d > 0) return 0;
  if (d == 0) return 1;
  return 0;
}
#endif

#if 0
const int cirs[][4] = {
  { 0, 58/2, 58/2, 0 },    // Constant Reactance Circle: 2j : R/2 = 58
  { 29/2, 0, 29/2, 1 },    // Constant Resistance Circle: 3 : R/4 = 29
  { 0, 115/2, 115/2, 0 },  // Constant Reactance Circle: 1j : R = 115
  { 58/2, 0, 58/2, 1 },    // Constant Resistance Circle: 1 : R/2 = 58
  { 0, 230/2, 230/2, 0 },  // Constant Reactance Circle: 1/2j : R*2 = 230
  { 86/2, 0, 86/2, 1 },    // Constant Resistance Circle: 1/3 : R*3/4 = 86
  { 0, 460/2, 460/2, 0 },  // Constant Reactance Circle: 1/4j : R*4 = 460
  { 115/2, 0, 115/2, 1 },  // Constant Resistance Circle: 0 : R
  { 173/2, 0, 173/2, 1 },  // Constant Resistance Circle: -1/3 : R*3/2 = 173
  { 0, 0, 0, 0 } // sentinel
};

static int
smith_grid3(int x, int y)
{
  int d;

  // offset to center
  x -= P_CENTER_X;
  y -= P_CENTER_Y;

  // outer circle
  d = circle_inout(x, y, P_RADIUS);
  if (d < 0)
    return 0;
  if (d == 0)
    return 1;

  // shift circle center to right origin
  x -= P_RADIUS /2;

  int i;
  for (i = 0; cirs[i][2]; i++) {
    d = circle_inout(x+cirs[i][0], y+cirs[i][1], cirs[i][2]);
    if (d == 0)
      return 1;
    if (d > 0 && cirs[i][3])
      return 0;
    d = circle_inout(x-cirs[i][0], y-cirs[i][1], cirs[i][2]);
    if (d == 0)
      return 1;
    if (d > 0 && cirs[i][3])
      return 0;
  }
  return 0;
}
#endif

#if 0
static int
rectangular_grid(int x, int y)
{
  //#define FREQ(x) (((x) * (fspan / 1000) / (WIDTH-1)) * 1000 + fstart)
  //int32_t n = FREQ(x-1) / fgrid;
  //int32_t m = FREQ(x) / fgrid;
  //if ((m - n) > 0)
  //if (((x * 6) % (WIDTH-1)) < 6)
  //if (((x - grid_offset) % grid_width) == 0)
  if (x == 0 || x == WIDTH-1)
    return 1;
  if ((y % GRIDY) == 0)
    return 1;
  if ((((x + grid_offset) * 10) % grid_width) < 10)
    return 1;
  return 0;
}
#endif

static int
rectangular_grid_x(int x)
{
  x -= CELLOFFSETX;
  if (x < 0) return 0;
  if (x == 0 || x == WIDTH)
    return 1;
  if ((((x + grid_offset) * 10) % grid_width) < 10)
    return 1;
  return 0;
}

static int
rectangular_grid_y(int y)
{
  if (y < 0)
    return 0;
  if ((y % GRIDY) == 0)
    return 1;
  return 0;
}

#if 0
int
set_strut_grid(int x)
{
  uint16_t *buf = spi_buffer;
  int y;

  for (y = 0; y < HEIGHT; y++) {
    int c = rectangular_grid(x, y);
    c |= smith_grid(x, y);
    *buf++ = c;
  }
  return y;
}

void
draw_on_strut(int v0, int d, int color)
{
  int v;
  int v1 = v0 + d;
  if (v0 < 0) v0 = 0;
  if (v1 < 0) v1 = 0;
  if (v0 >= HEIGHT) v0 = HEIGHT-1;
  if (v1 >= HEIGHT) v1 = HEIGHT-1;
  if (v0 == v1) {
    v = v0; d = 2;
  } else if (v0 < v1) {
    v = v0; d = v1 - v0 + 1;
  } else {
    v = v1; d = v0 - v1 + 1;
  }
  while (d-- > 0)
    spi_buffer[v++] |= color;
}
#endif

/*
 * calculate log10(abs(gamma))
 */ 
static float
logmag(const float *v)
{
  return log10f(v[0]*v[0] + v[1]*v[1]) * 10;
}

/*
 * calculate phase[-2:2] of coefficient
 */ 
static float
phase(const float *v)
{
  return (180.0f / VNA_PI) * atan2f(v[1], v[0]);
}

/*
 * calculate groupdelay
 */
static float
groupdelay(const float *v, const float *w, float deltaf)
{
#if 1
  // atan(w)-atan(v) = atan((w-v)/(1+wv))
  float r = w[0]*v[1] - w[1]*v[0];
  float i = w[0]*v[0] + w[1]*v[1];
  return atan2f(r, i) / (2 * VNA_PI * deltaf);
#else
  return (atan2f(w[0], w[1]) - atan2f(v[0], v[1])) / (2 * VNA_PI * deltaf);
#endif
}

/*
 * calculate abs(gamma)
 */
static float
linear(const float *v)
{
  return sqrtf(v[0]*v[0] + v[1]*v[1]);
}

/*
 * calculate vswr; (1+gamma)/(1-gamma)
 */ 
static float
swr(const float *v)
{
  float x = linear(v);
  if (x >= 1)
    return INFINITY;
  return (1 + x)/(1 - x);
}

static float
resistance(const float *v)
{
  const float z0 = 50;
  float d = z0 / ((1-v[0])*(1-v[0])+v[1]*v[1]);
  return (1 - v[0]*v[0] - v[1]*v[1]) * d;
}

static float
reactance(const float *v)
{
  const float z0 = 50;
  const float d = z0 / ((1-v[0])*(1-v[0])+v[1]*v[1]);
  return 2*v[1] * d;
}

#if 0
static float
mod_z(const float *v){
  float z0 = 50;
  float d = z0 / ((1-v[0])*(1-v[0])+v[1]*v[1]);
  float zr = (1 - v[0]*v[0] - v[1]*v[1])*d;
  float zi = 2*v[1]*d;

  return sqrtf(zr*zr + zi*zi);
}
#endif

static float
qualityfactor(const float *v)
{
  float i = 2*v[1];
  float r = 1 - v[0]*v[0] - v[1]*v[1];
  return fabsf(i / r);
}

static float
real(const float *v)
{
  return v[0];
}

static float
imag(const float *v)
{
  return v[1];
}

static void
cartesian_scale(const float *v, int *xp, int *yp, float scale)
{
  //float scale = 4e-3;
  int x = float2int(v[0] * P_RADIUS * scale);
  int y = float2int(v[1] * P_RADIUS * scale);
  if      (x < -P_RADIUS) x = -P_RADIUS;
  else if (x >  P_RADIUS) x =  P_RADIUS;
  if      (y < -P_RADIUS) y = -P_RADIUS;
  else if (y >  P_RADIUS) y =  P_RADIUS;
  *xp = P_CENTER_X + x;
  *yp = P_CENTER_Y - y;
}

float
groupdelay_from_array(int i, float array[POINTS_COUNT][2])
{
  int bottom = (i ==   0) ?   0 : i - 1;
  int top    = (i == sweep_points-1) ? sweep_points-1 : i + 1;
  float deltaf = frequencies[top] - frequencies[bottom];
  return groupdelay(array[bottom], array[top], deltaf);
}

// Calculate and cache point coordinates for trace
static void
trace_into_index(int t, float array[POINTS_COUNT][2])
{
  const float refpos = NGRIDY - get_trace_refpos(t);
  const float scale = 1 / get_trace_scale(t);
  const uint32_t point_count = sweep_points-1;
  index_t *index = trace_index[t];
  for (uint32_t i = 0; i <= point_count; i++){
    int y, x;
    float v = 0.0f;
    float *coeff = array[i];
    switch (trace[t].type) {
    case TRC_LOGMAG:
      v = logmag(coeff);
      break;
    case TRC_PHASE:
      v = phase(coeff);
      break;
    case TRC_DELAY:
      v = groupdelay_from_array(i, array);
      break;
    case TRC_LINEAR:
      v = linear(coeff);
      break;
    case TRC_SWR:
      v = (swr(coeff) - 1);
      break;
    case TRC_REAL:
      v = real(coeff);
      break;
    case TRC_IMAG:
      v = imag(coeff);
      break;
    case TRC_R:
      v = resistance(coeff);
      break;
    case TRC_X:
      v = reactance(coeff);
      break;
    case TRC_Q:
      v = qualityfactor(coeff);
      break;
    case TRC_SMITH:
    //case TRC_ADMIT:
    case TRC_POLAR:
      cartesian_scale(coeff, &x, &y, scale);
      goto set_index;
//  default:
//    continue;
    }
    v = refpos - v * scale;
    if (v <  0) v = 0;
    if (v > NGRIDY) v = NGRIDY;
    x = (i * (WIDTH) + (point_count>>1)) / point_count + CELLOFFSETX;
    y = float2int(v * GRIDY);
set_index:
    index[i] = INDEX(x, y);
  }
}

static void
format_smith_value(char *buf, int len, const float coeff[2], uint32_t frequency)
{
  char *format;
  float zr, zi;
  switch (marker_smith_format) {
  case MS_LIN:
    zr = linear(coeff);
    zi = phase(coeff);
    format = "%.2f %.1f" S_DEGREE;
    break;
  case MS_LOG:
    zr = logmag(coeff);
    zi = phase(coeff);
    format = (zr == -INFINITY) ? "-"S_INFINITY" dB" : "%.1fdB %.1f" S_DEGREE;
    break;
  case MS_REIM:
    zr = real(coeff);
    zi = imag(coeff);
    format = "%F%+Fj";
    break;
  case MS_RX:
    zr = resistance(coeff);
    zi = reactance(coeff);
    format = "%F%+Fj"S_OHM;
    break;
  case MS_RLC:
    zr = resistance(coeff);
    zi = reactance(coeff);
    if (zi < 0) {// Capacity
      format = "%F"S_OHM" %FF";
      zi = -1 / (2 * VNA_PI * frequency * zi);
    } else {     // Inductive
      format = "%F"S_OHM" %FH";
      zi = zi / (2 * VNA_PI * frequency);
    }
    break;
  default:
    return;
  }
  plot_printf(buf, len, format, zr, zi);
}

#if MAX_TRACE_TYPE != 13
#error "Redefined trace_type list, need check format_list"
#endif

typedef float (*get_value_cb_t)(const float *v);
static const struct {
  const char *name;
  const char *dname;
  get_value_cb_t get_value_cb;
} format_list[]={
// Type        format           delta format           get value
[TRC_LOGMAG] = {"%.2fdB",       S_DELTA"%.2fdB",       logmag       },
[TRC_PHASE]  = {"%.1f"S_DEGREE, S_DELTA"%.2f"S_DEGREE, phase        },
[TRC_DELAY]  = {"%.2Fs",        "%.2Fs",               NULL         }, // Custom
[TRC_LINEAR] = {"%.4f",         S_DELTA"%.3f",         linear       },
[TRC_SWR]    = {"%.4f",         S_DELTA"%.3f",         swr          },
[TRC_REAL]   = {"%.4f",         S_DELTA"%.3f",         real         },
[TRC_IMAG]   = {"%.4fj",        S_DELTA"%.3fj",        imag         },
[TRC_R]      = {"%.2F"S_OHM,    S_DELTA"%.2F"S_OHM,    resistance   },
[TRC_X]      = {"%.2F"S_OHM,    S_DELTA"%.2F"S_OHM,    reactance    },
[TRC_Q]      = {"%.3f",         S_DELTA"%.3f",         qualityfactor},
[TRC_SMITH]  = {NULL,           NULL,                  NULL         }, // Custom
[TRC_POLAR]  = {"%.2f%+.2fj",   "%.2f%+.2fj",          NULL         }  // Custom
};

static void
trace_get_value_string(int t, char *buf, int len, float array[POINTS_COUNT][2], int index, int index_ref)
{
  // Check correct input
  if (trace[t].type >= MAX_TRACE_TYPE) return;
  float v = 0.0f;
  float *coeff = array[index];
  float *coeff_ref;
  const char *format;
  // Get format data
  if (index_ref >=0){                           // Delta value
    coeff_ref = array[index_ref];
    format = format_list[trace[t].type].dname;
  }
  else{                                         // No delta
    coeff_ref = NULL;
    format = format_list[trace[t].type].name;
  }

  get_value_cb_t c = format_list[trace[t].type].get_value_cb;
  if (c){                                            // Run standard get value function from table
    v = c(coeff);                                    // Get value
    if (coeff_ref && v != INFINITY) v-=c(coeff_ref); // Calculate delta value
  }
  else { // Need custom calculations
    switch (trace[t].type) {
    case TRC_DELAY:
      v = groupdelay_from_array(index, array);
      if (coeff_ref) v-= groupdelay_from_array(index_ref, array);
      break;
    case TRC_SMITH:
      format_smith_value(buf, len, coeff, frequencies[index]);
      return;
    //case TRC_ADMIT:
    case TRC_POLAR:
      plot_printf(buf, len, format, coeff[0], coeff[1]);
      return;
    default:
      return;
    }
  }
  plot_printf(buf, len, format, v);
}

static int
trace_get_info(int t, char *buf, int len)
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
  return plot_printf(buf, len, format, get_trace_typename(t), scale);
}

static float time_of_index(int idx)
{
  return (idx / (float)FFT_SIZE) / (frequencies[1] - frequencies[0]);
}

static float distance_of_index(int idx)
{
  return velocity_factor * (SPEED_OF_LIGHT / 2) * time_of_index(idx);
}

static void
mark_map(int x, int y)
{
  if (y >= 0 && y < MAX_MARKMAP_Y && x >= 0 && x < MAX_MARKMAP_X)
    markmap[current_mappage][y] |= 1 << x;
}

static inline void
swap_markmap(void)
{
  current_mappage^= 1;
}

static void
clear_markmap(void)
{
  memset(markmap[current_mappage], 0, sizeof markmap[current_mappage]);
}

void
force_set_markmap(void)
{
  memset(markmap[current_mappage], 0xff, sizeof markmap[current_mappage]);
}

void
invalidate_rect(int x0, int y0, int x1, int y1)
{
  x0 /= CELLWIDTH;
  x1 /= CELLWIDTH;
  y0 /= CELLHEIGHT;
  y1 /= CELLHEIGHT;
  int x, y;
  for (y = y0; y <= y1; y++)
    for (x = x0; x <= x1; x++) 
      mark_map(x, y);
}

#define SWAP(x,y) {int t=x;x=y;y=t;}

static void
mark_cells_from_index(void)
{
  int t, i, j;
  /* mark cells between each neighbor points */
  map_t *map = &markmap[current_mappage][0];
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    index_t *index = &trace_index[t][0];
    int m0 = CELL_X(index[0]) / CELLWIDTH;
    int n0 = CELL_Y(index[0]) / CELLHEIGHT;
    map[n0] |= 1 << m0;
    for (i = 1; i < sweep_points; i++) {
      int m1 = CELL_X(index[i]) / CELLWIDTH;
      int n1 = CELL_Y(index[i]) / CELLHEIGHT;
      if (m0 == m1 && n0 == n1)
        continue;
      int x0 = m0; int x1 = m1; if (x0>x1) SWAP(x0, x1); m0 = m1;
      int y0 = n0; int y1 = n1; if (y0>y1) SWAP(y0, y1); n0 = n1;
      for (; y0 <= y1; y0++)
        for (j = x0; j <= x1; j++)
          map[y0] |= 1 << j;
    }
  }
}

static inline void
markmap_upperarea(void)
{
  // Hardcoded, Text info from upper area
  invalidate_rect(0, 0, AREA_WIDTH_NORMAL, 3*FONT_STR_HEIGHT);
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
  if (x1 < x0) { SWAP(x0, x1); SWAP(y0, y1); }
  int dx =-(x1 - x0);
  int dy = (y1 - y0), sy = 1; if (dy < 0) { dy = -dy; sy = -1; }
  int err = ((dy + dx) < 0 ? -dx : -dy) / 2;
  while (1) {
    if (y0 >= 0 && y0 < CELLHEIGHT && x0 >= 0 && x0 < CELLWIDTH)
      cell_buffer[y0 * CELLWIDTH + x0] |= c;
    if (x0 == x1 && y0 == y1)
      return;
    int e2 = err;
    if (e2 > dx) { err-= dy; x0++;  }
    if (e2 < dy) { err-= dx; y0+=sy;}
  }
}

// Give a little speedup then draw rectangular plot (50 systick on all calls, all render req 700 systick)
// Write more difficult algorithm for search indexes not give speedup
static int
search_index_range_x(int x1, int x2, index_t index[POINTS_COUNT], int *i0, int *i1)
{
  int i, j;
  int head = 0;
  int tail = sweep_points;
  int idx_x;

  // Search index point in cell
  while (1) {
    i = (head + tail) / 2;
    idx_x = CELL_X(index[i]);
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
  } while (x1 <= CELL_X(index[j]));
  *i0 = j;
  // Search index right from point
  do {
    if (i >=sweep_points-1) break;
    i++;
  } while (CELL_X(index[i]) < x2);
  *i1 = i;

  return TRUE;
}

static void
cell_blit_bitmap(int x, int y, uint16_t w, uint16_t h, const uint8_t *bmp)
{
  if (x <= -w)
    return;
  uint8_t bits = 0;
  int c = h+y, r;
  for (; y < c; y++) {
    for (r = 0; r < w; r++) {
      if ((r&7)==0) bits = *bmp++;
      if (y >= 0 && x+r >= 0 && y < CELLHEIGHT && x+r < CELLWIDTH && (0x80 & bits))
        cell_buffer[y*CELLWIDTH + x + r] = foreground_color;
      bits <<= 1;
    }
  }
}

static void
cell_drawstring(char *str, int x, int y)
{
  if (y <= -FONT_GET_HEIGHT || y >= CELLHEIGHT)
    return;
  while (*str) {
    if (x >= CELLWIDTH)
      return;
    uint8_t ch = *str++;
    uint16_t w = FONT_GET_WIDTH(ch);
    cell_blit_bitmap(x, y, w, FONT_GET_HEIGHT, FONT_GET_DATA(ch));
    x += w;
  }
}

// Include L/C match functions
#ifdef __USE_LC_MATCHING__
  #include "lc_matching.c"
#endif

#define REFERENCE_WIDTH    6
#define REFERENCE_HEIGHT   5
#define REFERENCE_X_OFFSET 5
#define REFERENCE_Y_OFFSET 2

// Reference bitmap
static const uint8_t reference_bitmap[]={
  _BMP8(0b11000000),
  _BMP8(0b11110000),
  _BMP8(0b11111100),
  _BMP8(0b11110000),
  _BMP8(0b11000000),
};

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
};

#elif _USE_BIG_MARKER_ == 1
#define MARKER_WIDTH       10
#define MARKER_HEIGHT      13
#define X_MARKER_OFFSET     4
#define Y_MARKER_OFFSET    13
#define MARKER_BITMAP(i)   (&marker_bitmap[(i)*2*MARKER_HEIGHT])
static const uint8_t marker_bitmap[]={
  // Marker Back plate
  _BMP16(0b1111111110000000),
  _BMP16(0b1111111110000000),
  _BMP16(0b1111111110000000),
  _BMP16(0b1111111110000000),
  _BMP16(0b1111111110000000),
  _BMP16(0b1111111110000000),
  _BMP16(0b1111111110000000),
  _BMP16(0b1111111110000000),
  _BMP16(0b1111111110000000),
  _BMP16(0b0111111100000000),
  _BMP16(0b0011111000000000),
  _BMP16(0b0001110000000000),
  _BMP16(0b0000100000000000),
  // Marker 1
  _BMP16(0b0000000000000000),
  _BMP16(0b0000110000000000),
  _BMP16(0b0001110000000000),
  _BMP16(0b0010110000000000),
  _BMP16(0b0000110000000000),
  _BMP16(0b0000110000000000),
  _BMP16(0b0000110000000000),
  _BMP16(0b0000110000000000),
  _BMP16(0b0000110000000000),
  _BMP16(0b0001111000000000),
  _BMP16(0b0000000000000000),
  _BMP16(0b0000000000000000),
  _BMP16(0b0000000000000000),
  // Marker 2
  _BMP16(0b0000000000000000),
  _BMP16(0b0001111000000000),
  _BMP16(0b0011001100000000),
  _BMP16(0b0011001100000000),
  _BMP16(0b0000011000000000),
  _BMP16(0b0000110000000000),
  _BMP16(0b0001100000000000),
  _BMP16(0b0011000000000000),
  _BMP16(0b0011111100000000),
  _BMP16(0b0000000000000000),
  _BMP16(0b0000000000000000),
  _BMP16(0b0000000000000000),
  _BMP16(0b0000000000000000),
  // Marker 3
  _BMP16(0b0000000000000000),
  _BMP16(0b0011111000000000),
  _BMP16(0b0110001100000000),
  _BMP16(0b0110001100000000),
  _BMP16(0b0000001100000000),
  _BMP16(0b0000111000000000),
  _BMP16(0b0000001100000000),
  _BMP16(0b0110001100000000),
  _BMP16(0b0110001100000000),
  _BMP16(0b0011111000000000),
  _BMP16(0b0000000000000000),
  _BMP16(0b0000000000000000),
  _BMP16(0b0000000000000000),
  // Marker 4
  _BMP16(0b0000000000000000),
  _BMP16(0b0000011000000000),
  _BMP16(0b0000111000000000),
  _BMP16(0b0001111000000000),
  _BMP16(0b0011011000000000),
  _BMP16(0b0110011000000000),
  _BMP16(0b0110011000000000),
  _BMP16(0b0111111100000000),
  _BMP16(0b0000011000000000),
  _BMP16(0b0000011000000000),
  _BMP16(0b0000000000000000),
  _BMP16(0b0000000000000000),
  _BMP16(0b0000000000000000),
};
#endif

static void
markmap_marker(int marker)
{
  int t;
  if (!markers[marker].enabled)
    return;
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    index_t index = trace_index[t][markers[marker].index];
    int x = CELL_X(index) - X_MARKER_OFFSET;
    int y = CELL_Y(index) - Y_MARKER_OFFSET;
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

//
// Marker search functions
//
static int greater(int x, int y) { return x > y; }
static int lesser(int x, int y) { return x < y; }

static int (*compare)(int x, int y) = lesser;

void
set_marker_search(int16_t mode)
{
  compare = (mode == MK_SEARCH_MIN) ? greater : lesser;
}

int
marker_search(void)
{
  int i;
  int found = 0;

  if (current_trace == TRACE_INVALID)
    return -1;

  int value = CELL_Y(trace_index[current_trace][0]);
  for (i = 1; i < sweep_points; i++) {
    index_t index = trace_index[current_trace][i];
    if ((*compare)(value, CELL_Y(index))) {
      value = CELL_Y(index);
      found = i;
    }
  }

  return found;
}

int
marker_search_dir(int16_t from, int16_t dir)
{
  int i;
  int found = -1;

  if (current_trace == TRACE_INVALID)
    return -1;

  int value = CELL_Y(trace_index[current_trace][from]);
  for (i = from + dir; i >= 0 && i < sweep_points; i+=dir) {
    index_t index = trace_index[current_trace][i];
    if ((*compare)(value, CELL_Y(index)))
      break;
    value = CELL_Y(index);
  }

  for (; i >= 0 && i < sweep_points; i+=dir) {
    index_t index = trace_index[current_trace][i];
    if ((*compare)(CELL_Y(index), value))
      break;
    found = i;
    value = CELL_Y(index);
  }
  return found;
}

int
distance_to_index(int8_t t, uint16_t idx, int16_t x, int16_t y)
{
  index_t index = trace_index[t][idx];
  x-= CELL_X(index);
  y-= CELL_Y(index);
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
  for (t = 0; t < TRACES_MAX; t++) {
    if (trace[t].enabled)
      trace_into_index(t, array[trace[t].channel]);
  }
  // Marker track on data update
  if (uistat.marker_tracking && active_marker != MARKER_INVALID)
    set_marker_index(active_marker, marker_search());
  // Current scan count
  sweep_count++;
  // Build cell list for update
  mark_cells_from_index(); // Trace graph update
  markmap_all_markers();   // Marker update
}

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
  cell_buffer = ili9341_get_cell_buffer();
  // Clear buffer ("0 : height" lines)
#if 0
  // use memset 350 system ticks for all screen calls
  // as understand it use 8 bit set, slow down on 32 bit systems
  memset(spi_buffer, DEFAULT_BG_COLOR, (h*CELLWIDTH)*sizeof(uint16_t));
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
  for (t = 0; t < TRACES_MAX; t++) {
    if (trace[t].enabled) {
      trace_type |= (1 << trace[t].type);
    }
  }
  // Draw rectangular plot (40 system ticks for all screen calls)
  if (trace_type & RECTANGULAR_GRID_MASK) {
    for (x = 0; x < w; x++) {
      if (rectangular_grid_x(x + x0)) {
        for (y = 0; y < h; y++) cell_buffer[y * CELLWIDTH + x] = c;
      }
    }
    for (y = 0; y < h; y++) {
      if (rectangular_grid_y(y + y0)) {
        for (x = 0; x < w; x++)
          if (x + x0 >= CELLOFFSETX && x + x0 <= WIDTH + CELLOFFSETX)
            cell_buffer[y * CELLWIDTH + x] = c;
      }
    }
  }
  // Smith greed line (1000 system ticks for all screen calls)
  if (trace_type & (1 << TRC_SMITH)) {
    for (y = 0; y < h; y++)
      for (x = 0; x < w; x++)
        if (smith_grid(x + x0, y + y0)) cell_buffer[y * CELLWIDTH + x] = c;
  }
  // Polar greed line (800 system ticks for all screen calls)
  else if (trace_type & (1 << TRC_POLAR)) {
    for (y = 0; y < h; y++)
      for (x = 0; x < w; x++)
        if (polar_grid(x + x0, y + y0)) cell_buffer[y * CELLWIDTH + x] = c;
  }
#if 0
  else if (trace_type & (1 << TRC_ADMIT)) {
    for (y = 0; y < h; y++)
      for (x = 0; x < w; x++)
        if (smith_grid3(x+x0, y+y0)
         // smith_grid2(x+x0, y+y0, 0.5))
           cell_buffer[y * CELLWIDTH + x] = c;
  }
#endif
#endif
//  PULSE;
// Draw traces (50-600 system ticks for all screen calls, depend from lines
// count and size)
#if 1
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    c = GET_PALTETTE_COLOR(LCD_TRACE_1_COLOR + t);
    // draw polar plot (check all points)
    i0 = 0;
    i1 = 0;
    uint32_t trace_type = (1 << trace[t].type);
    if (trace_type & ((1 << TRC_SMITH) | (1 << TRC_POLAR)))
      i1 = sweep_points - 1;
    else  // draw rectangular plot (search index range in cell, save 50-70
          // system ticks for all screen calls)
      search_index_range_x(x0, x0 + w, trace_index[t], &i0, &i1);
    if (i1==0) continue;
    index_t *index = trace_index[t];
    for (i = i0; i < i1; i++) {
      int x1 = CELL_X(index[i]) - x0;
      int y1 = CELL_Y(index[i]) - y0;
      int x2 = CELL_X(index[i + 1]) - x0;
      int y2 = CELL_Y(index[i + 1]) - y0;
      cell_drawline(x1, y1, x2, y2, c);
    }
  }
#else
  for (x = 0; x < area_width; x += 6)
    cell_drawline(x - x0, 0 - y0, area_width - x - x0, area_height - y0,
                  config.trace_color[0]);
#endif
//  PULSE;
// draw marker symbols on each trace (<10 system ticks for all screen calls)
#if 1
  for (i = 0; i < MARKERS_MAX; i++) {
    if (!markers[i].enabled)
      continue;
    for (t = 0; t < TRACES_MAX; t++) {
      if (!trace[t].enabled)
        continue;
      index_t index = trace_index[t][markers[i].index];
      int x = CELL_X(index) - x0 - X_MARKER_OFFSET;
      int y = CELL_Y(index) - y0 - Y_MARKER_OFFSET;
      // Check marker icon on cell
      if (x + MARKER_WIDTH >= 0 && x - MARKER_WIDTH < CELLWIDTH &&
          y + MARKER_HEIGHT >= 0 && y - MARKER_HEIGHT < CELLHEIGHT){
//        draw_marker(x, y, LCD_TRACE_1_COLOR + t, i);
          // Draw marker plate
          ili9341_set_foreground(LCD_TRACE_1_COLOR + t);
          cell_blit_bitmap(x, y, MARKER_WIDTH, MARKER_HEIGHT, MARKER_BITMAP(0));
          // Draw marker number
          ili9341_set_foreground(LCD_BG_COLOR);
          cell_blit_bitmap(x, y, MARKER_WIDTH, MARKER_HEIGHT, MARKER_BITMAP(i+1));
      }
    }
  }
#endif
// Draw trace and marker info on the top (50 system ticks for all screen calls)
#if 1
  if (n <= (3*FONT_STR_HEIGHT)/CELLHEIGHT)
    cell_draw_marker_info(x0, y0);
#endif
// L/C match data output
#ifdef __USE_LC_MATCHING__
  if (domain_mode & TD_LC_MATH)
    cell_draw_lc_match(x0, y0);
#endif
//  PULSE;
// Draw reference position (<10 system ticks for all screen calls)
  for (t = 0; t < TRACES_MAX; t++) {
    if (!trace[t].enabled)
      continue;
    uint32_t trace_type = (1 << trace[t].type);
    if (trace_type & ((1 << TRC_SMITH) | (1 << TRC_POLAR)))
      continue;

    int x = 0 - x0 + CELLOFFSETX - REFERENCE_X_OFFSET;
    if (x + REFERENCE_WIDTH >= 0 && x - REFERENCE_WIDTH < CELLWIDTH) {
      int y = HEIGHT - float2int(get_trace_refpos(t) * GRIDY) - y0 - REFERENCE_Y_OFFSET;
      if (y + REFERENCE_HEIGHT >= 0 && y - REFERENCE_HEIGHT < CELLHEIGHT){
        ili9341_set_foreground(LCD_TRACE_1_COLOR + t);
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
  ili9341_bulk_continue(OFFSETX + x0, OFFSETY + y0, w, h);
}

static void
draw_all_cells(bool flush_markmap)
{
  int m, n;
//  START_PROFILE
  for (m = 0; m < (area_width+CELLWIDTH-1) / CELLWIDTH; m++)
    for (n = 0; n < (area_height+CELLHEIGHT-1) / CELLHEIGHT; n++)
      if ((markmap[0][n] | markmap[1][n]) & (1 << m))
        draw_cell(m, n);
#if 0
  ili9341_bulk_finish();
  for (m = 0; m < (area_width+CELLWIDTH-1) / CELLWIDTH; m++)
    for (n = 0; n < (area_height+CELLHEIGHT-1) / CELLHEIGHT; n++) {
      if ((markmap[0][n] | markmap[1][n]) & (1 << m))
        ili9341_set_background(LCD_LOW_BAT_COLOR);
      else
        ili9341_set_background(LCD_NORMAL_BAT_COLOR);
      ili9341_fill(m*CELLWIDTH+OFFSETX, n*CELLHEIGHT, 2, 2);
    }
#endif
  if (flush_markmap) {
    // keep current map for update
    swap_markmap();
    // clear map for next plotting
    clear_markmap();
  }
  // Flush LCD buffer, wait completion (need call after end use ili9341_bulk_continue mode)
  ili9341_bulk_finish();
//  STOP_PROFILE
}

void
draw_all(bool flush)
{
  if (redraw_request & REDRAW_AREA)
    force_set_markmap();
  if (redraw_request & REDRAW_MARKER)
    markmap_upperarea();
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

  // mark cells on marker info
  markmap_upperarea();

  draw_all_cells(TRUE);
  // Force redraw all area after (disable artifacts after fast marker update area)
  redraw_request|=REDRAW_AREA;
}

void
request_to_draw_cells_behind_menu(void)
{
  // Values Hardcoded from ui.c
  invalidate_rect(LCD_WIDTH-MENU_BUTTON_WIDTH-OFFSETX, 0, LCD_WIDTH-OFFSETX, LCD_HEIGHT-1);
  redraw_request |= REDRAW_CELLS;
}

void
request_to_draw_cells_behind_numeric_input(void)
{
  // Values Hardcoded from ui.c
  invalidate_rect(0, LCD_HEIGHT-NUM_INPUT_HEIGHT, LCD_WIDTH-1, LCD_HEIGHT-1);
  redraw_request |= REDRAW_CELLS;
}

static void
cell_draw_marker_info(int x0, int y0)
{
  char buf[24];
  int t;
  if (active_marker == MARKER_INVALID)
    return;
  int idx = markers[active_marker].index;
  int j = 0;

  if (previous_marker != MARKER_INVALID && current_trace != TRACE_INVALID) {
    int t = current_trace;
    int mk;
    for (mk = 0; mk < MARKERS_MAX; mk++) {
      if (!markers[mk].enabled)
        continue;
      int xpos = 1 + (j%2)*(WIDTH/2) + CELLOFFSETX - x0;
      int ypos = 1 + (j/2)*(FONT_STR_HEIGHT) - y0;

      ili9341_set_foreground(LCD_TRACE_1_COLOR + t);
      if (mk == active_marker)
        cell_drawstring(S_SARROW, xpos, ypos);
      xpos += 6;
      plot_printf(buf, sizeof buf, "M%d", mk+1);
      cell_drawstring(buf, xpos, ypos);
      xpos += 19;
      //trace_get_info(t, buf, sizeof buf);
      uint32_t freq = frequencies[markers[mk].index];
      if (uistat.marker_delta && mk != active_marker) {
        uint32_t freq1 = frequencies[markers[active_marker].index];
        uint32_t delta = freq > freq1 ? freq - freq1 : freq1 - freq;
        plot_printf(buf, sizeof buf, S_DELTA"%qHz", delta);
      } else {
        plot_printf(buf, sizeof buf, "%qHz", freq);
      }
      cell_drawstring(buf, xpos, ypos);
      xpos += 116;
      int didx = -1; // delta value index
      if (uistat.marker_delta && mk != active_marker)
        didx = markers[active_marker].index;
      trace_get_value_string(t, buf, sizeof buf, measured[trace[t].channel], markers[mk].index, didx);
      ili9341_set_foreground(LCD_FG_COLOR);
      cell_drawstring(buf, xpos, ypos);
      j++;
    }

    // draw marker delta
    if (!uistat.marker_delta && active_marker != previous_marker) {
      int idx0 = markers[previous_marker].index;
      int xpos = (WIDTH/2+30) + CELLOFFSETX - x0;
      int ypos = 1 + (j/2)*(FONT_STR_HEIGHT) - y0;

      plot_printf(buf, sizeof buf, S_DELTA"%d-%d:", active_marker+1, previous_marker+1);
      ili9341_set_foreground(LCD_FG_COLOR);
      cell_drawstring(buf, xpos, ypos);
      xpos += 35;
      if ((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ) {
        uint32_t freq  = frequencies[idx];
        uint32_t freq1 = frequencies[idx0];
        uint32_t delta = freq >= freq1 ? freq - freq1 : freq1 - freq;
        plot_printf(buf, sizeof buf, "%c%qHz", freq >= freq1 ? '+' : '-', delta);
      } else {
        plot_printf(buf, sizeof buf, "%Fs (%Fm)", time_of_index(idx) - time_of_index(idx0), distance_of_index(idx) - distance_of_index(idx0));
      }
      cell_drawstring(buf, xpos, ypos);
    }
  } else {
    for (t = 0; t < TRACES_MAX; t++) {
      if (!trace[t].enabled)
        continue;
      int xpos = 1 + (j%2)*(WIDTH/2) + CELLOFFSETX - x0;
      int ypos = 1 + (j/2)*(FONT_STR_HEIGHT) - y0;

      ili9341_set_foreground(LCD_TRACE_1_COLOR + t);
      if (t == current_trace)
        cell_drawstring(S_SARROW, xpos, ypos);
      xpos += FONT_WIDTH;
      plot_printf(buf, sizeof buf, "CH%d", trace[t].channel);
      cell_drawstring(buf, xpos, ypos);
      xpos += 3*FONT_WIDTH + 4;

      int n = trace_get_info(t, buf, sizeof buf);
      cell_drawstring(buf, xpos, ypos);
      xpos += n * FONT_WIDTH + 2;
      //xpos += 60;
      trace_get_value_string(t, buf, sizeof buf, measured[trace[t].channel], idx, -1);
      ili9341_set_foreground(LCD_FG_COLOR);
      cell_drawstring(buf, xpos, ypos);
      j++;
    }

    // draw marker frequency
    int xpos = (WIDTH/2+40) + CELLOFFSETX - x0;
    int ypos = 1 + (j/2)*(FONT_STR_HEIGHT) - y0;

    ili9341_set_foreground(LCD_FG_COLOR);
    if (uistat.lever_mode == LM_MARKER)
      cell_drawstring(S_SARROW, xpos, ypos);
    xpos += FONT_WIDTH;
    plot_printf(buf, sizeof buf, "M%d:", active_marker+1);
    cell_drawstring(buf, xpos, ypos);
    xpos += 3*FONT_WIDTH + 4;

    if ((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ) {
      plot_printf(buf, sizeof buf, "%qHz", frequencies[idx]);
    } else {
      plot_printf(buf, sizeof buf, "%Fs (%Fm)", time_of_index(idx), distance_of_index(idx));
    }
    cell_drawstring(buf, xpos, ypos);
  }
  ili9341_set_foreground(LCD_FG_COLOR);
  if (electrical_delay != 0) {
    // draw electrical delay
    int xpos = 21 + CELLOFFSETX - x0;
    int ypos = 1 + ((j+1)/2)*(FONT_STR_HEIGHT) - y0;

    if (uistat.lever_mode == LM_EDELAY)
      cell_drawstring(S_SARROW, xpos, ypos);
    xpos += 5;

    float edelay = electrical_delay * 1e-12; // to seconds
    plot_printf(buf, sizeof buf, "Edelay %Fs %Fm", edelay, edelay * SPEED_OF_LIGHT * velocity_factor);
    cell_drawstring(buf, xpos, ypos);
  }
}

void
draw_frequencies(void)
{
  char buf1[32];
  char buf2[32]; buf2[0] = 0;
  if ((domain_mode & DOMAIN_MODE) == DOMAIN_FREQ) {
    if (FREQ_IS_CW()) {
      plot_printf(buf1, sizeof(buf1), " CW %qHz", get_sweep_frequency(ST_CW));
    } else if (FREQ_IS_STARTSTOP()) {
      plot_printf(buf1, sizeof(buf1), " START %qHz", get_sweep_frequency(ST_START));
      plot_printf(buf2, sizeof(buf2), " STOP %qHz", get_sweep_frequency(ST_STOP));
    } else if (FREQ_IS_CENTERSPAN()) {
      plot_printf(buf1, sizeof(buf1), " CENTER %qHz", get_sweep_frequency(ST_CENTER));
      plot_printf(buf2, sizeof(buf2), " SPAN %qHz", get_sweep_frequency(ST_SPAN));
    }
  } else {
    plot_printf(buf1, sizeof(buf1), " START 0s");
    plot_printf(buf2, sizeof(buf2), "STOP %Fs (%Fm)", time_of_index(sweep_points-1), distance_of_index(sweep_points-1));
  }
  ili9341_set_foreground(LCD_FG_COLOR);
  ili9341_set_background(LCD_BG_COLOR);
  ili9341_fill(0, FREQUENCIES_YPOS, LCD_WIDTH, FONT_GET_HEIGHT);
  if (uistat.lever_mode == LM_CENTER)
    buf1[0] = S_SARROW[0];
  if (uistat.lever_mode == LM_SPAN)
    buf2[0] = S_SARROW[0];
  ili9341_drawstring(buf1, FREQUENCIES_XPOS1, FREQUENCIES_YPOS);
  ili9341_drawstring(buf2, FREQUENCIES_XPOS2, FREQUENCIES_YPOS);
  plot_printf(buf1, sizeof(buf1), "bw:%uHz  %up", get_bandwidth_frequency(config.bandwidth), sweep_points);
  ili9341_set_foreground(LCD_BW_TEXT_COLOR);
  ili9341_drawstring(buf1, FREQUENCIES_XPOS3, FREQUENCIES_YPOS);
}

void
draw_cal_status(void)
{
  uint32_t i;
  int x = 0;
  int y = 100;
  ili9341_set_background(LCD_BG_COLOR);
  ili9341_set_foreground(LCD_FG_COLOR);
  ili9341_fill(0, y, OFFSETX, 7*(FONT_STR_HEIGHT));
  // Set 'C' string for slot status
  char c[4] = {'C', 0, 0, 0};
  if (cal_status & CALSTAT_APPLY) {
    if (cal_status & CALSTAT_INTERPOLATED){ili9341_set_foreground(LCD_NORMAL_BAT_COLOR); c[0] = 'c';}
    c[1] = '0' + lastsaveid;
    ili9341_drawstring(c, x, y);
  }
  ili9341_set_foreground(LCD_FG_COLOR);
  static const struct {char text, zero; uint16_t mask;} calibration_text[]={
    {'D', 0, CALSTAT_ED},
    {'R', 0, CALSTAT_ER},
    {'S', 0, CALSTAT_ES},
    {'T', 0, CALSTAT_ET},
    {'X', 0, CALSTAT_EX}
  };
  for (i = 0; i < ARRAY_COUNT(calibration_text); i++)
    if (cal_status & calibration_text[i].mask)
      ili9341_drawstring(&calibration_text[i].text, x, y+=FONT_STR_HEIGHT);

  if (cal_status & CALSTAT_APPLY){
    const properties_t *src = caldata_reference();
    if (src && src->_power != current_props._power)
      ili9341_set_foreground(LCD_LOW_BAT_COLOR);
  }
  c[0] = 'P';
  c[1] = current_props._power > 3 ? ('a') : (current_props._power * 2 + '2'); // 2,4,6,8 mA power or auto
  ili9341_drawstring(c, x, y);
}

// Draw battery level
#define BATTERY_TOP_LEVEL       4100
#define BATTERY_BOTTOM_LEVEL    3200
#define BATTERY_WARNING_LEVEL   3300

static void draw_battery_status(void)
{
  int16_t vbat = adc_vbat_read();
  if (vbat <= 0)
    return;
  uint8_t string_buf[16];
  // Set battery color
  ili9341_set_foreground(vbat < BATTERY_WARNING_LEVEL ? LCD_LOW_BAT_COLOR : LCD_NORMAL_BAT_COLOR);
  ili9341_set_background(LCD_BG_COLOR);
//  plot_printf(string_buf, sizeof string_buf, "V:%d", vbat);
//  ili9341_drawstringV(string_buf, 1, 60);
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
  ili9341_blitBitmap(3, 2, 8, x, string_buf);
}

void
request_to_redraw_grid(void)
{
  redraw_request |= REDRAW_AREA;
}

void
redraw_frame(void)
{
  ili9341_set_background(LCD_BG_COLOR);
  ili9341_clear_screen();
  draw_frequencies();
  draw_cal_status();
}

void
plot_init(void)
{
  force_set_markmap();
}
