/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 *
 *   Heavily messed about with by OneOfEleven July 2020
 *   DiSlord adaptation to use on NanoVNA
 */

// calculate physical component values to match an impendace to 'ref_impedance' (ie 50R)

#ifdef __USE_LC_MATCHING__

typedef struct
{
   float xps;   // Reactance parallel to source (can be NAN if not applicable)
   float xs;    // Serial reactance (can be 0.0 if not applicable)
   float xpl;   // Reactance parallel to load (can be NAN if not applicable)
} t_lc_match;

typedef struct
{
   uint32_t Hz;
   float R0;
   // L-Network solution structure
   t_lc_match matches[4];
   int16_t num_matches;
   uint16_t sweep_n;
} t_lc_match_array;

static t_lc_match_array lc_match_array;

static void lc_match_quadratic_equation(float a, float b, float c, float *x)
{
  const float d = (b * b) - (4.0f * a * c);
  if (d < 0){
    x[0] = x[1] = 0.0f;
    return;
  }
  const float sd = sqrtf(d);
  const float a2 = 2.0f * a;
  x[0] = (-b + sd) / a2;
  x[1] = (-b - sd) / a2;
}

// Calculate two solutions for ZL where (R + X * X / R) > R0
static void lc_match_calc_hi(float R0, float RL, float XL, t_lc_match *matches)
{
  float xp[2];

  const float a = R0 - RL;
  const float b = 2.0f * XL * R0;
  const float c = R0 * ((XL * XL) + (RL * RL));
  lc_match_quadratic_equation(a, b, c, xp);

  // found two impedances parallel to load
  //
  // now calculate serial impedances
  const float RL1 = -XL * xp[0];
  const float XL1 =  RL * xp[0];
  const float RL2 =  RL + 0.0f;
  const float XL2 =  XL + xp[0];
  matches[0].xs  = ((RL1 * XL2) - (RL2 * XL1)) / ((RL2 * RL2) + (XL2 * XL2));
  matches[0].xps = 0.0f;
  matches[0].xpl = xp[0];

  const float RL3 = -XL * xp[1];
  const float XL3 =  RL * xp[1];
  const float RL4 =  RL + 0.0f;
  const float XL4 =  XL + xp[1];
  matches[1].xs  = ((RL3 * XL4) - (RL4 * XL3)) / ((RL4 * RL4) + (XL4 * XL4));
  matches[1].xps = 0.0f;
  matches[1].xpl = xp[1];
}

// Calculate two solutions for ZL where R < R0
static void lc_match_calc_lo(float R0, float RL, float XL, t_lc_match *matches)
{
  float xs[2];
  // Calculate Xs
  const float a = 1.0f;
  const float b = 2.0f * XL;
  const float c = (RL * RL) + (XL * XL) - (R0 * RL);
  lc_match_quadratic_equation(a, b, c, xs);

  // got two serial impedances that change ZL to the Y.real = 1/R0
  //
  // now calculate impedances parallel to source
  const float RL1 = RL  + 0.0f;
  const float XL1 = XL  + xs[0];
  const float RL3 = RL1 * R0;
  const float XL3 = XL1 * R0;
  const float RL5 = RL1 - R0;
  const float XL5 = XL1 - 0.0f;
  matches[0].xs  = xs[0];
  matches[0].xps = ((RL5 * XL3) - (RL3 * XL5)) / ((RL5 * RL5) + (XL5 * XL5));
  matches[0].xpl = 0.0f;

  const float RL2 = RL  + 0.0f;
  const float XL2 = XL  + xs[1];
  const float RL4 = RL2 * R0;
  const float XL4 = XL2 * R0;
  const float RL6 = RL2 - R0;
  const float XL6 = XL2 - 0.0f;
  matches[1].xs  = xs[1];
  matches[1].xps = ((RL6 * XL4) - (RL4 * XL6)) / ((RL6 * RL6) + (XL6 * XL6));
  matches[1].xpl = 0.0f;
}

static int lc_match_calc(int index)
{
  const float R0 = lc_match_array.R0;
  // compute the impedance at the chosen frequency
  const float *coeff = measured[0][index];
  const float RL = resistance(coeff);
  const float XL = reactance(coeff);

  if (RL <= 0.5f)
    return -1;

  const float q_factor = XL / RL;
  const float vswr = swr(coeff);
  // no need for any matching
  if (vswr <= 1.1f || q_factor >= 100.0f)
    return 0;

  // only one solution is enough: just a serial reactance
  // this gives SWR < 1.1 if R is within the range 0.91 .. 1.1 of R0
  t_lc_match *matches = lc_match_array.matches;
  if ((RL * 1.1f) > R0  && RL < (R0 * 1.1f)){
    matches[0].xpl = 0.0f;
    matches[0].xps = 0.0f;
    matches[0].xs  = -XL;
    return 1;
  }

  if (RL >= R0)
  {   // two Hi-Z solutions
    lc_match_calc_hi(R0, RL, XL, &matches[0]);
    return 2;
  }

  // compute Lo-Z solutions
  lc_match_calc_lo(R0, RL, XL, &matches[0]);
  if ((RL + (XL * q_factor)) <= R0)
    return 2;

  // two more Hi-Z solutions exist
  lc_match_calc_hi(R0, RL, XL, &matches[2]);
  return 4;
}

// Mark to redraw area under L/C match text
static void lc_match_mark_area(void){
  // Update area
  int n = lc_match_array.num_matches; if (n < 0) n = 0;
  invalidate_rect(STR_LC_MATH_X                        , STR_LC_MATH_Y,
                  STR_LC_MATH_X + 3 * STR_LC_MATH_WIDTH, STR_LC_MATH_Y + (n + 2)*STR_LC_MATH_HEIGHT);
}

static void lc_match_process(void)
{
  const uint32_t am = (uint32_t)active_marker;
  if (am >=MARKERS_MAX || current_props._markers[am].enabled == false)
    return;

  const uint32_t index = current_props._markers[am].index;
  if (index >= sweep_points || frequencies[index] == 0)
    return;

  // Made calculation only one time for current sweep and frequency
  if (lc_match_array.sweep_n == sweep_count && lc_match_array.Hz == frequencies[index])
    return;

  lc_match_array.R0 = 50.0f;
  lc_match_array.Hz = frequencies[index];
  lc_match_array.sweep_n = sweep_count;

  // compute the possible LC matches
  lc_match_array.num_matches = lc_match_calc(index);
  lc_match_mark_area();
}

//
static void lc_match_x_str(uint32_t FHz, float X, int xp, int yp)
{
  if (isnan(X) || 0.0f == X || -0.0f == X)
    return;

  char type;
  char str[12];
#if 0
  float val;
  if (X < 0.0f) {val = 1.0f / (2.0f * VNA_PI * FHz * -X); type = 'F';}
  else          {val =    X / (2.0f * VNA_PI * FHz);      type = 'H';}
#else
  if (X < 0.0f) {X = -1.0 / X; type = 'F';}
  else          {              type = 'H';}
  float val = X / (2.0f * VNA_PI * FHz);
#endif
  plot_printf(str, sizeof(str), "%4.2F%c", val, type);
  cell_drawstring(str, xp, yp);
}

// Render L/C match to cell
static void cell_draw_lc_match(int x0, int y0)
{
  char s[32];
  lc_match_process();

  int xp = STR_LC_MATH_X - x0;
  int yp = STR_LC_MATH_Y - y0;

  ili9341_set_background(LCD_BG_COLOR);
  ili9341_set_foreground(LCD_LC_MATCH_COLOR);

  if (yp > -FONT_GET_HEIGHT && yp < CELLHEIGHT)
  {
     plot_printf(s, sizeof(s), "L/C match for source Z0 = %0.1f"S_OHM, lc_match_array.R0);
     cell_drawstring(s, xp, yp);
  }
#if 0
  yp += STR_LC_MATH_HEIGHT;
  if (yp > -FONT_GET_HEIGHT && yp < CELLHEIGHT)
  {
     plot_printf(s, sizeof(s), "%qHz %0.1f %c j%0.1f"S_OHM, match_array->Hz, match_array->RL, (match_array->XL >= 0) ? '+' : '-', fabsf(match_array->XL));
     cell_drawstring(s, xp, yp);
  }
#endif

  yp += STR_LC_MATH_HEIGHT;
  if (yp >= CELLHEIGHT) return;
  if (lc_match_array.num_matches < 0)
    cell_drawstring("No LC match for this", xp, yp);
  else if (lc_match_array.num_matches == 0)
    cell_drawstring("No need for LC match", xp, yp);
  else {
    cell_drawstring("Src shunt" , xp                      , yp);
    cell_drawstring("Series"    , xp +   STR_LC_MATH_WIDTH, yp);
    cell_drawstring("Load shunt", xp + 2*STR_LC_MATH_WIDTH, yp);
    for (int i = 0; i < lc_match_array.num_matches; i++){
      yp += STR_LC_MATH_HEIGHT;
      if (yp >= CELLHEIGHT) return;
      if (yp > -FONT_GET_HEIGHT){
        lc_match_x_str(lc_match_array.Hz, lc_match_array.matches[i].xps, xp                      , yp);
        lc_match_x_str(lc_match_array.Hz, lc_match_array.matches[i].xs , xp +   STR_LC_MATH_WIDTH, yp);
        lc_match_x_str(lc_match_array.Hz, lc_match_array.matches[i].xpl, xp + 2*STR_LC_MATH_WIDTH, yp);
      }
    }
  }
}
#endif
