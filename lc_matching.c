/*
 *   (c) Yury Kuchura
 *   kuchura@gmail.com
 *
 *   This code can be used on terms of WTFPL Version 2 (http://www.wtfpl.net/).
 *
 *   Heavily messed about with by OneOfEleven July 2020
 *   DiSlord adaptation to use on NanoVNA
 */

#ifdef __VNA_MEASURE_MODULE__
// Memory for measure cache data
static char measure_memory[128];

// Measure math functions
// quadratic function solver
static void match_quadratic_equation(float a, float b, float c, float *x)
{
  const float a_x_2 = 2.0f * a;
  const float d = (b * b) - (2.0f * a_x_2 * c);
  if (d < 0){
    x[0] = x[1] = 0.0f;
    return;
  }
  const float sd = vna_sqrtf(d);
  x[0] = (-b + sd) / a_x_2;
  x[1] = (-b - sd) / a_x_2;
}

// Search functions
// Type of get value function
typedef float (*get_value_t)(uint16_t idx);

// Search point get_value(x) = y
// Used bilinear interpolation, return value = frequency of this point
#define MEASURE_SEARCH_LEFT  -1
#define MEASURE_SEARCH_RIGHT  1
static float measure_search_value(uint16_t *idx, float y, get_value_t get, int16_t mode){
  uint16_t x = *idx;
  float y1, y2, y3;
  y1 = y2 = y3 = get(x);
  bool result = (y3 > y); // current position depend from start point
  for(; x < sweep_points; x+=mode) {
    y3 = get(x);
    if(result != (y3 > y)) break;
    y1 = y2;
    y2 = y3;
  }
  if (x >= sweep_points) return 0;
  x-=mode;
  *idx = x;
  // Now y1 > y, y2 > y, y3 <= y or y1 < y, y2 < y, y3 >= y
  const float a = 0.5f * (y1 + y3) - y2;
  const float b = 0.5f * (y3 - y1);
  const float c = y2 - y;
  float r[2];
  match_quadratic_equation(a, b, c, r);
  // Select result in middle 0 and 1 (in middle y2 and y3 result)
  float res = (r[0] > 0 && r[0] < 1.0) ? r[0] : r[1];
  // for search left need swap y1 and y3 points (use negative result)
  if (mode < 0) res=-res;
  return getFrequency(x) + getFrequencyStep() * res;
}

// Peak search, use bilinear interpolation for peak detect
#define MEASURE_SEARCH_MIN 0
#define MEASURE_SEARCH_MAX 1
static bool _greaterf(float x, float y) { return x > y; }
static bool _lesserf(float x, float y) { return x < y; }
static float search_peak_value(uint16_t *xp, get_value_t get, bool mode){
  bool (*compare)(float x, float y) = mode ? _greaterf : _lesserf;
  uint16_t x = 0;
  float y2 = get(x), ytemp;
  for(int i = 1; i < sweep_points; i++) {
    if(compare(ytemp = get(i), y2)) {
      y2 = ytemp;
      x = i;
    }
  }
  if (x < 1 || x >= sweep_points - 1) return y2;
  *xp = x;
  float y1 = get(x-1);
  float y3 = get(x+1);
  if (y1 == y3) return y2;
//  const float a = 0.5f * (y1 + y3) - y2;
//  const float b = 0.5f * (y3 - y1);
//  const float c = y2;
//  return c - b*b/(4*a);
  const float a = 8*(y1 - 2*y2 + y3);
  const float b = y3 - y1;
  const float c = y2;
  return c - b * b / a;
}

static float bilinear_interpolation(float y1, float y2, float y3, float k1){
  const float a = 0.5f * (y1 + y3) - y2;
  const float b = 0.5f * (y3 - y1);
  const float c = y2;
  return a * k1*k1 + b * k1 + c;
}

static bool measure_get_value(uint16_t ch, freq_t f, float *data){
  if (f < frequency0 || f > frequency1)
    return false;
  // Search k1
  uint16_t _points = sweep_points - 1;
  freq_t span = frequency1 - frequency0;
  uint32_t idx = (uint64_t)(f - frequency0) * (uint64_t)_points / span;
  if (idx < 1 && idx > _points)
    return false;
  uint64_t v = (uint64_t)span * idx + _points/2;
  freq_t src_f0 = frequency0 + (v       ) / _points;
  freq_t src_f1 = frequency0 + (v + span) / _points;
  freq_t delta = src_f1 - src_f0;
  float k1 = (delta == 0) ? 0.0 : (float)(f - src_f0) / delta;
#if 1
  // Bilinear interpolation by k1
  data[0] = bilinear_interpolation(measured[ch][idx-1][0], measured[ch][idx  ][0], measured[ch][idx+1][0],k1);
  data[1] = bilinear_interpolation(measured[ch][idx-1][1], measured[ch][idx  ][1], measured[ch][idx+1][1],k1);
#else
  // Linear Interpolate by k1
  float k0 = 1.0 - k1;
  data[0] = measured[ch][idx][0] * k0 + measured[ch][idx+1][0] * k1;
  data[1] = measured[ch][idx][1] * k0 + measured[ch][idx+1][1] * k1;
#endif
  return true;
}

#ifdef __USE_LC_MATCHING__
// calculate physical component values to match an impendace to 'ref_impedance' (ie 50R)
typedef struct
{
   float xps;   // Reactance parallel to source (can be NAN if not applicable)
   float xs;    // Serial reactance (can be 0.0 if not applicable)
   float xpl;   // Reactance parallel to load (can be NAN if not applicable)
} t_lc_match;

typedef struct
{
   freq_t Hz;
   float  R0;
   // L-Network solution structure
   t_lc_match matches[4];
   int16_t num_matches;
} lc_match_array_t;

// Size = 60 bytes
static lc_match_array_t *lc_match_array = (lc_match_array_t *)measure_memory;

// Calculate two solutions for ZL where (R + X * X / R) > R0
static void lc_match_calc_hi(float R0, float RL, float XL, t_lc_match *matches)
{
  float xp[2];

  const float a = R0 - RL;
  const float b = 2.0f * XL * R0;
  const float c = R0 * (XL * XL + RL * RL);
  match_quadratic_equation(a, b, c, xp);

  // found two impedances parallel to load
  //
  // now calculate serial impedances
  const float RL1 = -XL * xp[0];
  const float XL1 =  RL * xp[0];
  const float RL2 =  RL;// + 0.0f;
  const float XL2 =  XL + xp[0];
  matches[0].xs  = (RL1 * XL2 - RL2 * XL1) / (RL2 * RL2 + XL2 * XL2);
  matches[0].xps = 0.0f;
  matches[0].xpl = xp[0];

  const float RL3 = -XL * xp[1];
  const float XL3 =  RL * xp[1];
  const float RL4 =  RL;// + 0.0f;
  const float XL4 =  XL + xp[1];
  matches[1].xs  = (RL3 * XL4 - RL4 * XL3) / (RL4 * RL4 + XL4 * XL4);
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
  const float c = RL * RL + XL * XL - R0 * RL;
  match_quadratic_equation(a, b, c, xs);

  // got two serial impedances that change ZL to the Y.real = 1/R0
  //
  // now calculate impedances parallel to source
  const float RL1 = RL;//  + 0.0f;
  const float XL1 = XL  + xs[0];
  const float RL3 = RL1 * R0;
  const float XL3 = XL1 * R0;
  const float RL5 = RL1 - R0;
  const float XL5 = XL1;// - 0.0f;
  matches[0].xs  = xs[0];
  matches[0].xps = (RL5 * XL3 - RL3 * XL5) / (RL5 * RL5 + XL5 * XL5);
  matches[0].xpl = 0.0f;

  const float RL2 = RL;//  + 0.0f;
  const float XL2 = XL  + xs[1];
  const float RL4 = RL2 * R0;
  const float XL4 = XL2 * R0;
  const float RL6 = RL2 - R0;
  const float XL6 = XL2;// - 0.0f;
  matches[1].xs  = xs[1];
  matches[1].xps = (RL6 * XL4 - RL4 * XL6) / (RL6 * RL6 + XL6 * XL6);
  matches[1].xpl = 0.0f;
}

static int lc_match_calc(int index)
{
  const float R0 = lc_match_array->R0;
  // compute the impedance at the chosen frequency
  const float *coeff = measured[0][index];
  const float RL = resistance(index, coeff);
  const float XL = reactance(index, coeff);

  if (RL <= 0.5f)
    return -1;

  const float q_factor = XL / RL;
  const float vswr = swr(index, coeff);
  // no need for any matching
  if (vswr <= 1.1f || q_factor >= 100.0f)
    return 0;

  // only one solution is enough: just a serial reactance
  // this gives SWR < 1.1 if R is within the range 0.91 .. 1.1 of R0
  t_lc_match *matches = lc_match_array->matches;
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

static void prepare_lc_match(uint8_t mode, uint8_t update_mask)
{
  (void)mode;
  (void)update_mask;
  // Made calculation only one time for current sweep and frequency
  freq_t freq = get_marker_frequency(active_marker);
  if (freq == 0)// || lc_match_array->Hz == freq)
    return;

  lc_match_array->R0 = 50.0f;
  lc_match_array->Hz = freq;

  // compute the possible LC matches
  lc_match_array->num_matches = lc_match_calc(markers[active_marker].index);

  // Mark to redraw area under L/C match text
  int n = lc_match_array->num_matches;
  if (n < 0) n = 0;
  invalidate_rect(STR_MEASURE_X                        , STR_MEASURE_Y,
                  STR_MEASURE_X + 3 * STR_MEASURE_WIDTH, STR_MEASURE_Y + (n + 2)*STR_MEASURE_HEIGHT);
}

//
static void lc_match_x_str(uint32_t FHz, float X, int xp, int yp)
{
  if (isnan(X) || 0.0f == X || -0.0f == X)
    return;

  char type;
#if 0
  float val;
  if (X < 0.0f) {val = 1.0f / (2.0f * VNA_PI * FHz * -X); type = 'F';}
  else          {val =    X / (2.0f * VNA_PI * FHz);      type = 'H';}
#else
  if (X < 0.0f) {X = -1.0 / X; type = 'F';}
  else          {              type = 'H';}
  float val = X / ((2.0f * VNA_PI) * FHz);
#endif
  cell_printf(xp, yp, "%4.2F%c", val, type);
}

// Render L/C match to cell
static void draw_lc_match(int x0, int y0)
{
  int xp = STR_MEASURE_X - x0;
  int yp = STR_MEASURE_Y - y0;
  cell_printf(xp, yp, "L/C match for source Z0 = %0.1f"S_OHM, lc_match_array->R0);
#if 0
  yp += STR_MEASURE_HEIGHT;
  cell_printf(xp, yp, "%qHz %0.1f %c j%0.1f"S_OHM, match_array->Hz, match_array->RL, (match_array->XL >= 0) ? '+' : '-', vna_fabsf(match_array->XL));
#endif
  yp += STR_MEASURE_HEIGHT;
  if (yp >= CELLHEIGHT) return;
  if (lc_match_array->num_matches < 0)
    cell_printf(xp, yp, "No LC match for this");
  else if (lc_match_array->num_matches == 0)
    cell_printf(xp, yp, "No need for LC match");
  else {
    cell_printf(xp                      , yp, "Src shunt" );
    cell_printf(xp +   STR_MEASURE_WIDTH, yp, "Series"    );
    cell_printf(xp + 2*STR_MEASURE_WIDTH, yp, "Load shunt");
    for (int i = 0; i < lc_match_array->num_matches; i++){
      yp += STR_MEASURE_HEIGHT;
      if (yp >= CELLHEIGHT) return;
      lc_match_x_str(lc_match_array->Hz, lc_match_array->matches[i].xps, xp                      , yp);
      lc_match_x_str(lc_match_array->Hz, lc_match_array->matches[i].xs , xp +   STR_MEASURE_WIDTH, yp);
      lc_match_x_str(lc_match_array->Hz, lc_match_array->matches[i].xpl, xp + 2*STR_MEASURE_WIDTH, yp);
    }
  }
}
#endif // __USE_LC_MATCHING__

#ifdef __S21_MEASURE__
typedef struct {
  char *header;
  freq_t freq;    // resonant frequency
  freq_t freq1;   // fp
  float l;
  float c;
  float c1;       // capacitor parallel
  float r;
  float q;        // Q factor

//  freq_t f1;
//  freq_t f2;
//  float tan45;
} s21_analysis_t;
static s21_analysis_t *s21_measure = (s21_analysis_t *)measure_memory;

static float s21pow2(uint16_t i) {
  const float re = measured[1][i][0]; // S21 real
  const float im = measured[1][i][1]; // S21 imaginary
  return re*re+im*im; // S21^2
}

static float s21tan(uint16_t i) {
  const float re = measured[1][i][0]; // S21 real
  const float im = measured[1][i][1]; // S21 imaginary
  return im/re; // tan(S21)
}

// Phase Shift Measurement
// https://www.mikrocontroller.net/attachment/473317/Crystal_Motional_Parameters.pdf
static void analysis_lcshunt(void) {
  uint16_t xp = 0, x2;
  s21_measure->header = "LC-SHUNT";
  // Minimum search
  float ypeak = search_peak_value(&xp, s21pow2, MEASURE_SEARCH_MIN);
  // peak frequency, R
  float att = vna_sqrtf(ypeak);
  s21_measure->r = config._measure_r * att / (2 * (1 - att));
  if(s21_measure->r < 0) return;
  set_marker_index(0, xp);

  float tan45 = config._measure_r/(config._measure_r + 4 * s21_measure->r);
//  s21_measure->tan45 = tan45;
  // -45 degree search at left
  x2 = xp;
  float f1 = measure_search_value(&x2, -tan45, s21tan, MEASURE_SEARCH_LEFT);
  if (f1 == 0) return;
  set_marker_index(1, x2);

  // +45 degree search at right
  x2 = xp;
  float f2 = measure_search_value(&x2,  tan45, s21tan, MEASURE_SEARCH_RIGHT);
  if (f2 == 0) return;
  set_marker_index(2, x2);

  // L,C,Q calculations
  float bw = f2 - f1;
  float fpeak = vna_sqrtf(f2 * f1);
  s21_measure->freq = fpeak;
  s21_measure->q = fpeak / bw;
  s21_measure->l = s21_measure->q * s21_measure->r / ((2 * VNA_PI) * fpeak);
  s21_measure->c = 1 / ((2 * VNA_PI) * fpeak * s21_measure->q * s21_measure->r);
}

static void analysis_lcseries(void) {
  uint16_t xp=0, x2;
  s21_measure->header = "LC-SERIES";
  // Peak value and it frequency index search
  float ypeak = search_peak_value(&xp, s21pow2, MEASURE_SEARCH_MAX);
  if (xp == 0) return; // peak not found
  // motional resistance, Rm
  s21_measure->r = 2 * config._measure_r * (1.0f / vna_sqrtf(ypeak) - 1.0f);
  if(s21_measure->r < 0) return;
  set_marker_index(0, xp);

  const float tan45 = 1.0f; // tand(45) = 1.0f
  // Lookup +45 phase at left of xp index
  x2 = xp;
  float f1 = measure_search_value(&x2,  tan45, s21tan, MEASURE_SEARCH_LEFT);
  if (f1 == 0) return; // not found
  set_marker_index(1, x2);

  // Lookup -45 phase at right of xp index
  x2 = xp;
  float f2 = measure_search_value(&x2, -tan45, s21tan, MEASURE_SEARCH_RIGHT);
  if (f2 == 0) return; // not found
  set_marker_index(2, x2);

  // L,C,Q calculation
  float bw = f2 - f1;
  float fpeak = vna_sqrtf(f2*f1);
  // The total resistance, REFF, seen by the crystal is the sum of the load resistance (input and output) and the motional resistance, Rm:
  float reff = 2 * config._measure_r + s21_measure->r;

  s21_measure->freq = fpeak;
  s21_measure->l = reff / ((2 * VNA_PI) * bw);
  s21_measure->c = bw / ((2 * VNA_PI) * fpeak * fpeak * reff);
  // q = 2*pi * Fp * Ls / R
  s21_measure->q = (2 * VNA_PI) * fpeak * s21_measure->l / s21_measure->r;

//  s21_measure->f1 = f1;
//  s21_measure->f2 = f2;
}

static void analysis_xtalseries(void) {
  analysis_lcseries();
  s21_measure->header = "XTAL-SERIES";
  // search S21 min
  uint16_t xp=0;
  search_peak_value(&xp, s21pow2, MEASURE_SEARCH_MIN);
  if (xp == 0) return;
  set_marker_index(3, xp);

  freq_t freq1 = getFrequency(xp);
  if(freq1 < s21_measure->freq) return;
  s21_measure->freq1 = freq1;
  // df = f * c / (2*c1) => c1 = f * c / (2*df)
  s21_measure->c1 = s21_measure->c * s21_measure->freq / (2*(s21_measure->freq1 - s21_measure->freq));
}

static void draw_serial_result(int x0, int y0){
  int xp = STR_MEASURE_X - x0;
  int yp = STR_MEASURE_Y - y0;
  cell_printf(xp, yp, s21_measure->header);
  if (s21_measure->freq == 0 && s21_measure->freq1 == 0) {
    cell_printf(xp, yp+=STR_MEASURE_HEIGHT, "Not found");
    return;
  }
  if (s21_measure->freq)
  {
    cell_printf(xp, yp+=STR_MEASURE_HEIGHT, "Fs=%qHz", s21_measure->freq);
    cell_printf(xp, yp+=STR_MEASURE_HEIGHT, "Lm=%FH  Cm=%FF  Rm=%F" S_OHM, s21_measure->l, s21_measure->c, s21_measure->r);
    cell_printf(xp, yp+=STR_MEASURE_HEIGHT, "Q=%.3f", s21_measure->q);
//  cell_printf(xp, yp+=STR_MEASURE_HEIGHT, "tan45=%.4f", s21_measure->tan45);
//  cell_printf(xp, yp+=STR_MEASURE_HEIGHT, "F1=%qHz F2=%qHz", s21_measure->f1, s21_measure->f2);
  }
  if (s21_measure->freq1){
    cell_printf(xp, yp+=STR_MEASURE_HEIGHT, "Fp=%qHz", s21_measure->freq1);
    cell_printf(xp, yp+=STR_MEASURE_HEIGHT, "Cp=%FF", s21_measure->c1);
  }
}

static void prepare_series(uint8_t type, uint8_t update_mask)
{
  (void)update_mask;
  uint16_t n;
  // for detect completion
  s21_measure->freq  = 0;
  s21_measure->freq1 = 0;
  switch (type){
    case MEASURE_SHUNT_LC:    n = 4;    analysis_lcshunt(); break;
    case MEASURE_SERIES_LC:   n = 4;   analysis_lcseries(); break;
    case MEASURE_SERIES_XTAL: n = 6; analysis_xtalseries(); break;
    default: return;
  }
  // Prepare for update
  invalidate_rect(STR_MEASURE_X                        , STR_MEASURE_Y,
                  STR_MEASURE_X + 3 * STR_MEASURE_WIDTH, STR_MEASURE_Y + n * STR_MEASURE_HEIGHT);
  markmap_all_markers();
}
#endif // __S21_MEASURE__

#ifdef __S11_CABLE_MEASURE__
typedef struct {
  float R;
  float len;
  float loss;
  float C0;
  float a, b, c;
} s11_cable_measure_t;
static s11_cable_measure_t *s11_cable = (s11_cable_measure_t *)measure_memory;

static float s11imag(uint16_t i) {
  return measured[0][i][1];
}

static void draw_s11_cable(int x0, int y0){
  int xp = STR_MEASURE_X - x0;
  int yp = STR_MEASURE_Y - y0;
  cell_printf(xp, yp, "S11 CABLE");
  if (s11_cable->R){
    cell_printf(xp, yp+=STR_MEASURE_HEIGHT, "Z0 = %F" S_OHM, s11_cable->R);
//    cell_printf(xp, yp+=FONT_STR_HEIGHT, "C0 = %FF", s11_cable->C0);
  }
  if (s11_cable->len)
    cell_printf(xp, yp+=STR_MEASURE_HEIGHT, "Length = %Fm (K=%d%%)", s11_cable->len, velocity_factor);
  cell_printf(xp, yp+=STR_MEASURE_HEIGHT, "Loss = %FdB", s11_cable->loss);
}

static void prepare_s11_cable(uint8_t type, uint8_t update_mask)
{
  (void)type;
  freq_t f1;
  if (update_mask & MEASURE_UPD_SWEEP) {
    s11_cable->R = 0.0f;
    s11_cable->len = 0.0f;
    uint16_t x = 0;
    f1 = measure_search_value(&x,  0, s11imag, MEASURE_SEARCH_RIGHT);
    if (f1){
      s11_cable->len = velocity_factor * (SPEED_OF_LIGHT / 400.0f) / f1;
      float data[2];
      if (measure_get_value(0, f1/2, data)){
        s11_cable->R = vna_fabsf(reactance(0, data));
//        s11_cable->C0 = velocity_factor / (100.0f * SPEED_OF_LIGHT * s11_cable->R);
      }
    }
  }
  if ((update_mask & MEASURE_UPD_ALL) && active_marker != MARKER_INVALID) {
    int idx = markers[active_marker].index;
    s11_cable->loss = vna_fabsf(logmag(idx, measured[0][idx]) / 2);
  }
  // Prepare for update
  invalidate_rect(STR_MEASURE_X                        , STR_MEASURE_Y,
                  STR_MEASURE_X + 3 * STR_MEASURE_WIDTH, STR_MEASURE_Y + 4 * STR_MEASURE_HEIGHT);
}

#endif // __S11_CABLE_MEASURE__

#ifdef __S11_RESONANCE_MEASURE__
#define MEASURE_RESONANCE_COUNT   6
typedef struct {
  struct {
    freq_t f;
    float  r;
    float  x;
  } data[MEASURE_RESONANCE_COUNT];
  uint8_t count;
} s11_resonance_measure_t;
static s11_resonance_measure_t *s11_resonance = (s11_resonance_measure_t *)measure_memory;


static float s11_resonance_value(uint16_t i) {
  return measured[0][i][1];
}

static float s11_resonance_min(uint16_t i) {
  return fabsf(reactance(i, measured[0][i]));
}

static void draw_s11_resonance(int x0, int y0){
  int xp = STR_MEASURE_X - x0;
  int yp = STR_MEASURE_Y - y0;
  cell_printf(xp, yp, "S11 RESONANCE");
  if (s11_resonance->count == 0) {
    cell_printf(xp, yp+=STR_MEASURE_HEIGHT, "Not found");
    return;
  }
  for (int i = 0; i < s11_resonance->count; i++)
    cell_printf(xp, yp+=STR_MEASURE_HEIGHT, "%qHz, %F%+jF" S_OHM, s11_resonance->data[i].f, s11_resonance->data[i].r, s11_resonance->data[i].x);
}

static bool add_resonance_value(int i, uint16_t x, freq_t f) {
  float data[2];
  if (measure_get_value(0, f, data)) {
    s11_resonance->data[i].f = f;
    //set_marker_index(i, x);
    s11_resonance->data[i].r = resistance(x, data);
    s11_resonance->data[i].x = reactance(x, data);
    return true;
  }
  return false;
}

static void prepare_s11_resonance(uint8_t type, uint8_t update_mask)
{
  (void)type;
  if (update_mask & MEASURE_UPD_SWEEP) {
    int i;
    freq_t f;
    uint16_t x = 0;
    // Search resonances (X == 0)
    for (i = 0; i < MEASURE_RESONANCE_COUNT && i < MARKERS_MAX;) {
      f = measure_search_value(&x, 0.0f, s11_resonance_value, MEASURE_SEARCH_RIGHT);
      if (f == 0) break;
      if (add_resonance_value(i, x, f))
        i++;
      x++;
    }
    if (i == 0) { // Search minimum position, if resonances not found
      x = 0;
      search_peak_value(&x, s11_resonance_min, MEASURE_SEARCH_MIN);
      if (x && add_resonance_value(0, x, getFrequency(x)))
        i = 1;
    }
    s11_resonance->count = i;
  }
  // Prepare for update
  invalidate_rect(STR_MEASURE_X                        , STR_MEASURE_Y,
                  STR_MEASURE_X + 3 * STR_MEASURE_WIDTH, STR_MEASURE_Y + (MEASURE_RESONANCE_COUNT + 1) * STR_MEASURE_HEIGHT);
}
#endif //__S11_RESONANCE_MEASURE__

#endif // __VNA_MEASURE__
