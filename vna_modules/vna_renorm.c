/*
 * Copyright (c) 2019-2020, Dmitry (DiSlord) dislordlive@gmail.com
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

#ifdef __VNA_Z_RENORMALIZATION__
static void apply_renormalization(float data[4], uint16_t mask)
{
//  if (current_props._portz == 50.0f) return;
//  S -> Z parameter -> S
//  newS11 = (g*s21^2)/((1-g*s11)^2-g^2*s21^2)+((s11-g)*(1-g*s11))/((1-g*s11)^2-g^2*s21^2)
//  newS21 = ((1-g*s11)*s21)/((1-g*s11)^2-g^2*s21^2)+(g*(s11-g)*s21)/((1-g*s11)^2-g^2*s21^2)
#if 0
  complexf s11 = refl;
  complexf s21 = thru;
  float z = current_props._portz;
  float g = (z-50.f)/(z+50.f);
  complexf g_s11 = 1.f - g * s11;
  complexf g_s21 = g * s21;
  complexf s11_g = s11 - g;
  complexf denom_r = 1.f / (g_s11 * g_s11 - g_s21 * g_s21);

  refl = (g_s21 * s21 + s11_g * g_s11) * denom_r;
  thru = (g_s11 * s21 + s11_g * g_s21) * denom_r;
#elif 0
  float k = (1.0f - g*g);
  complexf pow = 2.0f*s11 - g*(s11*s11 - s21*s21) - g;
  complexf den = 1.0f / (k - g*pow);
  refl = (pow - s11*k) * den;
  thru =        s21*k  * den;
#endif
  float s11_re = mask & SWEEP_CH0_MEASURE ? data[0] : 0.0f;
  float s11_im = mask & SWEEP_CH0_MEASURE ? data[1] : 0.0f;

  float s21_re = mask & SWEEP_CH1_MEASURE ? data[2] : 0.0f;
  float s21_im = mask & SWEEP_CH1_MEASURE ? data[3] : 0.0f;

  float z = current_props._portz;
  float g = (z - 50.f) / (z + 50.f);
#if 0
  float g_s11_re = 1.0f - g * s11_re;                              // 1 - g * s11
  float g_s11_im = 0.0f - g * s11_im;

  float s11_g_re = s11_re - g;                                     // g_s11 = s11 - g
  float s11_g_im = s11_im;

  float g_s21_re = g * s21_re;                                     // g_s21 = g * s21
  float g_s21_im = g * s21_im;

// denom_r calc
  float g_s11_pow2_re = g_s11_re * g_s11_re - g_s11_im * g_s11_im; // g_s11 * g_s11
  float g_s11_pow2_im = 2 * g_s11_re * g_s11_im;

  float g_s21_pow2_re = g_s21_re * g_s21_re - g_s21_im * g_s21_im; // g_s21 * g_s21
  float g_s21_pow2_im = 2 * g_s21_re * g_s21_im;

  float denom_r_re = g_s11_pow2_re - g_s21_pow2_re;                // g_s11 * g_s11 - g_s21 * g_s21
  float denom_r_im = g_s11_pow2_im - g_s21_pow2_im;

  float d = denom_r_re * denom_r_re + denom_r_im * denom_r_im;
  float d_re = denom_r_re / d;                                     // d = 1 / denom_d
  float d_im =-denom_r_im / d;

  if (mask & SWEEP_CH0_MEASURE) {
    float r1_re = g_s21_re * s21_re - g_s21_im * s21_im;           // g_s21 * s21
    float r1_im = g_s21_re * s21_im + g_s21_im * s21_re;

    float r2_re = s11_g_re * g_s11_re - s11_g_im * g_s11_im;       // s11_g * g_s11
    float r2_im = s11_g_re * g_s11_im + s11_g_im * g_s11_re;

    float r_re = r1_re + r2_re;                                    // g_s21 * s21 + s11_g * g_s11
    float r_im = r1_im + r2_im;

    data[0] = r_re * d_re - r_im * d_im;                           // refl = (g_s21 * s21 + s11_g * g_s11) / denom_r
    data[1] = r_re * d_im + r_im * d_re;
  }

  if (mask & SWEEP_CH1_MEASURE) {
    float t1_re = g_s11_re * s21_re - g_s11_im * s21_im;           // g_s11 * s21
    float t1_im = g_s11_re * s21_im + g_s11_im * s21_re;

    float t2_re = s11_g_re * g_s21_re - s11_g_im * g_s21_im;       // s11_g * g_s21
    float t2_im = s11_g_re * g_s21_im + s11_g_im * g_s21_re;

    float t_re = t1_re + t2_re;                                    // g_s11 * s21 + s11_g * g_s21
    float t_im = t1_im + t2_im;

    data[2] = t_re * d_re - t_im * d_im;                           // thru = (g_s11 * s21 + s11_g * g_s21) / denom_r
    data[3] = t_re * d_im + t_im * d_re;
  }
#else
  float k = (1.0f - g*g);
  //complexf pow = 2.0f*s11 - g*(s11*s11 - s21*s21) - g;
  float pow_re = 2.0f * s11_re - g * (s11_re * s11_re - s11_im*s11_im - s21_re * s21_re + s21_im*s21_im) - g;
  float pow_im = 2.0f * (s11_im - g * (s11_re * s11_im - s21_re * s21_im));
  //complexf den = 1.0f / (k - g*pow);
  float d_re = k - g * pow_re;
  float d_im =     g * pow_im;
  float d = d_re*d_re + d_im*d_im;
  d_re/= d;
  d_im/= d;
  //refl = (pow - s11*k) * den;
  if (mask & SWEEP_CH0_MEASURE) {
	float r_re = pow_re - s11_re * k;
	float r_im = pow_im - s11_im * k;
    data[0] = r_re * d_re - r_im * d_im;
    data[1] = r_re * d_im + r_im * d_re;
  }
  // thru =        s21*k  * den;
  if (mask & SWEEP_CH1_MEASURE) {
	float r_re = s21_re * k;
	float r_im = s21_im * k;
    data[2] = r_re * d_re - r_im * d_im;
    data[3] = r_re * d_im + r_im * d_re;
  }
#endif
}
#endif
