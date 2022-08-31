/*
 * Copyright (c) 2019-2021, Dmitry (DiSlord) dislordlive@gmail.com
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
#ifndef __VNA_MATH_H
#define __VNA_MATH_H
// Use math.h functions if need
#include <math.h>

#ifndef __FPU_PRESENT
#define __FPU_PRESENT  0
#endif
#ifndef __FPU_USED
#define __FPU_USED     0
#endif

// VNA math used library
#ifdef __USE_VNA_MATH__
// Some functions implemented in hardware FPU
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
__attribute__((always_inline)) __STATIC_INLINE float vna_fabsf(float x){__asm__ ("vabs.f32 %0, %1" : "=t"(x) : "t"(x)); return x;}
__attribute__((always_inline)) __STATIC_INLINE float vna_sqrtf(float x){__asm__ ("vsqrt.f32 %0, %1" : "=t"(x) : "t"(x)); return x;}
__attribute__((always_inline)) __STATIC_INLINE float vna_fmaf(float x, float y, float z){__asm__ ("vfma.f32 %0, %1, %2" : "+t"(z) : "t"(x), "t"(y)); return z;}

#else
// Define inline functions
__attribute__((always_inline)) __STATIC_INLINE float vna_fabsf(float x){union {float f; uint32_t i;} u = {x}; u.i &= 0x7fffffff; return u.f;}
__attribute__((always_inline)) __STATIC_INLINE float vna_fmaf(float x, float y, float z){return z+x*y;}
// square root
float vna_sqrtf(float x);
#endif
//================================
// log
float vna_logf(float x);
float vna_log10f_x_10(float x);
float vna_expf(float x);
// atan
float vna_atanf(float x);
float vna_atan2f(float x, float y);
// modff
float vna_modff(float x, float *iptr);
#else
// Use defaults math functions
#define vna_fabsf        fabsf
#define vna_sqrtf        sqrtf
#define vna_logf         logf
#define vna_log10f_x_10 (logf(x) * (10.0f / logf(10.0f)))
#define vna_expf         expf
#define vna_atanf        atanf
#define vna_atan2f       atan2f
#define vna_modff        modff
#endif

// fft
void fft(float array[][2], const uint8_t dir);
#define fft_forward(array) fft(array, 0)
#define fft_inverse(array) fft(array, 1)

// cube root
float vna_cbrtf(float x);

// Return sin/cos value, angle have range 0.0 to 1.0 (0 is 0 degree, 1 is 360 degree)
void vna_sincosf(float angle, float * pSinVal, float * pCosVal);

#endif
