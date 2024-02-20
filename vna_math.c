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
#include "nanovna.h"
#include <stdint.h>

// Use table increase transform speed, but increase code size
// Use compact table, need 1/4 code size, and not decrease speed
// Used only if not defined __VNA_USE_MATH_TABLES__ (use self table for TTF or direct sin/cos calculations)
#define FFT_USE_SIN_COS_TABLE

// Use sin table and interpolation for sin/sos calculations
#ifdef __VNA_USE_MATH_TABLES__
// Use 512 table for calculation sin/cos value, also use this table for FFT
#define FAST_MATH_TABLE_SIZE 512

// Not use high part of table
#define GET_SIN_TABLE(idx) (((idx) < 256) ? sin_table_512[(idx)] : -sin_table_512[(idx)-256])

static const float sin_table_512[FAST_MATH_TABLE_SIZE/2 + 1] = {
	/*
	 * float has about 7.2 digits of precision
		for (int i = 0; i < FAST_MATH_TABLE_SIZE; i++) {
			printf("% .8f,%c", sin(2 * M_PI * i / FAST_MATH_TABLE_SIZE), i % 8 == 7 ? '\n' : ' ');
		}
	*/
	 0.00000000f, 0.01227154f, 0.02454123f, 0.03680722f, 0.04906767f, 0.06132074f, 0.07356456f, 0.08579731f,
	 0.09801714f, 0.11022221f, 0.12241068f, 0.13458071f, 0.14673047f, 0.15885814f, 0.17096189f, 0.18303989f,
	 0.19509032f, 0.20711138f, 0.21910124f, 0.23105811f, 0.24298018f, 0.25486566f, 0.26671276f, 0.27851969f,
	 0.29028468f, 0.30200595f, 0.31368174f, 0.32531029f, 0.33688985f, 0.34841868f, 0.35989504f, 0.37131719f,
	 0.38268343f, 0.39399204f, 0.40524131f, 0.41642956f, 0.42755509f, 0.43861624f, 0.44961133f, 0.46053871f,
	 0.47139674f, 0.48218377f, 0.49289819f, 0.50353838f, 0.51410274f, 0.52458968f, 0.53499762f, 0.54532499f,
	 0.55557023f, 0.56573181f, 0.57580819f, 0.58579786f, 0.59569930f, 0.60551104f, 0.61523159f, 0.62485949f,
	 0.63439328f, 0.64383154f, 0.65317284f, 0.66241578f, 0.67155895f, 0.68060100f, 0.68954054f, 0.69837625f,
	 0.70710678f, 0.71573083f, 0.72424708f, 0.73265427f, 0.74095113f, 0.74913639f, 0.75720885f, 0.76516727f,
	 0.77301045f, 0.78073723f, 0.78834643f, 0.79583690f, 0.80320753f, 0.81045720f, 0.81758481f, 0.82458930f,
	 0.83146961f, 0.83822471f, 0.84485357f, 0.85135519f, 0.85772861f, 0.86397286f, 0.87008699f, 0.87607009f,
	 0.88192126f, 0.88763962f, 0.89322430f, 0.89867447f, 0.90398929f, 0.90916798f, 0.91420976f, 0.91911385f,
	 0.92387953f, 0.92850608f, 0.93299280f, 0.93733901f, 0.94154407f, 0.94560733f, 0.94952818f, 0.95330604f,
	 0.95694034f, 0.96043052f, 0.96377607f, 0.96697647f, 0.97003125f, 0.97293995f, 0.97570213f, 0.97831737f,
	 0.98078528f, 0.98310549f, 0.98527764f, 0.98730142f, 0.98917651f, 0.99090264f, 0.99247953f, 0.99390697f,
	 0.99518473f, 0.99631261f, 0.99729046f, 0.99811811f, 0.99879546f, 0.99932238f, 0.99969882f, 0.99992470f,
	 1.00000000f, 0.99992470f, 0.99969882f, 0.99932238f, 0.99879546f, 0.99811811f, 0.99729046f, 0.99631261f,
	 0.99518473f, 0.99390697f, 0.99247953f, 0.99090264f, 0.98917651f, 0.98730142f, 0.98527764f, 0.98310549f,
	 0.98078528f, 0.97831737f, 0.97570213f, 0.97293995f, 0.97003125f, 0.96697647f, 0.96377607f, 0.96043052f,
	 0.95694034f, 0.95330604f, 0.94952818f, 0.94560733f, 0.94154407f, 0.93733901f, 0.93299280f, 0.92850608f,
	 0.92387953f, 0.91911385f, 0.91420976f, 0.90916798f, 0.90398929f, 0.89867447f, 0.89322430f, 0.88763962f,
	 0.88192126f, 0.87607009f, 0.87008699f, 0.86397286f, 0.85772861f, 0.85135519f, 0.84485357f, 0.83822471f,
	 0.83146961f, 0.82458930f, 0.81758481f, 0.81045720f, 0.80320753f, 0.79583690f, 0.78834643f, 0.78073723f,
	 0.77301045f, 0.76516727f, 0.75720885f, 0.74913639f, 0.74095113f, 0.73265427f, 0.72424708f, 0.71573083f,
	 0.70710678f, 0.69837625f, 0.68954054f, 0.68060100f, 0.67155895f, 0.66241578f, 0.65317284f, 0.64383154f,
	 0.63439328f, 0.62485949f, 0.61523159f, 0.60551104f, 0.59569930f, 0.58579786f, 0.57580819f, 0.56573181f,
	 0.55557023f, 0.54532499f, 0.53499762f, 0.52458968f, 0.51410274f, 0.50353838f, 0.49289819f, 0.48218377f,
	 0.47139674f, 0.46053871f, 0.44961133f, 0.43861624f, 0.42755509f, 0.41642956f, 0.40524131f, 0.39399204f,
	 0.38268343f, 0.37131719f, 0.35989504f, 0.34841868f, 0.33688985f, 0.32531029f, 0.31368174f, 0.30200595f,
	 0.29028468f, 0.27851969f, 0.26671276f, 0.25486566f, 0.24298018f, 0.23105811f, 0.21910124f, 0.20711138f,
	 0.19509032f, 0.18303989f, 0.17096189f, 0.15885814f, 0.14673047f, 0.13458071f, 0.12241068f, 0.11022221f,
	 0.09801714f, 0.08579731f, 0.07356456f, 0.06132074f, 0.04906767f, 0.03680722f, 0.02454123f, 0.01227154f,
	 0.00000000f,
	/*
	            -0.01227154f,-0.02454123f,-0.03680722f,-0.04906767f,-0.06132074f,-0.07356456f,-0.08579731f,
	-0.09801714f,-0.11022221f,-0.12241068f,-0.13458071f,-0.14673047f,-0.15885814f,-0.17096189f,-0.18303989f,
	-0.19509032f,-0.20711138f,-0.21910124f,-0.23105811f,-0.24298018f,-0.25486566f,-0.26671276f,-0.27851969f,
	-0.29028468f,-0.30200595f,-0.31368174f,-0.32531029f,-0.33688985f,-0.34841868f,-0.35989504f,-0.37131719f,
	-0.38268343f,-0.39399204f,-0.40524131f,-0.41642956f,-0.42755509f,-0.43861624f,-0.44961133f,-0.46053871f,
	-0.47139674f,-0.48218377f,-0.49289819f,-0.50353838f,-0.51410274f,-0.52458968f,-0.53499762f,-0.54532499f,
	-0.55557023f,-0.56573181f,-0.57580819f,-0.58579786f,-0.59569930f,-0.60551104f,-0.61523159f,-0.62485949f,
	-0.63439328f,-0.64383154f,-0.65317284f,-0.66241578f,-0.67155895f,-0.68060100f,-0.68954054f,-0.69837625f,
	-0.70710678f,-0.71573083f,-0.72424708f,-0.73265427f,-0.74095113f,-0.74913639f,-0.75720885f,-0.76516727f,
	-0.77301045f,-0.78073723f,-0.78834643f,-0.79583690f,-0.80320753f,-0.81045720f,-0.81758481f,-0.82458930f,
	-0.83146961f,-0.83822471f,-0.84485357f,-0.85135519f,-0.85772861f,-0.86397286f,-0.87008699f,-0.87607009f,
	-0.88192126f,-0.88763962f,-0.89322430f,-0.89867447f,-0.90398929f,-0.90916798f,-0.91420976f,-0.91911385f,
	-0.92387953f,-0.92850608f,-0.93299280f,-0.93733901f,-0.94154407f,-0.94560733f,-0.94952818f,-0.95330604f,
	-0.95694034f,-0.96043052f,-0.96377607f,-0.96697647f,-0.97003125f,-0.97293995f,-0.97570213f,-0.97831737f,
	-0.98078528f,-0.98310549f,-0.98527764f,-0.98730142f,-0.98917651f,-0.99090264f,-0.99247953f,-0.99390697f,
	-0.99518473f,-0.99631261f,-0.99729046f,-0.99811811f,-0.99879546f,-0.99932238f,-0.99969882f,-0.99992470f,
	-1.00000000f,-0.99992470f,-0.99969882f,-0.99932238f,-0.99879546f,-0.99811811f,-0.99729046f,-0.99631261f,
	-0.99518473f,-0.99390697f,-0.99247953f,-0.99090264f,-0.98917651f,-0.98730142f,-0.98527764f,-0.98310549f,
	-0.98078528f,-0.97831737f,-0.97570213f,-0.97293995f,-0.97003125f,-0.96697647f,-0.96377607f,-0.96043052f,
	-0.95694034f,-0.95330604f,-0.94952818f,-0.94560733f,-0.94154407f,-0.93733901f,-0.93299280f,-0.92850608f,
	-0.92387953f,-0.91911385f,-0.91420976f,-0.90916798f,-0.90398929f,-0.89867447f,-0.89322430f,-0.88763962f,
	-0.88192126f,-0.87607009f,-0.87008699f,-0.86397286f,-0.85772861f,-0.85135519f,-0.84485357f,-0.83822471f,
	-0.83146961f,-0.82458930f,-0.81758481f,-0.81045720f,-0.80320753f,-0.79583690f,-0.78834643f,-0.78073723f,
	-0.77301045f,-0.76516727f,-0.75720885f,-0.74913639f,-0.74095113f,-0.73265427f,-0.72424708f,-0.71573083f,
	-0.70710678f,-0.69837625f,-0.68954054f,-0.68060100f,-0.67155895f,-0.66241578f,-0.65317284f,-0.64383154f,
	-0.63439328f,-0.62485949f,-0.61523159f,-0.60551104f,-0.59569930f,-0.58579786f,-0.57580819f,-0.56573181f,
	-0.55557023f,-0.54532499f,-0.53499762f,-0.52458968f,-0.51410274f,-0.50353838f,-0.49289819f,-0.48218377f,
	-0.47139674f,-0.46053871f,-0.44961133f,-0.43861624f,-0.42755509f,-0.41642956f,-0.40524131f,-0.39399204f,
	-0.38268343f,-0.37131719f,-0.35989504f,-0.34841868f,-0.33688985f,-0.32531029f,-0.31368174f,-0.30200595f,
	-0.29028468f,-0.27851969f,-0.26671276f,-0.25486566f,-0.24298018f,-0.23105811f,-0.21910124f,-0.20711138f,
	-0.19509032f,-0.18303989f,-0.17096189f,-0.15885814f,-0.14673047f,-0.13458071f,-0.12241068f,-0.11022221f,
	-0.09801714f,-0.08579731f,-0.07356456f,-0.06132074f,-0.04906767f,-0.03680722f,-0.02454123f,-0.01227154f,
	-0.00000000f*/
};
//
#if FFT_SIZE == 256
#define FFT_SIN(i) sin_table_512[    2*(i)]
#define FFT_COS(i) ((i) >  64 ?-sin_table_512[2*(i)-128] : sin_table_512[128-2*(i)])
#elif FFT_SIZE == 512
#define FFT_SIN(i) sin_table_512[      (i)]
#define FFT_COS(i) ((i) > 128 ?-sin_table_512[  (i)-128] : sin_table_512[128-  (i)])
#else
#error "Need use bigger sin/cos table for new FFT size"
#endif

#else
#ifdef FFT_USE_SIN_COS_TABLE
#if FFT_SIZE == 256
static const float sin_table_256[] = {
	/*
	 * float has about 7.2 digits of precision
		for (uint8_t i = 0; i < FFT_SIZE - (FFT_SIZE / 4); i++) {
			printf("% .8f,%c", sin(2 * M_PI * i / FFT_SIZE), i % 8 == 7 ? '\n' : ' ');
		}
	*/
	 // for FFT_SIZE = 256
	 0.00000000,  0.02454123,  0.04906767,  0.07356456,  0.09801714,  0.12241068,  0.14673047,  0.17096189,
	 0.19509032,  0.21910124,  0.24298018,  0.26671276,  0.29028468,  0.31368174,  0.33688985,  0.35989504,
	 0.38268343,  0.40524131,  0.42755509,  0.44961133,  0.47139674,  0.49289819,  0.51410274,  0.53499762,
	 0.55557023,  0.57580819,  0.59569930,  0.61523159,  0.63439328,  0.65317284,  0.67155895,  0.68954054,
	 0.70710678,  0.72424708,  0.74095113,  0.75720885,  0.77301045,  0.78834643,  0.80320753,  0.81758481,
	 0.83146961,  0.84485357,  0.85772861,  0.87008699,  0.88192126,  0.89322430,  0.90398929,  0.91420976,
	 0.92387953,  0.93299280,  0.94154407,  0.94952818,  0.95694034,  0.96377607,  0.97003125,  0.97570213,
	 0.98078528,  0.98527764,  0.98917651,  0.99247953,  0.99518473,  0.99729046,  0.99879546,  0.99969882,
	 1.00000000,/*  0.99969882,  0.99879546,  0.99729046,  0.99518473,  0.99247953,  0.98917651,  0.98527764,
	 0.98078528,  0.97570213,  0.97003125,  0.96377607,  0.95694034,  0.94952818,  0.94154407,  0.93299280,
	 0.92387953,  0.91420976,  0.90398929,  0.89322430,  0.88192126,  0.87008699,  0.85772861,  0.84485357,
	 0.83146961,  0.81758481,  0.80320753,  0.78834643,  0.77301045,  0.75720885,  0.74095113,  0.72424708,
	 0.70710678,  0.68954054,  0.67155895,  0.65317284,  0.63439328,  0.61523159,  0.59569930,  0.57580819,
	 0.55557023,  0.53499762,  0.51410274,  0.49289819,  0.47139674,  0.44961133,  0.42755509,  0.40524131,
	 0.38268343,  0.35989504,  0.33688985,  0.31368174,  0.29028468,  0.26671276,  0.24298018,  0.21910124,
	 0.19509032,  0.17096189,  0.14673047,  0.12241068,  0.09801714,  0.07356456,  0.04906767,  0.02454123,
	 0.00000000, -0.02454123, -0.04906767, -0.07356456, -0.09801714, -0.12241068, -0.14673047, -0.17096189,
	-0.19509032, -0.21910124, -0.24298018, -0.26671276, -0.29028468, -0.31368174, -0.33688985, -0.35989504,
	-0.38268343, -0.40524131, -0.42755509, -0.44961133, -0.47139674, -0.49289819, -0.51410274, -0.53499762,
	-0.55557023, -0.57580819, -0.59569930, -0.61523159, -0.63439328, -0.65317284, -0.67155895, -0.68954054,
	-0.70710678, -0.72424708, -0.74095113, -0.75720885, -0.77301045, -0.78834643, -0.80320753, -0.81758481,
	-0.83146961, -0.84485357, -0.85772861, -0.87008699, -0.88192126, -0.89322430, -0.90398929, -0.91420976,
	-0.92387953, -0.93299280, -0.94154407, -0.94952818, -0.95694034, -0.96377607, -0.97003125, -0.97570213,
	-0.98078528, -0.98527764, -0.98917651, -0.99247953, -0.99518473, -0.99729046, -0.99879546, -0.99969882,*/
};
// full size table:
//   sin = sin_table_256[i   ]
//   cos = sin_table_256[i+64]
//#define FFT_SIN(i) sin_table_256[(i)]
//#define FFT_COS(i) sin_table_256[(i)+64]

// for size use only 0-64 indexes
//   sin = i > 64 ? sin_table_256[128-i] : sin_table_256[   i];
//   cos = i > 64 ?-sin_table_256[ i-64] : sin_table_256[64-i];
#define FFT_SIN(i) ((i) > 64 ? sin_table_256[128-(i)] : sin_table_256[   (i)])
#define FFT_COS(i) ((i) > 64 ?-sin_table_256[ (i)-64] : sin_table_256[64-(i)])

#elif FFT_SIZE == 512
static const float sin_table_512[] = {
	/*
	 * float has about 7.2 digits of precision
		for (int i = 0; i < FFT_SIZE - (FFT_SIZE / 4); i++) {
			printf("% .8f,%c", sin(2 * M_PI * i / FFT_SIZE), i % 8 == 7 ? '\n' : ' ');
		}
	*/
	 // For FFT_SIZE = 512
	 0.00000000,  0.01227154,  0.02454123,  0.03680722,  0.04906767,  0.06132074,  0.07356456,  0.08579731,
	 0.09801714,  0.11022221,  0.12241068,  0.13458071,  0.14673047,  0.15885814,  0.17096189,  0.18303989,
	 0.19509032,  0.20711138,  0.21910124,  0.23105811,  0.24298018,  0.25486566,  0.26671276,  0.27851969,
	 0.29028468,  0.30200595,  0.31368174,  0.32531029,  0.33688985,  0.34841868,  0.35989504,  0.37131719,
	 0.38268343,  0.39399204,  0.40524131,  0.41642956,  0.42755509,  0.43861624,  0.44961133,  0.46053871,
	 0.47139674,  0.48218377,  0.49289819,  0.50353838,  0.51410274,  0.52458968,  0.53499762,  0.54532499,
	 0.55557023,  0.56573181,  0.57580819,  0.58579786,  0.59569930,  0.60551104,  0.61523159,  0.62485949,
	 0.63439328,  0.64383154,  0.65317284,  0.66241578,  0.67155895,  0.68060100,  0.68954054,  0.69837625,
	 0.70710678,  0.71573083,  0.72424708,  0.73265427,  0.74095113,  0.74913639,  0.75720885,  0.76516727,
	 0.77301045,  0.78073723,  0.78834643,  0.79583690,  0.80320753,  0.81045720,  0.81758481,  0.82458930,
	 0.83146961,  0.83822471,  0.84485357,  0.85135519,  0.85772861,  0.86397286,  0.87008699,  0.87607009,
	 0.88192126,  0.88763962,  0.89322430,  0.89867447,  0.90398929,  0.90916798,  0.91420976,  0.91911385,
	 0.92387953,  0.92850608,  0.93299280,  0.93733901,  0.94154407,  0.94560733,  0.94952818,  0.95330604,
	 0.95694034,  0.96043052,  0.96377607,  0.96697647,  0.97003125,  0.97293995,  0.97570213,  0.97831737,
	 0.98078528,  0.98310549,  0.98527764,  0.98730142,  0.98917651,  0.99090264,  0.99247953,  0.99390697,
	 0.99518473,  0.99631261,  0.99729046,  0.99811811,  0.99879546,  0.99932238,  0.99969882,  0.99992470,
	 1.00000000,/*  0.99992470,  0.99969882,  0.99932238,  0.99879546,  0.99811811,  0.99729046,  0.99631261,
	 0.99518473,  0.99390697,  0.99247953,  0.99090264,  0.98917651,  0.98730142,  0.98527764,  0.98310549,
	 0.98078528,  0.97831737,  0.97570213,  0.97293995,  0.97003125,  0.96697647,  0.96377607,  0.96043052,
	 0.95694034,  0.95330604,  0.94952818,  0.94560733,  0.94154407,  0.93733901,  0.93299280,  0.92850608,
	 0.92387953,  0.91911385,  0.91420976,  0.90916798,  0.90398929,  0.89867447,  0.89322430,  0.88763962,
	 0.88192126,  0.87607009,  0.87008699,  0.86397286,  0.85772861,  0.85135519,  0.84485357,  0.83822471,
	 0.83146961,  0.82458930,  0.81758481,  0.81045720,  0.80320753,  0.79583690,  0.78834643,  0.78073723,
	 0.77301045,  0.76516727,  0.75720885,  0.74913639,  0.74095113,  0.73265427,  0.72424708,  0.71573083,
	 0.70710678,  0.69837625,  0.68954054,  0.68060100,  0.67155895,  0.66241578,  0.65317284,  0.64383154,
	 0.63439328,  0.62485949,  0.61523159,  0.60551104,  0.59569930,  0.58579786,  0.57580819,  0.56573181,
	 0.55557023,  0.54532499,  0.53499762,  0.52458968,  0.51410274,  0.50353838,  0.49289819,  0.48218377,
	 0.47139674,  0.46053871,  0.44961133,  0.43861624,  0.42755509,  0.41642956,  0.40524131,  0.39399204,
	 0.38268343,  0.37131719,  0.35989504,  0.34841868,  0.33688985,  0.32531029,  0.31368174,  0.30200595,
	 0.29028468,  0.27851969,  0.26671276,  0.25486566,  0.24298018,  0.23105811,  0.21910124,  0.20711138,
	 0.19509032,  0.18303989,  0.17096189,  0.15885814,  0.14673047,  0.13458071,  0.12241068,  0.11022221,
	 0.09801714,  0.08579731,  0.07356456,  0.06132074,  0.04906767,  0.03680722,  0.02454123,  0.01227154,
	 0.00000000, -0.01227154, -0.02454123, -0.03680722, -0.04906767, -0.06132074, -0.07356456, -0.08579731,
	-0.09801714, -0.11022221, -0.12241068, -0.13458071, -0.14673047, -0.15885814, -0.17096189, -0.18303989,
	-0.19509032, -0.20711138, -0.21910124, -0.23105811, -0.24298018, -0.25486566, -0.26671276, -0.27851969,
	-0.29028468, -0.30200595, -0.31368174, -0.32531029, -0.33688985, -0.34841868, -0.35989504, -0.37131719,
	-0.38268343, -0.39399204, -0.40524131, -0.41642956, -0.42755509, -0.43861624, -0.44961133, -0.46053871,
	-0.47139674, -0.48218377, -0.49289819, -0.50353838, -0.51410274, -0.52458968, -0.53499762, -0.54532499,
	-0.55557023, -0.56573181, -0.57580819, -0.58579786, -0.59569930, -0.60551104, -0.61523159, -0.62485949,
	-0.63439328, -0.64383154, -0.65317284, -0.66241578, -0.67155895, -0.68060100, -0.68954054, -0.69837625,
	-0.70710678, -0.71573083, -0.72424708, -0.73265427, -0.74095113, -0.74913639, -0.75720885, -0.76516727,
	-0.77301045, -0.78073723, -0.78834643, -0.79583690, -0.80320753, -0.81045720, -0.81758481, -0.82458930,
	-0.83146961, -0.83822471, -0.84485357, -0.85135519, -0.85772861, -0.86397286, -0.87008699, -0.87607009,
	-0.88192126, -0.88763962, -0.89322430, -0.89867447, -0.90398929, -0.90916798, -0.91420976, -0.91911385,
	-0.92387953, -0.92850608, -0.93299280, -0.93733901, -0.94154407, -0.94560733, -0.94952818, -0.95330604,
	-0.95694034, -0.96043052, -0.96377607, -0.96697647, -0.97003125, -0.97293995, -0.97570213, -0.97831737,
	-0.98078528, -0.98310549, -0.98527764, -0.98730142, -0.98917651, -0.99090264, -0.99247953, -0.99390697,
	-0.99518473, -0.99631261, -0.99729046, -0.99811811, -0.99879546, -0.99932238, -0.99969882, -0.99992470*/
};
// full size table:
//   sin = sin_table_512[i    ]
//   cos = sin_table_512[i+128]
//#define FFT_SIN(i) sin_table_512[(i)    ]
//#define FFT_COS(i) sin_table_512[(i)+128]

// for size use only 0-128 indexes
//   sin = i > 128 ? sin_table_512[256-i] : sin_table_512[    i];
//   cos = i > 128 ?-sin_table_512[i-128] : sin_table_512[128-i];
#define FFT_SIN(i) ((i) > 128 ? sin_table_512[256-(i)] : sin_table_512[    (i)])
#define FFT_COS(i) ((i) > 128 ?-sin_table_512[(i)-128] : sin_table_512[128-(i)])

#else
#error "Need build table for new FFT size"
#endif

#else
// Not use FFT_USE_SIN_COS_TABLE, use direct sin/cos calculations
#define FFT_SIN(k) sinf((2 * VNA_PI / FFT_SIZE) * (k))
#define FFT_COS(k) cosf((2 * VNA_PI / FFT_SIZE) * (k));

#endif // FFT_USE_SIN_COS_TABLE

#endif // __VNA_USE_MATH_TABLES__

#ifdef ARM_MATH_CM4
// Use CORTEX M4 rbit instruction (reverse bit order in 32bit value)
static uint32_t reverse_bits(uint32_t x, int n) {
	uint32_t result;
	 __asm volatile ("rbit %0, %1" : "=r" (result) : "r" (x) );
	return result>>(32-n); // made shift for correct result
}
#else
static uint16_t reverse_bits(uint16_t x, int n) {
  uint16_t result = 0;
  int i;
  for (i = 0; i < n; i++, x >>= 1)
    result = (result << 1) | (x & 1U);
  return result;
}
#endif

/***
 * dir = forward: 0, inverse: 1
 * https://www.nayuki.io/res/free-small-fft-in-multiple-languages/fft.c
 */
void fft(float array[][2], const uint8_t dir) {
// FFT_SIZE = 2^FFT_N
#if   FFT_SIZE == 256
 #define FFT_N     8
#elif FFT_SIZE == 512
 #define FFT_N     9
#else
 #error "Need define FFT_N for this FFT size"
#endif
  const uint16_t n = FFT_SIZE;
  const uint8_t levels = FFT_N; // log2(n)
  uint16_t i, j;
  for (i = 0; i < n; i++) {
    if ((j = reverse_bits(i, levels)) > i) {
      SWAP(float, array[i][0], array[j][0]);
      SWAP(float, array[i][1], array[j][1]);
    }
  }
  const uint16_t size = 2;
  uint16_t halfsize = size / 2;
  uint16_t tablestep = n / size;
  // Cooley-Tukey decimation-in-time radix-2 FFT
  for (;tablestep; tablestep>>=1, halfsize<<=1) {
    for (i = 0; i < n; i+= halfsize) {
      for (j = 0; j < n / size; i++, j+= tablestep) {
        const uint16_t l = i + halfsize;
        const float s = dir ? FFT_SIN(j) : -FFT_SIN(j);
        const float c = FFT_COS(j);
        const float tpre = array[l][0] * c - array[l][1] * s;
        const float tpim = array[l][0] * s + array[l][1] * c;
        array[l][0] = array[i][0] - tpre; array[i][0]+= tpre;
        array[l][1] = array[i][1] - tpim; array[i][1]+= tpim;
      }
    }
  }
}

// Return sin/cos value angle in range 0.0 to 1.0 (0 is 0 degree, 1 is 360 degree)
void vna_sincosf(float angle, float * pSinVal, float * pCosVal)
{
#ifndef __VNA_USE_MATH_TABLES__
  // Use default sin/cos functions
  angle *= 2 * VNA_PI;   // Convert to rad
  *pSinVal = sinf(angle);
  *pCosVal = cosf(angle);
#else
  uint16_t indexS, indexC;  // Index variable
  float f1, f2, d1, d2;     // Two nearest output values
  float fract, temp;

  // Round angle to range 0.0 to 1.0
  temp = vna_fabsf(angle);
  temp-= (uint32_t)temp;

  // Scale input from range 0.0 to 1.0 to table size
  temp*= FAST_MATH_TABLE_SIZE;

  indexS = temp;
  indexC = indexS + (FAST_MATH_TABLE_SIZE / 4); // cosine add 0.25 (pi/2) to read from sine table
  // Calculation of fractional value
  fract  = temp - indexS;
  // Align indexes to table
  indexS&= (FAST_MATH_TABLE_SIZE-1);
  indexC&= (FAST_MATH_TABLE_SIZE-1);

  // Read two nearest values of input value from the cos & sin tables
#if 0
  f1 = GET_SIN_TABLE(indexC  );
  f2 = GET_SIN_TABLE(indexC+1);
  d1 = GET_SIN_TABLE(indexS  );
  d2 = GET_SIN_TABLE(indexS+1);
#else
  if (indexC < 256){f1 = sin_table_512[indexC    +0];f2 = sin_table_512[indexC    +1];}
  else             {f1 =-sin_table_512[indexC-256+0];f2 =-sin_table_512[indexC-256+1];}
  if (indexS < 256){d1 = sin_table_512[indexS    +0];d2 = sin_table_512[indexS    +1];}
  else             {d1 =-sin_table_512[indexS-256+0];d2 =-sin_table_512[indexS-256+1];}
#endif

#if 1
  // 1e-7 error on 512 size table
  const float Dn = 2 * VNA_PI / FAST_MATH_TABLE_SIZE; // delta between the two points in table (fixed);
  float Df;
  // Calculation of cos value
  Df = f2 - f1; // delta between the values of the functions
  temp = Dn * (d1 + d2) + 2 * Df;
  temp = Df + (d1 * Dn + temp - fract * temp);
  temp = fract * temp - d1 * Dn;
  *pCosVal = f1 + fract * temp;
  // Calculation of sin value
  Df = d1 - d2; // delta between the values of the functions
  temp = Dn * (f1 + f2) + 2 * Df;
  temp = Df + (f1 * Dn + temp - fract * temp);
  temp = fract * temp - f1 * Dn;
  *pSinVal = d1 - fract * temp;
#else
  // 1e-5 error  on 512 size table
  // Calculation of sin and cos value, use simple linear interpolation
  *pCosVal = fract * (f2 - f1) + f1;
  *pSinVal = fract * (d2 - d1) + d1;
#endif
  if (angle < 0)
    *pSinVal = -*pSinVal;
#endif
}

//**********************************************************************************
//      VNA math
//**********************************************************************************
// Cleanup declarations if used default math.h functions
#undef vna_sqrtf
#undef vna_cbrtf
#undef vna_logf
#undef vna_atanf
#undef vna_atan2f
#undef vna_modff

//**********************************************************************************
// modff function - return fractional part and integer from float value x
//**********************************************************************************
float vna_modff(float x, float *iptr)
{
  union {float f; uint32_t i;} u = {x};
  int e = (int)((u.i>>23)&0xff) - 0x7f; // get exponent
  if (e <   0) {                        // no integral part
    if (iptr) *iptr = 0;
    return u.f;
  }
  if (e >= 23) x = 0;                   // no fractional part
  else {
    x = u.f; u.i&= ~(0x007fffff>>e);    // remove fractional part from u
    x-= u.f;                            // calc fractional part
  }
//if (iptr) *iptr = ((u.i&0x007fffff)|0x00800000)>>(23-e); // cut integer part from float as integer
  if (iptr) *iptr = u.f;                // cut integer part from float as float
  return x;
}

//**********************************************************************************
// square root
//**********************************************************************************
#if (__FPU_PRESENT == 0) && (__FPU_USED == 0)
#if 1
// __ieee754_sqrtf, remove some check (NAN, inf, normalization), small code optimization to arm
float vna_sqrtf(float x)
{
  int32_t ix,s,q,m,t;
  uint32_t r;
  union {float f; uint32_t i;} u = {x};
  ix = u.i;
#if 0
  // take care of Inf and NaN
  if((ix&0x7f800000)==0x7f800000) return x*x+x;	// sqrt(NaN)=NaN, sqrt(+inf)=+inf, sqrt(-inf)=sNaN
  // take care if x < 0
  if (ix <  0) return (x-x)/0.0f;
#endif
  if (ix == 0) return 0.0f;
  m = (ix>>23);
#if 0 //
  // normalize x
  if(m==0) {				// subnormal x
    for(int i=0;(ix&0x00800000)==0;i++) ix<<=1;
      m -= i-1;
  }
#endif
  m -= 127;	// unbias exponent
  ix = (ix&0x007fffff)|0x00800000;
  // generate sqrt(x) bit by bit
  ix<<= (m&1) ? 2 : 1;	// odd m, double x to make it even, and after multiple by 2
  m >>= 1;	       		// m = [m/2]
  q = s = 0;			// q = sqrt(x)
  r = 0x01000000;		// r = moving bit from right to left
  while(r!=0) {
    t = s+r;
    if(t<=ix) {
      s    = t+r;
      ix  -= t;
      q   += r;
    }
    ix += ix;
    r>>=1;
  }
  // use floating add to find out rounding direction
  if(ix!=0) {
	if ((1.0f - 1e-30f) >= 1.0f) // trigger inexact flag.
	  q += ((1.0f + 1e-30f) > 1.0f) ? 2 : (q&1);
  }
  ix = (q>>1)+0x3f000000;
  ix += (m <<23);
  u.i = ix;
  return u.f;
}
#else
// Simple implementation, but slow if no FPU used, and not usable if used hardware FPU sqrtf
float vna_sqrtf(float x)
{
  union {float x; uint32_t i;} u = {x};
  u.i = (1<<29) + (u.i >> 1) - (1<<22);
  // Two Babylonian Steps (simplified from:)
  // u.x = 0.5f * (u.x + x/u.x);
  // u.x = 0.5f * (u.x + x/u.x);
  u.x =       u.x + x/u.x;
  u.x = 0.25f*u.x + x/u.x;

  return u.x;
}
#endif
#endif

//**********************************************************************************
// Cube root
//**********************************************************************************
float vna_cbrtf(float x)
{
#if 1
  static const uint32_t
  B1 = 709958130, // B1 = (127-127.0/3-0.03306235651)*2**23
  B2 = 642849266; // B2 = (127-127.0/3-24/3-0.03306235651)*2**23

  float r,T;
  union {float f; uint32_t i;} u = {x};
  uint32_t hx = u.i & 0x7fffffff;

//	if (hx >= 0x7f800000)  // cbrt(NaN,INF) is itself
//		return x + x;
  // rough cbrtf to 5 bits
  if (hx < 0x00800000) {  // zero or subnormal?
    if (hx == 0)
      return x;  // cbrt(+-0) is itself
    u.f = x*0x1p24f;
    hx = u.i & 0x7fffffff;
    hx = hx/3 + B2;
  } else
    hx = hx/3 + B1;
  u.i &= 0x80000000;
  u.i |= hx;

  // First step Newton iteration (solving t*t-x/t == 0) to 16 bits.
  T = u.f;
  r = T*T*T;
  T*= (x+x+r)/(x+r+r);
// Second step Newton iteration to 47 bits.
  r = T*T*T;
  T*= (x+x+r)/(x+r+r);
  return T;
#else
  if (x == 0) {
    // would otherwise return something like 4.257959840008151e-109
    return 0;
  }
  float b = 1.0f; // use any value except 0
  float last_b_1 = 0;
  float last_b_2 = 0;
  while (last_b_1 != b && last_b_2 != b) {
    last_b_1 = b;
//    b = (b + x / (b * b)) / 2;
    b = (2 * b + x / b / b) / 3; // for small numbers, as suggested by  willywonka_dailyblah
    last_b_2 = b;
//    b = (b + x / (b * b)) / 2;
    b = (2 * b + x / b / b) / 3; //for small numbers, as suggested by  willywonka_dailyblah
  }
  return b;
#endif
}

//**********************************************************************************
// logf
//**********************************************************************************
float vna_logf(float x)
{
  const float MULTIPLIER = logf(2.0f);
#if 0
  // Give up to 0.006 error (2.5x faster original code)
  union {float f; int32_t i;} u = {x};
  const int      log_2 = ((u.i >> 23) & 255) - 128;
  if (u.i <=0) return -1/(x*x);                 // if <=0 return -inf
  u.i = (u.i&0x007FFFFF) + 0x3F800000;
  u.f = ((-1.0f/3) * u.f + 2) * u.f - (2.0f/3); // (1)
  return (u.f + log_2) * MULTIPLIER;
#elif 1
  // Give up to 0.00005 error (2x faster original code)
  // fast log2f approximation, give 0.0002 error
  union { float f; uint32_t i; } vx = { x };
  union { uint32_t i; float f; } mx = { (vx.i & 0x007FFFFF) | 0x3f000000 };
  // if <=0 return NAN
  if (vx.i <=0) return -1/(x*x);
  return vx.i * (MULTIPLIER / (1 << 23)) - (124.22544637f * MULTIPLIER) - (1.498030302f * MULTIPLIER) * mx.f - (1.72587999f * MULTIPLIER) / (0.3520887068f + mx.f);
#else
  // use original code (20% faster default)
  static const float
  ln2_hi = 6.9313812256e-01, /* 0x3f317180 */
  ln2_lo = 9.0580006145e-06, /* 0x3717f7d1 */
  two25  = 3.355443200e+07,  /* 0x4c000000 */
  /* |(log(1+s)-log(1-s))/s - Lg(s)| < 2**-34.24 (~[-4.95e-11, 4.97e-11]). */
  Lg1 = 0xaaaaaa.0p-24, /* 0.66666662693 */
  Lg2 = 0xccce13.0p-25, /* 0.40000972152 */
  Lg3 = 0x91e9ee.0p-25, /* 0.28498786688 */
  Lg4 = 0xf89e26.0p-26; /* 0.24279078841 */

  union {float f; uint32_t i;} u = {x};
  float hfsq,f,s,z,R,w,t1,t2,dk;
  uint32_t ix;
  int k;

  ix = u.i;
  k = 0;
  if (ix < 0x00800000 || ix>>31) {  /* x < 2**-126  */
    if (ix<<1 == 0)
      return -1/(x*x);  /* log(+-0)=-inf */
    if (ix>>31)
      return (x-x)/0.0f; /* log(-#) = NaN */
    /* subnormal number, scale up x */
    k -= 25;
    x *= two25;
    u.f = x;
    ix = u.i;
  } else if (ix >= 0x7f800000) {
    return x;
  } else if (ix == 0x3f800000)
    return 0;
   /* reduce x into [sqrt(2)/2, sqrt(2)] */
  ix += 0x3f800000 - 0x3f3504f3;
  k += (int)(ix>>23) - 0x7f;
  ix = (ix&0x007fffff) + 0x3f3504f3;
  u.i = ix;
  x = u.f;
  f = x - 1.0f;
  s = f/(2.0f + f);
  z = s*s;
  w = z*z;
  t1= w*(Lg2+w*Lg4);
  t2= z*(Lg1+w*Lg3);
  R = t2 + t1;
  hfsq = 0.5f * f * f;
  dk = k;
  return s*(hfsq+R) + dk*ln2_lo - hfsq + f + dk*ln2_hi;
#endif
}

float vna_log10f_x_10(float x)
{
  const float MULTIPLIER = (10.0f * logf(2.0f) / logf(10.0f));
#if 0
  // Give up to 0.006 error (2.5x faster original code)
  union {float f; int32_t i;} u = {x};
  const int      log_2 = ((u.i >> 23) & 255) - 128;
  if (u.i <=0) return -1/(x*x);                 // if <=0 return -inf
  u.i = (u.i&0x007FFFFF) + 0x3F800000;
  u.f = ((-1.0f/3) * u.f + 2) * u.f - (2.0f/3); // (1)
  return (u.f + log_2) * MULTIPLIER;
#else
  // Give up to 0.0001 error (2x faster original code)
  // fast log2f approximation, give 0.0004 error
  union { float f; uint32_t i; } vx = { x };
  union { uint32_t i; float f; } mx = { (vx.i & 0x007FFFFF) | 0x3f000000 };
  // if <=0 return NAN
  if (vx.i <=0) return -1/(x*x);
  return vx.i * (MULTIPLIER / (1 << 23)) - (124.22544637f * MULTIPLIER) - (1.498030302f * MULTIPLIER) * mx.f - (1.72587999f * MULTIPLIER) / (0.3520887068f + mx.f);
#endif
}
//**********************************************************************************
// atanf
//**********************************************************************************
// __ieee754_atanf
float vna_atanf(float x)
{
  static const float atanhi[] = {
    4.6364760399e-01, // atan(0.5)hi 0x3eed6338
    7.8539812565e-01, // atan(1.0)hi 0x3f490fda
    9.8279368877e-01, // atan(1.5)hi 0x3f7b985e
    1.5707962513e+00, // atan(inf)hi 0x3fc90fda
  };
  static const float atanlo[] = {
    5.0121582440e-09, // atan(0.5)lo 0x31ac3769
    3.7748947079e-08, // atan(1.0)lo 0x33222168
    3.4473217170e-08, // atan(1.5)lo 0x33140fb4
    7.5497894159e-08, // atan(inf)lo 0x33a22168
  };
  static const float aT[] = {
    3.3333328366e-01,
   -1.9999158382e-01,
    1.4253635705e-01,
   -1.0648017377e-01,
    6.1687607318e-02,
  };
  float w,s1,s2,z;
  uint32_t ix,sign;
  int id;
  union {float f; uint32_t i;} u = {x};
  ix = u.i;
  sign = ix>>31;
  ix &= 0x7fffffff;
  if (ix >= 0x4c800000) {  /* if |x| >= 2**26 */
    if (ix > 0x7f800000)
      return x;
    z = atanhi[3] + 0x1p-120f;
    return sign ? -z : z;
  }
  if (ix < 0x3ee00000) {   /* |x| < 0.4375 */
    if (ix < 0x39800000) {  /* |x| < 2**-12 */
      return x;
    }
    id = -1;
  } else {
    x = vna_fabsf(x);
    if (ix < 0x3f980000) {  /* |x| < 1.1875 */
      if (ix < 0x3f300000) {  /*  7/16 <= |x| < 11/16 */
        id = 0;
        x = (2.0f*x - 1.0f)/(2.0f + x);
      } else {                /* 11/16 <= |x| < 19/16 */
        id = 1;
        x = (x - 1.0f)/(x + 1.0f);
      }
    } else {
      if (ix < 0x401c0000) {  /* |x| < 2.4375 */
        id = 2;
        x = (x - 1.5f)/(1.0f + 1.5f*x);
      } else {                /* 2.4375 <= |x| < 2**26 */
        id = 3;
        x = -1.0f/x;
      }
    }
  }
  /* end of argument reduction */
  z = x*x;
  w = z*z;
  /* break sum from i=0 to 10 aT[i]z**(i+1) into odd and even poly */
  s1 = z*(aT[0]+w*(aT[2]+w*aT[4]));
  s2 = w*(aT[1]+w*aT[3]);
  if (id < 0)
    return x - x*(s1+s2);
  z = atanhi[id] - ((x*(s1+s2) - atanlo[id]) - x);
  return sign ? -z : z;
}

//**********************************************************************************
// atan2f
//**********************************************************************************
#if 0
// __ieee754_atan2f
float vna_atan2f(float y, float x)
{
  static const float pi    = 3.1415927410e+00; // 0x40490fdb
  static const float pi_lo =-8.7422776573e-08; // 0xb3bbbd2e
  float z;
  uint32_t m,ix,iy;
  union {float f; uint32_t i;} ux = {x};
  union {float f; uint32_t i;} uy = {y};
  ix = ux.i;
  iy = uy.i;

  if (ix == 0x3f800000)  /* x=1.0 */
    return vna_atanf(y);
  m = ((iy>>31)&1) | ((ix>>30)&2);  /* 2*sign(x)+sign(y) */
  ix &= 0x7fffffff;
  iy &= 0x7fffffff;

  /* when y = 0 */
  if (iy == 0) {
	switch (m) {
      case 0:
      case 1: return   y; // atan(+-0,+anything)=+-0
      case 2: return  pi; // atan(+0,-anything) = pi
      case 3: return -pi; // atan(-0,-anything) =-pi
    }
  }
  /* when x = 0 */
  if (ix == 0)
    return m&1 ? -pi/2 : pi/2;
  /* when x is INF */
  if (ix == 0x7f800000) {
    if (iy == 0x7f800000) {
      switch (m) {
        case 0: return  pi/4; /* atan(+INF,+INF) */
        case 1: return -pi/4; /* atan(-INF,+INF) */
        case 2: return 3*pi/4;  /*atan(+INF,-INF)*/
        case 3: return -3*pi/4; /*atan(-INF,-INF)*/
      }
    } else {
      switch (m) {
        case 0: return  0.0f;    /* atan(+...,+INF) */
        case 1: return -0.0f;    /* atan(-...,+INF) */
        case 2: return  pi; /* atan(+...,-INF) */
        case 3: return -pi; /* atan(-...,-INF) */
      }
    }
  }
  /* |y/x| > 0x1p26 */
  if (ix+(26<<23) < iy || iy == 0x7f800000)
    return m&1 ? -pi/2 : pi/2;

  /* z = atan(|y/x|) with correct underflow */
  if ((m&2) && iy+(26<<23) < ix)  /*|y/x| < 0x1p-26, x < 0 */
    z = 0.0;
  else
    z = vna_atanf(vna_fabsf(y/x));
  switch (m) {
    case 0: return z;              /* atan(+,+) */
    case 1: return -z;             /* atan(-,+) */
    case 2: return pi - (z-pi_lo); /* atan(+,-) */
    default: /* case 3 */
      return (z-pi_lo) - pi; /* atan(-,-) */
  }
}
#else
// Polynomial approximation to atan2f
float vna_atan2f(float y, float x)
{
  union {float f; int32_t i;} ux = {x};
  union {float f; int32_t i;} uy = {y};
  if (ux.i == 0 && uy.i == 0)
    return 0.0f;

  float ax, ay, r, s;
  ax = vna_fabsf(x);
  ay = vna_fabsf(y);
  r = (ay < ax) ? ay / ax : ax / ay;
  s = r * r;
  // Polynomial approximation to atan(a) on [0,1]
#if 0
  // give 0.31 degree error
  r*= 0.970562748477141f - 0.189514164974601f * s;
  //r*= vna_fmaf(-s, 0.189514164974601f, 0.970562748477141f);
#elif 0
  // give 0.04 degree error
  r*= 0.994949366116654f - s * (0.287060635532652f - 0.078037176446441f * s);
  //r*= vna_fmaf(-s, vna_fmaf(-s, 0.078037176446441f, 0.287060635532652f), 0.994949366116654f);
  //r*= 0.995354f − s * (0.288679f + 0.079331f * s);
#else
  // give 0.005 degree error
  r*= 0.999133448222780f - s * (0.320533292381664f - s * (0.144982490144465f - s * 0.038254464970299f));
  //r*= vna_fmaf(-s, vna_fmaf(-s, vna_fmaf(-s, 0.038254464970299f, 0.144982490144465f), 0.320533292381664f), 0.999133448222780f);
#endif
  // Map to full circle
  if (ay  > ax) r = VNA_PI/2.0f - r;
  if (ux.i < 0) r = VNA_PI      - r;
  if (uy.i < 0) r = -r;
  return r;
}
#endif

//**********************************************************************************
// Fast expf approximation
//**********************************************************************************
float vna_expf(float x)
{
  union { float f; int32_t i; } v;
  v.i = (int32_t)(12102203.0f*x) + 0x3F800000;
  int32_t m = (v.i >> 7) & 0xFFFF;  // copy mantissa
#if 1
  // cubic spline approximation, empirical values for small maximum relative error (8.34e-5):
  v.i += ((((((((1277*m) >> 14) + 14825)*m) >> 14) - 79749)*m) >> 11) - 626;
#else
  // quartic spline approximation, empirical values for small maximum relative error (1.21e-5):
  v.i += (((((((((((3537*m) >> 16) + 13668)*m) >> 18) + 15817)*m) >> 14) - 80470)*m) >> 11);
#endif
  return v.f;
}
