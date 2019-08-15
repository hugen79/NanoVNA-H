/*
 * Copyright (c) 2016-2017, TAKAHASHI Tomohiro (TTRFTECH) edy555@gmail.com
 * All rights reserved.
 *
 *"2016-2019 Copyright @edy555, licensed under GPL. https://github.com/ttrftech/NanoVNA"
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
#include "usbcfg.h"
#include "si5351.h"
#include "nanovna.h"

#include <chprintf.h>
#include <shell.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#define ENABLED_DUMP

static void apply_error_term_at(int i);
static void cal_interpolate(int s);

void sweep(void);

static MUTEX_DECL(mutex);

int32_t frequency_offset = 5000;
int32_t frequency = 10000000;
int8_t drive_strength = DRIVE_STRENGTH_AUTO;
int8_t frequency_updated = FALSE;
int8_t sweep_enabled = TRUE;
int8_t cal_auto_interpolate = TRUE;
int8_t redraw_requested = FALSE;

static THD_WORKING_AREA(waThread1, 768);
static THD_FUNCTION(Thread1, arg)
{
    (void)arg;
    chRegSetThreadName("sweep");

    while (1) {
      if (sweep_enabled) {
        chMtxLock(&mutex);
        sweep();
        chMtxUnlock(&mutex);
      } else {
        __WFI();
        ui_process();
      }

      /* calculate trace coordinates */
      plot_into_index(measured);
      /* plot trace as raster */
      draw_all_cells();
    }
}

void
pause_sweep(void)
{
  sweep_enabled = FALSE;
}

void
resume_sweep(void)
{
  sweep_enabled = TRUE;
}

void
toggle_sweep(void)
{
  sweep_enabled = !sweep_enabled;
}

static void cmd_pause(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)chp;
    (void)argc;
    (void)argv;
    pause_sweep();
}

static void cmd_resume(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)chp;
    (void)argc;
    (void)argv;
    resume_sweep();
}

static void cmd_reset(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void)argc;
    (void)argv;

    chprintf(chp, "Performing reset\r\n");

    rccEnableWWDG(FALSE);

    WWDG->CFR = 0x60;
    WWDG->CR = 0xff;

    /* wait forever */
    while (1)
      ;
}

int set_frequency(int freq)
{
    int delay = 0;
    if (frequency == freq)
          return delay;

        if (freq > FREQ_HARMONICS && frequency <= FREQ_HARMONICS) {
          tlv320aic3204_set_gain(40, 38);
          delay += 3;
        }
        if (freq <= FREQ_HARMONICS && frequency > FREQ_HARMONICS) {
          tlv320aic3204_set_gain(5, 5);
          delay += 3;
        }

        int8_t ds = drive_strength;
        if (ds == DRIVE_STRENGTH_AUTO) {
          ds = freq > FREQ_HARMONICS ? SI5351_CLK_DRIVE_STRENGTH_8MA : SI5351_CLK_DRIVE_STRENGTH_2MA;
        }
        delay += si5351_set_frequency_with_offset(freq, frequency_offset, ds);

        frequency = freq;
    return delay;
}

static void cmd_offset(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 1) {
        chprintf(chp, "usage: offset {frequency offset(Hz)}\r\n");
        return;
    }
    frequency_offset = atoi(argv[0]);
    set_frequency(frequency);
}

static void cmd_freq(BaseSequentialStream *chp, int argc, char *argv[])
{
    int freq;
    if (argc != 1) {
        chprintf(chp, "usage: freq {frequency(Hz)}\r\n");
        return;
    }
    pause_sweep();
    chMtxLock(&mutex);
    freq = atoi(argv[0]);
    set_frequency(freq);
    chMtxUnlock(&mutex);
}

static void cmd_power(BaseSequentialStream *chp, int argc, char *argv[])
{
    if (argc != 1) {
    	chprintf(chp, "usage: power {0-3|-1}\r\n");
        return;
    }
    drive_strength = atoi(argv[0]);
    set_frequency(frequency);
}

static void cmd_time(BaseSequentialStream *chp, int argc, char *argv[])
{
    RTCDateTime timespec;
    (void)argc;
    (void)argv;
    rtcGetTime(&RTCD1, &timespec);
    chprintf(chp, "%d/%d/%d %d\r\n", timespec.year+1980, timespec.month, timespec.day, timespec.millisecond);
}


static void cmd_da
