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
#include "nanovna.h"
#include <string.h>

uint16_t lastsaveid = 0;
#if SAVEAREA_MAX >= 8
#error "Increase checksum_ok type for save more cache slots"
#endif
// properties CRC check cache (max 8 slots)
static uint8_t checksum_ok = 0;

static uint32_t
checksum(const void *start, size_t len)
{
  uint32_t *p = (uint32_t*)start;
  uint32_t value = 0;
  // align by sizeof(uint32_t)
  len = (len + sizeof(uint32_t)-1)/sizeof(uint32_t);
  while (len-- > 0)
    value = __ROR(value, 31) + *p++;
  return value;
}

int
config_save(void)
{
  // Apply magic word and calculate checksum
  config.magic = CONFIG_MAGIC;
  config.checksum = checksum(&config, sizeof config - sizeof config.checksum);

  // write to flash
  flash_program_half_word_buffer((uint16_t*)SAVE_CONFIG_ADDR, (uint16_t*)&config, sizeof(config_t));
  return 0;
}

int
config_recall(void)
{
  const config_t *src = (const config_t*)SAVE_CONFIG_ADDR;

  if (src->magic != CONFIG_MAGIC || checksum(src, sizeof *src - sizeof src->checksum) != src->checksum)
    return -1;
  // duplicated saved data onto sram to be able to modify marker/trace
  memcpy(&config, src, sizeof(config_t));
  return 0;
}

int
caldata_save(uint32_t id)
{
  if (id >= SAVEAREA_MAX)
    return -1;

  // Apply magic word and calculate checksum
  current_props.magic = PROPS_MAGIC;
  current_props.checksum = checksum(&current_props, sizeof current_props - sizeof current_props.checksum);

  // write to flash
  uint16_t *dst = (uint16_t*)(SAVE_PROP_CONFIG_ADDR + id * SAVE_PROP_CONFIG_SIZE);
  flash_program_half_word_buffer(dst, (uint16_t*)&current_props, sizeof(properties_t));

  lastsaveid = id;
  return 0;
}

const properties_t *
get_properties(uint32_t id)
{
  if (id >= SAVEAREA_MAX)
    return NULL;
  // point to saved area on the flash memory
  properties_t *src = (properties_t*)(SAVE_PROP_CONFIG_ADDR + id * SAVE_PROP_CONFIG_SIZE);
  // Check crc cache mask (made it only 1 time)
  if (checksum_ok&(1<<id))
    return src;
  if (src->magic != PROPS_MAGIC || checksum(src, sizeof *src - sizeof src->checksum) != src->checksum)
    return NULL;
  checksum_ok|=1<<id;
  return src;
}

int
caldata_recall(uint32_t id)
{
  lastsaveid = NO_SAVE_SLOT;
  if (id == NO_SAVE_SLOT) return 0;
  // point to saved area on the flash memory
  const properties_t *src = get_properties(id);
  if (src == NULL){
    load_default_properties();
    return 1;
  }
  // active configuration points to save data on flash memory
  lastsaveid = id;
  // duplicated saved data onto sram to be able to modify marker/trace
  memcpy(&current_props, src, sizeof(properties_t));
  return 0;
}

// Used in interpolate, get current calibration slot data
const properties_t *
caldata_reference(void)
{
  return get_properties(lastsaveid);
}

void
clear_all_config_prop_data(void)
{
  lastsaveid = NO_SAVE_SLOT;
  checksum_ok = 0;
  // unlock and erase flash pages
  flash_erase_pages(SAVE_CONFIG_ADDR, SAVE_FULL_AREA_SIZE);
}

