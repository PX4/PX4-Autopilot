/****************************************************************************
 * boards/arm/rp23xx/common/src/rp23xx_uniqueid.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <assert.h>
#include <nuttx/config.h>

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "pico.h"
#include "rp23xx_uniqueid.h"
#include "rp23xx_rom.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SYS_INFO_CHIP_INFO                      0x0001

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef int (*rom_get_sys_info_fn)
  (uint32_t *out_buffer, uint32_t out_buffer_word_size, uint32_t flags);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_uniqueid[CONFIG_BOARDCTL_UNIQUEID_SIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_uniqueid_initialize
 *
 * Description:
 *   The RP23XX doesn't have a unique ID, so we load the ID from the
 *   connected flash chip.  We use the flash ID to seed a simple xorshift
 *   PRNG.  The PRNG then generates CONFIG_BOARDCTL_UNIQUEID_SIZE bytes,
 *   which we will use as the board's unique ID.
 *
 *   Retrieving the flash id is somewhat slow and complex, so we only do
 *   this during initialization and store the result for later use.
 *
 * Assumptions/Limitations:
 *   This uniqueid implementation requires a flash chip.  It should not be
 *   used on boards without flash.
 *
 ****************************************************************************/

void rp23xx_uniqueid_initialize(void)
{
  uint64_t x;

  rom_get_sys_info_fn func = (rom_get_sys_info_fn)
    rom_func_lookup(ROM_FUNC_GET_SYS_INFO);

  union
  {
    uint32_t words[9];
    uint8_t bytes[9 * 4];
  } out;

  memset(out.bytes, 0x00, 9 * 4);

  int rc = func(out.words, 9, SYS_INFO_CHIP_INFO);

  if (rc != 4)
    {
      PANIC();
    }

  /* xorshift PRNG: */

  x = *(uint64_t *)(out.bytes);
  for (int i = 0; i < CONFIG_BOARDCTL_UNIQUEID_SIZE; i++)
    {
      x ^= x >> 12;
      x ^= x << 25;
      x ^= x >> 27;
      g_uniqueid[i] = (uint8_t)((x * 0x2545f4914f6cdd1dull) >> 32);
    }
}

/****************************************************************************
 * Name: board_uniqueid
 *
 * Description:
 *   Return a unique ID associated with the board.
 *
 * Input Parameters:
 *   uniqueid - A reference to a writable memory location provided by the
 *     caller to receive the board unique ID.  The memory memory referenced
 *     by this pointer must be at least CONFIG_BOARDCTL_UNIQUEID_SIZE in
 *     length.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwize a negated errno value is
 *   returned indicating the nature of the failure.
 *
 ****************************************************************************/

int board_uniqueid(FAR uint8_t *uniqueid)
{
  memcpy(uniqueid, g_uniqueid, CONFIG_BOARDCTL_UNIQUEID_SIZE);
  return OK;
}
