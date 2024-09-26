/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_board_spiflash.c
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

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/spi/spi.h>

#include "esp32_spiflash.h"
#include "esp32_board_spiflash.h"

#define CONFIG_ESP32_SPIFLASH_SPIFFS


/****************************************************************************
 * Name: setup_spiffs
 *
 * Description:
 *   Register a mtd driver and mount a SPIFFS over it.
 *
 * Parameters:
 *   path   - Path name used to register the mtd driver.
 *   mtd    - Pointer to a pre-allocated mtd partition.
 *   mnt_pt - Mount point
 *   priv   - Privileges
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESP32_SPIFLASH_SPIFFS
static int setup_spiffs(const char *path, struct mtd_dev_s *mtd,
                        const char *mnt_pt, int priv)
{
  int ret = OK;

  ret = register_mtddriver(path, mtd, priv, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to register MTD: %d\n", ret);
      return -ENOMEM;
    }

  if (mnt_pt != NULL)
    {
      ret = nx_mount(path, mnt_pt, "spiffs", 0, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to mount the FS volume: %d\n", ret);
          return ret;
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: init_storage_partition
 *
 * Description:
 *   Initialize partition that is dedicated to general use.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int init_storage_partition(void)
{
  int ret = OK;
  struct mtd_dev_s *mtd;

  mtd = esp32_spiflash_alloc_mtdpart(CONFIG_ESP32_STORAGE_MTD_OFFSET,
                                     CONFIG_ESP32_STORAGE_MTD_SIZE,
                                     false);
  if (!mtd)
    {
      ferr("ERROR: Failed to alloc MTD partition of SPI Flash\n");
      return -ENOMEM;
    }

  const char *path = "/dev/esp32flash";
  ret = setup_spiffs(path, mtd, NULL, 0755);
  if (ret < 0)
    {
      ferr("ERROR: Failed to setup spiffs\n");
      return ret;
    }

  return ret;
}

#define CONFIG_ESP32_PARAM_MTD_OFFSET   0x310000
#define CONFIG_ESP32_PARAM_MTD_SIZE     0x50000

static int init_param_partition(void)
{
  int ret = OK;
  struct mtd_dev_s *mtd;

  mtd = esp32_spiflash_alloc_mtdpart(CONFIG_ESP32_PARAM_MTD_OFFSET,
                                     CONFIG_ESP32_PARAM_MTD_SIZE,
                                     false);
  if (!mtd)
    {
      ferr("ERROR: Failed to alloc PARAM MTD partition of SPI Flash\n");
      return -ENOMEM;
    }

  ret = register_mtddriver("/fs/mtd_params", mtd, 0777, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to register PARAM MTD: %d\n", ret);
      return ret;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_spiflash_init
 *
 * Description:
 *   Initialize the SPI Flash and register the MTD.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int esp32_spiflash_init(void)
{
  int ret = OK;


  ret = init_storage_partition();
  if (ret < 0)
    {
      return ret;
    }

  ret = init_param_partition();
  if (ret < 0)
    {
      return ret;
    }


  return ret;
}
