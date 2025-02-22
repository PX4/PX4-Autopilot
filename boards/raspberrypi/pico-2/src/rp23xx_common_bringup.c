/****************************************************************************
 * boards/arm/rp23xx/common/src/rp23xx_common_bringup.c
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

#include <debug.h>
#include <stddef.h>
#include <string.h>
#include <sys/stat.h>

#include <nuttx/fs/fs.h>

#include <arch/board/board.h>

#include "rp23xx_pico.h"
#include "rp23xx_common_bringup.h"

#ifdef CONFIG_RP23XX_PWM
#include "rp23xx_pwm.h"
#include "rp23xx_pwmdev.h"
#endif

#if defined(CONFIG_ADC) && defined(CONFIG_RP23XX_ADC)
#include "rp23xx_adc.h"
#endif

#ifdef CONFIG_WATCHDOG
#  include "rp23xx_wdt.h"
#endif

#if defined(CONFIG_RP23XX_ROMFS_ROMDISK_DEVNAME)
#  include <rp23xx_romfsimg.h>
#endif

#if defined(CONFIG_RP23XX_BOARD_HAS_WS2812) && defined(CONFIG_WS2812)
#  include "rp23xx_ws2812.h"
#ifdef CONFIG_WS2812_HAS_WHITE
#  define HAS_WHITE true
#else /* CONFIG_WS2812_HAS_WHITE */
#  define HAS_WHITE false
#endif /* CONFIG_WS2812_HAS_WHITE */
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_common_bringup
 ****************************************************************************/

int rp23xx_common_bringup(void)
{
  int ret = 0;

#ifdef CONFIG_RP23XX_I2C_DRIVER
  #ifdef CONFIG_RP23XX_I2C0
  ret = board_i2cdev_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C0.\n");
    }
  #endif

  #ifdef CONFIG_RP23XX_I2C1
  ret = board_i2cdev_initialize(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C1.\n");
    }
  #endif
#endif

#ifdef CONFIG_RP23XX_SPI_DRIVER
  #ifdef CONFIG_RP23XX_SPI0
  ret = board_spidev_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SPI0.\n");
    }
  #endif

  #ifdef CONFIG_RP23XX_SPI1
  ret = board_spidev_initialize(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SPI1.\n");
    }
  #endif
#endif

#ifdef CONFIG_RP23XX_PWM
#  ifdef CONFIG_RP23XX_PWM0
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp23xx_pwmdev_initialize(0,
                                 CONFIG_RP23XX_PWM0A_GPIO,
                                 CONFIG_RP23XX_PWM0B_GPIO,
                                 (0
#      ifdef CONFIG_RP23XX_PWM0A_INVERT
                                  | RP23XX_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM0B_INVERT
                                  | RP23XX_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM0_PHASE_CORRECT
                                  | RP23XX_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp23xx_pwmdev_initialize(0,
                                 CONFIG_RP23XX_PWM0A_GPIO,
                                 (0
#      ifdef CONFIG_RP23XX_PWM0A_INVERT
                                  | RP23XX_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM0_PHASE_CORRECT
                                  | RP23XX_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PWM0.\n");
    }
#  endif

#  ifdef CONFIG_RP23XX_PWM1
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp23xx_pwmdev_initialize(1,
                                 CONFIG_RP23XX_PWM1A_GPIO,
                                 CONFIG_RP23XX_PWM1B_GPIO,
                                 (0
#      ifdef CONFIG_RP23XX_PWM1A_INVERT
                                  | RP23XX_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM1B_INVERT
                                  | RP23XX_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM1_PHASE_CORRECT
                                  | RP23XX_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp23xx_pwmdev_initialize(1,
                                 CONFIG_RP23XX_PWM1A_GPIO,
                                 (0
#      ifdef CONFIG_RP23XX_PWM1A_INVERT
                                  | RP23XX_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM1_PHASE_CORRECT
                                  | RP23XX_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PWM1.\n");
    }
#  endif

#  ifdef CONFIG_RP23XX_PWM2
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp23xx_pwmdev_initialize(2,
                                 CONFIG_RP23XX_PWM2A_GPIO,
                                 CONFIG_RP23XX_PWM2B_GPIO,
                                 (0
#      ifdef CONFIG_RP23XX_PWM2A_INVERT
                                  | RP23XX_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM2B_INVERT
                                  | RP23XX_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM2_PHASE_CORRECT
                                  | RP23XX_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp23xx_pwmdev_initialize(2,
                                 CONFIG_RP23XX_PWM2A_GPIO,
                                 (0
#      ifdef CONFIG_RP23XX_PWM2A_INVERT
                                  | RP23XX_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM2_PHASE_CORRECT
                                  | RP23XX_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PWM2.\n");
    }
#  endif

#  ifdef CONFIG_RP23XX_PWM3
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp23xx_pwmdev_initialize(3,
                                 CONFIG_RP23XX_PWM3A_GPIO,
                                 CONFIG_RP23XX_PWM3B_GPIO,
                                 (0
#      ifdef CONFIG_RP23XX_PWM3A_INVERT
                                  | RP23XX_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM3B_INVERT
                                  | RP23XX_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM3_PHASE_CORRECT
                                  | RP23XX_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp23xx_pwmdev_initialize(3,
                                 CONFIG_RP23XX_PWM3A_GPIO,
                                 (0
#      ifdef CONFIG_RP23XX_PWM3A_INVERT
                                  | RP23XX_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM3_PHASE_CORRECT
                                  | RP23XX_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PWM3.\n");
    }
#  endif

#  ifdef CONFIG_RP23XX_PWM4
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp23xx_pwmdev_initialize(4,
                                 CONFIG_RP23XX_PWM4A_GPIO,
                                 CONFIG_RP23XX_PWM4B_GPIO,
                                 (0
#      ifdef CONFIG_RP23XX_PWM4A_INVERT
                                  | RP23XX_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM4B_INVERT
                                  | RP23XX_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM4_PHASE_CORRECT
                                  | RP23XX_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp23xx_pwmdev_initialize(4,
                                 CONFIG_RP23XX_PWM4A_GPIO,
                                 (0
#      ifdef CONFIG_RP23XX_PWM4A_INVERT
                                  | RP23XX_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM4_PHASE_CORRECT
                                  | RP23XX_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PWM4.\n");
    }
#  endif

#  ifdef CONFIG_RP23XX_PWM5
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp23xx_pwmdev_initialize(5,
                                 CONFIG_RP23XX_PWM5A_GPIO,
                                 CONFIG_RP23XX_PWM5B_GPIO,
                                 (0
#      ifdef CONFIG_RP23XX_PWM5A_INVERT
                                  | RP23XX_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM5B_INVERT
                                  | RP23XX_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM5_PHASE_CORRECT
                                  | RP23XX_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#  else
  ret = rp23xx_pwmdev_initialize(5,
                                 CONFIG_RP23XX_PWM5A_GPIO,
                                 (0
#      ifdef CONFIG_RP23XX_PWM5A_INVERT
                                  | RP23XX_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM5_PHASE_CORRECT
                                  | RP23XX_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#  endif
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PWM5.\n");
    }
#  endif

#  ifdef CONFIG_RP23XX_PWM6
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp23xx_pwmdev_initialize(6,
                                 CONFIG_RP23XX_PWM6A_GPIO,
                                 CONFIG_RP23XX_PWM6B_GPIO,
                                 (0
#      ifdef CONFIG_RP23XX_PWM6A_INVERT
                                  | RP23XX_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM6B_INVERT
                                  | RP23XX_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM6_PHASE_CORRECT
                                  | RP23XX_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp23xx_pwmdev_initialize(6,
                                 CONFIG_RP23XX_PWM6A_GPIO,
                                 (0
#      ifdef CONFIG_RP23XX_PWM6A_INVERT
                                  | RP23XX_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM6_PHASE_CORRECT
                                  | RP23XX_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PWM6.\n");
    }
#  endif

#  ifdef CONFIG_RP23XX_PWM7
#    if defined(CONFIG_PWM_NCHANNELS) && CONFIG_PWM_NCHANNELS == 2
  ret = rp23xx_pwmdev_initialize(7,
                                 CONFIG_RP23XX_PWM7A_GPIO,
                                 CONFIG_RP23XX_PWM7B_GPIO,
                                 (0
#      ifdef CONFIG_RP23XX_PWM7A_INVERT
                                  | RP23XX_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM7B_INVERT
                                  | RP23XX_PWM_CSR_B_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM7_PHASE_CORRECT
                                  | RP23XX_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    else
  ret = rp23xx_pwmdev_initialize(7,
                                 CONFIG_RP23XX_PWM7A_GPIO,
                                 (0
#      ifdef CONFIG_RP23XX_PWM7A_INVERT
                                  | RP23XX_PWM_CSR_A_INV
#      endif
#      ifdef CONFIG_RP23XX_PWM7_PHASE_CORRECT
                                  | RP23XX_PWM_CSR_PH_CORRECT
#      endif
                                 ));
#    endif
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PWM7.\n");
    }
#  endif
#endif

#ifdef CONFIG_RP23XX_SPISD
  /* Mount the SPI-based MMC/SD block driver */

  ret = board_spisd_initialize(0, CONFIG_RP23XX_SPISD_SPI_CH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SPI device to MMC/SD: %d\n",
           ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount procfs at %s: %d\n", "/proc", ret);
    }
#endif

#ifdef CONFIG_RP23XX_I2S
  ret = board_i2sdev_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2S.\n");
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = rp23xx_dev_gpio_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
    }
#endif

  /* Initialize ADC */

#if defined(CONFIG_ADC) && defined(CONFIG_RP23XX_ADC)

#  ifdef CONFIG_RP23XX_ADC_CHANNEL0
#    define ADC_0 true
#  else
#    define ADC_0 false
#  endif

#  ifdef CONFIG_RP23XX_ADC_CHANNEL1
#    define ADC_1 true
#  else
#    define ADC_1 false
#  endif

#  ifdef CONFIG_RP23XX_ADC_CHANNEL2
#    define ADC_2 true
#  else
#    define ADC_2 false
#  endif

#  ifdef CONFIG_RP23XX_ADC_CHANNEL3
#    define ADC_3 true
#  else
#    define ADC_3 false
#  endif

#  ifdef CONFIG_RP23XX_ADC_TEMPERATURE
#    define ADC_TEMP true
#  else
#    define ADC_TEMP false
#  endif

  ret = rp23xx_adc_setup("/dev/adc0", ADC_0, ADC_1, ADC_2, ADC_3, ADC_TEMP);
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to initialize ADC Driver: %d\n", ret);
    }

#endif /* defined(CONFIG_ADC) && defined(CONFIG_RP23XX_ADC) */

  /* Initialize board neo-pixel */

#if defined(CONFIG_RP23XX_BOARD_HAS_WS2812) && defined(CONFIG_WS2812)

  if (rp23xx_ws2812_setup("/dev/leds0",
                          CONFIG_RP23XX_WS2812_GPIO_PIN,
                          CONFIG_RP23XX_WS2812_PWR_GPIO,
                          CONFIG_WS2812_LED_COUNT,
                          HAS_WHITE) == NULL)
    {
      syslog(LOG_ERR, "Failed to initialize WS2812: %d\n", errno);
    }
#endif

#ifdef CONFIG_WATCHDOG
  /* Configure watchdog timer */

  ret = rp23xx_wdt_init();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize watchdog drivers: %d\n",
             ret);
    }
#endif

#if defined(CONFIG_RP23XX_ROMFS_ROMDISK_DEVNAME)
  /* Register the ROM disk */

  ret = romdisk_register(CONFIG_RP23XX_ROMFS_ROMDISK_MINOR,
                         rp23xx_romfs_img,
                         NSECTORS(rp23xx_romfs_img_len),
                         CONFIG_RP23XX_ROMFS_ROMDISK_SECTSIZE);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: romdisk_register failed: %d\n", -ret);
    }
  else
    {
      /* Mount the file system */

      ret = nx_mount(CONFIG_RP23XX_ROMFS_ROMDISK_DEVNAME,
                     CONFIG_RP23XX_ROMFS_MOUNT_MOUNTPOINT,
                     "romfs",
                     MS_RDONLY,
                     NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: nx_mount(%s,%s,romfs) failed: %d\n",
                 CONFIG_RP23XX_ROMFS_ROMDISK_DEVNAME,
                 CONFIG_RP23XX_ROMFS_MOUNT_MOUNTPOINT,
                 ret);
        }
    }

#endif
  return ret;
}
