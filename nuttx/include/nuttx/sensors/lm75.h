/****************************************************************************
 * include/nuttx/lm75.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __NUTTX_SENSORS_LM75_H
#define __NUTTX_SENSORS_LM75_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************
 * CONFIG_I2C - Enables support for I2C drivers
 * CONFIG_I2C_LM75 - Enables support for the LM-75 driver
 */

#define CONFIG_LM75_BASEADDR 0x48

/* IOCTL Commands ***********************************************************/

#define SNIOC_READCONF     _SNIOC(0x0001) /* Arg: uint8_t* pointer */
#define SNIOC_WRITECONF    _SNIOC(0x0002) /* Arg: uint8_t value */
#define SNIOC_SHUTDOWN     _SNIOC(0x0003) /* Arg: None */
#define SNIOC_POWERUP      _SNIOC(0x0004) /* Arg: None */
#define SNIOC_FAHRENHEIT   _SNIOC(0x0005) /* Arg: None */
#define SNIOC_CENTIGRADE   _SNIOC(0x0006) /* Arg: None */
#define SNIOC_READTHYS     _SNIOC(0x0007) /* Arg: b16_t* pointer */
#define SNIOC_WRITETHYS    _SNIOC(0x0008) /* Arg: b16_t value */
#define SNIOC_READTOS      _SNIOC(0x0009) /* Arg: b16_t* pointer */
#define SNIOC_WRITRETOS    _SNIOC(0x000a) /* Arg: b16_t value */

/* LM-75 Register Definitions ***********************************************/
/* LM-75 Registers addresses */

#define LM75_TEMP_REG      0x00     /* Temperature Register */
#define LM75_CONF_REG      0x01     /* Configuration Register */
#define LM75_THYS_REG      0x02     /* Temperature Register */
#define LM75_TOS_REG       0x03     /* Over-temp Shutdown Threshold Register */

/* Configuration Register Bit Definitions */

#define LM75_CONF_SHUTDOWN (1 << 0) /* Bit 0: Put LM75 goes in low power shutdown mode */
#define LM75_CONF_INTMODE  (1 << 1) /* Bit 1: 0=Comparator 1=Interrupt mode */
#define LM75_CONF_POLARITY (1 << 2) /* Bit 2: 0=O.S. Active low 1=active high */
#define LM75_CONF_FAULTQ   (3)      /* Bits 3-4: # faults before setting O.S. */

/* NOTE: When temperature values are read, they are return as b16_t, fixed
 * precision integer values (see include/fixedmath.h).
 */

/****************************************************************************
 * Global Data
 ****************************************************************************/

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: lm75_register
 *
 * Description:
 *   Register the LM-75 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   i2c - An instance of the I2C interface to use to communicate with LM75
 *   addr - The I2C address of the LM-75.  The base I2C address of the LM75
 *   is 0x48.  Bits 0-3 can be controlled to get 8 unique addresses from 0x48
 *   through 0x4f.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

EXTERN int lm75_register(FAR const char *devpath, FAR struct i2c_dev_s *i2c,
                         uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __NUTTX_SENSORS_LM75_H */
