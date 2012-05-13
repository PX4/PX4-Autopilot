/********************************************************************************************
 * drivers/input/tsc2007.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "1.2V to 3.6V, 12-Bit, Nanopower, 4-Wire Micro TOUCH SCREEN CONTROLLER
 *    with I2C Interface," SBAS405A March 2007, Revised, March 2009, Texas
 *    Instruments Incorporated
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
 ********************************************************************************************/

/* The TSC2007 is an analog interface circuit for a human interface touch screen device.
 * All peripheral functions are controlled through the command byte and onboard state
 * machines.
 */

#ifndef __DRIVERS_INPUT_TSC2007_H
#define __DRIVERS_INPUT_TSC2007_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/* TSC2007 Address */

#define TSC2007_ADDRESS_MASK      (0xf8)    /* Bits 3-7: Invariant part of TSC2007 address */
#define TSC2007_ADDRESS           (0x90)    /* Bits 3-7: Always set at '10010' */
#define TSC2007_A1                (1 << 2)  /* Bit 2: A1 */
#define TSC2007_A0                (1 << 1)  /* Bit 1: A1 */
#define TSC2007_READ              (1 << 0)  /* Bit0=1: Selects read operation */
#define TSC2007_WRITE             (0)       /* Bit0=0: Selects write operation */

/* TSC2007 Command Byte */

#define TSC2007_CMD_FUNC_SHIFT    (4)       /* Bits 4-7: Converter function select bits */
#define TSC2007_CMD_FUNC_MASK     (15 << TSC2007_CMD_FUNC_SHIFT)
#  define TSC2007_CMD_FUNC_TEMP0  (0 << TSC2007_CMD_FUNC_SHIFT)  /* Measure TEMP0 */
#  define TSC2007_CMD_FUNC_AUX    (2 << TSC2007_CMD_FUNC_SHIFT)  /* Measure AUX */
#  define TSC2007_CMD_FUNC_TEMP1  (4 << TSC2007_CMD_FUNC_SHIFT)  /* Measure TEMP1 */
#  define TSC2007_CMD_FUNC_XON    (8 << TSC2007_CMD_FUNC_SHIFT)  /* Activate X-drivers */
#  define TSC2007_CMD_FUNC_YON    (9 << TSC2007_CMD_FUNC_SHIFT)  /* Activate Y-drivers */
#  define TSC2007_CMD_FUNC_YXON   (10 << TSC2007_CMD_FUNC_SHIFT) /* Activate Y+, X-drivers */
#  define TSC2007_CMD_FUNC_SETUP  (11 << TSC2007_CMD_FUNC_SHIFT) /* Setup command */
#  define TSC2007_CMD_FUNC_XPOS   (12 << TSC2007_CMD_FUNC_SHIFT) /* Measure X position */
#  define TSC2007_CMD_FUNC_YPOS   (13 << TSC2007_CMD_FUNC_SHIFT) /* Measure Y position */
#  define TSC2007_CMD_FUNC_Z1POS  (14 << TSC2007_CMD_FUNC_SHIFT) /* Measure Z1 position */
#  define TSC2007_CMD_FUNC_Z2POS  (15 << TSC2007_CMD_FUNC_SHIFT) /* Measure Z2 positionn */
#define TSC2007_CMD_PWRDN_SHIFT   (2)       /* Bits 2-3: Power-down bits */
#define TSC2007_CMD_PWRDN_MASK    (3 << TSC2007_CMD_PWRDN_SHIFT)
#  define TSC2007_CMD_PWRDN_IRQEN (0 << TSC2007_CMD_PWRDN_SHIFT)  /* 00: Power down between cycles; PENIRQ enabled */
#  define TSC2007_CMD_ADCON_IRQDIS (1 << TSC2007_CMD_PWRDN_SHIFT) /* 01: A/D converter on; PENIRQ disabled */
#  define TSC2007_CMD_ADCOFF_IRQEN (2 << TSC2007_CMD_PWRDN_SHIFT) /* 10: A/D converter off; PENIRQ enabled. */
                                                                  /* 11: A/D converter on; PENIRQ disabled. */
#define TSC2007_CMD_12BIT         (0)       /* Bit 1: 0=12-bit */
#define TSC2007_CMD_8BIT          (1 << 1)  /* Bit 1: 1=8-bit */
                                            /* Bit 0: Don't care */

/* TSC2007 Setup Command */

#define TSC2007_SETUP_CMD         TSC2007_CMD_FUNC_SETUP         /* Bits 4-7: Setup command */
                                            /* Bits 2-3: Must be zero */
#define TSC2007_CMD_USEMAV        (0)       /* Bit 1: 0: Use the onboard MAV filter (default) */
#define TSC2007_CMD_BYPASSMAV     (1 << 1)  /* Bit 1: 1: Bypass the onboard MAV filter */
#define TSC2007_CMD_PU_50KOHM     (0)       /* Bit 0: 0: RIRQ = 50kOhm (default). */
#define TSC2007_CMD_PU_90KOHM     (1 << 1)  /* Bit 0: 1: 1: RIRQ = 90kOhm */

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __DRIVERS_INPUT_TSC2007_H */
