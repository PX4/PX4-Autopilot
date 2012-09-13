/************************************************************************************
 * arch/arm/src/imx/imx_i2c.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_IMX_I2C_H
#define __ARCH_ARM_IMX_I2C_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
 
/************************************************************************************
 * Definitions
 ************************************************************************************/

/* I2C Register Offsets *************************************************************/

#define I2C_IADR_OFFSET             0x0000
#define I2C_IFDR_OFFSET             0x0004
#define I2C_I2CR_OFFSET             0x0008
#define I2C_I2SR_OFFSET             0x000c
#define I2C_I2DR_OFFSET             0x0010

/* I2C Register Addresses ***********************************************************/

#define IMX_I2C_IADR                (IMX_I2C_VBASE + I2C_IADR_OFFSET) 
#define IMX_I2C_IFDR                (IMX_I2C_VBASE + I2C_IFDR_OFFSET)
#define IMX_I2C_I2CR                (IMX_I2C_VBASE + I2C_I2CR_OFFSET)
#define IMX_I2C_I2SR                (IMX_I2C_VBASE + I2C_I2SR_OFFSET) 
#define IMX_I2C_I2DR                (IMX_I2C_VBASE + I2C_I2DR_OFFSET) 

/* I2C Register Bit Definitions *****************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_IMX_I2C_H */
