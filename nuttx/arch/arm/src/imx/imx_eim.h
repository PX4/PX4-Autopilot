/************************************************************************************
 * arch/arm/src/imx/imx_eim.h
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

#ifndef __ARCH_ARM_IMX_WIEM_H
#define __ARCH_ARM_IMX_WIEM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
 
/************************************************************************************
 * Definitions
 ************************************************************************************/

/* EIM Register Offsets ************************************************************/

#define EIM_CS0H_OFFSET             0x00
#define EIM_CS0L_OFFSET             0x04
#define EIM_CS1H_OFFSET             0x08
#define EIM_CS1L_OFFSET             0x0c
#define EIM_CS2H_OFFSET             0x10
#define EIM_CS2L_OFFSET             0x14
#define EIM_CS3H_OFFSET             0x18
#define EIM_CS3L_OFFSET             0x1c
#define EIM_CS4H_OFFSET             0x20
#define EIM_CS4L_OFFSET             0x24
#define EIM_CS5H_OFFSET             0x28
#define EIM_CS5L_OFFSET             0x2c
#define EIM_WEIM_OFFSET             0x30

/* EIM Register Addresses ***********************************************************/

#define IMX_EIM_CS0H                (EIM_BASE_ADDR + EIM_CS0H_OFFSET)                
#define IMX_EIM_CS0L                (EIM_BASE_ADDR + EIM_CS0L_OFFSET)                
#define IMX_EIM_CS1H                (EIM_BASE_ADDR + EIM_CS1H_OFFSET)                
#define IMX_EIM_CS1L                (EIM_BASE_ADDR + EIM_CS1L_OFFSET)                
#define IMX_EIM_CS2H                (EIM_BASE_ADDR + EIM_CS2H_OFFSET)                
#define IMX_EIM_CS2L                (EIM_BASE_ADDR + EIM_CS2L_OFFSET)                
#define IMX_EIM_CS3H                (EIM_BASE_ADDR + EIM_CS3H_OFFSET)                
#define IMX_EIM_CS3L                (EIM_BASE_ADDR + EIM_CS3L_OFFSET)                
#define IMX_EIM_CS4H                (EIM_BASE_ADDR + EIM_CS4H_OFFSET)                
#define IMX_EIM_CS4L                (EIM_BASE_ADDR + EIM_CS4L_OFFSET)                
#define IMX_EIM_CS5H                (EIM_BASE_ADDR + EIM_CS5H_OFFSET)                
#define IMX_EIM_CS5L                (EIM_BASE_ADDR + EIM_CS5L_OFFSET)                
#define IMX_EIM_WEIM                (EIM_BASE_ADDR + EIM_WEIM_OFFSET)

/* EIM Register Bit Definitions *****************************************************/

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#endif  /* __ARCH_ARM_IMX_EIM_H */
