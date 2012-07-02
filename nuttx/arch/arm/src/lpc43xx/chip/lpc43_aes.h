/************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_aes.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_AES_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_AES_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* The AES is controlled through a set of simple API calls located in the LPC43xx
 * ROM.  This value holds the pointer to the AES driver table.
 */

#define LPC43_ROM_AES_DRIVER_TABLE LPC43_ROM_DRIVER_TABLE2

/************************************************************************************
 * Public Types
 ************************************************************************************/

enum lpc43_aescmd_e
{
  AES_API_CMD_ENCODE_ECB = 0,
  AES_API_CMD_DECODE_ECB = 1, 
  AES_API_CMD_ENCODE_CBC = 2, 
  AES_API_CMD_DECODE_CBC = 3 
};

struct lpc43_aes_s
{
  /* Initialize the AES engine */

  void (*aes_Init)(void);

  /* Offset 0x04 -- Defines AES engine operation mode.  See enum lpc43_aescmd_e */

  unsigned int (*aes_SetMode)(unsigned int cmd);

  /* Load 128-bit AES user keys */

  void (*aes_LoadKey1)(void);
  void (*aes_LoadKey2)(void);

  /* Loads randomly generated key in AES engine. To update the RNG and load a new
   * random number, use the API call otp_GenRand before aes_LoadKeyRNG.
   */

  void (*aes_LoadKeyRNG)(void);

  /* Loads 128-bit AES software defined user key (16 bytes) */

  void (*aes_LoadKeySW)(unsigned char *key);

  /* Loads 128-bit AES initialization vector (16 bytes) */
 
  void (*aes_LoadIV_SW)(unsigned char *iv);

  /* Loads 128-bit AES IC specific initialization vector, which is used to decrypt
   * a boot image.
   */

  void (*aes_LoadIV_IC)(void);
};

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_AES_H */
