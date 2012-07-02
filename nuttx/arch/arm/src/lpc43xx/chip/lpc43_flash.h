/************************************************************************************
 * arch/arm/src/lpc43xx/chip/lpc43_flash.h
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

#ifndef __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_FLASH_H
#define __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_FLASH_H

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

#define LPC43_ROM_IAP_DRIVER_TABLE LPC43_ROM_DRIVER_TABLE0

#define IAP_LOCATION *(volatile unsigned int *)LPC43_ROM_IAP_DRIVER_TABLE;

/* General usage:
 *
 * Declare a function pointer in your code like:
 *
 *  iap_t iap = (iap_t)IAP_LOCATION;
 *
 * Then call the IAP using the function pointe like:
 *
 *  unsigned long command[6];
 *  unsigned long result[5];
 *  ...
 *  iap(command, result);
 */

/* IAP Commands
 *
 *   See tables 1042-1053 in the "LPC43xx User Manual" (UM10503), Rev. 1.2, 8 June
 *   2012, NXP for definitions descriptions of each IAP command.
 */

#define IAP_INIT            49   /* Initialization */
#define IAP_WRITE_PREPARE   50   /* Prepare sectors for write operation */
#define IAP_WRITE           51   /* Copy RAM to Flash */
#define IAP_ERASE_SECTOR    52   /* Erase sectors */
#define IAP_BLANK_CHECK     53   /* Blank check sectors */
#define IAP_PART_ID         54   /* Read part ID */
#define IAP_BOOT_VERSION    55   /* Read Boot Code version */
#define IAP_SERIAL_NUMBER   58   /* Read device serial number */
#define IAP_COMPARE         56   /* Compare */
#define IAP_REINVOKE        57   /* Reinvoke ISP */
#define IAP_ERASE_PAGE      59   /* Erase page */
#define IAP_SET_BANK        60   /* Set active boot flash bank */

/* ISP/IAP return codes */

/* Command is executed successfully. Sent by ISP handler only when command given by
 * the host has been completely and successfully executed.
 */

#define CMD_SUCCESS 0

/* Invalid command */

#define INVALID_COMMAND

/* Source address is not on word boundary. */

#define SRC_ADDR_ERROR 2

/* Destination address not on word or 256 byte boundary. */

#define DST_ADDR_ERROR 3

/* Source address is not mapped in the memory map. Count value is taken into
 * consideration where applicable.
 */

#define SRC_ADDR_NOT_MAPPED 4

/* Destination address is not mapped in the memory map. Count value is taken into
 * consideration where applicable.
 */

#define DST_ADDR_NOT_MAPPED 5

/* Byte count is not multiple of 4 or is not a permitted value. */

#define COUNT_ERROR 6

/* Sector number is invalid or end sector number is greater than start sector number. */

#define INVALID_SECTOR 7

/* Sector is not blank. */

#define SECTOR_NOT_BLANK 8

/* Command to prepare sector for write operation was not executed. */

#define SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION 9

/* Source and destination data not equal. */

#define COMPARE_ERROR 10

/* Flash programming hardware interface is busy. */

#define BUSY 11

/* Insufficient number of parameters or invalid parameter. */

#define PARAM_ERROR 12

/* Address is not on word boundary. */

#define ADDR_ERROR 13

/* Address is not mapped in the memory map. Count value is taken in to consideration
 * where applicable.
 */

#define ADDR_NOT_MAPPED 14

/* Command is locked. */

#define CMD_LOCKED 15

/* Unlock code is invalid. */

#define INVALID_CODE 16

/* Invalid baud rate setting. */

#define INVALID_BAUD_RATE 17

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* IAP function pointer */

typedef void (*iap_t)(unsigned int *cmd, unsigned int *result);

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC43XX_CHIP_LPC43_FLASH_H */
