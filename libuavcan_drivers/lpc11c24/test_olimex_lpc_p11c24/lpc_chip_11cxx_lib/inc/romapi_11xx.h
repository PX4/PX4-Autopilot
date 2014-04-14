/*
 * @brief LPC11xx ROM API declarations and functions
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#ifndef __ROMAPI_11XX_H_
#define __ROMAPI_11XX_H_

#include "error.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup ROMAPI_11XX CHIP: LPC11XX ROM API declarations and functions
 * @ingroup CHIP_11XX_Drivers
 * @{
 */

/**
 * @brief LPC11XX High level ROM API structure
 */
typedef struct {
	const uint32_t usbdApiBase;				/*!< USBD API function table base address */
	const uint32_t reserved0;				/*!< Reserved */
	const uint32_t candApiBase;				/*!< CAN API function table base address */
	const uint32_t pwrApiBase;				/*!< Power API function table base address */
	const uint32_t reserved1;				/*!< Reserved */
	const uint32_t reserved2;				/*!< Reserved */
	const uint32_t reserved3;				/*!< Reserved */
	const uint32_t reserved4;				/*!< Reserved */
} LPC_ROM_API_T;

/**
 * @brief LPC11XX IAP_ENTRY API function type
 */
typedef void (*IAP_ENTRY_T)(unsigned int[], unsigned int[]);

static INLINE void iap_entry(unsigned int cmd_param[], unsigned int status_result[])
{
	((IAP_ENTRY_T) IAP_ENTRY_LOCATION)(cmd_param, status_result);
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __ROMAPI_11XX_H_ */
