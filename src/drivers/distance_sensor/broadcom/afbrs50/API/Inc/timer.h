/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides driver functionality for PIT (periodic interrupt timer).
 *
 * @copyright	Copyright c 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *
 *****************************************************************************/

#ifndef TIMER_H
#define TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup	timer Timer Module
 * @ingroup		driver
 * @brief		Timer Hardware Driver Module
 * @details		Provides driver functionality for the timer peripherals.
 * 				This module actually implements the #argus_timer interface that
 * 				is required for the Argus API. It contains two timing
 * 				functionalities: A periodic interrupt/callback timer and
 * 				an lifetime counter.
 *
 * 				Note that the current implementation only features a single
 * 				callback timer interval and does not yet support the feature
 * 				of multiple intervalls at a time.
 *
 * @addtogroup 	timer
 * @{
 *****************************************************************************/

/*******************************************************************************
 * Include Files
 ******************************************************************************/

#include "platform/argus_timer.h"

/*!***************************************************************************
 * @brief	Initializes the timer hardware.
 *****************************************************************************/
void Timer_Init(void);

#ifdef __cplusplus
}
#endif

/*! @} */
#endif /* TIMER_H */
