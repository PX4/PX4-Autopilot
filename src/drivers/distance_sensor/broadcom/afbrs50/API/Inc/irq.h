/*************************************************************************//**
 * @file
 * @brief    	This file is part of the AFBR-S50 API.
 * @details		This file provides functionality to globally turn IRQs on/off.
 *
 * @copyright	Copyright c 2016-2019, Avago Technologies GmbH.
 * 				All rights reserved.
 *
 *****************************************************************************/

#ifndef IRQ_H
#define IRQ_H

#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup	IRQ IRQ: Global Interrupt Control
 * @ingroup		driver
 * @brief		Global IRQ Module
 * @details		This module provides functionality to globally enable/disable
 * 				interrupts by turning the I-bit in the CPSR on/off.
 * @addtogroup 	IRQ
 * @{
 *****************************************************************************/

#include "platform/argus_irq.h"

#ifdef __cplusplus
}
#endif

/*! @} */
#endif /* IRQ_H */
