/*
 * @brief LPC11xx selective CMSIS inclusion file
 *
 * Copyright(C) NXP Semiconductors, 2012
 * All rights reserved.
 *
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
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#ifndef __CMSIS_H_
#define __CMSIS_H_

#include "lpc_types.h"
#include "sys_config.h"

/* Select correct CMSIS include file based on CHIP_* definition */
#if defined(CHIP_LPC110X)
#include "cmsis_110x.h"
typedef LPC110X_IRQn_Type IRQn_Type;
#elif defined(CHIP_LPC1125)
#include "cmsis_1125.h"
typedef LPC1125_IRQn_Type IRQn_Type;
#elif defined(CHIP_LPC11AXX)
#include "cmsis_11axx.h"
typedef LPC11AXX_IRQn_Type IRQn_Type;
#elif defined(CHIP_LPC11CXX)
#include "cmsis_11cxx.h"
typedef LPC11CXX_IRQn_Type IRQn_Type;
#elif defined(CHIP_LPC11EXX)
#include "cmsis_11exx.h"
typedef LPC11EXX_IRQn_Type IRQn_Type;
#elif defined(CHIP_LPC11UXX)
#include "cmsis_11uxx.h"
typedef LPC11UXX_IRQn_Type IRQn_Type;
#elif defined(CHIP_LPC11XXLV)
#include "cmsis_11lvxx.h"
typedef LPC11XXLV_IRQn_Type IRQn_Type;
#else
#error "No CHIP_* definition is defined"
#endif

/* Cortex-M0 processor and core peripherals */
#include "core_cm0.h"

#endif /* __CMSIS_H_ */
