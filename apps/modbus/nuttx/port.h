/*
 * FreeModbus Libary: NuttX Port
 * Based on the FreeModbus Linux port by:
 *
 *   Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: port.h,v 1.1 2006/08/01 20:58:49 wolti Exp $
 */

#ifndef __APPS_MODBUS_NUTTX_PORT_H
#define __APPS_MODBUS_NUTTX_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include <assert.h>

/* ----------------------- Defines ------------------------------------------*/

#define INLINE
#define PR_BEGIN_EXTERN_C  extern "C" {
#define PR_END_EXTERN_C    }

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif

#define ENTER_CRITICAL_SECTION( ) vMBPortEnterCritical()
#define EXIT_CRITICAL_SECTION( ) vMBPortExitCritical()

#define MB_PORT_HAS_CLOSE 1

#ifndef true
#  define true   true
#endif

#ifndef false
#  define false  false
#endif

/* ----------------------- Type definitions ---------------------------------*/

typedef enum
{
  MB_LOG_ERROR = 0,
  MB_LOG_WARN  = 1,
  MB_LOG_INFO  = 2,
  MB_LOG_DEBUG = 3
} eMBPortLogLevel;

/* ----------------------- Function prototypes ------------------------------*/

void vMBPortEnterCritical(void);
void vMBPortExitCritical(void);
void vMBPortLog(eMBPortLogLevel eLevel, const char * szModule,
                const char * szFmt, ...);
void vMBPortTimerPoll(void);
bool xMBPortSerialPoll(void);
bool xMBPortSerialSetTimeout(uint32_t dwTimeoutMs);

#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif
