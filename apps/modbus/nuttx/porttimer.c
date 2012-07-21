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
 * File: $Id: porttimer.c,v 1.1 2006/08/01 20:58:50 wolti Exp $
 */

/* ----------------------- Standard includes --------------------------------*/
#include <nuttx/config.h>

#include <sys/time.h>
#include <stdlib.h>
#include <assert.h>

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include <apps/modbus/mb.h>
#include <apps/modbus/mbport.h>

/* ----------------------- Defines ------------------------------------------*/

/* ----------------------- Static variables ---------------------------------*/
uint32_t        ulTimeOut;
bool            bTimeoutEnable;

static struct timeval xTimeLast;

/* ----------------------- Start implementation -----------------------------*/
bool
xMBPortTimersInit( uint16_t usTim1Timerout50us )
{
    ulTimeOut = usTim1Timerout50us / 20U;
    if( ulTimeOut == 0 )
        ulTimeOut = 1;

    return xMBPortSerialSetTimeout( ulTimeOut );
}

void
xMBPortTimersClose(  )
{
    /* Does not use any hardware resources. */
}

void
vMBPortTimerPoll(  )
{
    uint32_t        ulDeltaMS;
    struct timeval  xTimeCur;

    /* Timers are called from the serial layer because we have no high
     * res timer in Win32. */
    if( bTimeoutEnable )
    {
        if( gettimeofday( &xTimeCur, NULL ) != 0 )
        {
            /* gettimeofday failed - retry next time. */
        }
        else
        {
            ulDeltaMS = ( xTimeCur.tv_sec - xTimeLast.tv_sec ) * 1000L +
                ( xTimeCur.tv_usec - xTimeLast.tv_usec ) * 1000L;
            if( ulDeltaMS > ulTimeOut )
            {
                bTimeoutEnable = false;
                ( void )pxMBPortCBTimerExpired(  );
            }
        }
    }
}

void
vMBPortTimersEnable(  )
{
    int             res = gettimeofday( &xTimeLast, NULL );

    ASSERT( res == 0 );
    bTimeoutEnable = true;
}

void
vMBPortTimersDisable(  )
{
    bTimeoutEnable = false;
}
