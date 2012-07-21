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
 * File: $Id: portevent.c,v 1.1 2006/08/01 20:58:49 wolti Exp $
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include <apps/modbus/mb.h>
#include <apps/modbus/mbport.h>

#include "port.h"

/* ----------------------- Variables ----------------------------------------*/
static eMBEventType eQueuedEvent;
static bool     xEventInQueue;

/* ----------------------- Start implementation -----------------------------*/
bool
xMBPortEventInit( void )
{
    xEventInQueue = false;
    return true;
}

bool
xMBPortEventPost( eMBEventType eEvent )
{
    xEventInQueue = true;
    eQueuedEvent = eEvent;
    return true;
}

bool
xMBPortEventGet( eMBEventType * eEvent )
{
    bool            xEventHappened = false;

    if( xEventInQueue )
    {
        *eEvent = eQueuedEvent;
        xEventInQueue = false;
        xEventHappened = true;
    }
    else
    {
        /* Poll the serial device. The serial device timeouts if no
         * characters have been received within for t3.5 during an
         * active transmission or if nothing happens within a specified
         * amount of time. Both timeouts are configured from the timer
         * init functions.
         */
        ( void )xMBPortSerialPoll(  );

        /* Check if any of the timers have expired. */
        vMBPortTimerPoll(  );

    }
    return xEventHappened;
}
