/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006 Christian Walter <wolti@sil.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * File: $Id: mbutils.c,v 1.6 2007/02/18 23:49:07 wolti Exp $
 */

/* ----------------------- System includes ----------------------------------*/
#include <nuttx/config.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include <apps/modbus/mb.h>
#include <apps/modbus/mbproto.h>

/* ----------------------- Defines ------------------------------------------*/
#define BITS_uint8_t      8U

/* ----------------------- Start implementation -----------------------------*/
void
xMBUtilSetBits( uint8_t * ucByteBuf, uint16_t usBitOffset, uint8_t ucNBits,
                uint8_t ucValue )
{
    uint16_t        usWordBuf;
    uint16_t        usMask;
    uint16_t        usByteOffset;
    uint16_t        usNPreBits;
    uint16_t        usValue = ucValue;

    ASSERT( ucNBits <= 8 );
    ASSERT( ( size_t )BITS_uint8_t == sizeof( uint8_t ) * 8 );

    /* Calculate byte offset for first byte containing the bit values starting
     * at usBitOffset. */
    usByteOffset = ( uint16_t )( ( usBitOffset ) / BITS_uint8_t );

    /* How many bits precede our bits to set. */
    usNPreBits = ( uint16_t )( usBitOffset - usByteOffset * BITS_uint8_t );

    /* Move bit field into position over bits to set */
    usValue <<= usNPreBits;

    /* Prepare a mask for setting the new bits. */
    usMask = ( uint16_t )( ( 1 << ( uint16_t ) ucNBits ) - 1 );
    usMask <<= usBitOffset - usByteOffset * BITS_uint8_t;

    /* copy bits into temporary storage. */
    usWordBuf = ucByteBuf[usByteOffset];
    usWordBuf |= ucByteBuf[usByteOffset + 1] << BITS_uint8_t;

    /* Zero out bit field bits and then or value bits into them. */
    usWordBuf = ( uint16_t )( ( usWordBuf & ( ~usMask ) ) | usValue );

    /* move bits back into storage */
    ucByteBuf[usByteOffset] = ( uint8_t )( usWordBuf & 0xFF );
    ucByteBuf[usByteOffset + 1] = ( uint8_t )( usWordBuf >> BITS_uint8_t );
}

uint8_t
xMBUtilGetBits( uint8_t * ucByteBuf, uint16_t usBitOffset, uint8_t ucNBits )
{
    uint16_t        usWordBuf;
    uint16_t        usMask;
    uint16_t        usByteOffset;
    uint16_t        usNPreBits;

    /* Calculate byte offset for first byte containing the bit values starting
     * at usBitOffset. */
    usByteOffset = ( uint16_t )( ( usBitOffset ) / BITS_uint8_t );

    /* How many bits precede our bits to set. */
    usNPreBits = ( uint16_t )( usBitOffset - usByteOffset * BITS_uint8_t );

    /* Prepare a mask for setting the new bits. */
    usMask = ( uint16_t )( ( 1 << ( uint16_t ) ucNBits ) - 1 );

    /* copy bits into temporary storage. */
    usWordBuf = ucByteBuf[usByteOffset];
    usWordBuf |= ucByteBuf[usByteOffset + 1] << BITS_uint8_t;

    /* throw away unneeded bits. */
    usWordBuf >>= usNPreBits;

    /* mask away bits above the requested bitfield. */
    usWordBuf &= usMask;

    return ( uint8_t ) usWordBuf;
}

eMBException
prveMBError2Exception( eMBErrorCode eErrorCode )
{
    eMBException    eStatus;

    switch ( eErrorCode )
    {
        case MB_ENOERR:
            eStatus = MB_EX_NONE;
            break;

        case MB_ENOREG:
            eStatus = MB_EX_ILLEGAL_DATA_ADDRESS;
            break;

        case MB_ETIMEDOUT:
            eStatus = MB_EX_SLAVE_BUSY;
            break;

        default:
            eStatus = MB_EX_SLAVE_DEVICE_FAILURE;
            break;
    }

    return eStatus;
}
