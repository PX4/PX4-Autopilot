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
 * File: $Id: mbfunc.h,v 1.12 2006/12/07 22:10:34 wolti Exp $
 */

#ifndef _MB_FUNC_H
#define _MB_FUNC_H

#ifdef __cplusplus
PR_BEGIN_EXTERN_C
#endif
#ifdef CONFIG_MB_FUNC_OTHER_REP_SLAVEID_BUF
    eMBException eMBFuncReportSlaveID( uint8_t * pucFrame, uint16_t * usLen );
#endif

#ifdef CONFIG_MB_FUNC_READ_INPUT_ENABLED 
eMBException    eMBFuncReadInputRegister( uint8_t * pucFrame, uint16_t * usLen );
#endif

#ifdef CONFIG_MB_FUNC_READ_HOLDING_ENABLED
eMBException    eMBFuncReadHoldingRegister( uint8_t * pucFrame, uint16_t * usLen );
#endif

#ifdef CONFIG_MB_FUNC_WRITE_HOLDING_ENABLED
eMBException    eMBFuncWriteHoldingRegister( uint8_t * pucFrame, uint16_t * usLen );
#endif

#ifdef CONFIG_MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED
eMBException    eMBFuncWriteMultipleHoldingRegister( uint8_t * pucFrame, uint16_t * usLen );
#endif

#ifdef CONFIG_MB_FUNC_READ_COILS_ENABLED
eMBException    eMBFuncReadCoils( uint8_t * pucFrame, uint16_t * usLen );
#endif

#ifdef CONFIG_MB_FUNC_WRITE_COIL_ENABLED
eMBException    eMBFuncWriteCoil( uint8_t * pucFrame, uint16_t * usLen );
#endif

#ifdef CONFIG_MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED
eMBException    eMBFuncWriteMultipleCoils( uint8_t * pucFrame, uint16_t * usLen );
#endif

#ifdef CONFIG_MB_FUNC_READ_DISCRETE_INPUTS_ENABLED
eMBException    eMBFuncReadDiscreteInputs( uint8_t * pucFrame, uint16_t * usLen );
#endif

#ifdef CONFIG_MB_FUNC_READWRITE_HOLDING_ENABLED
eMBException    eMBFuncReadWriteMultipleHoldingRegister( uint8_t * pucFrame, uint16_t * usLen );
#endif

#ifdef __cplusplus
PR_END_EXTERN_C
#endif
#endif
