/***************************************************************************
 * pinsn.h
 * External Declarations associated libinsn.a
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
 ***************************************************************************/

#ifndef __PINSN_H
#define __PINSN_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <stdint.h>

/***************************************************************************
 * Global Function Prototypes
 ***************************************************************************/

/* Opcode generators */

extern void insn_GenerateSimple(enum pcode_e opcode);
extern void insn_GenerateDataOperation(enum pcode_e opcode, int32_t data);
extern void insn_GenerateDataSize(uint32_t dwDataSize);
extern void insn_GenerateFpOperation(uint8_t fpOpcode);
extern void insn_GenerateIoOperation(uint16_t ioOpcode, uint16_t fileNumber);
extern void insn_BuiltInFunctionCall(uint16_t libOpcode);
extern void insn_GenerateLevelReference(enum pcode_e opcode, uint16_t level,
                                        int32_t offset);
extern void insn_GenerateProcedureCall(uint16_t level, int32_t offset);
extern void insn_GenerateLineNumber(uint16_t includeNumber, uint32_t lineNumber);
extern void insn_SetStackLevel(uint32_t level);

/* Opcode relocation */

extern int  insn_Relocate(OPTYPE *op, uint32_t pcOffset, uint32_t roOffset);
extern void insn_FixupProcedureCall(uint8_t *progData, uint32_t symValue);

/* POFF-wrapped INSNS access helpers */

extern uint32_t insn_GetOpCode(poffHandle_t handle, OPTYPE *ptr);
extern void insn_ResetOpCodeRead(poffHandle_t handle);
extern void insn_AddOpCode(poffHandle_t handle, OPTYPE *ptr);
extern void insn_ResetOpCodeWrite(poffHandle_t handle);
extern void insn_AddTmpOpCode(poffProgHandle_t progHandle, OPTYPE *ptr);
extern void insn_ResetTmpOpCodeWrite(poffProgHandle_t progHandle);

/* INSN-specific disassembler */

extern void insn_DisassemblePCode(FILE* lfile, OPTYPE *pop);

#endif /* __PINSN_H */
