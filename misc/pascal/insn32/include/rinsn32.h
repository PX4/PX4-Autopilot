/***********************************************************************
 * rinsn32.h
 * 32-bit register module instruction definitions
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
 ***********************************************************************/

#ifndef __RINSN32_H
#define __RINSN32_H

/***********************************************************************
 * Included Files
 ***********************************************************************/

#include <stdint.h>

/***********************************************************************
 * Pre-processor Definitions
 ***********************************************************************/

/* 32-bit op-code bit definitions
 *
 * Machine model:
 *
 *   SPB 32-bit Pascal stack base address
 *   SP  32-bit Pascal stack pointer
 *   LSP 32-bit level stack pointer
 *   CSB 32-bit character stack base address
 *   PC  32-bit program counter
 *   ZRO 32-bit register containing zero (like MIPS)
 *   Rn  32-bit general purpose registers
 *
 * Parameters passed in registers up to a limit.  Parameters
 * beyond the limit are passed on the stack.  Return values
 * are provided in these same registers.
 *
 * Instruction forms
 *
 *   FORM 1: <op>          <roperand1>, <operand2>
 *   FORM 2: <op> <rdest>,              <operand2>
 *   FORM 3: <op> <rdest>, <roperand1>, <operand2>
 *                <rsrc>,  <roperand1>, <operand2>
 *   FORM 4: <op> <pc-offset>
 *
 *   Where
 *     <op>        operation code
 *     [cc]        optional conditional execution
 *     [S]         optional set condition code
 *     <rdest>     register holding the result
 *     <rsrc>      register holding data to be stored.
 *     <roperand1> register holding the first operand
 *     <operand2>  One of <roperand2> or <immediate>
 *     <roperand2> register holding the second operand
 *     <immediate> Immediate constant data (short).
 *     <pc-offset> Word offset from current PC
 *
 * NOTE: This instruction set is inspired by the ARM instruction
 * set, but is intended to be a general register model.  It does
 * not exploit certain ARM-isms such as "flexible" operands, conditional
 * execution, post-incrementing, pre-incrementing, pc-relative addressing,
 * optional register writeback, or ldm/stm with register sets, etc.
 */

#define FORM1(o)  (((o) & 0xe0) == 0x00)
#define FORM1R(o) (((o) & 0xf0) == 0x00)
#define FORM1I(o) (((o) & 0xf0) == 0x10)
#define FORM1O(o) ((o) & 0x0f)

#define FORM2(o)  (((o) & 0xe0) == 0x20)
#define FORM2R(o) (((o) & 0xf0) == 0x20)
#define FORM2I(o) (((o) & 0xf0) == 0x30)
#define FORM2O(o) ((o) & 0x0f)

#define FORM3(o)  (((o) & 0x80) == 0x80)
#define FORM3R(o) (((o) & 0xc0) == 0x80)
#define FORM3I(o) (((o) & 0xc0) == 0xc0)
#define FORM3O(o) ((o) & 0x3f)

#define FORM4(o)  (((o) & 0xc0) == 0x40)
#define FORM3O(o) ((o) & 0x3f)

struct rinsn_u
{
  uint8_t opcode;
  union {
    struct {
      uint32_t rop1;
      uint32_t rop2;
    } r1;
    struct {
      uint32_t rop1;
      uint32_t im2;
    } i1;
    struct {
      uint32_t rdest;
      uint32_t rop;
    } r2;
    struct {
      uint32_t rdest;
      uint32_t im;
    } i2;
    struct {
      uint32_t rdest;
      uint32_t rop1;
      uint32_t rop2;
    } r3;
    struct {
      uint32_t rdest;
      uint32_t rop1;
      uint32_t im2;
    } i3;
    struct {
      uint32_t offset;
    } i4;
  } f;
};
typedef struct rinsn_u RINSN32;

/* FORM 1: Comparisons */

#define rCMP   (0x00) /* Form 1: set <cc> after <roperand1> - <roperand2> */
#define rCMPI  (0x10) /* Form 1: set <cc> after <roperand1> - <immediate> */
#define rCMN   (0x01) /* Form 1: set <cc> after <roperand1> + <roperand2> */
#define rCMNI  (0x11) /* Form 1: set <cc> after <roperand1> + <immediate> */

/* FORM 2: Data movement */

#define rMOV   (0x20) /* Form 1: <rdest> = <roperand2> */
#define rMOVI  (0x30) /* Form 1: <rdist> = <immediate> */
#define rMVN   (0x21) /* Form 1: <rdest> = ~<roperand2> */
#define rMVNI  (0x31) /* Form 1: <rdist> = ~<immediate> */

/* FORM 4: Program control */

#define rB     (0x40) /*               PC += <pcoffset> << 2 */
#define rBEQ   (0x41) /* if <cc>->EQ,  PC += <pcoffset> << 2 */
#define rBNE   (0x42) /* if <cc>->NEQ, PC += <pcoffset> << 2 */
#define rBLT   (0x43) /* if <cc>->LT,  PC += <pcoffset> << 2 */
#define rBGTE  (0x44) /* if <cc>->GTE, PC += <pcoffset> << 2 */
#define rBGT   (0x45) /* if <cc>->GT,  PC += <pcoffset> << 2 */
#define rBLTE  (0x46) /* if <cc>->KTE, PC += <pcoffset> << 2 */
#define rBL    (0x47) /* LR=next PC,   PC += <pcoffset> << 2 */

/* FORM 3: Arithmetic and logical operations */

#define rADD   (0x80) /* Form 3: <rdest> =  <roperand1> + <roperand2> */
#define rADDI  (0xc0) /* Form 3: <rdest> =  <roperand1> + <immediate> */
#define rSUB   (0x81) /* Form 3: <rdest> =  <roperand1> - <roperand2> */
#define rSUBI  (0xc1) /* Form 3: <rdest> =  <roperand1> - <immediate> */
#define rRSB   (0x82) /* Form 3: <rdest> = -<roperand1> + <roperand2> */
#define rRSBI  (0xc2) /* Form 3: <rdest> = -<roperand2> + <immediate> */
#define rMUL   (0x83) /* Form 3: <rdest> =  <roperand1> * <roperand2> */
#define rMULI  (0xc3) /* Form 3: <rdest> =  <roperand1> * <immediate> */
#define rDIV   (0x84) /* Form 3: <rdest> =  <roperand1> / <roperand2> */
#define rDIVI  (0xc4) /* Form 3: <rdest> =  <roperand1> / <immediate> */
#define rMOD   (0x85) /* Form 3: <rdest> =  <roperand1> % <roperand2> */
#define rMODI  (0xc5) /* Form 3: <rdest> =  <roperand1> % <immediate> */
#define rSLL   (0x86) /* Form 3: <rdest> =  <roperand1> << <roperand2> */
#define rSLLI  (0xc6) /* Form 3: <rdest> =  <roperand1> << <immediate> */
#define rSRL   (0x87) /* Form 3: <rdest> =  <roperand1> >> <roperand2> */
#define rSRLI  (0xc7) /* Form 3: <rdest> =  <roperand1> >> <immediate> */
#define rSRA   (0x88) /* Form 3: <rdest> =  <roperand1> >> <roperand2> */
#define rSRAI  (0xc8) /* Form 3: <rdest> =  <roperand1> >> <immediate> */
#define rOR    (0x89) /* Form 3: <rdest> =  <roperand1> | <roperand2> */
#define rORI   (0xc9) /* Form 3: <rdest> =  <roperand1> | <immediate> */
#define rAND   (0x8a) /* Form 3: <rdest> =  <roperand1> & <roperand2> */
#define rANDI  (0xca) /* Form 3: <rdest> =  <roperand1> & <immediate> */
#define rXOR   (0x8b) /* Form 3: <rdest> =  <roperand1> xor <roperand2> */
#define rXORI  (0xcb) /* Form 3: <rdest> =  <roperand1> xor <immediate> */
#define rANDN  (0x8c) /* Form 3: <rdest> =  <roperand1> & ~<roperand2> */
#define rANDNI (0xcc) /* Form 3: <rdest> =  <roperand1> & ~<immediate> */

/* FORM 3: Loads and stores */

#define rLD    (0x90) /* Form 3: <rdest> = (<roperand1> + <roperand2>) */
#define rLDI   (0xd0) /* Form 3: <rdest> = (<roperand1> + (<immediate> << 2)) */
#define rLDH   (0x91) /* Form 3: <rdest> = (<roperand1> + <roperand2>) */
#define rLDIH  (0xd1) /* Form 3: <rdest> = (<roperand1> + (<immediate> << 1)) */
#define rLDB   (0x92) /* Form 3: <rdest> = (<roperand1> + <roperand2>) */
#define rLDIB  (0xd2) /* Form 3: <rdest> = (<roperand1> + <immediate>) */

#define rLDM   (0xd3) /* Form 3: Load <immediate> registers.  Source
                       * address is roperand1, first dest register is
                       * <rdest>, register count is <immediate> */

#define rST    (0x94) /* Form 3: (<roperand1> + <roperand2>) = <rsrc> */
#define rSTI   (0xd4) /* Form 3: (<roperand1> + (<immediate> << 2)) = <rsrc> */
#define rSTH   (0x95) /* Form 3: (<roperand1> + <roperand2>) = <rsrc> */
#define rSTIH  (0xd5) /* Form 3: (<roperand1> + (<immediate> << 1)) = <rsrc> */
#define rSTB   (0x96) /* Form 3: (<roperand1> + <roperand2>) = <rsrc> */
#define rSTIB  (0xd6) /* Form 3: (<roperand1> + <immediate>) = <rsrc> */

#define rSTM   (0xd7) /* Form 3: Store <immediate> registers.  Destination
                       * address is roperand1, first source register is
                       * <rsrc>, register count is <immediate> */

#endif /* __RINSN32_H */
