/***********************************************************************
 * toolchain/nxflat/arm/disarm.c
 * ARM Disassembler
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derived from XFLAT:
 *
 *   Copyright (c) 2006, Cadenux, LLC.  All rights reserved.
 *   Copyright (c) 2006, Gregory Nutt.  All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Which simply lifted it from the BFD:
 *
 * Copyright 1998, 1999, 2000, 2001, 2002 Free Software Foundation, Inc.
 * This file is derived from parts of of the binutils package
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 ***********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>

#define BDISP(x) ((((x) & 0xffffff) ^ 0x800000) - 0x800000) /* 26 bit */

struct arm_opcode
{
  u_int32_t value;
  u_int32_t mask;
  char *assembler;
};

static char * arm_conditional[] =
{"eq", "ne", "cs", "cc", "mi", "pl", "vs", "vc",
 "hi", "ls", "ge", "lt", "gt", "le", "", "nv"};

typedef struct
{
  const char *name;
  const char *description;
  const char *reg_names[16];
}
arm_regname;

static arm_regname regnames[] =
{
  { "raw" , "Select raw register names",
    { "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r13", "r14", "r15"}},
  { "gcc",  "Select register names used by GCC",
    { "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "sl",  "fp",  "ip",  "sp",  "lr",  "pc" }},
  { "std",  "Select register names used in ARM's ISA documentation",
    { "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "sp",  "lr",  "pc" }},
  { "apcs", "Select register names used in the APCS",
    { "a1", "a2", "a3", "a4", "v1", "v2", "v3", "v4", "v5", "v6", "sl",  "fp",  "ip",  "sp",  "lr",  "pc" }},
  { "atpcs", "Select register names used in the ATPCS",
    { "a1", "a2", "a3", "a4", "v1", "v2", "v3", "v4", "v5", "v6", "v7",  "v8",  "IP",  "SP",  "LR",  "PC" }},
  { "special-atpcs", "Select special register names used in the ATPCS",
    { "a1", "a2", "a3", "a4", "v1", "v2", "v3", "WR", "v5", "SB", "SL",  "FP",  "IP",  "SP",  "LR",  "PC" }}
};

static unsigned int regname_selected = 1;

#define NUM_ARM_REGNAMES  NUM_ELEM (regnames)
#define arm_regnames      regnames[regname_selected].reg_names

static char * arm_fp_const[] =
{"0.0", "1.0", "2.0", "3.0", "4.0", "5.0", "0.5", "10.0"};

static char * arm_shift[] = 
{"lsl", "lsr", "asr", "ror"};

static struct arm_opcode arm_opcodes[] =
  {
    /* ARM instructions.  */
    {0xe1a00000, 0xffffffff, "nop\t\t\t(mov r0,r0)"},
    {0x012FFF10, 0x0ffffff0, "bx%c\t%0-3r"},
    {0x00000090, 0x0fe000f0, "mul%c%20's\t%16-19r, %0-3r, %8-11r"},
    {0x00200090, 0x0fe000f0, "mla%c%20's\t%16-19r, %0-3r, %8-11r, %12-15r"},
    {0x01000090, 0x0fb00ff0, "swp%c%22'b\t%12-15r, %0-3r, [%16-19r]"},
    {0x00800090, 0x0fa000f0, "%22?sumull%c%20's\t%12-15r, %16-19r, %0-3r, %8-11r"},
    {0x00a00090, 0x0fa000f0, "%22?sumlal%c%20's\t%12-15r, %16-19r, %0-3r, %8-11r"},

    /* V5J instruction.  */
    {0x012fff20, 0x0ffffff0, "bxj%c\t%0-3r"},

    /* XScale instructions.  */
    {0x0e200010, 0x0fff0ff0, "mia%c\tacc0, %0-3r, %12-15r"},
    {0x0e280010, 0x0fff0ff0, "miaph%c\tacc0, %0-3r, %12-15r"},
    {0x0e2c0010, 0x0ffc0ff0, "mia%17'T%17`B%16'T%16`B%c\tacc0, %0-3r, %12-15r"},
    {0x0c400000, 0x0ff00fff, "mar%c\tacc0, %12-15r, %16-19r"},
    {0x0c500000, 0x0ff00fff, "mra%c\t%12-15r, %16-19r, acc0"},
    {0xf450f000, 0xfc70f000, "pld\t%a"},
    
    /* V5 Instructions.  */
    {0xe1200070, 0xfff000f0, "bkpt\t0x%16-19X%12-15X%8-11X%0-3X"},
    {0xfa000000, 0xfe000000, "blx\t%B"},
    {0x012fff30, 0x0ffffff0, "blx%c\t%0-3r"},
    {0x016f0f10, 0x0fff0ff0, "clz%c\t%12-15r, %0-3r"},
    {0xfc100000, 0xfe100000, "ldc2%22'l\t%8-11d, cr%12-15d, %A"},
    {0xfc000000, 0xfe100000, "stc2%22'l\t%8-11d, cr%12-15d, %A"},
    {0xfe000000, 0xff000010, "cdp2\t%8-11d, %20-23d, cr%12-15d, cr%16-19d, cr%0-3d, {%5-7d}"},
    {0xfe000010, 0xff100010, "mcr2\t%8-11d, %21-23d, %12-15r, cr%16-19d, cr%0-3d, {%5-7d}"},
    {0xfe100010, 0xff100010, "mrc2\t%8-11d, %21-23d, %12-15r, cr%16-19d, cr%0-3d, {%5-7d}"},

    /* V5E "El Segundo" Instructions.  */    
    {0x000000d0, 0x0e1000f0, "ldr%cd\t%12-15r, %s"},
    {0x000000f0, 0x0e1000f0, "str%cd\t%12-15r, %s"},
    {0x01000080, 0x0ff000f0, "smlabb%c\t%16-19r, %0-3r, %8-11r, %12-15r"},
    {0x010000a0, 0x0ff000f0, "smlatb%c\t%16-19r, %0-3r, %8-11r, %12-15r"},
    {0x010000c0, 0x0ff000f0, "smlabt%c\t%16-19r, %0-3r, %8-11r, %12-15r"},
    {0x010000e0, 0x0ff000f0, "smlatt%c\t%16-19r, %0-3r, %8-11r, %12-15r"},

    {0x01200080, 0x0ff000f0, "smlawb%c\t%16-19r, %0-3r, %8-11r, %12-15r"},
    {0x012000c0, 0x0ff000f0, "smlawt%c\t%16-19r, %0-3r, %8-11r, %12-15r"},

    {0x01400080, 0x0ff000f0, "smlalbb%c\t%12-15r, %16-19r, %0-3r, %8-11r"},
    {0x014000a0, 0x0ff000f0, "smlaltb%c\t%12-15r, %16-19r, %0-3r, %8-11r"},
    {0x014000c0, 0x0ff000f0, "smlalbt%c\t%12-15r, %16-19r, %0-3r, %8-11r"},
    {0x014000e0, 0x0ff000f0, "smlaltt%c\t%12-15r, %16-19r, %0-3r, %8-11r"},

    {0x01600080, 0x0ff0f0f0, "smulbb%c\t%16-19r, %0-3r, %8-11r"},
    {0x016000a0, 0x0ff0f0f0, "smultb%c\t%16-19r, %0-3r, %8-11r"},
    {0x016000c0, 0x0ff0f0f0, "smulbt%c\t%16-19r, %0-3r, %8-11r"},
    {0x016000e0, 0x0ff0f0f0, "smultt%c\t%16-19r, %0-3r, %8-11r"},

    {0x012000a0, 0x0ff0f0f0, "smulwb%c\t%16-19r, %0-3r, %8-11r"},
    {0x012000e0, 0x0ff0f0f0, "smulwt%c\t%16-19r, %0-3r, %8-11r"},

    {0x01000050, 0x0ff00ff0,  "qadd%c\t%12-15r, %0-3r, %16-19r"},
    {0x01400050, 0x0ff00ff0, "qdadd%c\t%12-15r, %0-3r, %16-19r"},
    {0x01200050, 0x0ff00ff0,  "qsub%c\t%12-15r, %0-3r, %16-19r"},
    {0x01600050, 0x0ff00ff0, "qdsub%c\t%12-15r, %0-3r, %16-19r"},

    {0x0c400000, 0x0ff00000, "mcrr%c\t%8-11d, %4-7d, %12-15r, %16-19r, cr%0-3d"},
    {0x0c500000, 0x0ff00000, "mrrc%c\t%8-11d, %4-7d, %12-15r, %16-19r, cr%0-3d"},

    /* ARM Instructions.  */
    {0x00000090, 0x0e100090, "str%c%6's%5?hb\t%12-15r, %s"},
    {0x00100090, 0x0e100090, "ldr%c%6's%5?hb\t%12-15r, %s"},
    {0x00000000, 0x0de00000, "and%c%20's\t%12-15r, %16-19r, %o"},
    {0x00200000, 0x0de00000, "eor%c%20's\t%12-15r, %16-19r, %o"},
    {0x00400000, 0x0de00000, "sub%c%20's\t%12-15r, %16-19r, %o"},
    {0x00600000, 0x0de00000, "rsb%c%20's\t%12-15r, %16-19r, %o"},
    {0x00800000, 0x0de00000, "add%c%20's\t%12-15r, %16-19r, %o"},
    {0x00a00000, 0x0de00000, "adc%c%20's\t%12-15r, %16-19r, %o"},
    {0x00c00000, 0x0de00000, "sbc%c%20's\t%12-15r, %16-19r, %o"},
    {0x00e00000, 0x0de00000, "rsc%c%20's\t%12-15r, %16-19r, %o"},
    {0x0120f000, 0x0db0f000, "msr%c\t%22?SCPSR%C, %o"},
    {0x010f0000, 0x0fbf0fff, "mrs%c\t%12-15r, %22?SCPSR"},
    {0x01000000, 0x0de00000, "tst%c%p\t%16-19r, %o"},
    {0x01200000, 0x0de00000, "teq%c%p\t%16-19r, %o"},
    {0x01400000, 0x0de00000, "cmp%c%p\t%16-19r, %o"},
    {0x01600000, 0x0de00000, "cmn%c%p\t%16-19r, %o"},
    {0x01800000, 0x0de00000, "orr%c%20's\t%12-15r, %16-19r, %o"},
    {0x01a00000, 0x0de00000, "mov%c%20's\t%12-15r, %o"},
    {0x01c00000, 0x0de00000, "bic%c%20's\t%12-15r, %16-19r, %o"},
    {0x01e00000, 0x0de00000, "mvn%c%20's\t%12-15r, %o"},
    {0x04000000, 0x0e100000, "str%c%22'b%t\t%12-15r, %a"},
    {0x06000000, 0x0e100ff0, "str%c%22'b%t\t%12-15r, %a"},
    {0x04000000, 0x0c100010, "str%c%22'b%t\t%12-15r, %a"},
    {0x06000010, 0x0e000010, "undefined"},
    {0x04100000, 0x0c100000, "ldr%c%22'b%t\t%12-15r, %a"},
    {0x08000000, 0x0e100000, "stm%c%23?id%24?ba\t%16-19r%21'!, %m%22'^"},
    {0x08100000, 0x0e100000, "ldm%c%23?id%24?ba\t%16-19r%21'!, %m%22'^"},
    {0x0a000000, 0x0e000000, "b%24'l%c\t%b"},
    {0x0f000000, 0x0f000000, "swi%c\t%0-23x"},

    /* Floating point coprocessor (FPA) instructions */
    {0x0e000100, 0x0ff08f10, "adf%c%P%R\t%12-14f, %16-18f, %0-3f"},
    {0x0e100100, 0x0ff08f10, "muf%c%P%R\t%12-14f, %16-18f, %0-3f"},
    {0x0e200100, 0x0ff08f10, "suf%c%P%R\t%12-14f, %16-18f, %0-3f"},
    {0x0e300100, 0x0ff08f10, "rsf%c%P%R\t%12-14f, %16-18f, %0-3f"},
    {0x0e400100, 0x0ff08f10, "dvf%c%P%R\t%12-14f, %16-18f, %0-3f"},
    {0x0e500100, 0x0ff08f10, "rdf%c%P%R\t%12-14f, %16-18f, %0-3f"},
    {0x0e600100, 0x0ff08f10, "pow%c%P%R\t%12-14f, %16-18f, %0-3f"},
    {0x0e700100, 0x0ff08f10, "rpw%c%P%R\t%12-14f, %16-18f, %0-3f"},
    {0x0e800100, 0x0ff08f10, "rmf%c%P%R\t%12-14f, %16-18f, %0-3f"},
    {0x0e900100, 0x0ff08f10, "fml%c%P%R\t%12-14f, %16-18f, %0-3f"},
    {0x0ea00100, 0x0ff08f10, "fdv%c%P%R\t%12-14f, %16-18f, %0-3f"},
    {0x0eb00100, 0x0ff08f10, "frd%c%P%R\t%12-14f, %16-18f, %0-3f"},
    {0x0ec00100, 0x0ff08f10, "pol%c%P%R\t%12-14f, %16-18f, %0-3f"},
    {0x0e008100, 0x0ff08f10, "mvf%c%P%R\t%12-14f, %0-3f"},
    {0x0e108100, 0x0ff08f10, "mnf%c%P%R\t%12-14f, %0-3f"},
    {0x0e208100, 0x0ff08f10, "abs%c%P%R\t%12-14f, %0-3f"},
    {0x0e308100, 0x0ff08f10, "rnd%c%P%R\t%12-14f, %0-3f"},
    {0x0e408100, 0x0ff08f10, "sqt%c%P%R\t%12-14f, %0-3f"},
    {0x0e508100, 0x0ff08f10, "log%c%P%R\t%12-14f, %0-3f"},
    {0x0e608100, 0x0ff08f10, "lgn%c%P%R\t%12-14f, %0-3f"},
    {0x0e708100, 0x0ff08f10, "exp%c%P%R\t%12-14f, %0-3f"},
    {0x0e808100, 0x0ff08f10, "sin%c%P%R\t%12-14f, %0-3f"},
    {0x0e908100, 0x0ff08f10, "cos%c%P%R\t%12-14f, %0-3f"},
    {0x0ea08100, 0x0ff08f10, "tan%c%P%R\t%12-14f, %0-3f"},
    {0x0eb08100, 0x0ff08f10, "asn%c%P%R\t%12-14f, %0-3f"},
    {0x0ec08100, 0x0ff08f10, "acs%c%P%R\t%12-14f, %0-3f"},
    {0x0ed08100, 0x0ff08f10, "atn%c%P%R\t%12-14f, %0-3f"},
    {0x0ee08100, 0x0ff08f10, "urd%c%P%R\t%12-14f, %0-3f"},
    {0x0ef08100, 0x0ff08f10, "nrm%c%P%R\t%12-14f, %0-3f"},
    {0x0e000110, 0x0ff00f1f, "flt%c%P%R\t%16-18f, %12-15r"},
    {0x0e100110, 0x0fff0f98, "fix%c%R\t%12-15r, %0-2f"},
    {0x0e200110, 0x0fff0fff, "wfs%c\t%12-15r"},
    {0x0e300110, 0x0fff0fff, "rfs%c\t%12-15r"},
    {0x0e400110, 0x0fff0fff, "wfc%c\t%12-15r"},
    {0x0e500110, 0x0fff0fff, "rfc%c\t%12-15r"},
    {0x0e90f110, 0x0ff8fff0, "cmf%c\t%16-18f, %0-3f"},
    {0x0eb0f110, 0x0ff8fff0, "cnf%c\t%16-18f, %0-3f"},
    {0x0ed0f110, 0x0ff8fff0, "cmfe%c\t%16-18f, %0-3f"},
    {0x0ef0f110, 0x0ff8fff0, "cnfe%c\t%16-18f, %0-3f"},
    {0x0c000100, 0x0e100f00, "stf%c%Q\t%12-14f, %A"},
    {0x0c100100, 0x0e100f00, "ldf%c%Q\t%12-14f, %A"},
    {0x0c000200, 0x0e100f00, "sfm%c\t%12-14f, %F, %A"},
    {0x0c100200, 0x0e100f00, "lfm%c\t%12-14f, %F, %A"},

    /* Floating point coprocessor (VFP) instructions */
    {0x0eb00bc0, 0x0fff0ff0, "fabsd%c\t%1z, %0z"},
    {0x0eb00ac0, 0x0fbf0fd0, "fabss%c\t%1y, %0y"},
    {0x0e300b00, 0x0ff00ff0, "faddd%c\t%1z, %2z, %0z"},
    {0x0e300a00, 0x0fb00f50, "fadds%c\t%1y, %2y, %1y"},
    {0x0eb40b40, 0x0fff0f70, "fcmp%7'ed%c\t%1z, %0z"},
    {0x0eb40a40, 0x0fbf0f50, "fcmp%7'es%c\t%1y, %0y"},
    {0x0eb50b40, 0x0fff0f70, "fcmp%7'ezd%c\t%1z"},
    {0x0eb50a40, 0x0fbf0f70, "fcmp%7'ezs%c\t%1y"},
    {0x0eb00b40, 0x0fff0ff0, "fcpyd%c\t%1z, %0z"},
    {0x0eb00a40, 0x0fbf0fd0, "fcpys%c\t%1y, %0y"},
    {0x0eb70ac0, 0x0fff0fd0, "fcvtds%c\t%1z, %0y"},
    {0x0eb70bc0, 0x0fbf0ff0, "fcvtsd%c\t%1y, %0z"},
    {0x0e800b00, 0x0ff00ff0, "fdivd%c\t%1z, %2z, %0z"},
    {0x0e800a00, 0x0fb00f50, "fdivs%c\t%1y, %2y, %0y"},
    {0x0d100b00, 0x0f700f00, "fldd%c\t%1z, %A"},
    {0x0c900b00, 0x0fd00f00, "fldmia%0?xd%c\t%16-19r%21'!, %3z"},
    {0x0d300b00, 0x0ff00f00, "fldmdb%0?xd%c\t%16-19r!, %3z"},
    {0x0d100a00, 0x0f300f00, "flds%c\t%1y, %A"},
    {0x0c900a00, 0x0f900f00, "fldmias%c\t%16-19r%21'!, %3y"},
    {0x0d300a00, 0x0fb00f00, "fldmdbs%c\t%16-19r!, %3y"},
    {0x0e000b00, 0x0ff00ff0, "fmacd%c\t%1z, %2z, %0z"},
    {0x0e000a00, 0x0fb00f50, "fmacs%c\t%1y, %2y, %0y"},
    {0x0e200b10, 0x0ff00fff, "fmdhr%c\t%2z, %12-15r"},
    {0x0e000b10, 0x0ff00fff, "fmdlr%c\t%2z, %12-15r"},
    {0x0c400b10, 0x0ff00ff0, "fmdrr%c\t%0z, %12-15r, %16-19r"},
    {0x0e300b10, 0x0ff00fff, "fmrdh%c\t%12-15r, %2z"},
    {0x0e100b10, 0x0ff00fff, "fmrdl%c\t%12-15r, %2z"},
    {0x0c500b10, 0x0ff00ff0, "fmrrd%c\t%12-15r, %16-19r, %0z"},
    {0x0c500a10, 0x0ff00fd0, "fmrrs%c\t%12-15r, %16-19r, %4y"},
    {0x0e100a10, 0x0ff00f7f, "fmrs%c\t%12-15r, %2y"},
    {0x0ef1fa10, 0x0fffffff, "fmstat%c"},
    {0x0ef00a10, 0x0fff0fff, "fmrx%c\t%12-15r, fpsid"},
    {0x0ef10a10, 0x0fff0fff, "fmrx%c\t%12-15r, fpscr"},
    {0x0ef80a10, 0x0fff0fff, "fmrx%c\t%12-15r, fpexc"},
    {0x0ef90a10, 0x0fff0fff, "fmrx%c\t%12-15r, fpinst\t@ Impl def"},
    {0x0efa0a10, 0x0fff0fff, "fmrx%c\t%12-15r, fpinst2\t@ Impl def"},
    {0x0ef00a10, 0x0ff00fff, "fmrx%c\t%12-15r, <impl def 0x%16-19x>"},
    {0x0e100b00, 0x0ff00ff0, "fmscd%c\t%1z, %2z, %0z"},
    {0x0e100a00, 0x0fb00f50, "fmscs%c\t%1y, %2y, %0y"},
    {0x0e000a10, 0x0ff00f7f, "fmsr%c\t%2y, %12-15r"},
    {0x0c400a10, 0x0ff00fd0, "fmsrr%c\t%12-15r, %16-19r, %4y"},
    {0x0e200b00, 0x0ff00ff0, "fmuld%c\t%1z, %2z, %0z"},
    {0x0e200a00, 0x0fb00f50, "fmuls%c\t%1y, %2y, %0y"},
    {0x0ee00a10, 0x0fff0fff, "fmxr%c\tfpsid, %12-15r"},
    {0x0ee10a10, 0x0fff0fff, "fmxr%c\tfpscr, %12-15r"},
    {0x0ee80a10, 0x0fff0fff, "fmxr%c\tfpexc, %12-15r"},
    {0x0ee90a10, 0x0fff0fff, "fmxr%c\tfpinst, %12-15r\t@ Impl def"},
    {0x0eea0a10, 0x0fff0fff, "fmxr%c\tfpinst2, %12-15r\t@ Impl def"},
    {0x0ee00a10, 0x0ff00fff, "fmxr%c\t<impl def 0x%16-19x>, %12-15r"},
    {0x0eb10b40, 0x0fff0ff0, "fnegd%c\t%1z, %0z"},
    {0x0eb10a40, 0x0fbf0fd0, "fnegs%c\t%1y, %0y"},
    {0x0e000b40, 0x0ff00ff0, "fnmacd%c\t%1z, %2z, %0z"},
    {0x0e000a40, 0x0fb00f50, "fnmacs%c\t%1y, %2y, %0y"},
    {0x0e100b40, 0x0ff00ff0, "fnmscd%c\t%1z, %2z, %0z"},
    {0x0e100a40, 0x0fb00f50, "fnmscs%c\t%1y, %2y, %0y"},
    {0x0e200b40, 0x0ff00ff0, "fnmuld%c\t%1z, %2z, %0z"},
    {0x0e200a40, 0x0fb00f50, "fnmuls%c\t%1y, %2y, %0y"},
    {0x0eb80bc0, 0x0fff0fd0, "fsitod%c\t%1z, %0y"},
    {0x0eb80ac0, 0x0fbf0fd0, "fsitos%c\t%1y, %0y"},
    {0x0eb10bc0, 0x0fff0ff0, "fsqrtd%c\t%1z, %0z"},
    {0x0eb10ac0, 0x0fbf0fd0, "fsqrts%c\t%1y, %0y"},
    {0x0d000b00, 0x0f700f00, "fstd%c\t%1z, %A"},
    {0x0c800b00, 0x0fd00f00, "fstmia%0?xd%c\t%16-19r%21'!, %3z"},
    {0x0d200b00, 0x0ff00f00, "fstmdb%0?xd%c\t%16-19r!, %3z"},
    {0x0d000a00, 0x0f300f00, "fsts%c\t%1y, %A"},
    {0x0c800a00, 0x0f900f00, "fstmias%c\t%16-19r%21'!, %3y"},
    {0x0d200a00, 0x0fb00f00, "fstmdbs%c\t%16-19r!, %3y"},
    {0x0e300b40, 0x0ff00ff0, "fsubd%c\t%1z, %2z, %0z"},
    {0x0e300a40, 0x0fb00f50, "fsubs%c\t%1y, %2y, %0y"},
    {0x0ebc0b40, 0x0fbe0f70, "fto%16?sui%7'zd%c\t%1y, %0z"},
    {0x0ebc0a40, 0x0fbe0f50, "fto%16?sui%7'zs%c\t%1y, %0y"},
    {0x0eb80b40, 0x0fff0fd0, "fuitod%c\t%1z, %0y"},
    {0x0eb80a40, 0x0fbf0fd0, "fuitos%c\t%1y, %0y"},

    /* Cirrus coprocessor instructions.  */
    {0x0d100400, 0x0f500f00, "cfldrs%c\tmvf%12-15d, %A"},
    {0x0c100400, 0x0f500f00, "cfldrs%c\tmvf%12-15d, %A"},
    {0x0d500400, 0x0f500f00, "cfldrd%c\tmvd%12-15d, %A"},
    {0x0c500400, 0x0f500f00, "cfldrd%c\tmvd%12-15d, %A"}, 
    {0x0d100500, 0x0f500f00, "cfldr32%c\tmvfx%12-15d, %A"},
    {0x0c100500, 0x0f500f00, "cfldr32%c\tmvfx%12-15d, %A"},
    {0x0d500500, 0x0f500f00, "cfldr64%c\tmvdx%12-15d, %A"},
    {0x0c500500, 0x0f500f00, "cfldr64%c\tmvdx%12-15d, %A"},
    {0x0d000400, 0x0f500f00, "cfstrs%c\tmvf%12-15d, %A"},
    {0x0c000400, 0x0f500f00, "cfstrs%c\tmvf%12-15d, %A"},
    {0x0d400400, 0x0f500f00, "cfstrd%c\tmvd%12-15d, %A"},
    {0x0c400400, 0x0f500f00, "cfstrd%c\tmvd%12-15d, %A"},
    {0x0d000500, 0x0f500f00, "cfstr32%c\tmvfx%12-15d, %A"},
    {0x0c000500, 0x0f500f00, "cfstr32%c\tmvfx%12-15d, %A"},
    {0x0d400500, 0x0f500f00, "cfstr64%c\tmvdx%12-15d, %A"},
    {0x0c400500, 0x0f500f00, "cfstr64%c\tmvdx%12-15d, %A"},
    {0x0e000450, 0x0ff00ff0, "cfmvsr%c\tmvf%16-19d, %12-15r"},
    {0x0e100450, 0x0ff00ff0, "cfmvrs%c\t%12-15r, mvf%16-19d"},
    {0x0e000410, 0x0ff00ff0, "cfmvdlr%c\tmvd%16-19d, %12-15r"},
    {0x0e100410, 0x0ff00ff0, "cfmvrdl%c\t%12-15r, mvd%16-19d"},
    {0x0e000430, 0x0ff00ff0, "cfmvdhr%c\tmvd%16-19d, %12-15r"},
    {0x0e100430, 0x0ff00fff, "cfmvrdh%c\t%12-15r, mvd%16-19d"},
    {0x0e000510, 0x0ff00fff, "cfmv64lr%c\tmvdx%16-19d, %12-15r"},
    {0x0e100510, 0x0ff00fff, "cfmvr64l%c\t%12-15r, mvdx%16-19d"},
    {0x0e000530, 0x0ff00fff, "cfmv64hr%c\tmvdx%16-19d, %12-15r"},
    {0x0e100530, 0x0ff00fff, "cfmvr64h%c\t%12-15r, mvdx%16-19d"},
    {0x0e100610, 0x0ff0fff0, "cfmval32%c\tmvax%0-3d, mvfx%16-19d"},
    {0x0e000610, 0x0ff0fff0, "cfmv32al%c\tmvfx%0-3d, mvax%16-19d"},
    {0x0e100630, 0x0ff0fff0, "cfmvam32%c\tmvax%0-3d, mvfx%16-19d"},
    {0x0e000630, 0x0ff0fff0, "cfmv32am%c\tmvfx%0-3d, mvax%16-19d"},
    {0x0e100650, 0x0ff0fff0, "cfmvah32%c\tmvax%0-3d, mvfx%16-19d"},
    {0x0e000650, 0x0ff0fff0, "cfmv32ah%c\tmvfx%0-3d, mvax%16-19d"},
    {0x0e000670, 0x0ff0fff0, "cfmv32a%c\tmvfx%0-3d, mvax%16-19d"},
    {0x0e100670, 0x0ff0fff0, "cfmva32%c\tmvax%0-3d, mvfx%16-19d"},
    {0x0e000690, 0x0ff0fff0, "cfmv64a%c\tmvdx%0-3d, mvax%16-19d"},
    {0x0e100690, 0x0ff0fff0, "cfmva64%c\tmvax%0-3d, mvdx%16-19d"},
    {0x0e1006b0, 0x0ff0fff0, "cfmvsc32%c\tdspsc, mvfx%16-19d"},
    {0x0e0006b0, 0x0ff0fff0, "cfmv32sc%c\tmvfx%0-3d, dspsc"},
    {0x0e000400, 0x0ff00fff, "cfcpys%c\tmvf%12-15d, mvf%16-19d"},
    {0x0e000420, 0x0ff00fff, "cfcpyd%c\tmvd%12-15d, mvd%16-19d"},
    {0x0e000460, 0x0ff00fff, "cfcvtsd%c\tmvd%12-15d, mvf%16-19d"},
    {0x0e000440, 0x0ff00fff, "cfcvtds%c\tmvf%12-15d, mvd%16-19d"},
    {0x0e000480, 0x0ff00fff, "cfcvt32s%c\tmvf%12-15d, mvfx%16-19d"},
    {0x0e0004a0, 0x0ff00fff, "cfcvt32d%c\tmvd%12-15d, mvfx%16-19d"},
    {0x0e0004c0, 0x0ff00fff, "cfcvt64s%c\tmvf%12-15d, mvdx%16-19d"},
    {0x0e0004e0, 0x0ff00fff, "cfcvt64d%c\tmvd%12-15d, mvdx%16-19d"},
    {0x0e100580, 0x0ff00fff, "cfcvts32%c\tmvfx%12-15d, mvf%16-19d"},
    {0x0e1005a0, 0x0ff00fff, "cfcvtd32%c\tmvfx%12-15d, mvd%16-19d"},
    {0x0e1005c0, 0x0ff00fff, "cftruncs32%c\tmvfx%12-15d, mvf%16-19d"},
    {0x0e1005e0, 0x0ff00fff, "cftruncd32%c\tmvfx%12-15d, mvd%16-19d"},
    {0x0e000550, 0x0ff00ff0, "cfrshl32%c\tmvfx%16-19d, mvfx%0-3d, %12-15r"},
    {0x0e000570, 0x0ff00ff0, "cfrshl64%c\tmvdx%16-19d, mvdx%0-3d, %12-15r"},
    {0x0e000500, 0x0ff00f00, "cfsh32%c\tmvfx%12-15d, mvfx%16-19d, #%I"},
    {0x0e200500, 0x0ff00f00, "cfsh64%c\tmvdx%12-15d, mvdx%16-19d, #%I"},
    {0x0e100490, 0x0ff00ff0, "cfcmps%c\t%12-15r, mvf%16-19d, mvf%0-3d"},
    {0x0e1004b0, 0x0ff00ff0, "cfcmpd%c\t%12-15r, mvd%16-19d, mvd%0-3d"},
    {0x0e100590, 0x0ff00ff0, "cfcmp32%c\t%12-15r, mvfx%16-19d, mvfx%0-3d"},
    {0x0e1005b0, 0x0ff00ff0, "cfcmp64%c\t%12-15r, mvdx%16-19d, mvdx%0-3d"},
    {0x0e300400, 0x0ff00fff, "cfabss%c\tmvf%12-15d, mvf%16-19d"},
    {0x0e300420, 0x0ff00fff, "cfabsd%c\tmvd%12-15d, mvd%16-19d"},
    {0x0e300440, 0x0ff00fff, "cfnegs%c\tmvf%12-15d, mvf%16-19d"},
    {0x0e300460, 0x0ff00fff, "cfnegd%c\tmvd%12-15d, mvd%16-19d"},
    {0x0e300480, 0x0ff00ff0, "cfadds%c\tmvf%12-15d, mvf%16-19d, mvf%0-3d"},
    {0x0e3004a0, 0x0ff00ff0, "cfaddd%c\tmvd%12-15d, mvd%16-19d, mvd%0-3d"},
    {0x0e3004c0, 0x0ff00ff0, "cfsubs%c\tmvf%12-15d, mvf%16-19d, mvf%0-3d"},
    {0x0e3004e0, 0x0ff00ff0, "cfsubd%c\tmvd%12-15d, mvd%16-19d, mvd%0-3d"},
    {0x0e100400, 0x0ff00ff0, "cfmuls%c\tmvf%12-15d, mvf%16-19d, mvf%0-3d"},
    {0x0e100420, 0x0ff00ff0, "cfmuld%c\tmvd%12-15d, mvd%16-19d, mvd%0-3d"},
    {0x0e300500, 0x0ff00fff, "cfabs32%c\tmvfx%12-15d, mvfx%16-19d"},
    {0x0e300520, 0x0ff00fff, "cfabs64%c\tmvdx%12-15d, mvdx%16-19d"},
    {0x0e300540, 0x0ff00fff, "cfneg32%c\tmvfx%12-15d, mvfx%16-19d"},
    {0x0e300560, 0x0ff00fff, "cfneg64%c\tmvdx%12-15d, mvdx%16-19d"},
    {0x0e300580, 0x0ff00ff0, "cfadd32%c\tmvfx%12-15d, mvfx%16-19d, mvfx%0-3d"},
    {0x0e3005a0, 0x0ff00ff0, "cfadd64%c\tmvdx%12-15d, mvdx%16-19d, mvdx%0-3d"},
    {0x0e3005c0, 0x0ff00ff0, "cfsub32%c\tmvfx%12-15d, mvfx%16-19d, mvfx%0-3d"},
    {0x0e3005e0, 0x0ff00ff0, "cfsub64%c\tmvdx%12-15d, mvdx%16-19d, mvdx%0-3d"},
    {0x0e100500, 0x0ff00ff0, "cfmul32%c\tmvfx%12-15d, mvfx%16-19d, mvfx%0-3d"},
    {0x0e100520, 0x0ff00ff0, "cfmul64%c\tmvdx%12-15d, mvdx%16-19d, mvdx%0-3d"},
    {0x0e100540, 0x0ff00ff0, "cfmac32%c\tmvfx%12-15d, mvfx%16-19d, mvfx%0-3d"},
    {0x0e100560, 0x0ff00ff0, "cfmsc32%c\tmvfx%12-15d, mvfx%16-19d, mvfx%0-3d"},
    {0x0e000600, 0x0ff00f00, "cfmadd32%c\tmvax%5-7d, mvfx%12-15d, mvfx%16-19d, mvfx%0-3d"},
    {0x0e100600, 0x0ff00f00, "cfmsub32%c\tmvax%5-7d, mvfx%12-15d, mvfx%16-19d, mvfx%0-3d"},
    {0x0e200600, 0x0ff00f00, "cfmadda32%c\tmvax%5-7d, mvax%12-15d, mvfx%16-19d, mvfx%0-3d"},
    {0x0e300600, 0x0ff00f00, "cfmsuba32%c\tmvax%5-7d, mvax%12-15d, mvfx%16-19d, mvfx%0-3d"},

    /* Generic coprocessor instructions */
    {0x0e000000, 0x0f000010, "cdp%c\t%8-11d, %20-23d, cr%12-15d, cr%16-19d, cr%0-3d, {%5-7d}"},
    {0x0e100010, 0x0f100010, "mrc%c\t%8-11d, %21-23d, %12-15r, cr%16-19d, cr%0-3d, {%5-7d}"},
    {0x0e000010, 0x0f100010, "mcr%c\t%8-11d, %21-23d, %12-15r, cr%16-19d, cr%0-3d, {%5-7d}"},
    {0x0c000000, 0x0e100000, "stc%c%22'l\t%8-11d, cr%12-15d, %A"},
    {0x0c100000, 0x0e100000, "ldc%c%22'l\t%8-11d, cr%12-15d, %A"},

    /* The rest.  */
    {0x00000000, 0x00000000, "undefined instruction %0-31x"},
    {0x00000000, 0x00000000, 0}
  };

static inline void print_address(FILE *stream, u_int32_t offset)
{
  fprintf(stream, "0x%08x", offset);
}

static void arm_decode_shift(u_int32_t given, FILE *stream)
{
  fprintf(stream, "%s", arm_regnames[given & 0xf]);
  
  if ((given & 0xff0) != 0)
    {
      if ((given & 0x10) == 0)
	{
	  int amount = (given & 0xf80) >> 7;
	  int shift = (given & 0x60) >> 5;
	  
	  if (amount == 0)
	    {
	      if (shift == 3)
		{
		  fprintf(stream, ", rrx");
		  return;
		}
	      
	      amount = 32;
	    }
	  
	  fprintf(stream, ", %s #%d", arm_shift[shift], amount);
	}
      else
	fprintf(stream, ", %s %s", arm_shift[(given & 0x60) >> 5],
	      arm_regnames[(given & 0xf00) >> 8]);
    }
}

int print_insn_arm(u_int32_t pc, FILE *stream, u_int32_t given)
{
  struct arm_opcode *insn;

  for (insn = arm_opcodes; insn->assembler; insn++)
    {
      if ((given & insn->mask) == insn->value)
	{
	  char * c;
	  
	  for (c = insn->assembler; *c; c++)
	    {
	      if (*c == '%')
		{
		  switch (*++c)
		    {
		    case '%':
		      fprintf(stream, "%%");
		      break;

		    case 'a':
		      if (((given & 0x000f0000) == 0x000f0000)
			  && ((given & 0x02000000) == 0))
			{
			  int offset = given & 0xfff;
			  
			  fprintf(stream, "[pc");
 
			  if (given & 0x01000000)
			    {
			      if ((given & 0x00800000) == 0)
				offset = - offset;
			  
			      /* Pre-indexed.  */
			      fprintf(stream, ", #%d]", offset);

			      offset += pc + 8;

			      /* Cope with the possibility of write-back
				 being used.  Probably a very dangerous thing
				 for the programmer to do, but who are we to
				 argue ?  */
			      if (given & 0x00200000)
				fprintf(stream, "!");
			    }
			  else
			    {
			      /* Post indexed.  */
			      fprintf(stream, "], #%d", offset);

			      /* ie ignore the offset.  */
			      offset = pc + 8;
			    }
			  
			  fprintf(stream, "\t; ");
			  print_address(stream, offset);
			}
		      else
			{
			  fprintf(stream, "[%s", 
				arm_regnames[(given >> 16) & 0xf]);
			  if ((given & 0x01000000) != 0)
			    {
			      if ((given & 0x02000000) == 0)
				{
				  int offset = given & 0xfff;
				  if (offset)
				    fprintf(stream, ", %s#%d",
					  (((given & 0x00800000) == 0)
					   ? "-" : ""), offset);
				}
			      else
				{
				  fprintf(stream, ", %s",
					(((given & 0x00800000) == 0)
					 ? "-" : ""));
				  arm_decode_shift (given, stream);
				}

			      fprintf(stream, "]%s", 
				    ((given & 0x00200000) != 0) ? "!" : "");
			    }
			  else
			    {
			      if ((given & 0x02000000) == 0)
				{
				  int offset = given & 0xfff;
				  if (offset)
				    fprintf(stream, "], %s#%d",
					  (((given & 0x00800000) == 0)
					   ? "-" : ""), offset);
				  else 
				    fprintf(stream, "]");
				}
			      else
				{
				  fprintf(stream, "], %s",
					(((given & 0x00800000) == 0) 
					 ? "-" : ""));
				  arm_decode_shift (given, stream);
				}
			    }
			}
		      break;

		    case 's':
                      if ((given & 0x004f0000) == 0x004f0000)
			{
                          /* PC relative with immediate offset.  */
			  int offset = ((given & 0xf00) >> 4) | (given & 0xf);
			  
			  if ((given & 0x00800000) == 0)
			    offset = -offset;
			  
			  fprintf(stream, "[pc, #%d]\t; ", offset);
			  
			  print_address(stream, offset + pc + 8);
			}
		      else
			{
			  fprintf(stream, "[%s", 
				arm_regnames[(given >> 16) & 0xf]);
			  if ((given & 0x01000000) != 0)
			    {
                              /* Pre-indexed.  */
			      if ((given & 0x00400000) == 0x00400000)
				{
                                  /* Immediate.  */
                                  int offset = ((given & 0xf00) >> 4) | (given & 0xf);
				  if (offset)
				    fprintf(stream, ", %s#%d",
					  (((given & 0x00800000) == 0)
					   ? "-" : ""), offset);
				}
			      else
				{
                                  /* Register.  */
				  fprintf(stream, ", %s%s",
					(((given & 0x00800000) == 0)
					 ? "-" : ""),
                                        arm_regnames[given & 0xf]);
				}

			      fprintf(stream, "]%s", 
				    ((given & 0x00200000) != 0) ? "!" : "");
			    }
			  else
			    {
                              /* Post-indexed.  */
			      if ((given & 0x00400000) == 0x00400000)
				{
                                  /* Immediate.  */
                                  int offset = ((given & 0xf00) >> 4) | (given & 0xf);
				  if (offset)
				    fprintf(stream, "], %s#%d",
					  (((given & 0x00800000) == 0)
					   ? "-" : ""), offset);
				  else 
				    fprintf(stream, "]");
				}
			      else
				{
                                  /* Register.  */
				  fprintf(stream, "], %s%s",
					(((given & 0x00800000) == 0)
					 ? "-" : ""),
                                        arm_regnames[given & 0xf]);
				}
			    }
			}
		      break;
			  
		    case 'b':
		      print_address(stream, BDISP (given) * 4 + pc + 8);
		      break;

		    case 'c':
		      fprintf(stream, "%s",
			    arm_conditional [(given >> 28) & 0xf]);
		      break;

		    case 'm':
		      {
			int started = 0;
			int reg;

			fprintf(stream, "{");
			for (reg = 0; reg < 16; reg++)
			  if ((given & (1 << reg)) != 0)
			    {
			      if (started)
				fprintf(stream, ", ");
			      started = 1;
			      fprintf(stream, "%s", arm_regnames[reg]);
			    }
			fprintf(stream, "}");
		      }
		      break;

		    case 'o':
		      if ((given & 0x02000000) != 0)
			{
			  int rotate = (given & 0xf00) >> 7;
			  int immed = (given & 0xff);
			  immed = (((immed << (32 - rotate))
				    | (immed >> rotate)) & 0xffffffff);
			  fprintf(stream, "#%d\t; 0x%x", immed, immed);
			}
		      else
			arm_decode_shift (given, stream);
		      break;

		    case 'p':
		      if ((given & 0x0000f000) == 0x0000f000)
			fprintf(stream, "p");
		      break;

		    case 't':
		      if ((given & 0x01200000) == 0x00200000)
			fprintf(stream, "t");
		      break;

		    case 'A':
		      fprintf(stream, "[%s", arm_regnames [(given >> 16) & 0xf]);
		      if ((given & 0x01000000) != 0)
			{
			  int offset = given & 0xff;
			  if (offset)
			    fprintf(stream, ", %s#%d]%s",
				  ((given & 0x00800000) == 0 ? "-" : ""),
				  offset * 4,
				  ((given & 0x00200000) != 0 ? "!" : ""));
			  else
			    fprintf(stream, "]");
			}
		      else
			{
			  int offset = given & 0xff;
			  if (offset)
			    fprintf(stream, "], %s#%d",
				  ((given & 0x00800000) == 0 ? "-" : ""),
				  offset * 4);
			  else
			    fprintf(stream, "]");
			}
		      break;

		    case 'B':
		      /* Print ARM V5 BLX(1) address: pc+25 bits.  */
		      {
			u_int32_t address;
			u_int32_t offset = 0;
			
			if (given & 0x00800000)
			  /* Is signed, hi bits should be ones.  */
			  offset = (-1) ^ 0x00ffffff;

			/* Offset is (SignExtend(offset field)<<2).  */
			offset += given & 0x00ffffff;
			offset <<= 2;
			address = offset + pc + 8;
			
			if (given & 0x01000000)
			  /* H bit allows addressing to 2-byte boundaries.  */
			  address += 2;

		        print_address(stream, address);
		      }
		      break;

		    case 'I':
		      /* Print a Cirrus/DSP shift immediate.  */
		      /* Immediates are 7bit signed ints with bits 0..3 in
			 bits 0..3 of opcode and bits 4..6 in bits 5..7
			 of opcode.  */
		      {
			int imm;

			imm = (given & 0xf) | ((given & 0xe0) >> 1);

			/* Is ``imm'' a negative number?  */
			if (imm & 0x40)
			  imm |= (-1 << 7);

			fprintf(stream, "%d", imm);
		      }

		      break;

		    case 'C':
		      fprintf(stream, "_");
		      if (given & 0x80000)
			fprintf(stream, "f");
		      if (given & 0x40000)
			fprintf(stream, "s");
		      if (given & 0x20000)
			fprintf(stream, "x");
		      if (given & 0x10000)
			fprintf(stream, "c");
		      break;

		    case 'F':
		      switch (given & 0x00408000)
			{
			case 0:
			  fprintf(stream, "4");
			  break;
			case 0x8000:
			  fprintf(stream, "1");
			  break;
			case 0x00400000:
			  fprintf(stream, "2");
			  break;
			default:
			  fprintf(stream, "3");
			}
		      break;
			
		    case 'P':
		      switch (given & 0x00080080)
			{
			case 0:
			  fprintf(stream, "s");
			  break;
			case 0x80:
			  fprintf(stream, "d");
			  break;
			case 0x00080000:
			  fprintf(stream, "e");
			  break;
			default:
			  fprintf(stream, "<illegal precision>");
			  break;
			}
		      break;
		    case 'Q':
		      switch (given & 0x00408000)
			{
			case 0:
			  fprintf(stream, "s");
			  break;
			case 0x8000:
			  fprintf(stream, "d");
			  break;
			case 0x00400000:
			  fprintf(stream, "e");
			  break;
			default:
			  fprintf(stream, "p");
			  break;
			}
		      break;
		    case 'R':
		      switch (given & 0x60)
			{
			case 0:
			  break;
			case 0x20:
			  fprintf(stream, "p");
			  break;
			case 0x40:
			  fprintf(stream, "m");
			  break;
			default:
			  fprintf(stream, "z");
			  break;
			}
		      break;

		    case '0': case '1': case '2': case '3': case '4': 
		    case '5': case '6': case '7': case '8': case '9':
		      {
			int bitstart = *c++ - '0';
			int bitend = 0;
			while (*c >= '0' && *c <= '9')
			  bitstart = (bitstart * 10) + *c++ - '0';

			switch (*c)
			  {
			  case '-':
			    c++;
			    
			    while (*c >= '0' && *c <= '9')
			      bitend = (bitend * 10) + *c++ - '0';
			    
			    if (!bitend)
			      return -1;
			    
			    switch (*c)
			      {
			      case 'r':
				{
				  int32_t reg;
				  
				  reg = given >> bitstart;
				  reg &= (2 << (bitend - bitstart)) - 1;
				  
				  fprintf(stream, "%s", arm_regnames[reg]);
				}
				break;
			      case 'd':
				{
				  int32_t reg;
				  
				  reg = given >> bitstart;
				  reg &= (2 << (bitend - bitstart)) - 1;
				  
				  fprintf(stream, "%d", reg);
				}
				break;
			      case 'x':
				{
				  int32_t reg;
				  
				  reg = given >> bitstart;
				  reg &= (2 << (bitend - bitstart)) - 1;
				  
				  fprintf(stream, "0x%08x", reg);
				  
				  /* Some SWI instructions have special
				     meanings.  */
				  if ((given & 0x0fffffff) == 0x0FF00000)
				    fprintf(stream, "\t; IMB");
				  else if ((given & 0x0fffffff) == 0x0FF00001)
				    fprintf(stream, "\t; IMBRange");
				}
				break;
			      case 'X':
				{
				  int32_t reg;
				  
				  reg = given >> bitstart;
				  reg &= (2 << (bitend - bitstart)) - 1;
				  
				  fprintf(stream, "%01x", reg & 0xf);
				}
				break;
			      case 'f':
				{
				  int32_t reg;
				  
				  reg = given >> bitstart;
				  reg &= (2 << (bitend - bitstart)) - 1;
				  
				  if (reg > 7)
				    fprintf(stream, "#%s",
					  arm_fp_const[reg & 7]);
				  else
				    fprintf(stream, "f%d", reg);
				}
				break;
			      default:
				return -1;
			      }
			    break;

			  case 'y':
			  case 'z':
			    {
			      int single = *c == 'y';
			      int regno;

			      switch (bitstart)
				{
				case 4: /* Sm pair */
				  fprintf(stream, "{");
				  /* Fall through.  */
				case 0: /* Sm, Dm */
				  regno = given & 0x0000000f;
				  if (single)
				    {
				      regno <<= 1;
				      regno += (given >> 5) & 1;
				    }
				  break;

				case 1: /* Sd, Dd */
				  regno = (given >> 12) & 0x0000000f;
				  if (single)
				    {
				      regno <<= 1;
				      regno += (given >> 22) & 1;
				    }
				  break;

				case 2: /* Sn, Dn */
				  regno = (given >> 16) & 0x0000000f;
				  if (single)
				    {
				      regno <<= 1;
				      regno += (given >> 7) & 1;
				    }
				  break;

				case 3: /* List */
				  fprintf(stream, "{");
				  regno = (given >> 12) & 0x0000000f;
				  if (single)
				    {
				      regno <<= 1;
				      regno += (given >> 22) & 1;
				    }
				  break;

				  
				default:
				  return -1;
				}

			      fprintf(stream, "%c%d", single ? 's' : 'd', regno);

			      if (bitstart == 3)
				{
				  int count = given & 0xff;

				  if (single == 0)
				    count >>= 1;

				  if (--count)
				    {
				      fprintf(stream, "-%c%d",
					    single ? 's' : 'd',
					    regno + count);
				    }

				  fprintf(stream, "}");
				}
			      else if (bitstart == 4)
				fprintf(stream, ", %c%d}", single ? 's' : 'd',
				      regno + 1);

			      break;
			    }

			  case '`':
			    c++;
			    if ((given & (1 << bitstart)) == 0)
			      fprintf(stream, "%c", *c);
			    break;
			  case '\'':
			    c++;
			    if ((given & (1 << bitstart)) != 0)
			      fprintf(stream, "%c", *c);
			    break;
			  case '?':
			    ++c;
			    if ((given & (1 << bitstart)) != 0)
			      fprintf(stream, "%c", *c++);
			    else
			      fprintf(stream, "%c", *++c);
			    break;
			  default:
			    return -1;
			  }
			break;

		      default:
			return -1;
		      }
		    }
		}
	      else
		fprintf(stream, "%c", *c);
	    }
	  return 0;
	}
    }
  return -1;
}
