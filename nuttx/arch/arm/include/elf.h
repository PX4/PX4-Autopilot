/****************************************************************************
 * arch/arm/include/syscall.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference: "ELF for the ARM® Architecture," ARM IHI 0044D, current through
 *   ABI release 2.08, October 28, 2009, ARM Limited.
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
 ****************************************************************************/

#ifndef __ARCH_ARM_INCLUDE_ELF_H
#define __ARCH_ARM_INCLUDE_ELF_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 4.3.1 ELF Identification.  Should have:
 *
 * e_machine         = EM_ARM
 * e_ident[EI_CLASS] = ELFCLASS32
 * e_ident[EI_DATA]  = ELFDATA2LSB (little endian) or ELFDATA2MSB (big endian)
 */

#if 0 /* Defined in include/elf32.h */
#define EM_ARM                   40
#endif

/* Table 4-2, ARM-specific e_flags */

#define EF_ARM_EABI_MASK         0xff000000
#define EF_ARM_EABI_UNKNOWN      0x00000000
#define EF_ARM_EABI_VER1         0x01000000
#define EF_ARM_EABI_VER2         0x02000000
#define EF_ARM_EABI_VER3         0x03000000
#define EF_ARM_EABI_VER4         0x04000000
#define EF_ARM_EABI_VER5         0x05000000

#define EF_ARM_BE8               0x00800000 

/* Table 4-4, Processor specific section types */

#define SHT_ARM_EXIDX            0x70000001 /* Exception Index table */
#define SHT_ARM_PREEMPTMAP       0x70000002 /* BPABI DLL dynamic linking pre-emption map */
#define SHT_ARM_ATTRIBUTES       0x70000003 /* Object file compatibility attributes */
#define SHT_ARM_DEBUGOVERLAY     0x70000004
#define SHT_ARM_OVERLAYSECTION   0x70000005

/* 4.7.1 Relocation codes
 *
 * S (when used on its own) is the address of the symbol.
 * A is the addend for the relocation.
 * P is the address of the place being relocated (derived from r_offset).
 * Pa is the adjusted address of the place being relocated, defined as (P & 0xFFFFFFFC).
 * T is 1 if the target symbol S has type STT_FUNC and the symbol addresses a Thumb instruction;
 *   it is 0 otherwise.
 * B(S) is the addressing origin of the output segment defining the symbol S.
 * GOT_ORG is the addressing origin of the Global Offset Table
 * GOT(S) is the address of the GOT entry for the symbol S.
 */

#define R_ARM_NONE               0             /* No relocation */
#define R_ARM_PC24               1             /* ARM       ((S + A) | T) - P */
#define R_ARM_ABS32              2             /* Data      (S + A) | T */
#define R_ARM_REL32              3             /* Data      ((S + A) | T) - P */
#define R_ARM_LDR_PC_G0          4             /* ARM       S + A - P */
#define R_ARM_ABS16              5             /* Data      S + A */
#define R_ARM_ABS12              6             /* ARM       S + A */
#define R_ARM_THM_ABS5           7             /* Thumb16   S + A */
#define R_ARM_ABS8               8             /* Data      S + A */
#define R_ARM_SBREL32            9             /* Data      ((S + A) | T) - B(S) */
#define R_ARM_THM_CALL           10            /* Thumb32   ((S + A) | T) - P */
#define R_ARM_THM_PC8            11            /* Thumb16   S + A - Pa */
#define R_ARM_BREL_ADJ           12            /* Data      ?B(S) + A */
#define R_ARM_TLS_DESC           13            /* Data     */
#define R_ARM_THM_SWI8           14            /* Obsolete */
#define R_ARM_XPC25              15            /* Obsolete */
#define R_ARM_THM_XPC22          16            /* Obsolete */
#define R_ARM_TLS_DTPMOD32       17            /* Data      Module[S] */
#define R_ARM_TLS_DTPOFF32       18            /* Data      S + A - TLS */
#define R_ARM_TLS_TPOFF32        19            /* Data      S + A - tp */
#define R_ARM_COPY               20            /* Miscellaneous */
#define R_ARM_GLOB_DAT           21            /* Data      (S + A) | T */
#define R_ARM_JUMP_SLOT          22            /* Data      (S + A) | T */
#define R_ARM_RELATIVE           23            /* Data      B(S) + A */
#define R_ARM_GOTOFF32           24            /* Data      ((S + A) | T) - GOT_ORG */
#define R_ARM_BASE_PREL          25            /* Data      B(S) + A - P */
#define R_ARM_GOT_BREL           26            /* Data      GOT(S) + A - GOT_ORG */
#define R_ARM_PLT32              27            /* ARM       ((S + A) | T) - P */
#define R_ARM_CALL               28            /* ARM       ((S + A) | T) - P */
#define R_ARM_JUMP24             29            /* ARM       ((S + A) | T) - P */
#define R_ARM_THM_JUMP24         30            /* Thumb32   ((S + A) | T) - P */
#define R_ARM_BASE_ABS           31            /* Data      B(S) + A */
#define R_ARM_ALU_PCREL_7_0      32            /* Obsolete */
#define R_ARM_ALU_PCREL_15_8     33            /* Obsolete */
#define R_ARM_ALU_PCREL_23_15    34            /* Obsolete */
#define R_ARM_LDR_SBREL_11_0_NC  35            /* ARM       S + A - B(S) */
#define R_ARM_ALU_SBREL_19_12_NC 36            /* ARM       S + A - B(S) */
#define R_ARM_ALU_SBREL_27_20_CK 37            /* ARM       S + A - B(S) */
#define R_ARM_TARGET1            38            /* Miscellaneous (S + A) | T or ((S + A) | T) - P */
#define R_ARM_SBREL31            39            /* Data      ((S + A) | T) - B(S) */
#define R_ARM_V4BX               40            /* Miscellaneous */
#define R_ARM_TARGET2            41            /* Miscellaneous */
#define R_ARM_PREL31             42            /* Data      ((S + A) | T) - P */
#define R_ARM_MOVW_ABS_NC        43            /* ARM       (S + A) | T */
#define R_ARM_MOVT_ABS           44            /* ARM       S + A */
#define R_ARM_MOVW_PREL_NC       45            /* ARM       ((S + A) | T) - P */
#define R_ARM_MOVT_PREL          46            /* ARM       S + A - P */
#define R_ARM_THM_MOVW_ABS_NC    47            /* Thumb32   (S + A) | T */
#define R_ARM_THM_MOVT_ABS       48            /* Thumb32   S + A */
#define R_ARM_THM_MOVW_PREL_NC   49            /* Thumb32   ((S + A) | T) - P */
#define R_ARM_THM_MOVT_PREL      50            /* Thumb32   S + A - P */
#define R_ARM_THM_JUMP19         51            /* Thumb32   ((S + A) | T) - P */
#define R_ARM_THM_JUMP6          52            /* Thumb16   S + A - P */
#define R_ARM_THM_ALU_PREL_11_0  53            /* Thumb32   ((S + A) | T) - Pa */
#define R_ARM_THM_PC12           54            /* Thumb32   S + A - Pa */
#define R_ARM_ABS32_NOI          55            /* Data      S + A */
#define R_ARM_REL32_NOI          56            /* Data      S + A - P */
#define R_ARM_ALU_PC_G0_NC       57            /* ARM       ((S + A) | T) - P */
#define R_ARM_ALU_PC_G0          58            /* ARM       ((S + A) | T) - P */
#define R_ARM_ALU_PC_G1_NC       59            /* ARM       ((S + A) | T) - P */
#define R_ARM_ALU_PC_G1          60            /* ARM       ((S + A) | T) - P */
#define R_ARM_ALU_PC_G2          61            /* ARM       ((S + A) | T) - P */
#define R_ARM_LDR_PC_G1          62            /* ARM       S + A - P */
#define R_ARM_LDR_PC_G2          63            /* ARM       S + A - P */
#define R_ARM_LDRS_PC_G0         64            /* ARM       S + A - P */
#define R_ARM_LDRS_PC_G1         65            /* ARM       S + A - P */
#define R_ARM_LDRS_PC_G2         66            /* ARM       S + A - P */
#define R_ARM_LDC_PC_G0          67            /* ARM       S + A - P */
#define R_ARM_LDC_PC_G1          68            /* ARM       S + A - P */
#define R_ARM_LDC_PC_G2          69            /* ARM       S + A - P */
#define R_ARM_ALU_SB_G0_NC       70            /* ARM       ((S + A) | T) - B(S) */
#define R_ARM_ALU_SB_G0          71            /* ARM       ((S + A) | T) - B(S) */
#define R_ARM_ALU_SB_G1_NC       72            /* ARM       ((S + A) | T) - B(S) */
#define R_ARM_ALU_SB_G1          73            /* ARM       ((S + A) | T) - B(S) */
#define R_ARM_ALU_SB_G2          74            /* ARM       ((S + A) | T) - B(S) */
#define R_ARM_LDR_SB_G0          75            /* ARM       S + A - B(S) */
#define R_ARM_LDR_SB_G1          76            /* ARM       S + A - B(S) */
#define R_ARM_LDR_SB_G2          77            /* ARM       S + A - B(S) */
#define R_ARM_LDRS_SB_G0         78            /* ARM       S + A - B(S) */
#define R_ARM_LDRS_SB_G1         79            /* ARM       S + A - B(S) */
#define R_ARM_LDRS_SB_G2         80            /* ARM       S + A - B(S) */
#define R_ARM_LDC_SB_G0          81            /* ARM       S + A - B(S) */
#define R_ARM_LDC_SB_G1          82            /* ARM       S + A - B(S) */
#define R_ARM_LDC_SB_G2          83            /* ARM       S + A - B(S) */
#define R_ARM_MOVW_BREL_NC       84            /* ARM       ((S + A) | T) - B(S) */
#define R_ARM_MOVT_BREL          85            /* ARM       S + A - B(S) */
#define R_ARM_MOVW_BREL          86            /* ARM       ((S + A) | T) - B(S) */
#define R_ARM_THM_MOVW_BREL_NC   87            /* Thumb32   ((S + A) | T) - B(S) */
#define R_ARM_THM_MOVT_BREL      88            /* Thumb32   S + A - B(S) */
#define R_ARM_THM_MOVW_BREL      89            /* Thumb32   ((S + A) | T) - B(S) */
#define R_ARM_TLS_GOTDESC        90            /* Data */
#define R_ARM_TLS_CALL           91            /* ARM */
#define R_ARM_TLS_DESCSEQ        92            /* ARM       TLS relaxation */
#define R_ARM_THM_TLS_CALL       93            /* Thumb32 */
#define R_ARM_PLT32_ABS          94            /* Data      PLT(S) + A */
#define R_ARM_GOT_ABS            95            /* Data      GOT(S) + A */
#define R_ARM_GOT_PREL           96            /* Data      GOT(S) + A - P */
#define R_ARM_GOT_BREL12         97            /* ARM       GOT(S) + A - GOT_ORG */
#define R_ARM_GOTOFF12           98            /* ARM       S + A - GOT_ORG */
#define R_ARM_GOTRELAX           99            /* Miscellaneous */
#define R_ARM_GNU_VTENTRY        100           /* Data */
#define R_ARM_GNU_VTINHERIT      101           /* Data */
#define R_ARM_THM_JUMP11         102           /* Thumb16   S + A - P */
#define R_ARM_THM_JUMP8          103           /* Thumb16   S + A - P */
#define R_ARM_TLS_GD32           104           /* Data      GOT(S) + A - P */
#define R_ARM_TLS_LDM32          105           /* Data      GOT(S) + A - P */
#define R_ARM_TLS_LDO32          106           /* Data      S + A - TLS */
#define R_ARM_TLS_IE32           107           /* Data      GOT(S) + A - P */
#define R_ARM_TLS_LE32           108           /* Data      S + A - tp */
#define R_ARM_TLS_LDO12          109           /* ARM       S + A - TLS */
#define R_ARM_TLS_LE12           110           /* ARM       S + A - tp */
#define R_ARM_TLS_IE12GP         111           /* ARM       GOT(S) + A - GOT_ORG */
#define R_ARM_ME_TOO             128           /* Obsolete */
#define R_ARM_THM_TLS_DESCSEQ16  129           /* Thumb16 */
#define R_ARM_THM_TLS_DESCSEQ32  130           /* Thumb32 */

/* 5.2.1 Platform architecture compatibility data */

#define PT_ARM_ARCHEXT_FMTMSK       0xff000000
#define PT_ARM_ARCHEXT_PROFMSK      0x00ff0000
#define PT_ARM_ARCHEXT_ARCHMSK      0x000000ff

#define PT_ARM_ARCHEXT_FMT_OS       0x00000000
#define PT_ARM_ARCHEXT_FMT_ABI      0x01000000

#define PT_ARM_ARCHEXT_PROF_NONE    0x00000000
#define PT_ARM_ARCHEXT_PROF_ARM     0x00410000
#define PT_ARM_ARCHEXT_PROF_RT      0x00520000
#define PT_ARM_ARCHEXT_PROF_MC      0x004d0000
#define PT_ARM_ARCHEXT_PROF_CLASSIC 0x00530000

#define PT_ARM_ARCHEXT_ARCH_UNKNOWN 0x00
#define PT_ARM_ARCHEXT_ARCHv4       0x01
#define PT_ARM_ARCHEXT_ARCHv4T      0x02
#define PT_ARM_ARCHEXT_ARCHv5T      0x03
#define PT_ARM_ARCHEXT_ARCHv5TE     0x04
#define PT_ARM_ARCHEXT_ARCHv5TEJ    0x05
#define PT_ARM_ARCHEXT_ARCHv6       0x06
#define PT_ARM_ARCHEXT_ARCHv6KZ     0x07
#define PT_ARM_ARCHEXT_ARCHv6T2     0x08
#define PT_ARM_ARCHEXT_ARCHv6K      0x09
#define PT_ARM_ARCHEXT_ARCHv7       0x0a
#define PT_ARM_ARCHEXT_ARCHv6M      0x0b
#define PT_ARM_ARCHEXT_ARCHv6SM     0x0c
#define PT_ARM_ARCHEXT_ARCHv7EM     0x0d

/* Table 5-6, ARM-specific dynamic array tags */

#define DT_ARM_RESERVED1         0x70000000
#define DT_ARM_SYMTABSZ          0x70000001
#define DT_ARM_PREEMPTMAP        0x70000002
#define DT_ARM_RESERVED2         0x70000003

#endif /* __ARCH_ARM_INCLUDE_ELF_H */
