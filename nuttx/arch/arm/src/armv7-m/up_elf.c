/****************************************************************************
 * arch/arm/src/armv7-m/up_elf.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <elf32.h>
#include <errno.h>
#include <debug.h>

#include <arch/elf.h>
#include <nuttx/arch.h>
#include <nuttx/binfmt/elf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arch_checkarch
 *
 * Description:
 *   Given the ELF header in 'hdr', verify that the ELF file is appropriate
 *   for the current, configured architecture.  Every architecture that uses
 *   the ELF loader must provide this function.
 *
 * Input Parameters:
 *   hdr - The ELF header read from the ELF file.
 *
 * Returned Value:
 *   True if the architecture supports this ELF file.
 *
 ****************************************************************************/

bool arch_checkarch(FAR const Elf32_Ehdr *ehdr)
{
  /* Make sure it's an ARM executable */

  if (ehdr->e_machine != EM_ARM)
    {
      bdbg("Not for ARM: e_machine=%04x\n", ehdr->e_machine);
      return -ENOEXEC;
    }

  /* Make sure that 32-bit objects are supported */

  if (ehdr->e_ident[EI_CLASS] != ELFCLASS32)
    {
      bdbg("Need 32-bit objects: e_ident[EI_CLASS]=%02x\n", ehdr->e_ident[EI_CLASS]);
      return -ENOEXEC;
    }

  /* Verify endian-ness */

#ifdef CONFIG_ENDIAN_BIG
  if (ehdr->e_ident[EI_DATA] != ELFDATA2MSB)
#else
  if (ehdr->e_ident[EI_DATA] != ELFDATA2LSB)
#endif
    {
      bdbg("Wrong endian-ness: e_ident[EI_DATA]=%02x\n", ehdr->e_ident[EI_DATA]);
      return -ENOEXEC;
    }

  /* TODO:  Check ABI here. */
  return OK;
}

/****************************************************************************
 * Name: arch_relocate and arch_relocateadd
 *
 * Description:
 *   Perform on architecture-specific ELF relocation.  Every architecture
 *   that uses the ELF loader must provide this function.
 *
 * Input Parameters:
 *   rel - The relocation type
 *   sym - The ELF symbol structure containing the fully resolved value.
 *   addr - The address that requires the relocation.
 *
 * Returned Value:
 *   Zero (OK) if the relocation was successful.  Otherwise, a negated errno
 *   value indicating the cause of the relocation failure.
 *
 ****************************************************************************/

int arch_relocate(FAR const Elf32_Rel *rel, FAR const Elf32_Sym *sym,
                  uintptr_t addr)
{
  int32_t offset;
  uint32_t upper_insn;
  uint32_t lower_insn;

  switch (ELF32_R_TYPE(rel->r_info))
    {
    case R_ARM_NONE:
      {
        /* No relocation */
      }
      break;

    case R_ARM_PC24:
    case R_ARM_CALL:
    case R_ARM_JUMP24:
      {
        bvdbg("Performing PC24 [%d] link at addr %08lx [%08lx] to sym '%s' st_value=%08lx\n",
              ELF32_R_TYPE(rel->r_info), (long)addr, (long)(*(uint32_t*)addr),
              sym, (long)sym->st_value);

        offset = (*(uint32_t*)addr & 0x00ffffff) << 2;
        if (offset & 0x02000000)
          {
            offset -= 0x04000000;
          }

        offset += sym->st_value - addr;
        if (offset & 3 || offset <= (int32_t) 0xfe000000 || offset >= (int32_t) 0x02000000)
          {
            bdbg("  ERROR: PC24 [%d] relocation out of range, offset=%08lx\n",
                 ELF32_R_TYPE(rel->r_info), offset);

            return -EINVAL;
          }

        offset >>= 2;

        *(uint32_t*)addr &= 0xff000000;
        *(uint32_t*)addr |= offset & 0x00ffffff;
      }
      break;

    case R_ARM_ABS32:
    case R_ARM_TARGET1:  /* New ABI:  TARGET1 always treated as ABS32 */
      {
        bvdbg("Performing ABS32 link at addr=%08lx [%08lx] to sym=%p st_value=%08lx\n",
              (long)addr, (long)(*(uint32_t*)addr), sym, (long)sym->st_value);

        *(uint32_t*)addr += sym->st_value;
      }
      break;

    case R_ARM_THM_CALL:
    case R_ARM_THM_JUMP24:
      {
        uint32_t S;
        uint32_t J1;
        uint32_t J2;

        /* Thumb BL and B.W instructions. Encoding:
         *
         * upper_insn:
         *
         *  1   1   1   1   1   1
         *  5   4   3   2   1   0   9   8   7   6   5   4   3   2   1   0
         * +----------+---+-------------------------------+--------------+
         * |1   1   1 |OP1|     OP2                       |              | 32-Bit Instructions
         * +----------+---+--+-----+----------------------+--------------+
         * |1   1   1 | 1   0|  S  |              imm10                  | BL Instruction
         * +----------+------+-----+-------------------------------------+
         *
         * lower_insn:
         *
         *  1   1   1   1   1   1
         *  5   4   3   2   1   0   9   8   7   6   5   4   3   2   1   0
         * +---+---------------------------------------------------------+
         * |OP |                                                         | 32-Bit Instructions
         * +---+--+---+---+---+------------------------------------------+
         * |1   1 |J1 | 1 |J2 |                 imm11                    | BL Instruction
         * +------+---+---+---+------------------------------------------+
         *
         * The branch target is encoded in these bits:
         *
         *   S     = upper_insn[10]
         *   imm10 = upper_insn[0:9]
         *   imm11 = lower_insn[0:10]
         *   J1    = lower_insn[13]
         *   J2    = lower_insn[11]
         */

        upper_insn = (uint32_t)(*(uint16_t*)addr);
        lower_insn = (uint32_t)(*(uint16_t*)(addr + 2));

        bvdbg("Performing THM_JUMP24 [%d] link at addr=%08lx [%04x %04x] to sym=%p st_value=%08lx\n",
              ELF32_R_TYPE(rel->r_info), (long)addr, (int)upper_insn, (int)lower_insn,
              sym, (long)sym->st_value);

        /* Extract the 25-bit offset from the 32-bit instruction:
         *
         *   offset[24]    = S
         *   offset[23]    = ~(J1 ^ S)
         *   offset[22]    = ~(J2 ^ S)]
         *   offset[12:21] = imm10
         *   offset[1:11]  = imm11
         *   offset[0]     = 0
         */

        S   = (upper_insn >> 10) & 1;
        J1  = (lower_insn >> 13) & 1;
        J2  = (lower_insn >> 11) & 1;

        offset = (S << 24) |                       /* S -   > offset[24] */
                 ((~(J1 ^ S) & 1) << 23) |         /* J1    -> offset[23] */
                 ((~(J2 ^ S) & 1) << 22) |         /* J2    -> offset[22] */
                 ((upper_insn & 0x03ff) << 12) |   /* imm10 -> offset[12:21] */
                 ((lower_insn & 0x07ff) << 1);     /* imm11 -> offset[1:11] */
                                                   /* 0     -> offset[0] */

        /* Sign extend */

        if (offset & 0x01000000)
          {
            offset -= 0x02000000;
          }

        /* And perform the relocation */

        bvdbg("  S=%d J1=%d J2=%d offset=%08lx branch target=%08lx\n",
              S, J1, J2, (long)offset, offset + sym->st_value - addr);

        offset += sym->st_value - addr;

        /* Is this a function symbol?  If so, then the branch target must be
         * an odd Thumb address
         */

        if (ELF32_ST_TYPE(sym->st_info) == STT_FUNC && (offset & 1) == 0)
          {
            bdbg("  ERROR: JUMP24 [%d] requires odd offset, offset=%08lx\n",
                 ELF32_R_TYPE(rel->r_info), offset);

            return -EINVAL;
          }

        /* Check the range of the offset */

        if (offset <= (int32_t)0xff000000 || offset >= (int32_t)0x01000000)
          {
            bdbg("  ERROR: JUMP24 [%d] relocation out of range, branch taget=%08lx\n",
                 ELF32_R_TYPE(rel->r_info), offset);

            return -EINVAL;
          }

        /* Now, reconstruct the 32-bit instruction using the new, relocated
         * branch target.
         */

        S  = (offset >> 24) & 1;
        J1 = S ^ (~(offset >> 23) & 1);
        J2 = S ^ (~(offset >> 22) & 1);
 
        upper_insn = ((upper_insn & 0xf800) | (S << 10) | ((offset >> 12) & 0x03ff));
        *(uint16_t*)addr = (uint16_t)upper_insn;

        lower_insn = ((lower_insn & 0xd000) | (J1 << 13) | (J2 << 11) | ((offset >> 1) & 0x07ff));
        *(uint16_t*)(addr + 2) = (uint16_t)lower_insn;

        bvdbg("  S=%d J1=%d J2=%d insn [%04x %04x]\n",
              S, J1, J2, (int)upper_insn, (int)lower_insn);
      }
      break;

    case R_ARM_V4BX:
      {
        bvdbg("Performing V4BX link at addr=%08lx [%08lx]\n",
              (long)addr, (long)(*(uint32_t*)addr));

         /* Preserve only Rm and the condition code */

        *(uint32_t*)addr &= 0xf000000f;

        /* Change instruction to 'mov pc, Rm' */

        *(uint32_t*)addr |= 0x01a0f000;
      }
      break;

    case R_ARM_PREL31:
      {
        bvdbg("Performing PREL31 link at addr=%08lx [%08lx] to sym=%p st_value=%08lx\n",
              (long)addr, (long)(*(uint32_t*)addr), sym, (long)sym->st_value);

        offset           = *(uint32_t*)addr + sym->st_value - addr;
        *(uint32_t*)addr = offset & 0x7fffffff;
      }
      break;

    case R_ARM_MOVW_ABS_NC:
    case R_ARM_MOVT_ABS:
      {
        bvdbg("Performing MOVx_ABS [%d] link at addr=%08lx [%08lx] to sym=%p st_value=%08lx\n",
              ELF32_R_TYPE(rel->r_info), (long)addr, (long)(*(uint32_t*)addr),
              sym, (long)sym->st_value);

        offset = *(uint32_t*)addr;
        offset = ((offset & 0xf0000) >> 4) | (offset & 0xfff);
        offset = (offset ^ 0x8000) - 0x8000;

        offset += sym->st_value;
        if (ELF32_R_TYPE(rel->r_info) == R_ARM_MOVT_ABS)
          {
            offset >>= 16;
          }

        *(uint32_t*)addr &= 0xfff0f000;
        *(uint32_t*)addr |= ((offset & 0xf000) << 4) | (offset & 0x0fff);
      }
      break;

    case R_ARM_THM_MOVW_ABS_NC:
    case R_ARM_THM_MOVT_ABS:
      {
        /* Thumb BL and B.W instructions. Encoding:
         *
         * upper_insn:
         *
         *  1   1   1   1   1   1
         *  5   4   3   2   1   0   9   8   7   6   5   4   3   2   1   0
         * +----------+---+-------------------------------+--------------+
         * |1   1   1 |OP1|     OP2                       |              | 32-Bit Instructions
         * +----------+---+--+-----+----------------------+--------------+
         * |1   1   1 | 1   0|  i  | 1  0   1   1   0   0 |    imm4      | MOVT Instruction
         * +----------+------+-----+----------------------+--------------+
         *
         * lower_insn:
         *
         *  1   1   1   1   1   1
         *  5   4   3   2   1   0   9   8   7   6   5   4   3   2   1   0
         * +---+---------------------------------------------------------+
         * |OP |                                                         | 32-Bit Instructions
         * +---+----------+--------------+-------------------------------+
         * |0  |   imm3   |      Rd      |            imm8               | MOVT Instruction
         * +---+----------+--------------+-------------------------------+
         *
         * The 16-bit immediate value is encoded in these bits:
         *
         *   i    = imm16[11]    = upper_insn[10]
         *   imm4 = imm16[12:15] = upper_insn[3:0]
         *   imm3 = imm16[8:10]  = lower_insn[14:12]
         *   imm8 = imm16[0:7]   = lower_insn[7:0]
         */

        upper_insn = (uint32_t)(*(uint16_t*)addr);
        lower_insn = (uint32_t)(*(uint16_t*)(addr + 2));

        bvdbg("Performing THM_MOVx [%d] link at addr=%08lx [%04x %04x] to sym=%p st_value=%08lx\n",
              ELF32_R_TYPE(rel->r_info), (long)addr, (int)upper_insn, (int)lower_insn,
              sym, (long)sym->st_value);

        /* Extract the 16-bit offset from the 32-bit instruction */

        offset = ((upper_insn & 0x000f) << 12) | /* imm4 -> imm16[8:10] */
                 ((upper_insn & 0x0400) << 1) |  /* i    -> imm16[11] */
                 ((lower_insn & 0x7000) >> 4) |  /* imm3 -> imm16[8:10] */
                  (lower_insn & 0x00ff);         /* imm8 -> imm16[0:7] */

        /* Sign extend */

        offset = (offset ^ 0x8000) - 0x8000;

        /* And perform the relocation */

        bvdbg("  offset=%08lx branch target=%08lx\n",
              (long)offset, offset + sym->st_value);

        offset += sym->st_value;

        /* Update the immediate value in the instruction.  For MOVW we want the bottom
         * 16-bits; for MOVT we want the top 16-bits.
         */

        if (ELF32_R_TYPE(rel->r_info) == R_ARM_THM_MOVT_ABS)
          {
            offset >>= 16;
          }

        upper_insn = ((upper_insn & 0xfbf0) | ((offset & 0xf000) >> 12) | ((offset & 0x0800) >> 1));
        *(uint16_t*)addr = (uint16_t)upper_insn;

        lower_insn = ((lower_insn & 0x8f00) | ((offset & 0x0700) << 4) | (offset & 0x00ff));
        *(uint16_t*)(addr + 2) = (uint16_t)lower_insn;

        bvdbg("  insn [%04x %04x]\n",
             (int)upper_insn, (int)lower_insn);
      }
      break;

    default:
      bdbg("Unsupported relocation: %d\n", ELF32_R_TYPE(rel->r_info));
      return -EINVAL;
    }

  return OK;
}

int arch_relocateadd(FAR const Elf32_Rela *rel, FAR const Elf32_Sym *sym,
                     uintptr_t addr)
{
  bdbg("RELA relocation not supported\n");
  return -ENOSYS;
}

