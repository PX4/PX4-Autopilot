/***************************************************************************
 * poff.h
 * Definitions for the PCode Object File Format (POFF)
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

#ifndef __POFF_H
#define __POFF_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include <stdint.h>
#include "keywords.h"
#include "config.h"

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

/* Definitions for the fh_ident field of the poffHdr_t */

#define FHI_MAG0            0    /* fh_ident[] indices */
#define FHI_MAG1            1
#define FHI_MAG2            2
#define FHI_MAG3            3
#define FHI_NIDENT          4

#define FHI_POFF_MAG0       'P'
#define FHI_POFF_MAG1       'O'
#define FHI_POFF_MAG2       'F'
#define FHI_POFF_MAG3       'F'
#define FHI_POFF_MAG        "POFF"

/* Definitions for fh_version */

#define FHV_NONE             0
#define FHV_CURRENT          1

/* Definitions for the fh_type */

#define FHT_NONE             0    /* Shouldn't happen */
#define FHT_EXEC             1    /* Pascal program executable */
#define FHT_SHLIB            2    /* Pascal shared library */
#define FHT_PROGRAM          3    /* Pascal program object */
#define FHT_UNIT             4    /* Pascal unit object */
#define FHT_NTYPES           5

/* Definitions for fh_arch */

#define FHAW_INSN16          0    /* Data width is 16 bits */
#define FHAW_INSN32          1    /* Data width is 32 bits */

#define FHAC_PCODE           0    /* Stack oriented P-Code machine class */
#define FHAC_REGM            1    /* Generalized register machine class */

#define MK_FH_ARCH(c,w)      (((c)<<4)|(w))
#define GET_FH_CLASS(fha)    ((fha) >> 4)
#define GET_FH_WIDTH(fha)    ((fha) & 0x0f)

#define FHA_PCODE_INSN16     MK_FH_ARCH(FHAC_PCODE,FHAW_INSN16)
#define FHA_PCODE_INSN32     MK_FH_ARCH(FHAC_PCODE,FHAW_INSN32)
#define FHA_REGM_INSN16      MK_FH_ARCH(FHAC_REGM,FHAW_INSN16)
#define FHA_REGM_INSN32      MK_FH_ARCH(FHAC_REGM,FHAW_INSN32)

#ifdef CONFIG_INSN16
# define FHA_PCODE           FHA_PCODE_INSN16
# define FHA_REGM            FHA_REGM_INSN16
#endif
#ifdef CONFIG_INSN32
# define FHA_PCODE           FHA_PCODE_INSN32
# define FHA_REGM            FHA_REGM_INSN16
#endif

/* Definitions for sh_type */

#define SHT_NULL             0    /* Shouldn't happen */
#define SHT_PROGDATA         1    /* Program data */
#define SHT_SYMTAB           2    /* Symbol table */
#define SHT_STRTAB           3    /* String table */
#define SHT_REL              4    /* Relocation data */
#define SHT_FILETAB          5    /* File table */
#define SHT_LINENO           6    /* Line number data */
#define SHT_DEBUG            7    /* Procedure/Function info */
#define SHT_NTYPES           8

/* Definitions for sh_flags */

#define SHF_WRITE            0x01 /* Section is write-able */
#define SHF_ALLOC            0x02 /* Memory must be allocated for setion */
#define SHF_EXEC             0x04 /* Section contains program data */

/* Values for st_type */

#define STT_NONE             0    /* Should not occur */
#define STT_DATA             1    /* Stack data section symbol */
#define STT_RODATA           2    /* Read only data section symbol */
#define STT_PROC             3    /* Procedure entry point */
#define STT_FUNC             4    /* Function entry point */
#define STT_NTYPES           5

/* Values for st_align.  Any power of two numeric value can be
 * used, but the following are defined for convenience.
 */

#define STA_NONE             0    /* Should not occur */
#define STA_8BIT             1    /* 8-bit byte alignment */
#define STA_16BIT            2    /* 16-bit half word alignment */
#define STA_32BIT            4    /* 32-bit word alignment */
#define STA_64BIT            8    /* 32-bit double word alignment */

/* Values for st_flags */

#define STF_NONE             0x00
#define STF_UNDEFINED        0x01 /* Symbol is undefined (imported) */

/* P-Code relocation types (see RLI_type) */

#define RLT_NONE             0    /* Should not occur */
#define RLT_PCAL             1    /* PCAL to external proc/func */
#define RLT_LDST             2    /* LA or LAX to external stack loc */
#define RLT_NTYPES           3

/* The following are used with relocation table rl_info field */

#define RLI_SYM(x)           ((x) >> 8)   /* Symbol index */
#define RLI_TYPE(x)          ((x) & 0xff) /* Rloc type */
#define RLI_MAKE(s,t)        (((uint32_t)(s) << 8) | ((t) & 0xff))

/***************************************************************************
 * Public Types
 ***************************************************************************/

/* POFF file header */

struct poffFileHeader_s
{
  /* fh_ident holds the four characters 'P', 'O', 'F', 'F'
   * See the FHI_ definitions above.
   */

  uint8_t fh_ident[FHI_NIDENT];

  /* fh_version holds the version of the POFF file format.  This should
   * always be FHV_CURRENT.
   */

  uint8_t fh_version;

  /* fh_type holds the type of binary carry by the POFF file.
   * See the FHT_ definitions above.
   */

  uint8_t fh_type;

  /* fh_arch holds the mach architecture identifier.  See the FHA_
   * definitions above.
   */

  uint8_t fh_arch;

  /* Pad so that the next field is aligned */

  uint8_t fh_padding;

  /* fh_shsize is the size a section header.  This should be
   * sizeof(poffSectionHeader_t)
   */

  uint16_t fh_shsize;

  /* fh_num is the number of section headers in section header
   * list.  The total size of the section header block is then
   * fh_shsize * fh_shnum.
   */

  uint16_t fh_shnum;

  /* fh_name is an offset into the string table section data.
   * It refers to a name associated with fh_type that determines
   * the specific instances of the type.
   */

  uint32_t fh_name;

  /* For fhi_type = {FHI_EXEC or FHI_PROGRAM}, fh_entry holds the
   * entry point into the program.  For FHI_PROGRAM, this entry point
   * is a instruction space label.  For FHI_EXEC, this entry point
   * is an instruction space address offset (from address zero).
   */

  uint32_t fh_entry;

  /* fh_shoff is the file offset to the beginning of the table of file
   * headers.  fh_shoff will most likely be sizeof(poffFileHeader_t).
   */

  uint32_t fh_shoff;
};
typedef struct poffFileHeader_s poffFileHeader_t;

/* POFF section header */

struct poffSectionHeader_s
{
  /* sh_type is the type of section described by this header.
   * See the SHT_ definitions above.
   */

  uint8_t sh_type;

  /* These flags describe the characteristics of the section.  See the
   * SHF_ definitions above.
   */

  uint8_t sh_flags;

  /* If the section holds a table of fixed sized entries, sh_entsize
   * gives the size of one entry.  The number of entries can then be
   * obtained by dividing sh_size by sh_entsize.
   */

  uint16_t sh_entsize;

  /* sh_name is an offset into the string table section data.
   * It refers to a name associated with section.
   */
  
  uint32_t sh_name;

  /* If the section is loaded into memory (SHF_ALLOC), then this
   * address holds the address at which the data must be loaded
   * (if applicable).
   */

  uint32_t sh_addr;

  /* sh_offset is the offset from the beginning of the file to
   * beginning of data associated with this section.
   */

  uint32_t sh_offset;

  /* sh_size provides the total size of the section data in bytes.
   * If the section holds a table of fixed sized entries, then
   * sh_size be equal to sh_entsize times the number of entries.
   */

  uint32_t sh_size;
};
typedef struct poffSectionHeader_s poffSectionHeader_t;

/* Structures which may appear as arrays in sections */

/* Relocation data section array entry structure */

struct poffRelocation_s
 {
   /* This value includes the symbol table index plus the
    * relocation type.  See the RLI_* macros above.
    */

   uint32_t rl_info;

   /* This is the section data offset to the instruction/data
    * to be relocated.  The effected section is implicit in the
    * relocation type.
    */

   uint32_t rl_offset;           /* Offset to pcode */
};
typedef struct poffRelocation_s poffRelocation_t;

/* Symbol section array entry structure */

struct poffSymbol_s
{
  /* st_type is the type of symbol described by this entry.
   * See the STT_ definitions above.
   */

  uint8_t st_type;

  /* For data section symbols, the following provides the required
   * data space alignment for the symbol memory representation.  For
   * procedures and functions, this value is ignored. See the STT_
   * definitions above.
   */

  uint8_t st_align;

  /* These flags describe the characteristics of the symbol.  See the
   * STF_ definitions above.
   */

  uint8_t st_flags;
  uint8_t st_pad;

  /* st_name is an offset into the string table section data.
   * It refers to a name associated with symbol.
   */
  
  uint32_t st_name;

  /* st_value is the value associated with symbol.  For defined data
   * section symbols, this is the offset into the initialized data
   * section data; for defined procedures and functions, this the
   * offset into program section data.  For undefined symbols, this
   * valid can be used as as addend.
   */

  uint32_t st_value;

  /* For data section symbols, this is the size of the initialized
   * data region associated with the symbol.
   */

  uint32_t st_size;
};
typedef struct poffSymbol_s poffSymbol_t;

/* The file table section just consists of a list of offsets
 * into the string table.  The file table index is used elsewhere
 * (such as in the line number array) to refer to a specific
 * file.
 */

typedef uint32_t poffFileTab_t;

/* Line number section array entry structure.  Line numbers are
 * associated with executable program data sections.
 */

struct poffLineNumber_s
{
   /* This is the source file line number */

   uint16_t ln_lineno;

   /* This is an index (not a byte offset) to an entry in the file
    * section table.  This can be used to identify the name of the
    * file for which the line number applies.
    */

   uint16_t ln_fileno;

   /* This is an offset to the beginning of the instruction in the
    * program data section.  At present, this is limited to a single
    * program data section.
    */

   uint32_t ln_poffset;
};
typedef struct poffLineNumber_s poffLineNumber_t;

/* The debug info section consists of a list of poffDebugFuncInfo_t
 * entries, each following a sublist of poffDebugArgInfo_t entries.
 */

/* poffDebugFuncInfo_t provides description of function input
 * parameters and return values.
 */

struct poffDebugFuncInfo_s
{
  /* This is the address or label of the function/procedure entry
   * point.
   */

  uint32_t df_value;

  /* This is the size of the value returned by the function in
   * bytes (zero for procedures).
   */

  uint32_t df_size;

  /* This is the number of parameters accepted by the function/
   * procedure.
   */

  uint32_t df_nparms;
};
typedef struct poffDebugFuncInfo_s poffDebugFuncInfo_t;

/* poffDebugArgInfo_t provides description of one function input
 * parameter.
 */

struct poffDebugArgInfo_s
{
  /* This is the size, in bytes, of one input paramter */

  uint32_t da_size;
};
typedef struct poffDebugArgInfo_s poffDebugArgInfo_t;

#endif /* __POFF_H */
