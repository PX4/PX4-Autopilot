/****************************************************************************
 * include/nxflat.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NXFLAT_H
#define __INCLUDE_NXFLAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NXFLAT_MAX_STRING_SIZE 64     /* Largest size of string (w/zterminator) */
#define NXFLAT_MAGIC          "NxFT"  /* NXFLAT magic number */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * The NXFLAT file header
 *
 * The elements within this structure are stored in network order (i.e.,
 * ntohs() and ntohl() should be used to access fields within the
 * header.
 ****************************************************************************/

struct nxflat_hdr_s
{
  /* The "magic number" identifying the file type.  This field should contain
   * "NxFT". NOTE that there is no other versioning information other than
   * this magic number.
   */

  char h_magic[4];

  /* The following fields provide the memory map for the nxflat binary.
   *
   * h_entry      - Offset to the first executable insruction from
   *                the beginning of the file.
   * h_datastart  - Offset to the beginning of the data segment from
   *                the beginning of the file.  This field can also
   *                interpreted as the size of the ISpace segment.
   * h_dataend    - Offset to the end of the data segment from the
   *                beginning of  the file.
   * h_bssend     - Offset to the end of bss segment from the beginning
   *                of the file.
   *
   * The text segment can be considered to be the contiguous (unrelocated)
   * address space range from address zero through (but not including)
   * h_datastart.
   *
   * The size of the data/bss segment includes (as a minimum) the data
   * and bss regions (bss_end - data_start) as well as the size of the
   * stack.  At run time, this region will also include program arguments
   * and environement variables.
   * 
   * The bss segment is data_end through bss_end.
   */

  uint32_t h_entry;
  uint32_t h_datastart;
  uint32_t h_dataend;
  uint32_t h_bssend;

  /* Size of stack, in bytes */

  uint32_t h_stacksize;

  /* Relocation entries:
   *
   * h_relocstart - Offset to the beginning of an array of relocation
   *                records (struct nxflat_reloc).  The offset is
   *                relative to the start of the file
   */

  uint32_t h_relocstart;       /* Offset of relocation records */

  /* Imported symbol table (NOTE no symbols are exported):
   *
   * h_importsymbols - Offset to the beginning of an array of imported
   *                   symbol structures (struct nxflat_import_s).  The
   *                   h_importsymbols offset is relative to the
   *                   beginning of the file.  Each entry of the
   *                   array contains an uint32_t offset (again from
   *                   the beginning of the file) to the name of
   *                   a symbol string.  This string is null-terminated.
   */

  uint32_t h_importsymbols;    /* Offset to list of imported symbols */

  /* 16-bit counts
   *
   * h_reloccount  - The number of relocation records in the arry
   * h_importcount - The number of records in the h_importsymbols array.
   */

  uint16_t h_reloccount;       /* Number of relocation records */
  uint16_t h_importcount;      /* Number of imported symbols */
};

/****************************************************************************
 * NXFLAT Relocation types.
 *
 * The relocation records are an array of the following type.
 ****************************************************************************/

struct nxflat_reloc_s
{
  uint32_t r_info;             /* Bit-encoded relocation info */
};

/* Pack the type and the offset into one 32-bit value */

#define NXFLAT_RELOC(t,o)       (((uint32_t)((t) & 3) << 30) | ((o) & 0x3fffffff))

/* The top three bits of the relocation info is the relocation type (see the
 * NXFLAT_RELOC_TYPE_* definitions below.  This is an unsigned value.
 */

#define NXFLAT_RELOC_TYPE(r)    ((uint32_t)(r) >> 30)

/* The bottom 28 bits of the relocation info is the (non-negative) offset into
 * the D-Space that needs the fixup.
 */

#define NXFLAT_RELOC_OFFSET(r)  ((uint32_t)(r) & 0x3fffffff)

/* These are possible values for the relocation type:
 *
 * NXFLAT_RELOC_TYPE_REL32I  Meaning: Object file contains a 32-bit offset
 *                                    into I-Space at the offset.
 *                           Fixup:   Add mapped I-Space address to the offset.
 * NXFLAT_RELOC_TYPE_REL32D  Meaning: Object file contains a 32-bit offset
 *                                    into D-Space at the offset.
 *                           Fixup:   Add allocated D-Space address to the
 *                                    offset.
 * NXFLAT_RELOC_TYPE_REL32ID Meaning: Object file contains a 32-bit offset
 *                                    into I-Space at the offset that will
 *                                    unfortunately be references relative
 *                                    to the GOT
 *                           Fixup:   Add allocated the mapped I-Space
 *                                    address MINUS the allocated D-Space
 *                                    address to the offset.
 */

#define NXFLAT_RELOC_TYPE_REL32I  0
#define NXFLAT_RELOC_TYPE_REL32D  1
#undef  NXFLAT_RELOC_TYPE_REL32ID   /* May not need */
#define NXFLAT_RELOC_TYPE_NUM     2 /* Number of relocation types */

/****************************************************************************
 * NXFLAT Imported symbol type 
 *
 * The imported symbols are an array of the following type.  The fields
 * in each element are stored in native machine order.
 ****************************************************************************/

struct nxflat_import_s
{
  uint32_t i_funcname;    /* Offset to name of imported function */
  uint32_t i_funcaddress; /* Resolved address of imported function */
};

#endif /* __INCLUDE_NXFLAT_H */
