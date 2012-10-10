/***********************************************************************
 * toolchain/nxflat/ldnxflat.c
 * Convert ELF (or any BFD format) to NXFLAT binary format
 *
 * ldnxflat takes a fully resolvable elf binary which was linked with -r 
 * and resolves all references, then generates relocation table entries for
 * any relocation entries in data sections. This is designed to work with
 * the options -fpic -msingle-pic-base (and -mno-got or -membedded-pic, but
 * will and GOT relocations as well).
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *
 * Modified from ldelf2xflat (see http://xflat.org):
 *
 *   Copyright (c) 2002, 2006, Cadenux, LLC.  All rights reserved.
 *   Copyright (c) 2002, 2006, Gregory Nutt.  All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Extended from the FLAT ldnxflat.c (original copyright below )
 *
 *   Copyright (C) 2000 NETsilicon, Inc.
 *   Copyright (C) 2000 WireSpeed Communications Corp
 *
 *   author : Joe deBlaquiere ( joe@wirespeed.com )
 *
 *   converted from elf2flt.c ( original copyright below )
 *
 *   elf2flt copyright :
 *
 *   (c) 1999, Greg Ungerer <gerg@moreton.com.au>
 *   (c) 1999, Phil Blundell, Nexus Electronics Ltd <pb@nexus.co.uk>
 *
 *   Hacked this about badly to fully support relocating binaries.
 *
 *   Originally obj-res.c
 *
 *   (c) 1998, Kenneth Albanowski <kjahds@kjahds.com>
 *   (c) 1998, D. Jeff Dionne
 *   (c) 1998, The Silver Hammer Group Ltd.
 *   (c) 1996, 1997 Dionne & Associates
 *   jeff@ryeham.ee.ryerson.ca
 *
 *   Relocation added March 1997, Kresten Krab Thorup 
 *   krab@california.daimi.aau.dk
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

/***********************************************************************
 * Included Files
 ***********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdarg.h>
#include <errno.h>

#include <sys/types.h>
#include <netinet/in.h>

#include "bfd.h"
#include "arch/arch.h"
#include "nxflat.h"

/***********************************************************************
 * Compilation Switches
 ***********************************************************************/

/* #define RELOCS_IN_NETWORK_ORDER 1 */

#define LIBS_CAN_INCLUDE_LIBS 1

/* Debug output */

#if 0
#  define message(fmt, arg...) printf("%s: " fmt, __func__, ##arg)
#  define dbg(fmt, arg...)     if (verbose) printf("%s: " fmt, __func__, ##arg)
#  define vdbg(fmt, arg...)    if (verbose > 1) printf("%s: " fmt, __func__, ##arg)
#else
#  define message(fmt, arg...) printf(fmt, ##arg)
#  define dbg(fmt, arg...)     if (verbose) printf(fmt, ##arg)
#  define vdbg(fmt, arg...)     if (verbose > 1) printf(fmt, ##arg)
#endif
#define err(fmt, arg...)       fprintf(stderr, "ERROR -- " fmt, ##arg)
#define warn(fmt, arg...)      fprintf(stderr, "WARNING -- " fmt, ##arg)

/***********************************************************************
 * Definitions
 ***********************************************************************/

#ifndef PARAMS
#  define PARAMS(x)          x
#endif

#ifdef __CYGWIN32__
#  define O_PLATFORM         O_BINARY
#else
#  define O_PLATFORM         0
#endif

#define MAX_SECTIONS       16
#define DEFAULT_STACK_SIZE 4096

#define IS_GLOBAL(x)       ((((x)->flags)&(BSF_GLOBAL))!=0)

#define NXFLAT_HDR_SIZE     sizeof(struct nxflat_hdr_s)

/* The names of these fields have changed in later versions of binutils
 * (after 2.13 and before 2.15) and the meaning of _rawsize is has also
 * changed somewhat.  In the same timeframe, the name of the section
 * structure changed.
 */

#if 0
#  define COOKED_SIZE _cooked_size
#  define RAW_SIZE    _raw_size
#  define bfd_section sec
#else
#  define COOKED_SIZE size
#  define RAW_SIZE    rawsize
#endif

#define NXFLAT_RELOC_TARGET_TEXT    0
#define NXFLAT_RELOC_TARGET_DATA    1
#define NXFLAT_RELOC_TARGET_RODATA  2
#define NXFLAT_RELOC_TARGET_BSS     3
#define NXFLAT_RELOC_TARGET_UNKNOWN 4

/***********************************************************************
 * Private Types
 ***********************************************************************/

/* Needs to match definition in include/elf/internal.h.  This is from binutils-2.19.1 */

struct elf_internal_sym
{
  bfd_vma       st_value;            /* Value of the symbol */
  bfd_vma       st_size;             /* Associated symbol size */
  unsigned long st_name;             /* Symbol name, index in string tbl */
  unsigned char st_info;             /* Type and binding attributes */
  unsigned char st_other;            /* Visibilty, and target specific */
  unsigned int  st_shndx;            /* Associated section index */
};

typedef struct
{
  /* The BFD symbol. */

  asymbol symbol;

  /* ELF symbol information.  */

  struct elf_internal_sym internal_elf_sym;

  /* Backend specific information.  */

  union
    {
      unsigned int hppa_arg_reloc;
      void *mips_extr;
      void *any;
    }
  tc_data;

  /* Version information.  This is from an Elf_Internal_Versym structure in a 
   * SHT_GNU_versym section.  It is zero if there is no version information. */

  u_int16_t version;

} elf_symbol_type;

typedef struct _segment_info
{
  const char *name;
  bfd_vma low_mark;
  bfd_vma high_mark;
  size_t size;
  void *contents;
  asection *subsect[MAX_SECTIONS];
  int nsubsects;
} segment_info;

typedef void (*func_type) (asymbol * sym, void *arg1, void *arg2, void *arg3);

/* This structure defines the got entry for one symbol */

struct nxflat_got_s
{
  asymbol   *sym;                /* Symbol */
  u_int32_t  offset;             /* GOT offset for this symbol */
};

/***********************************************************************
 * Private Variable Data
 ***********************************************************************/

static int verbose = 0;
static int dsyms = 0;
static int stack_size = 0;
static int nerrors = 0;

static int32_t counter = 0;

static const char *program_name = NULL;
static const char *bfd_filename = NULL;
static const char *entry_name = NULL;
static char *out_filename = NULL;

static segment_info text_info;
static segment_info data_info;
static segment_info bss_info;

static asymbol **symbol_table = NULL;
static int32_t number_of_symbols = 0;

static asymbol *entry_symbol = NULL;
static asymbol *dynimport_begin_symbol = NULL;
static asymbol *dynimport_end_symbol = NULL;

struct nxflat_reloc_s *nxflat_relocs;
static int nxflat_nrelocs;

static struct nxflat_got_s *got_offsets; /* realloc'ed array of GOT entry descriptions */
static u_int32_t got_size;                /* The size of the GOT to be allocated */
int    ngot_offsets;                      /* Number of GOT offsets in got_offsets[] */

/***********************************************************************
 * Private Constant Data
 ***********************************************************************/

static const char default_exe_entry_name[] = "_start";
static const char dynimport_begin_name[] = "__dynimport_begin";
static const char dynimport_end_name[] = "__dynimport_end";

/***********************************************************************
 * Private Functions
 ***********************************************************************/

/***********************************************************************
 * nxflat_swap32
 ***********************************************************************/

#ifdef ARCH_BIG_ENDIAN
static inline u_int32_t nxflat_swap32(u_int32_t little)
{
  u_int32_t big =
    ((little >> 24) & 0xff) |
    (((little >> 16) & 0xff) << 8) |
    (((little >> 8) & 0xff) << 16) | ((little & 0xff) << 24);
  return big;
}
#endif

/***********************************************************************
 * get_xflat32
 ***********************************************************************/

static inline u_int32_t get_xflat32(u_int32_t * addr32)
{
  return ntohl(*addr32);
}

/***********************************************************************
 * put_xflat32
 ***********************************************************************/

static void inline put_xflat32(u_int32_t * addr32, u_int32_t val32)
{
  *addr32 = htonl(val32);
}

/***********************************************************************
 * put_xflat16
 ***********************************************************************/

static void inline put_xflat16(u_int16_t * addr16, u_int16_t val16)
{
#if 1
  *addr16 = htons(val16);
#else
  u_int32_t *addr32 = (u_int32_t *) (((u_int32_t) addr16) & ~3);
  u_int32_t ndx = ((((u_int32_t) addr16) >> 1) & 1);

  union
    {
      u_int16_t hw[2];
      u_int32_t w;
    } uword;

  /* Fetch the 32 bit value */

  uword.w = get_xflat32(addr32);

  /* Add in the 16 bit value */

  uword.hw[ndx] = val16;

  /* Then save the 32-bit value */

  put_xflat32(addr32, uword.w);
#endif
}

/***********************************************************************
 * nxflat_write
 ***********************************************************************/

static void nxflat_write(int fd, const char *buffer, int buflen)
{
  vdbg("Writing fd: %d buffer: %p buflen: %d\n", fd, buffer, buflen);

  /* Dump the entire buffer if very strong debug is selected */

  if (verbose > 2)
    {
      static int offset = 0;
      const unsigned char *ptr = (const unsigned char *)buffer;
      int i;
      int j;

      for (i = 0; i < buflen; i += 32)
        {
          printf("%08x:", offset + i);
          for (j = 0; j < 32 && (i + j) < buflen; j++)
            {
              printf(" %02x", *ptr++);
            }
          printf("\n");
        }
      offset += buflen;
    }

  /* Write the data to file, handling errors and interruptions */

  do
    {
      ssize_t nwritten = write(fd, buffer, buflen);
      if (nwritten < 0)
        {
           if (errno != EINTR)
             {
               err("Write to output file failed: %s\n", strerror(errno));
               exit(1);
             }
        }
      else
        {
           buffer += nwritten;
           buflen -= nwritten;
        }
    }
  while (buflen > 0);
}

/***********************************************************************
 * get_symbols
 ***********************************************************************/

static asymbol **get_symbols(bfd *abfd, int32_t *num)
{
  int32_t storage_needed;

  if (dsyms)
    {
      storage_needed = bfd_get_dynamic_symtab_upper_bound(abfd);
    }
  else
    {
      storage_needed = bfd_get_symtab_upper_bound(abfd);
    }

  if (storage_needed < 0)
    {
      abort();
    }

  if (storage_needed == 0)
    {
      return NULL;
    }

  symbol_table = (asymbol**)malloc(storage_needed);

  if (dsyms)
    {
      number_of_symbols = bfd_canonicalize_dynamic_symtab(abfd, symbol_table);
    }
  else
    {
      number_of_symbols = bfd_canonicalize_symtab(abfd, symbol_table);
    }

  if (number_of_symbols < 0)
    {
      abort();
    }

  *num = number_of_symbols;

  vdbg("Read %ld symbols\n", (long)number_of_symbols);
  return symbol_table;
}

/***********************************************************************
 * traverse_global_symbols
 ***********************************************************************/

static void
traverse_global_symbols(void *arg1, void *arg2, void *arg3, func_type fn)
{
  int i;
  for (i = 0; i < number_of_symbols; i++)
    {
      /* Check if it is a global function symbol defined in this */

      if (IS_GLOBAL(symbol_table[i]))
        {
          /* Yes, process the symbol */

          fn(symbol_table[i], arg1, arg2, arg3);
        }
    }
}

/***********************************************************************
 * check_special_symbol
 ***********************************************************************/

static void check_special_symbol(asymbol * sym,
                                 void *arg1, void *arg2, void *arg3)
{
  if ((entry_name) && (strcmp(entry_name, sym->name) == 0))
    {
      entry_symbol = sym;
    }
  else if (strcmp(dynimport_begin_name, sym->name) == 0)
    {
      dynimport_begin_symbol = sym;
    }
  else if (strcmp(dynimport_end_name, sym->name) == 0)
    {
      dynimport_end_symbol = sym;
    }
  counter++;
}

/***********************************************************************
 * find_special_symbols
 ***********************************************************************/

static void inline find_special_symbols(void)
{
  counter = 0;
  traverse_global_symbols(NULL, NULL, NULL, check_special_symbol);

  if (entry_symbol == NULL)
    {
      err("Executable entry point \"%s\" not found\n", entry_name);
      exit(1);
    }

  if (dynimport_begin_symbol == NULL)
    {
      warn("Special symbol \"%s\" not found\n", dynimport_begin_name);
      dynimport_end_symbol = NULL;
    }
  else if (dynimport_end_symbol == NULL)
    {
      err("Symbol \"%s\" found, but missing \"%s\"\n",
          dynimport_begin_name, dynimport_end_name);
      exit(1);
    }
}

/***********************************************************************
 * put_special_symbol
 ***********************************************************************/

static void
put_special_symbol(asymbol *begin_sym, asymbol *end_sym,
                   u_int32_t *addr, u_int16_t *count,
                   u_int32_t elem_size, u_int32_t offset)
{
  u_int32_t file_offset = 0;
  u_int32_t elems = 0;

  u_int32_t begin_sym_value;
  u_int32_t begin_sect_vma;

  /* We'll assume its okay if this symbol was not found. */

  if (begin_sym != NULL)
    {
      vdbg("begin: '%s' end: '%s' offset: %08lx\n",
            begin_sym->name, end_sym->name, (long)(NXFLAT_HDR_SIZE+offset));

      /* Get the value of the beginning symbol and the section that it is
       * defined in. */

      begin_sym_value = begin_sym->value;
      if (begin_sym->section == NULL)
        {
          err("No section for symbol \"%s\"\n", begin_sym->name);
          exit(1);
        }
      else
        {
          /* Get the file offset to the beginning symbol */

          begin_sect_vma = begin_sym->section->vma;

          file_offset = NXFLAT_HDR_SIZE +       /* Size of the NXFLAT header */
            begin_sect_vma +    /* Virtual address of section */
            begin_sym_value +   /* Value of the symbol */
            offset;             /* Additional file offset */

          /* If there is a begin symbol, then there MUST be a corresponding
           * ending symbol.  We must have this to get the size of the data
           * structure.  This size will be used to determined the number of
           * elements in the array. */

          if (end_sym == NULL)
            {
              /* No matching end symbol */

              err("ERROR: Begin sym \"%s\" found, no corresponding end\n",
                  begin_sym->name);
              exit(1);
            }
          else if (end_sym->section == NULL)
            {
              /* No section associated with the end symbol */

              err("No section for symbol \"%s\"\n", end_sym->name);
              exit(1);
            }
          else if (end_sym->section != begin_sym->section)
            {
              /* Section associated with the end symbol is not the same as the
               * section associated with the begin symbol. */

              err("Begin sym \"%s\" is defined in section \"%s\"\n",
                  begin_sym->name, begin_sym->section->name);
              err("  but sym \"%s\" is defined in section \"%s\"\n",
                  end_sym->name, end_sym->section->name);
              exit(1);
            }
          else if (end_sym->value < begin_sym_value)
            {
              /* End symbol is before the begin symbol? */

              err("Begin sym \"%s\" lies at offset %d in section \"%s\"\n",
                  begin_sym->name, begin_sym_value, begin_sym->section->name);
              err("  but sym \"%s\" is before that at offset: %ld\n",
                  end_sym->name, (long)end_sym->value);
              exit(1);
            }
          else
            {
              /* Get the size of the structure in bytes */

              u_int32_t array_size = end_sym->value - begin_sym_value;

              /* Get the number of elements in the structure. */

              elems = array_size / elem_size;

              /* Verify that there are an even number of elements in the array. */

              if (elems * elem_size != array_size)
                {
                  err("Array size (%d) is not a multiple of the element size (%d)\n",
                      array_size, elem_size);
                  exit(1);
                }
            }
        }

      dbg("Symbol %s: value: %08x section offset: %08x file offset: %08x count: %d\n",
          begin_sym->name, begin_sym_value, begin_sect_vma, file_offset, elems);
    }

  put_xflat32(addr, file_offset);
  put_xflat16(count, elems);
}

/***********************************************************************
 * put_entry_point
 ***********************************************************************/

static void put_entry_point(struct nxflat_hdr_s *hdr)
{
  u_int32_t entry_point = 0;

  if (entry_symbol)
    {
      struct bfd_section *sect;

      /* Does this symbol lie in the text section? */

      sect = entry_symbol->section;
      if (sect == NULL)
        {
          err("No section for entry symbol \"%s\"\n", entry_symbol->name);
          exit(1);
        }

      /* Get the file offset to the entry point symbol */

      entry_point = NXFLAT_HDR_SIZE + sect->vma + entry_symbol->value;

      printf("Entry symbol \"%s\": %08x in section \"%s\"\n",
             entry_symbol->name, entry_point, sect->name);

      dbg("  HDR: %08lx + Section VMA: %08lx + Symbol Value: %08lx\n",
          (long)NXFLAT_HDR_SIZE, (long)sect->vma, (long)entry_symbol->value);
    }

  /* Does the entry point lie within the text region? */

  if ((entry_point < NXFLAT_HDR_SIZE) ||
      (entry_point >= NXFLAT_HDR_SIZE + text_info.size))
    {

      /* No... One special case: A shared library may not need an
       * initialization entry point. */

      if (entry_point == 0)
        {
          /* Complain a little in this case... The used might have intended to
           * specify one. */

          warn("Library has no initialization entry point\n");
        }
      else
        {
          /* Otherwise, complain a lot.  We either have a program with no
           * entry_point or a bogus entry_point. */

          err("Invalid entry point: %08x\n", entry_point);
          err("  Valid TEXT range: %08lx - %08lx\n",
              (long)NXFLAT_HDR_SIZE, (long)(NXFLAT_HDR_SIZE + text_info.size));
          exit(1);
        }
    }

  /* Put the entry point into the NXFLAT header. */

  put_xflat32(&hdr->h_entry, entry_point);
}

/***********************************************************************
 * get_reloc_type
 ***********************************************************************/

static int get_reloc_type(asection *sym_section, segment_info **sym_segment)
{
  int i;

  /* Locate the address referred to by section type.  In the context in which
   * this runs, we can no longer use the flags field (it is zero for some
   * reason).  But we can search for matches with the buffered section
   * pointers. */

  /* Check if the symbol is defined in a BSS section */

  for (i = 0; i < bss_info.nsubsects; i++)
    {
      if (bss_info.subsect[i] == sym_section)
        {
          /* Yes... */

          vdbg("Sym section %s is BSS\n", sym_section->name);
 
          if (sym_segment)
            {
              *sym_segment = &bss_info;
            }
          return NXFLAT_RELOC_TARGET_BSS;
        }
    }

  /* Check if the symbol is defined in a TEXT section */

  for (i = 0; i < text_info.nsubsects; i++)
    {
      if (text_info.subsect[i] == sym_section)
        {
          /* Yes... */

          vdbg("Sym section %s is CODE\n", sym_section->name);

          if (sym_segment)
            {
              *sym_segment = &text_info;
            }
          return NXFLAT_RELOC_TARGET_TEXT;
        }
    }

  /* Check if the symbol is defined in a DATA section */

  for (i = 0; i < data_info.nsubsects; i++)
    {
      if (data_info.subsect[i] == sym_section)
        {
          /* Yes... */

          vdbg("Sym section %s is DATA\n", sym_section->name);

          if (sym_segment)
            {
              *sym_segment = &data_info;
            }
          return NXFLAT_RELOC_TARGET_DATA;
        }
    }

  err("Could not find region for sym_section \"%s\" (%p)\n",
      sym_section->name, sym_section);

  return NXFLAT_RELOC_TARGET_UNKNOWN;
}

/***********************************************************************
 * find_got_entry
 ***********************************************************************/

/* Find the GOT entry for a particular symbol index */

static struct nxflat_got_s *find_got_entry(asymbol *sym)
{
  int i;
  for (i = 0; i < ngot_offsets; i++)
    {
      if (got_offsets[i].sym == sym)
        {
           return &got_offsets[i];
        }
    }
  return NULL;
}

/***********************************************************************
 * alloc_got_entry
 ***********************************************************************/

/* Allocate a new got entry */

static void alloc_got_entry(asymbol *sym)
{
  struct nxflat_got_s *newgot;
  int noffsets;

  /* First, make sure that we don't already have an entry for this symbol */

  if (find_got_entry(sym) == 0)
    {
      /* Realloc the array of GOT offsets to hold one more */

      noffsets = ngot_offsets + 1;
      newgot = (struct nxflat_got_s *)realloc(got_offsets, sizeof(struct nxflat_got_s) * noffsets);
      if (!newgot)
        {
           err("Failed to extend the GOT offset table. noffsets: %d\n", noffsets);
        }
      else
        {
          /* Add the new symbol offset to the end of the reallocated table */

          newgot[ngot_offsets].sym    = sym;
          newgot[ngot_offsets].offset = got_size;

          /* Update counts and sizes */

          got_offsets   = newgot;
          ngot_offsets  = noffsets;
          got_size      = sizeof(u_int32_t) * noffsets;
        }
    }
}

/***********************************************************************
 * relocate_rel32
 ***********************************************************************/

static void
relocate_rel32(arelent *relp, int32_t *target, symvalue sym_value)
{
  reloc_howto_type      *how_to      = relp->howto;
  asymbol               *rel_sym     = *relp->sym_ptr_ptr;
  asection              *rel_section = rel_sym->section;
  int32_t                value;
  int32_t                temp;
  int32_t                saved;

  if (verbose > 1)
    {
      vdbg("  Original location %p is %08lx ",
#ifdef ARCH_BIG_ENDIAN
           target, (long)nxflat_swap32(*target));
#else
          target, (long)*target);
#endif
      if (verbose > 2)
        {
          printf("rsh %d ", how_to->rightshift);
          printf(" sz %d ", how_to->size);
          printf("bit %d ", how_to->bitsize);
          printf("rel %d ", how_to->pc_relative);
          printf("smask %08lx ", (long)how_to->src_mask);
          printf("dmask %08lx ", (long)how_to->dst_mask);
          printf("off %d ", how_to->pcrel_offset);
        }

      printf("\n");
    }

#ifdef ARCH_BIG_ENDIAN
  saved = temp = (int32_t) nxflat_swap32(*target);
#else
  saved = temp = *target;
#endif
  /* Mask  and sign extend */

  temp &= how_to->src_mask;
  temp <<= (32 - how_to->bitsize);
  temp >>= (32 - how_to->bitsize);

  /* Calculate the new value:  Current value + VMA - current PC */

  value = temp + sym_value + rel_section->vma - relp->address;

  /* Offset */

  temp += (value >> how_to->rightshift);

  /* Mask upper bits from rollover */

  temp &= how_to->dst_mask;

  /* Replace data that was masked */

  temp |= saved & (~how_to->dst_mask);

  vdbg("  Modified location: %08lx\n", (long)temp);
#ifdef ARCH_BIG_ENDIAN
  *target = (long)nxflat_swap32(temp);
#else
  *target = (long)temp;
#endif
 }

/***********************************************************************
 * relocate_abs32
 ***********************************************************************/

static void
relocate_abs32(arelent *relp, int32_t *target, symvalue sym_value)
{
  reloc_howto_type      *how_to      = relp->howto;
  asymbol               *rel_sym     = *relp->sym_ptr_ptr;
  asection              *rel_section = rel_sym->section;
  struct nxflat_reloc_s *relocs;
  int32_t                temp;
  int32_t                saved;
  int                    reloc_type;

  /* ABS32 links from .text are easy - since the fetches will
   * always be base relative. the ABS32 refs from data will be
   * handled the same
   */

  if (verbose > 1)
    {
      vdbg("  Original location %p is %08lx ",
#ifdef ARCH_BIG_ENDIAN
           target, (long)nxflat_swap32(*target));
#else
          target, (long)*target);
#endif
      if (verbose > 2)
        {
          printf("rsh %d ", how_to->rightshift);
          printf(" sz %d ", how_to->size);
          printf("bit %d ", how_to->bitsize);
          printf("rel %d ", how_to->pc_relative);
          printf("smask %08lx ", (long)how_to->src_mask);
          printf("dmask %08lx ", (long)how_to->dst_mask);
          printf("off %d ", how_to->pcrel_offset);
        }

      printf("\n");
    }

#ifdef ARCH_BIG_ENDIAN
  saved = temp = (int32_t) nxflat_swap32(*target);
#else
  saved = temp = *target;
#endif
  /* Mask  and sign extend */

  temp &= how_to->src_mask;
  temp <<= (32 - how_to->bitsize);
  temp >>= (32 - how_to->bitsize);

  /* Offset */

  temp += (sym_value + rel_section->vma) >> how_to->rightshift;

  /* Mask upper bits from rollover */

  temp &= how_to->dst_mask;

  /* Replace data that was masked */

  temp |= saved & (~how_to->dst_mask);

  vdbg("  Modified location: %08lx\n", (long)temp);
#ifdef ARCH_BIG_ENDIAN
  *target = (long)nxflat_swap32(temp);
#else
  *target = (long)temp;
#endif
  /* Determine where the symbol lies */

  switch (get_reloc_type(rel_section, NULL))
    {
      case NXFLAT_RELOC_TARGET_UNKNOWN:
      default:
        {
          err("Symbol relocation section type is unknown\n");
          nerrors++;
        }
        /* Fall through and do something wrong */

      case NXFLAT_RELOC_TARGET_BSS:
      case NXFLAT_RELOC_TARGET_DATA:
        {
          vdbg("Symbol '%s' lies in D-Space\n", rel_sym->name);
          reloc_type = NXFLAT_RELOC_TYPE_REL32D;
        }
        break;

      case NXFLAT_RELOC_TARGET_TEXT:
        {
          vdbg("Symbol '%s' lies in I-Space\n", rel_sym->name);
          reloc_type = NXFLAT_RELOC_TYPE_REL32I;
        }
        break;
    }

  /* Re-allocate memory to include this relocation */

  relocs = (struct nxflat_reloc_s*)
  realloc(nxflat_relocs, sizeof(struct nxflat_reloc_s) * (nxflat_nrelocs + 1));
  if (!relocs)
    {
      err("Failed to re-allocate memory ABS32 relocations (%d relocations)\n",
          nxflat_nrelocs);
      nerrors++;
    }
  else
    {
      /* Reallocation was successful.  Update globals */
               
      nxflat_nrelocs++;
      nxflat_relocs = relocs;

      /* Then add the relocation at the end of the table */

      nxflat_relocs[nxflat_nrelocs-1].r_info =
      NXFLAT_RELOC(reloc_type, relp->address + got_size);
 
      vdbg("relocs[%d]: type: %d offset: %08x\n",
      nxflat_nrelocs-1,
      NXFLAT_RELOC_TYPE(nxflat_relocs[nxflat_nrelocs-1].r_info),
      NXFLAT_RELOC_OFFSET(nxflat_relocs[nxflat_nrelocs-1].r_info));
    }
 }

/***********************************************************************
 * resolve_segment_relocs
 ***********************************************************************/

static void
resolve_segment_relocs(bfd *input_bfd, segment_info *inf, asymbol **syms)
{
  arelent **relpp;
  int relsize;
  int relcount;
  int i;
  int j;

  for (i = 0; i < inf->nsubsects; i++)
    {
      relcount = inf->subsect[i]->reloc_count;
      vdbg("Section %s has %08x relocs\n", inf->subsect[i]->name, relcount);

      if (0 >= relcount)
        {
          continue;
        }

      relsize = bfd_get_reloc_upper_bound(input_bfd, inf->subsect[i]);
      vdbg("Section %s reloc size: %08x\n", inf->subsect[i]->name, relsize);

      if (0 >= relsize)
        {
          continue;
        }

      relpp = (arelent**)malloc((size_t) relsize);
      relcount = bfd_canonicalize_reloc(input_bfd, inf->subsect[i], relpp, syms);

      if (relcount < 0)
        {
          err("bfd_canonicalize_reloc failed!\n");
          exit(1);
        }

      vdbg("Section %s can'd %08x relocs\n", inf->subsect[i]->name, relcount);

      for (j = 0; j < relcount; j++)
        {
          /* Get information about this symbol */

          reloc_howto_type *how_to      = relpp[j]->howto;
          asymbol          *rel_sym     = *relpp[j]->sym_ptr_ptr;
          asection         *rel_section = rel_sym->section;
          int32_t          *target      = (int32_t*)(inf->contents + relpp[j]->address);
          symvalue          sym_value;

          /* If the symbol is a thumb function, then set bit 1 of the value */

          sym_value = rel_sym->value;
#ifdef NXFLAT_THUMB2
          if ((((elf_symbol_type *)rel_sym)->internal_elf_sym.st_info & 0x0f) == STT_ARM_TFUNC)
            {
               sym_value |= 1;
            }
          else
#endif

          /* If the symbol lies in D-Space, then we need to add the size of the GOT
           * table to the symbol value
           */

          if ((rel_section->flags & SEC_CODE) == 0 && (rel_section->flags & SEC_ALLOC) != 0)
            {
              sym_value += got_size;
            }

          dbg("rel %-3d: sym [%20s] s_addr @ %08lx val %08lx-%08lx rel %08lx how %s\n",
               j, rel_sym->name, (long)relpp[j]->address, (long)rel_sym->value,
               (long)sym_value, (long)relpp[j]->addend, how_to->name);

          switch (how_to->type)
            {
            case R_ARM_PLT32:
            case R_ARM_PC24:
              {
                int32_t temp;
                int32_t saved;

                dbg("performing PC24 link at addr %08lx [%08lx] to sym '%s' [%08lx]\n",
                    (long)relpp[j]->address, (long)*target, rel_sym->name, (long)sym_value);

                /* Can't fix what we ain't got */

                if ((SEC_IN_MEMORY & rel_section->flags) == 0)
                  {
                    err("Section %s not loaded into mem!\n", rel_section->name);
                    exit(1);
                  }

                /* PC24 -> can only fix text to text refs */

                if ((SEC_CODE & rel_section->flags) == 0)
                  {
                    err("Section %s not code!\n", rel_section->name);
                    exit(1);
                  }

                if ((SEC_CODE & inf->subsect[i]->flags) == 0)
                  {
                    err("Section %s not code!\n", rel_section->name);
                    exit(1);
                  }

                if (verbose > 1)
                  {
                    vdbg(" Original opcode @ %p is %08lx ",
#ifdef ARCH_BIG_ENDIAN
                         target, (long)nxflat_swap32(*target));
#else
                         target, (long)*target);
#endif
                    if (verbose > 2)
                      {
                        printf("rsh %d ", how_to->rightshift);
                        printf(" sz %d ", how_to->size);
                        printf("bit %d ", how_to->bitsize);
                        printf("rel %d ", how_to->pc_relative);
                        printf("smask %08lx ", (long)how_to->src_mask);
                        printf("dmask %08lx ", (long)how_to->dst_mask);
                        printf("off %d ", how_to->pcrel_offset);
                      }
                    printf("\n");
                  }

                if (how_to->pcrel_offset)
                  {
#ifdef ARCH_BIG_ENDIAN
                    saved = temp = (int32_t)nxflat_swap32(*target);
#else
                    saved = temp = *target;
#endif
                    /* mask */
                    temp &= how_to->src_mask;

                    /* sign extend */
                    temp <<= (32 - how_to->bitsize);
                    temp >>= (32 - how_to->bitsize);

                    /* offset */
                    temp +=
                      ((sym_value + rel_section->vma)
                       - relpp[j]->address) >> how_to->rightshift;

                    /* demote */
                    /* temp >>= how_to->rightshift; */

                    /* mask upper bits from rollover */
                    temp &= how_to->dst_mask;

                    /* replace data that was masked */
                    temp |= saved & (~how_to->dst_mask);
                  }
                else
                  {
                    err("Do not know how pcrel_offset\n");
                    exit(1);
                  }

                vdbg("  Modified opcode: %08lx\n", (long)temp);
#ifdef ARCH_BIG_ENDIAN
                *target = (long)nxflat_swap32(temp);
#else
                *target = (long)temp;
#endif
              }
              break;

            case R_ARM_ABS32:
              {
                dbg("Performing ABS32 link at addr %08lx [%08lx] to sym '%s' [%08lx]\n",
                     (long)relpp[j]->address, (long)*target, rel_sym->name, (long)sym_value);

                relocate_abs32(relpp[j], target, sym_value);
              }
              break;

            case R_ARM_REL32:
              {
                dbg("Performing REL32 link at addr %08lx [%08lx] to sym '%s' [%08lx]\n",
                     (long)relpp[j]->address, (long)*target, rel_sym->name, (long)sym_value);

                /* The only valid REL32 relocation would be to relocate a reference from
                 * I-Space to another symbol in I-Space.  That should be handled by the
                 * partially linking logic so we don't expect to see any R_ARM_REL32
                 * relocations here.
                 */

                switch (get_reloc_type(rel_section, NULL))
                  {
                    case NXFLAT_RELOC_TARGET_UNKNOWN:
                    default:
                      {
                        err("Symbol relocation section type is unknown\n");
                        nerrors++;
                      }
                      break;

                    case NXFLAT_RELOC_TARGET_BSS:
                    case NXFLAT_RELOC_TARGET_DATA:
                      {
                        err("Cannot perform REL32 relocation: Symbol '%s' lies in D-Space\n",
                            rel_sym->name);
                        nerrors++;
                      }
                      break;

                    case NXFLAT_RELOC_TARGET_TEXT:
                      {
                        vdbg("Symbol '%s' lies in I-Space\n", rel_sym->name);
                        relocate_rel32(relpp[j], target, sym_value - relpp[j]->address);
                      }
                      break;
                  }
              }
              break;

#ifdef NXFLAT_THUMB2
            case R_ARM_THM_XPC22:
            case R_ARM_THM_CALL:
            case R_ARM_THM_JUMP24:
              /* Thumb BL (branch long instruction).  */
              {
                u_int16_t *pinsn     = (u_int16_t*)target;
                u_int16_t upper_insn = pinsn[0];
                u_int16_t lower_insn = pinsn[1];

                /* Fetch the addend.  We use the Thumb-2 encoding (backwards
                 * compatible with Thumb-1) involving the J1 and J2 bits
                 */

                int32_t s     = (upper_insn & (1 << 10)) >> 10;
                int32_t upper = upper_insn & 0x3ff;
                int32_t lower = lower_insn & 0x7ff;
                int32_t j1    = (lower_insn & (1 << 13)) >> 13;
                int32_t j2    = (lower_insn & (1 << 11)) >> 11;
                int32_t i1    = j1 ^ s ? 0 : 1;
                int32_t i2    = j2 ^ s ? 0 : 1;
                int32_t temp;
                int32_t signbit;

                temp = (i1 << 23) | (i2 << 22) | (upper << 12) | (lower << 1);

                /* Sign extend */

                temp = (temp | ((s ? 0 : 1) << 24)) - (1 << 24);

                dbg("Performing THM link at addr %08lx [%04x %04x] to sym '%s' [%08lx]\n",
                    (long)relpp[j]->address, upper_insn, lower_insn,
                    rel_sym->name, (long)sym_value);
                vdbg("  Original INSN: %04x %04x temp: %08lx\n",
                    upper_insn, lower_insn, (long)temp);

                /* Add the branch offset (really needs a range check) */

                temp += (sym_value + rel_section->vma - relpp[j]->address);

                if ((lower_insn & 0x5000) == 0x4000)
                  {
                    /* For a BLX instruction, make sure that the relocation is rounded up
                     * to a word boundary.  This follows the semantics of the instruction
                     * which specifies that bit 1 of the target address will come from bit
                     * 1 of the base address.
                     */

                    temp = (temp + 2) & ~ 3;
                  }

                /* Put RELOCATION back into the insn.  Assumes two's complement.
                 * We use the Thumb-2 encoding, which is safe even if dealing with
                 * a Thumb-1 instruction by virtue of our overflow check above.
                 */

                signbit = (temp < 0) ? 1 : 0;
                upper_insn = (upper_insn & ~0x7ff) | ((temp >> 12) & 0x3ff) | (signbit << 10);
                lower_insn = (lower_insn & ~0x2fff)
                           | (((!((temp >> 23) & 1)) ^ signbit) << 13)
                           | (((!((temp >> 22) & 1)) ^ signbit) << 11)
                           | ((temp >> 1) & 0x7ff);

                vdbg("  Modified INSN: %04x %04x temp: %08lx Sec VMA: %08lx\n",
                    upper_insn, lower_insn, (long)temp, (long)rel_section->vma);

                /* Put the relocated value back in the object file:  */

                pinsn[0] = upper_insn;
                pinsn[1] = lower_insn;
              }
              break;
#endif

            case R_ARM_GOTOFF:
              {
                int reltype;

                /* Relocation is relative to the start of the global offset
                 * table.  This is used for things link known offsets to
                 * constant strings in D-Space.  I think we can just ignore
                 * this relocation. The usual assembly language sequence
                 * is like:
                 * 
                 *      ldr  r0, .L9       <- r0 holds GOT-relative offset to 'n'
                 *      add  r0, sl, r0    <- Adding SL produces address of 'n'
                 *      ...
                 * .L9:
                 *     .word  n(GOTOFF)
                 */

                dbg("Perfoming GOTOFF reloc at addr %08lx [%08lx] to sym '%s' [%08lx]\n",
                    (long)relpp[j]->address, (long)*target, rel_sym->name, (long)sym_value);

                /* For this location, we need to set the value to the value
                 * of the symbol in D-Space.  (There is obviously a problem if
                 * the symbol lies in I-Space because the offset is not relative
                 * to the PIC address which points to the GOT).
                 */

                /* Check if symbols lies in I- or D-Space */
 
                reltype = get_reloc_type(rel_section, NULL);
                if (reltype == NXFLAT_RELOC_TARGET_TEXT)
                  {
                    err("Symbol in GOT32 relocation is in TEXT\n");
                    err("  At addr %08lx to sym '%s' [%08lx]\n",
                        (long)relpp[j]->address, rel_sym->name, (long)sym_value);
                  }
                else
                  {
                    vdbg("  Original value: %08lx\n", (long)*target);
                    *target = sym_value;
                    vdbg("  Modified value: %08lx\n", (long)*target);
                 }
              }
              break;

            case R_ARM_GOT32:
            case R_ARM_GOT_PREL:
              {
                struct nxflat_got_s *got_entry;

                /* Relocation is to the entry for this symbol in the global
                 * offset table. This relocation type is used to set the 32-bit
                 * address of global variables.  The usual assembly language sequence
                 * is like:
                 * 
                 *      ldr  r3, .L4       <- r3 holds GOT-relative offset to address of 'n'
                 *      ldr  r1, [sl,r3]   <- r1 holds (relocated) address of 'n'
                 *      ...
                 * .L4:
                 *     .word  n(GOT)
                 */

                dbg("Performing GOT32 reloc at addr %08lx [%08lx] to sym '%s' [%08lx]\n",
                    (long)relpp[j]->address, (long)*target, rel_sym->name, (long)sym_value);

                /* There should be an entry for the relocation allocated in the GOT */

                got_entry = find_got_entry(rel_sym);
                if (!got_entry)
                  {
                     err("No GOT entry from for symobl '%s'\n", rel_sym->name);
                     nerrors++;
                  }
                else
                  {
                    /* The fixup is simply to provide the GOT offset as the relocation value */

                    vdbg("  Original value: %08lx\n", (long)*target);
                    *target = got_entry->offset;
                    vdbg("  Modified value: %08lx\n", (long)*target);
                  }
              }
              break;

            case R_ARM_GOTPC:
              {
                /* Use the global offset table as a symbol value */

                dbg("Performing GOTPC reloc at addr %08lx [%08lx] to sym '%s' [%08lx]\n",
                    (long)relpp[j]->address, (long)*target, rel_sym->name, (long)sym_value);
 
                /* Check if this is TEXT section relocation */
 
                if ((inf->subsect[i]->flags & SEC_CODE) != 0 &&
                         (inf->subsect[i]->flags & SEC_ALLOC) != 0)
                  {
                    /* The GOT always begins at offset 0 */

                    vdbg("  Original value: %08lx\n", (long)*target);
                    *target = 0;
                    vdbg("  Modified value: %08lx\n", (long)*target);
                  }
                else
                  {
                    err("Attempted GOTPC relocation in outside of I-Space section\n");
                    err("  At addr %08lx [%08lx] to sym '%s' [%08lx]\n",
                        (long)relpp[j]->address, (long)*target,
                        rel_sym->name, (long)sym_value);
                    nerrors++;
                  }
              }
              break;

            default:
              err("Do not know how to handle reloc %d type %s @ %p!\n",
                  how_to->type, how_to->name, how_to);
              nerrors++;
              break;
            }
        }

      /* Mark the section as having no relocs */

      inf->subsect[i]->flags &= !(SEC_RELOC);
      inf->subsect[i]->reloc_count = 0;
      free(relpp);
    }
}

/***********************************************************************
 * allocate_segment_got
 ***********************************************************************/

/* The GOT lies at the beginning of D-Space.  Before we can process
 * any relocation data, we need to determine the size of the GOT.
 */

static void allocate_segment_got(bfd *input_bfd, segment_info *inf, asymbol **syms)
{
   arelent **relpp;
  int relsize;
  int relcount;
  int i;
  int j;

  for (i = 0; i < inf->nsubsects; i++)
    {
      relcount = inf->subsect[i]->reloc_count;
      vdbg("Section %s has %08x relocs\n", inf->subsect[i]->name, relcount);

      if (0 >= relcount)
        {
          continue;
        }

      relsize = bfd_get_reloc_upper_bound(input_bfd, inf->subsect[i]);
      vdbg("Section %s reloc size: %08x\n", inf->subsect[i]->name, relsize);

      if (0 >= relsize)
        {
          continue;
        }

      relpp    = (arelent**)malloc((size_t) relsize);
      relcount = bfd_canonicalize_reloc(input_bfd, inf->subsect[i], relpp, syms);

      if (relcount < 0)
        {
          err("bfd_canonicalize_reloc failed!\n");
          exit(1);
        }

      vdbg("Section %s can'd %08x relocs\n", inf->subsect[i]->name, relcount);

      for (j = 0; j < relcount; j++)
        {
          /* Get information about this symbol */

          reloc_howto_type *how_to  = relpp[j]->howto;
          asymbol          *rel_sym = *relpp[j]->sym_ptr_ptr;

          dbg("rel %-3d: sym [%20s] s_addr @ %08lx rel %08lx how %s\n",
               j, rel_sym->name, (long)relpp[j]->address,
               (long)relpp[j]->addend, how_to->name);

          switch (how_to->type)
            {
              case R_ARM_GOT32:
              case R_ARM_GOT_PREL:
                /* This symbol requires a global offset table entry.  */
                {
                  alloc_got_entry(rel_sym);

                  dbg("  Created GOT entry %d for sym %p (offset %d)\n",
                      ngot_offsets-1, got_offsets[ngot_offsets-1].sym, got_offsets[ngot_offsets-1].offset);
                }
                break;

              case R_ARM_GOTOFF32:
              case R_ARM_GOTPC:
                /* These are relative to the GOT, but do not require GOT entries */
                break;

              default:
                break;
            }
        }
      free(relpp);
    }
}

/***********************************************************************
 * dump_symbol
 ***********************************************************************/

static void dump_symbol(asymbol * psym)
{
  struct elf_internal_sym *isym =
    (struct elf_internal_sym *)&((elf_symbol_type *)psym)->internal_elf_sym;

  if (bfd_is_com_section(psym->section))
    {
      /* Common Global - unplaced */

      printf("Sym[%24s] @            sz %04lx ",
             psym->name, (long)psym->value);
      printf("align %04x ", (u_int32_t)isym->st_value);
    }
  else
    {
      printf("Sym[%24s] @ %04lx align            ",
             psym->name, (long)psym->value);
      printf("sz %04x ", (u_int32_t)isym->st_size);
    }

  /* Symbol type */

  printf("tp %02x ", isym->st_info);

  /* Tag thumb specific attributes */

#ifdef NXFLAT_THUMB2
  if ((isym->st_info & 0x0f) == STT_ARM_TFUNC || (isym->st_info & 0x0f) == STT_ARM_16BIT)
    {
      putchar('T');
    }
  else
    {
      putchar(' ');
    }
#endif

  /* Common attributes */

  printf("|%c", psym->flags & BSF_OBJECT ? 'O' : '.');
  printf("%c",  psym->flags & BSF_DYNAMIC ? 'D' : '.');
  printf("%c",  psym->flags & BSF_FILE ? 'F' : '.');
  printf("%c",  psym->flags & BSF_INDIRECT ? 'I' : '.');
  printf("%c",  psym->flags & BSF_WARNING ? 'W' : '.');
  printf("%c",  psym->flags & BSF_CONSTRUCTOR ? 'C' : '.');
  printf("%c",  psym->flags & BSF_NOT_AT_END ? 'N' : '.');
  printf("%c",  psym->flags & BSF_OLD_COMMON ? 'c' : '.');
  printf("%c",  psym->flags & BSF_SECTION_SYM ? 'S' : '.');
  printf("%c",  psym->flags & BSF_WEAK ? 'w' : '.');
  printf("%c",  psym->flags & BSF_KEEP_G ? 'G' : '.');
  printf("%c",  psym->flags & BSF_KEEP ? 'K' : '.');
  printf("%c",  psym->flags & BSF_FUNCTION ? 'f' : '.');
  printf("%c",  psym->flags & BSF_DEBUGGING ? 'd' : '.');
  printf("%c",  psym->flags & BSF_GLOBAL ? 'g' : '.');
  printf("%c|", psym->flags & BSF_LOCAL ? 'l' : '.');
  printf("\n");
}

/***********************************************************************
 * check_symbol_overlap
 ***********************************************************************/

static void check_symbol_overlap(asymbol ** symbols, int number_of_symbols)
{
  int i;
  int j;

  for (i = 0; i < number_of_symbols; i++)
    {
      elf_symbol_type *sym_i;
      bfd_vma base_i;
      bfd_vma top_i;
      bfd_vma size_i;

      sym_i = (elf_symbol_type *) symbols[i];
      base_i = sym_i->symbol.section->vma + sym_i->internal_elf_sym.st_value;
      size_i = sym_i->internal_elf_sym.st_size;

      if (0 == size_i)
        {
          if (sym_i->symbol.section->flags & SEC_CODE)
            {
              /* must be an internal branch - ignore */

              vdbg("Sym [%20s] is zero len, skipping!\n", sym_i->symbol.name);
              continue;
            }
          else
            {
              /* pointer - fake size up */
              size_i = 4;
            }
        }

      top_i = base_i + size_i;

      dbg("Sym [%20s] base %08lx, top %08lx\n",
          sym_i->symbol.name, (long)base_i, (long)top_i);

      for (j = (i + 1); j < number_of_symbols; j++)
        {
          elf_symbol_type *sym_j;
          bfd_vma base_j;
          bfd_vma top_j;
          bfd_vma size_j = 0;

          sym_j = (elf_symbol_type *) symbols[j];
          base_j =
            sym_j->symbol.section->vma + sym_j->internal_elf_sym.st_value;

          if (0 == size_j)
            {
              if (sym_j->symbol.section->flags & SEC_CODE)
                {
                  /* must be an internal branch - ignore */
                  continue;
                }
              else
                {
                  /* pointer - fake size up */
                  size_j = 4;
                }
            }

          top_j = base_j + sym_j->internal_elf_sym.st_size;

          if (0 == sym_j->internal_elf_sym.st_size)
            {
              continue;
            }

          if ((base_j < top_i) && (top_j > base_i))
            {
              /* symbols overlap - bad bad bad bad */

              if (verbose)
                {
                  warn("Symbols '%s'[%6s] and '%s'[%6s] OVERLAP!\n",
                      sym_i->symbol.name, sym_i->symbol.section->name,
                      sym_j->symbol.name, sym_j->symbol.section->name);
                  warn("  Sym '%s' base %08lx, top %08lx\n",
                      sym_i->symbol.name, (long)base_i, (long)top_i);
                  warn("  Sym '%s' base %08lx, top %08lx\n",
                      sym_j->symbol.name, (long)base_j, (long)top_j);
                }
            }

        }
    }
}

/***********************************************************************
 * map_common_symbols
 ***********************************************************************/

static void
map_common_symbols(bfd * input_bfd, asymbol ** symbols, int number_of_symbols)
{
  asection *bss_s;
  int i;
  int j;

  bfd_vma baseaddr;
  bfd_vma align;
  bfd_vma size;
  bfd_vma symbase;
  bfd_vma offset;

  bss_s = bss_info.subsect[0];
  baseaddr = 0;

  vdbg("Before map high mark %08lx cooked %08lx raw %08lx \n",
      (long)bss_info.high_mark, (long)bss_info.subsect[0]->COOKED_SIZE,
      (long)bss_info.subsect[0]->RAW_SIZE);
  vdbg("Checking overlap before mapping\n");

  check_symbol_overlap(symbols, number_of_symbols);

  vdbg("Mapping COMMONS\n");

  if (NULL == bss_s)
    {
      warn("NULL section passed to map_common_symbols\n");
      return;
    }

  vdbg("Assigning COMMON symbols to section %s\n", bss_s->name);

  for (i = 0; i < number_of_symbols; i++)
    {
      if (bfd_is_com_section(symbols[i]->section))
        {
          if (verbose)
            {
              message("COMMON sym[%04d] ", i);
              dump_symbol(symbols[i]);
            }

          /* get parameters of unmapped symbol */
#if 0
          align = ((elf_symbol_type *) symbols[i])->internal_elf_sym.st_value;
#else
          /* Ignore alignment - just make sure we're word aligned we're not
           * worrying about page boundaries since we're flat mem and we're not
           * really concerned with cache alignment - maybe someday */

          align = 0x04;
#endif
          size = ((elf_symbol_type *) symbols[i])->internal_elf_sym.st_size;

          if (0 == size)
            {
              dbg("Aero size symbol assumed to be a ptr size 4\n");
              size = 0x04;
            }

          if (size % 0x04)
            {
              dbg("non-mod4 symbol rounded up 4\n");
              size = ((size >> 2) + 1) << 2;
            }

          /* INSERT SYMBOL AT END OF BSS - MUCH MO BETTA */

          /* calulate transaction effects - insert blank b4 sym to get align */

          baseaddr = bss_s->COOKED_SIZE;
          symbase = ((baseaddr + align - 1) / align) * align;
          offset = (symbase + size) - baseaddr;

          vdbg("  ba: %08lx sb: %08lx al: %04lx sz: %04lx of: %04lx\n",
              (long)baseaddr, (long)symbase, (long)align, (long)size, (long)offset);

          /* Add space to bss segment and section */

          bss_info.high_mark += offset;
          bss_info.size += offset;
          bss_s->COOKED_SIZE += offset;
          bss_s->RAW_SIZE += offset;

          /* find all end markers and offset */

          for (j = 0; j < number_of_symbols; j++)
            {
              if (bss_s == symbols[j]->section)
                {
                  if (verbose)
                    {
                      message("Checking endsym? %08lx sym[%04d] ", (long)baseaddr, j);
                      dump_symbol(symbols[j]);
                    }

                  if (symbols[j]->value >= baseaddr)
                    {
                      symbols[j]->value += offset;
                      ((elf_symbol_type *) symbols[j])->internal_elf_sym.
                        st_value += offset;
                      if (verbose > 1)
                        {
                          message("Sym MOVED to sym[%04d] ", j);
                          dump_symbol(symbols[j]);
                        }
                    }
                }
            }

          /* stuff sym at base */

          symbols[i]->section = bss_s;
          symbols[i]->value = symbase;
          symbols[i]->flags = BSF_OBJECT | BSF_GLOBAL;
          ((elf_symbol_type *) symbols[i])->internal_elf_sym.st_value = symbase;

          if (verbose)
            {
              message("NEW sym[%04d] ", i);
              dump_symbol(symbols[i]);
            }
        }
    }

  check_symbol_overlap(symbols, number_of_symbols);

  vdbg("After map high mark %08lx cooked %08lx raw %08lx \n",
      (long)bss_info.high_mark, (long)bss_info.subsect[0]->COOKED_SIZE,
      (long)bss_info.subsect[0]->RAW_SIZE);
}

/***********************************************************************
 * resolve_relocs
 ***********************************************************************/

static void resolve_relocs(bfd *input_bfd, asymbol **symbols)
{
  resolve_segment_relocs(input_bfd, &text_info, symbols);
  resolve_segment_relocs(input_bfd, &data_info, symbols);
  resolve_segment_relocs(input_bfd, &bss_info, symbols);
}

/***********************************************************************
 * allocate_got
 ***********************************************************************/

/* The GOT lies at the beginning of D-Space.  Before we can process
 * any relocation data, we need to determine the size of the GOT.
 */

static void allocate_got(bfd *input_bfd, asymbol **symbols)
{
  allocate_segment_got(input_bfd, &text_info, symbols);
  allocate_segment_got(input_bfd, &data_info, symbols);
  allocate_segment_got(input_bfd, &bss_info, symbols);
}

/***********************************************************************
 * output_got
 ***********************************************************************/

/* Pull the sections that make up this segment in off disk */

static void output_got(int fd)
{
  struct nxflat_reloc_s *relocs;
  u_int32_t *got;
  int reloc_size;
  int reloc_type;
  int nrelocs;
  int i;
  int j;

  if (ngot_offsets > 0)
    {
      /* Allocate memory for the GOT */

      got = (u_int32_t*)malloc(got_size);
      if (!got)
        {
           err("Failed to allocate memory for the GOT (%d bytes, %d offsets)\n",
               got_size, ngot_offsets);
           exit(1);
        }

      /* Re-allocate memory for the relocations to include the GOT relocations */

      nrelocs    = ngot_offsets + nxflat_nrelocs;
      reloc_size = sizeof(struct nxflat_reloc_s) * nrelocs;
      relocs     = (struct nxflat_reloc_s*)realloc(nxflat_relocs, reloc_size);
      if (!relocs)
        {
           err("Failed to re-allocate memory for the GOT relocations (%d bytes, %d relocations)\n",
               reloc_size, nrelocs);
           exit(1);
        }

      /* Then initialize the GOT contents with the value associated with each symbol */

      for (i = 0; i < ngot_offsets; i++)
        {
          asymbol  *rel_sym     = got_offsets[i].sym;
          asection *rel_section = rel_sym->section;
          symvalue  sym_value   = rel_sym->value;

          /* j is the offset index into the relocatino table */

          j = i + nxflat_nrelocs;

          /* If the symbol is a thumb function, then set bit 1 of the value */

#ifdef NXFLAT_THUMB2
          if ((((elf_symbol_type *)rel_sym)->internal_elf_sym.st_info & 0x0f) == STT_ARM_TFUNC)
            {
               sym_value |= 1;
            }
#endif
          /* Determine where the symbol lies */

          switch (get_reloc_type(rel_section, NULL))
            {
              /* If the symbol lies in D-Space, then we need to add the size of the GOT
               * table to the symbol value
               */

            case NXFLAT_RELOC_TARGET_BSS:
            case NXFLAT_RELOC_TARGET_DATA:
              {
                vdbg("Symbol '%s' lies in D-Space\n", rel_sym->name);
                reloc_type = NXFLAT_RELOC_TYPE_REL32D;
                sym_value += got_size;
              }
              break;

              /* If the symbol lies in I-Space */

            case NXFLAT_RELOC_TARGET_TEXT:
              {
                vdbg("Symbol '%s' lies in I-Space\n", rel_sym->name);
                reloc_type = NXFLAT_RELOC_TYPE_REL32I;
              }
              break;

            case NXFLAT_RELOC_TARGET_UNKNOWN:
            default:
              {
                err("Relocation type is unknown\n");
                nerrors++;
                continue;
              }
            }

          /* Then save the symbol offset in the got */

          got[i] = sym_value;
          vdbg("GOT[%d]: sym name: '%s' value: %08lx->%08lx\n",
                i, rel_sym->name, (long)rel_sym->value, (long)sym_value);

          /* And output the relocation information associate with the GOT entry */

          relocs[j].r_info = NXFLAT_RELOC(reloc_type, sizeof(u_int32_t) * i);
 
          vdbg("relocs[%d]: type: %d offset: %08x\n",
                j, NXFLAT_RELOC_TYPE(relocs[j].r_info), NXFLAT_RELOC_OFFSET(relocs[j].r_info));
        }

      /* Write the GOT on the provided file descriptor */

      if (verbose > 1)
        {
           printf("GOT:\n");
           for (i = 0; i < ngot_offsets; i++)
             {
               printf("  Offset %-3ld: %08x\n",
                     (long)(sizeof(u_int32_t) * i), got[i]);
             }

           printf("Relocations:\n");
           for (i = 0; i < nrelocs; i++)
             {
               printf("  Offset %-3ld: %08x\n",
                     (long)(sizeof(struct nxflat_reloc_s) * i), relocs[i].r_info);
             }
        }

      nxflat_write(fd, (const char *)got, got_size);
      free(got);

      /* Return the relocation table (via global variables) */

      nxflat_relocs  = relocs;
      nxflat_nrelocs = nrelocs;
    }
}

/***********************************************************************
 * is_unwanted_section
 ***********************************************************************/

/* Return 1 if this is a section that we want to throw away but can only
 * identify by name.   Normally, any section with appropriate-looking flags
 * will get copied into the output file.
 */

static int is_unwanted_section(asection * s)
{
  if (!strcmp(s->name, ".hash"))
    return 1;
  if (!strcmp(s->name, ".dynstr") || !strcmp(s->name, ".dynsym"))
    return 1;
  if (s->flags & SEC_DEBUGGING)
    return 1;
  return 0;
}

/***********************************************************************
 * register_section
 ***********************************************************************/

/* Mark this section for inclusion in some segment.  */
static void register_section(asection * s, segment_info * inf)
{
  vdbg("registering section %s to %s segment\n", s->name, inf->name);
  inf->subsect[inf->nsubsects++] = s;
}

/***********************************************************************
 * dump_sections
 ***********************************************************************/

/* Print out the sections that make up this segment for debugging.  */
static void dump_sections(segment_info * inf)
{
  int i;
  printf("      [ ");
  for (i = 0; i < inf->nsubsects; i++)
    {
      printf("%s ", inf->subsect[i]->name);
    }
  printf("]\n");
}

/***********************************************************************
 * load_sections
 ***********************************************************************/

/* Pull the sections that make up this segment in off disk */

static void load_sections(bfd *bfd, segment_info *inf)
{
  void *ptr;
  int i;

  if (inf->size > 0)
    {
      inf->contents = malloc(inf->size);
      if (!inf->contents)
        {
          err("Failed to allocate memory for section contents.\n");
          exit(1);
        }

      ptr = inf->contents;
      for (i = 0; i < inf->nsubsects; i++)
        {
          if (!bfd_get_section_contents(bfd, inf->subsect[i], ptr,
                                        0, inf->subsect[i]->COOKED_SIZE))
            {
              err("Failed to read section contents.\n");
              exit(1);
            }
          ptr += inf->subsect[i]->COOKED_SIZE;
          inf->subsect[i]->flags |= SEC_IN_MEMORY;
        }
    }
}

/***********************************************************************
 * stack_nxflat_segment
 ***********************************************************************/

static void stack_nxflat_segment(segment_info * inf)
{
  bfd_vma min_addr = 0x7fffffff;
  bfd_vma max_addr = 0x00000000;
  int i;

  for (i = 0; i < inf->nsubsects; i++)
    {
      bfd_vma vma = inf->subsect[i]->vma;
      if (vma < min_addr)
        min_addr = vma;

      vma += inf->subsect[i]->COOKED_SIZE;
      if (vma > max_addr)
        max_addr = vma;
    }

  inf->low_mark = min_addr;
  inf->high_mark = max_addr;
  inf->size = max_addr - min_addr;
}

/***********************************************************************
 * show_usage
 ***********************************************************************/

static void show_usage(void)
{
  fprintf(stderr, "Usage: %s [options] <bfd-filename>\n\n", program_name);
  fprintf(stderr, "Where options are one or more of the following.  Note\n");
  fprintf(stderr, "that a space is always required between the option and\n");
  fprintf(stderr, "any following arguments.\n\n");
  fprintf(stderr, "  -d Use dynamic symbol table [Default: symtab]\n");
  fprintf(stderr, "  -e <entry-point>\n");
  fprintf(stderr, "     Entry point to module [Default: %s]\n",
          default_exe_entry_name);
  fprintf(stderr, "  -o <out-filename>\n");
  fprintf(stderr, "     Output to <out-filename> [Default: <bfd-filename>.nxf]\n");
  fprintf(stderr, "  -s <stack-size>\n");
  fprintf(stderr, "     Set stack size to <stack-size> [Default: %d]\n", DEFAULT_STACK_SIZE);
  fprintf(stderr, "  -v Verbose outpu.t If -v is applied twice, additional\n");
  fprintf(stderr, "     debug output is enabled [Default: no verbose output].\n");
  fprintf(stderr, "\n");
  exit(2);
}

/***********************************************************************
 * parse_args
 ***********************************************************************/

static void parse_args(int argc, char **argv)
{
  int opt;

  /* Save our name (for show_usage) */

  program_name = argv[0];

  /* Set some default values */

  stack_size = 0;
  entry_name = NULL;

  if (argc < 2)
    {
      err("Missing required arguments\n\n");
      show_usage();
    }

  /* Get miscellaneous options from the command line. */

  while ((opt = getopt(argc, argv, "de:lo:s:v")) != -1)
    {
      switch (opt)
        {
        case 'd':
          dsyms++;
          break;

        case 'e':
          entry_name = strdup(optarg);
          break;

        case 'o':
          out_filename = optarg;
          break;

        case 's':
          stack_size = atoi(optarg);
          break;

        case 'v':
          verbose++;
          break;

        case 'l':
        default:
          err("%s Unknown option\n\n", argv[0]);
          show_usage();
          break;
        }
    }

  /* The very last thing is also the name of the BFD input file */

  bfd_filename = argv[argc - 1];

  /* Verify that an appropriate stack size is selected. */

  if (stack_size == 0)
    {
      /* Executables must have a stack_size selected. */

      printf("Using default stack size: %d\n", DEFAULT_STACK_SIZE);
      stack_size = DEFAULT_STACK_SIZE;
    }

  if (entry_name == NULL)
    {
      printf("Using entry_point: %s\n", default_exe_entry_name);
      entry_name = default_exe_entry_name;
    }
}

/***********************************************************************
 * Public Functions
 ***********************************************************************/

/***********************************************************************
 * main
 ***********************************************************************/

int main(int argc, char **argv, char **envp)
{
  struct nxflat_hdr_s hdr;
  bfd *bf;
  asection *s;
  asymbol **symbol_table;
  int32_t number_of_symbols;
  u_int32_t offset;
  int fd;
  int i;

  /* Parse the incoming command line */

  parse_args(argc, argv);

  /* Open the BFD input file */

  if (!(bf = bfd_openr(argv[argc - 1], 0)))
    {
      err("Failed to open %s\n", argv[argc - 1]);
      exit(1);
    }

  /* Verify the format of the BFD file */

  if (bfd_check_format(bf, bfd_object) == 0)
    {
      err("File is not an object file\n");
      exit(2);
    }

  /* Read the symbol table from the file */

  symbol_table = get_symbols(bf, &number_of_symbols);

  /* Find all of the special symbols that we will need in the symbol table that 
   * we just read. */

  find_special_symbols();

  /* Walk the list of sections, figuring out where each one goes and how much
   * storage it requires. */

  text_info.low_mark = data_info.low_mark = bss_info.low_mark = -1;
  text_info.high_mark = data_info.high_mark = bss_info.high_mark = 0;
  text_info.contents = data_info.contents = bss_info.contents = NULL;
  text_info.size = data_info.size = bss_info.size = 0;
  text_info.nsubsects = data_info.nsubsects = bss_info.nsubsects = 0;
  text_info.name = "text";
  data_info.name = "data";
  bss_info.name = "bss";

  for (s = bf->sections; s != NULL; s = s->next)
    {
      dbg("Reading section %s\n", s->name);

      /* ignore blatantly useless sections */

      if (!is_unwanted_section(s))
        {
          if (s->flags == SEC_ALLOC)
            {
              vdbg("  Section %s is ALLOC only\n", s->name);
              register_section(s, &bss_info);
            }
          else if ((s->flags & SEC_CODE) != 0 && (s->flags & SEC_ALLOC) != 0)
            {
              vdbg("  Section %s is CODE\n", s->name);
              register_section(s, &text_info);
            }
          else if ((s->flags & SEC_DATA) != 0 && (s->flags & SEC_ALLOC) != 0)
            {
              vdbg("  Section %s is DATA\n", s->name);
              register_section(s, &data_info);
            }
          else
            {
              vdbg("WARNING: ignoring section %s\n", s->name);
            }
        }
    }

  /* Fixup high and low water VMA address */

  stack_nxflat_segment(&text_info);
  stack_nxflat_segment(&data_info);
  stack_nxflat_segment(&bss_info);

  /* Check for a data offset due to the presence of a GOT */

  printf("INPUT SECTIONS:\n");
  printf("SECT LOW      HIGH     SIZE\n");
  if (text_info.nsubsects == 0)
    {
      warn("TEXT Not found  Not found ( Not found )\n");
    }
  else
    {
      printf("TEXT %08lx %08lx %08lx\n",
             (long)text_info.low_mark, (long)text_info.high_mark,
             (long)text_info.size);

      if (text_info.low_mark != 0)
        {
          err("Text section must be origined at zero");
          exit(1);
        }
    }

  if (data_info.nsubsects == 0)
    {
      warn("DATA Not found  Not found ( Not found )\n");
    }
  else
    {
      printf("DATA %08lx %08lx %08lx\n",
             (long)data_info.low_mark, (long)data_info.high_mark,
             (long)data_info.size);

      if (data_info.low_mark != 0)
        {
          err("data section must be origined at zero");
          exit(1);
        }
    }

  if (bss_info.nsubsects == 0)
    {
      warn("BSS  Not found  Not found ( Not found )\n");
    }
  else
    {
      printf("BSS  %08lx %08lx %08lx\n",
             (long)bss_info.low_mark, (long)bss_info.high_mark,
             (long)bss_info.size);

      /* If data is present, then BSS was be origined immediately after the
       * data. */

      if (data_info.nsubsects > 0)
        {
          /* There is data... Account for possible ALIGN 0x10 at end of data */

          u_int32_t bss_start1 = data_info.high_mark;
          u_int32_t bss_start2 = ((bss_start1 + 0x0f) & ~0x0f);

          if ((bss_info.low_mark < bss_start1) &&
              (bss_info.low_mark > bss_start2))
            {
              err("BSS must be origined immediately after the data section\n");
              exit(1);
            }
        }

      /* If there is no data, then the BSS must be origined at zero */

      else if (bss_info.low_mark != 0)
        {
          err("BSS section (with no data section) must be origined at zero\n");
          exit(1);
        }
    }

  if (verbose)
    {
      message("TEXT: ");
      dump_sections(&text_info);
      message("DATA: ");
      dump_sections(&data_info);
      message("BSS: ");
      dump_sections(&bss_info);
    }

  /* Slurp the section contents in.  No need to load BSS since we know it
   * isn't initialised. */

  load_sections(bf, &text_info);
  load_sections(bf, &data_info);

  /* Unmapped 'common' symbols need to be stuffed into bss */

  map_common_symbols(bf, symbol_table, number_of_symbols);

  /* Dump symbol information */

  if (verbose)
    {
      for (i = 0; i < number_of_symbols; i++)
        {
          message("sym[%04d] ", i);
          dump_symbol(symbol_table[i]);
        }
    }

  /* The GOT lies at the beginning of D-Space.  Before we can process
   * any relocation data, we need to determine the size of the GOT.
   */

  allocate_got(bf, symbol_table);

  /* Then process all of the relocations */

  resolve_relocs(bf, symbol_table);

  /* Fill in the NXFLAT file header */

  memcpy(hdr.h_magic, NXFLAT_MAGIC, 4);

  offset = NXFLAT_HDR_SIZE + text_info.size;
  put_xflat32(&hdr.h_datastart, offset);

  offset += got_size + data_info.size;
  put_xflat32(&hdr.h_dataend, offset);
  put_xflat32(&hdr.h_relocstart, offset);

  offset += bss_info.size;
  put_xflat32(&hdr.h_bssend, offset);

  put_xflat32(&hdr.h_stacksize, stack_size);
  put_xflat16(&hdr.h_reloccount, nxflat_nrelocs + ngot_offsets);

  put_entry_point(&hdr);

  put_special_symbol(dynimport_begin_symbol, dynimport_end_symbol,
                     &hdr.h_importsymbols, &hdr.h_importcount,
                     sizeof(struct nxflat_import_s), text_info.size + got_size);

  /* Open the output file */

  if (!out_filename)
    {
      out_filename = malloc(strlen(bfd_filename) + 5);  /* 5 to add suffix */
      strcpy(out_filename, bfd_filename);
      strcat(out_filename, ".nxf");
    }

  fd = open(out_filename, O_WRONLY | O_PLATFORM | O_CREAT | O_TRUNC, 0744);
  if (fd < 0)
    {
      err("Failed open output file %s: %s\n", out_filename, strerror(errno));
      exit(4);
    }

  /* Write the data in the following order in order to match the NXFLAT header
   * offsets:  HDR, ISPACE, GOT, DSPACE, RELOCS.
   */

  nxflat_write(fd, (const char *)&hdr, NXFLAT_HDR_SIZE);
  nxflat_write(fd, (const char *)text_info.contents, text_info.size);

  if (ngot_offsets > 0)
    {
      output_got(fd);
    }

  nxflat_write(fd, (const char *)data_info.contents, data_info.size);

  if (nxflat_relocs)
    {
      vdbg("Number of GOT relocations: %d\n", ngot_offsets);

#ifdef RELOCS_IN_NETWORK_ORDER
      for (i = 0; i < nxflat_nrelocs; i++)
        {
          nxflat_relocs[i] = htonl(nxflat_relocs[i]);
        }
#endif
      nxflat_write(fd, (const char *)nxflat_relocs, sizeof(struct nxflat_reloc_s) * nxflat_nrelocs);
    }

  /* Finished! */

  close(fd);

  if (nerrors > 0)
    {
       fprintf(stderr, "%d Errors detected\n", nerrors);
       return 1;
    }
  return 0;
}
