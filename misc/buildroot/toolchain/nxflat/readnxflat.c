/***********************************************************************
 * toolchain/nxflat/readnxflat.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Modified from readxflat (see http://xflat.org):
 *
 *   Copyright (c) 2002, 2006, Cadenux, LLC.  All rights reserved.
 *   Copyright (c) 2002, 2006, Gregory Nutt.  All rights reserved.
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

/***********************************************************************
 * Compilation Flags
 ***********************************************************************/

#define SWAP_BYTES 1

/***********************************************************************
 * Included Files
 ***********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <netinet/in.h>         /* ntohl and friends */
#include "nxflat.h"

/***********************************************************************
 * Compilation Switches
 ***********************************************************************/

/* #define RELOCS_IN_NETWORK_ORDER */

/***********************************************************************
 * Definitions
 ***********************************************************************/
 
#define NXFLAT_HDR_SIZE   sizeof(struct nxflat_hdr_s)

/***********************************************************************
 * Private Data
 ***********************************************************************/

static const char *program_name;
static const char *nxflat_filename;

static int dump_header = 0;
static int dump_relocs = 0;
static int dump_imports = 0;
static int dump_text = 0;
static int dump_data = 0;
static int verbose = 0;

static int num_errors = 0;

#ifdef ARCH_BIG_ENDIAN
static int big_endian = 1;      /* Assume big-endian */
#else
static int big_endian = 0;      /* Assume little-endian */
#endif

/***********************************************************************
 * Private Constant Data
 ***********************************************************************/

static const char unknown[] = "UNKNOWN";

static const char hdr_reloc_rel32i[]  = "RELOC_REL32I";
static const char hdr_reloc_rel32d[]  = "RELOC_REL32D";
#ifdef NXFLAT_RELOC_TYPE_REL32ID
static const char hdr_reloc_rel32id[] = "RELOC_REL32ID";
#endif

static const char *reloc_type_string[] = {
  hdr_reloc_rel32i,
  hdr_reloc_rel32d,
#ifdef NXFLAT_RELOC_TYPE_REL32ID
  hdr_reloc_rel32id,
#else
  unknown,
#endif
  unknown
};

/***********************************************************************
 * Public Function Prototypes
 ***********************************************************************/

extern void __attribute__ ((weak)) print_insn_arm(u_int32_t pc,
                                                  FILE * stream,
                                                  u_int32_t given);

/***********************************************************************
 * Private Functions
 ***********************************************************************/

/***********************************************************************
 * swap32
 ***********************************************************************/

static inline u_int32_t nxflat_swap32(u_int32_t little)
{
  u_int32_t big =
    ((little >> 24) & 0xff) |
    (((little >> 16) & 0xff) << 8) |
    (((little >> 8) & 0xff) << 16) | ((little & 0xff) << 24);
  return big;
}

/***********************************************************************
 * get_nxflat32
 ***********************************************************************/

static inline u_int32_t get_nxflat32(u_int32_t * addr32)
{
  return ntohl(*addr32);
}

/***********************************************************************
 * get_nxflat16
 ***********************************************************************/

static inline u_int16_t get_nxflat16(u_int16_t * addr16)
{
  return ntohs(*addr16);
}

/***********************************************************************
 * dump_hex_data
 ***********************************************************************/

static void dump_hex_data(FILE * in_stream, struct nxflat_hdr_s *header)
{
  u_int32_t data_start = get_nxflat32(&header->h_datastart);
  u_int32_t data_end = get_nxflat32(&header->h_dataend);
  int32_t words_left = (data_end - data_start) / sizeof(u_int32_t);
  u_int32_t addr;
  u_int32_t buffer[64];

  printf("\nXFLAT DATA SEGMENT:\n\n");

  /* Seek to the beginning of data in the file */

  if (fseek(in_stream, data_start, SEEK_SET) != 0)
    {
      fprintf(stderr,
              "ERROR: Failed to seek to data in file: offset: %08x\n",
              data_start);
      return;
    }

  /* Now dump all of the data reading 64 words at a time */

  addr = 0;
  while (words_left > 0)
    {
      size_t nread = fread(buffer, sizeof(u_int32_t), 64, in_stream);
      if (nread >= 0)
        {
          union
            {
              u_int32_t l[4];
              unsigned char b[16];
            } row;
          int32_t i, j, k;

          for (i = 0; i < nread; i += 4)
            {
              for (j = 0; j < 4; j++)
                {
                  row.l[j] = buffer[i + j];
                }

              printf("%08x: ", addr);

              for (j = 0; j < 4; j++)
                {
                  printf("%08x ", row.l[j]);
                }

              printf("  ");

              for (j = 0; j < 4 * sizeof(u_int32_t); j += sizeof(u_int32_t))
                {
                  for (k = 0; k < sizeof(u_int32_t); k++)
                    {
                      if (isprint(row.b[j + k]))
                        putchar(row.b[j + k]);
                      else
                        putchar('.');
                    }
                }

              putchar('\n');
              addr += 4 * sizeof(u_int32_t);
            }
          words_left -= nread;
        }
      else
        break;
    }
  putchar('\n');
}

/***********************************************************************
 * disassemble_text
 ***********************************************************************/

static void disassemble_text(FILE * in_stream, struct nxflat_hdr_s *header)
{
  if (print_insn_arm)
    {
      u_int32_t text_start = NXFLAT_HDR_SIZE;
      u_int32_t text_end = get_nxflat32(&header->h_datastart);
      int32_t insns_left = (text_end - text_start) / sizeof(u_int32_t);
      u_int32_t addr;
      u_int32_t buffer[64];

      printf("\nXFLAT TEXT:\n\n");

      /* Seek to the beginning of text in the file */

      if (fseek(in_stream, text_start, SEEK_SET) != 0)
        {
          fprintf(stderr,
                  "ERROR: Failed to seek to text in file: offset: %08x\n",
                  text_start);
          return;
        }

      /* Now dump all of the data reading 64 insns at a time */

      addr = text_start;
      while (insns_left > 0)
        {
          size_t nread = fread(buffer, sizeof(u_int32_t), 64, in_stream);
          if (nread > 0)
            {
              int i;
              for (i = 0; i < nread; i++)
                {
                  u_int32_t insn = buffer[i];
                  if (big_endian)
                    {
                      insn = nxflat_swap32(insn);
                    }

                  printf("%08x %08x\t", addr, insn);
                  print_insn_arm(addr, stdout, insn);
                  putchar('\n');
                  addr += sizeof(u_int32_t);
                }
              insns_left -= nread;
            }
          else
            break;
          putchar('\n');
        }
    }
}

/***********************************************************************
 * dump_imported_symbols
 ***********************************************************************/

static void dump_imported_symbols(FILE *in_stream, struct nxflat_hdr_s *header)
{
  struct nxflat_import_s import;
  u_int32_t import_offset;
  u_int32_t data_start;
  u_int32_t struct_offset;
  u_int32_t name_offset;
  char imported_symbol_name[NXFLAT_MAX_STRING_SIZE];
  int status;
  int i;

  printf("\nIMPORTED SYMBOLS:\n");
  printf("      OFFSET   ADDRESS  SYMBOL NAME\n\n");

  import_offset = get_nxflat32(&header->h_importsymbols);
  data_start    = get_nxflat32(&header->h_datastart);

  for (i = 0; i < get_nxflat16(&header->h_importcount); i++)
    {
      /* Seek to the next imported symbol */

      struct_offset = i * sizeof(struct nxflat_import_s) + import_offset;

      if (fseek(in_stream, struct_offset, SEEK_SET) != 0)
        {
          fprintf(stderr, "ERROR: fseek to imported symbol %d struct failed\n",
                  i);
          fprintf(stderr, "       struct_offset: %d: %s\n",
                  struct_offset, strerror(errno));
          exit(1);
        }

      /* Read the next import entry. */

      status = fread((void *)&import,
                     sizeof(struct nxflat_import_s), 1, in_stream);
      if (status != 1)
        {
          if (ferror(in_stream))
            {
              fprintf(stderr, "ERROR: Read imported symbol %d struct failed: %s\n",
                      i + 1, strerror(errno));
            }
          else
            {
              fprintf(stderr, "ERROR: Read imported symbol %d struct: End-of-file after %d\n",
                      i + 1, struct_offset);
            }

          exit(1);
        }

      if (big_endian)
        {
          import.i_funcname    = nxflat_swap32(import.i_funcname);
          import.i_funcaddress = nxflat_swap32(import.i_funcaddress);
        }

      if (verbose)
        {
          /* Print the raw info */

          printf("[Import: %4d Offset: %08x Name: %08x Address: %08x]\n",
                 i + 1, struct_offset, import.i_funcname, import.i_funcaddress);
        }

      /* Seek to the function name in the file */

      name_offset = import.i_funcname + NXFLAT_HDR_SIZE;

      if (fseek(in_stream, name_offset, SEEK_SET) != 0)
        {
          fprintf(stderr, "ERROR: fseek to imported symbol %d name failed\n",
                  i);
          fprintf(stderr, "       name_offset: %d %s\n",
                  name_offset, strerror(errno));
          exit(1);
        }

      /* Then, read the imported symbol name (assuming it is less than
       * NXFLAT_MAX_STRING_SIZE in length). */

      status = fread((void *)imported_symbol_name, NXFLAT_MAX_STRING_SIZE,
                     1, in_stream);

      if (status != 1)
        {
          if (ferror(in_stream))
            {
              fprintf(stderr, "ERROR: Read imported symbol %d name failed: %s\n",
                      i + 1, strerror(errno));
            }
          else
            {
              fprintf(stderr, "ERROR: Read imported symbol %d name: End-of-file after offset=%d\n",
                      i + 1, name_offset);
            }
          exit(1);
        }

      imported_symbol_name[NXFLAT_MAX_STRING_SIZE - 1] = '\0';

      /* And print it */

      printf("%5d %08x ", i + 1, (int)struct_offset - data_start);

      if (import.i_funcaddress)
        {
          printf("%08x ", import.i_funcaddress);
        }
      else
        {
          printf("UNKNOWN  ");
        }

      printf("%s\n", imported_symbol_name);
    }
}

/***********************************************************************
 * dump_relocation_entries
 ***********************************************************************/

static void dump_relocation_entries(FILE * in_stream, struct nxflat_hdr_s *header)
{
  struct nxflat_reloc_s reloc;
  int status;
  int i;

  /* Seek to the beginning of the relocation records. */

  if (0 != fseek(in_stream, get_nxflat32(&header->h_relocstart), SEEK_SET))
    {
      fprintf(stderr, "ERROR: fseek to reloc records failed: %s\n",
              strerror(errno));
      exit(1);
    }

  printf("\nRELOCATION ENTRIES:\n");
  printf("      OFFSET   RELOC TYPE\n\n");

  for (i = 0; i < get_nxflat16(&header->h_reloccount); i++)
    {
      /* Read the next reloction entry. */

      status = fread((void *)&reloc, sizeof(struct nxflat_reloc_s), 1, in_stream);
      if (status != 1)
        {
          if (ferror(in_stream))
            {
              fprintf(stderr, "ERROR: Read reloc record %d: %s\n",
                      i + 1, strerror(errno));
            }
          else
            {
              fprintf(stderr, "ERROR: Read reloc record %d: End-of-file\n",
                      i + 1);
            }
          exit(1);
        }
 
#ifdef RELOCS_IN_NETWORK_ORDER
      {
        u_int32_t *ptmp;
        ptmp = (u_int32_t *) & reloc;
        *ptmp = get_nxflat32(ptmp);
      }
#endif

      if (NXFLAT_RELOC_TYPE(reloc.r_info) >= NXFLAT_RELOC_TYPE_NUM)
        {
          printf("%5d %08x UNKNOWN(%d)\n", i + 1,
                 NXFLAT_RELOC_OFFSET(reloc.r_info), NXFLAT_RELOC_TYPE(reloc.r_info));
          fprintf(stderr, "Error eloc type out of range(%d)\n",
                  NXFLAT_RELOC_TYPE(reloc.r_info));
          num_errors++;
        }
      else
        {
          printf("%5d %08x %-13s\n",
                 i + 1, NXFLAT_RELOC_OFFSET(reloc.r_info),
                 reloc_type_string[NXFLAT_RELOC_TYPE(reloc.r_info)]);
        }
    }
}

/***********************************************************************
 * dump_hdr
 ***********************************************************************/

static void dump_hdr(struct nxflat_hdr_s *header)
{
  /* Print the contents of the FLT header */

  printf("\nXFLAT HEADER:\n");
  printf("\nMagic           %c%c%c%c\n",
         header->h_magic[0], header->h_magic[1],
         header->h_magic[2], header->h_magic[3]);

  printf("\nMEMORY MAP:\n");
  printf("  Text start    %08lx\n", (long)NXFLAT_HDR_SIZE);
  printf("  Entry point   %08x\n", get_nxflat32(&header->h_entry));
  printf("  Data start    %08x\n", get_nxflat32(&header->h_datastart));
  printf("  Data end      %08x\n", get_nxflat32(&header->h_dataend) - 1);
  printf("  Bss start     %08x\n", get_nxflat32(&header->h_dataend));
  printf("  Bss end       %08x\n", get_nxflat32(&header->h_bssend) - 1);
  printf("TOTAL SIZE      %08x\n\n", get_nxflat32(&header->h_bssend));
  printf("Stack size      %08x\n", get_nxflat32(&header->h_stacksize));
  printf("\nRELOCATIONS:\n");
  printf("  Reloc start   %08x\n", get_nxflat32(&header->h_relocstart));
  printf("  reloc count   %d\n", get_nxflat16(&header->h_reloccount));
  printf("\nIMPORTED SYMBOLS:\n");
  printf("  Import start  %08x\n", get_nxflat32(&header->h_importsymbols));
  printf("  Import count  %d\n", get_nxflat16(&header->h_importcount));
}

/***********************************************************************
 * show_usage
 ***********************************************************************/

static void show_usage(void)
{
  fprintf(stderr, "Usage: %s [options] <flat-filename>\n\n", program_name);
#if 1
  fprintf(stderr, "Where options are one or more of the following:\n\n");
#else
  fprintf(stderr, "Where options are one or more of the following.  Note\n");
  fprintf(stderr, "that a space is always required between the option and\n");
  fprintf(stderr, "any following arguments\n\n");
#endif
  fprintf(stderr, "  -h Dump the XFLAT file header     [not dumped]\n");
  fprintf(stderr, "  -r Dump relocation entries        [not dumped]\n");
  fprintf(stderr, "  -i Dump the imported symbol table [not dumped]\n");
  fprintf(stderr, "  -x Dump xFLT loader pathname      [not dumped]\n");
  fprintf(stderr, "  -c Disassemble the text section   [not dumped]\n");
  fprintf(stderr, "  -d Dump data section (hex)        [not dumped]\n");
  fprintf(stderr, "  -a Dump all of the above          [not dumped]\n");
#ifdef ARCH_BIG_ENDIAN
  fprintf(stderr, "  -b Assume little-endian byteorder [big endian]\n");
#else
  fprintf(stderr, "  -b Assume big-endian byteorder    [little endian]\n");
#endif
  fprintf(stderr, "  -v Output verbose debug info      [no output]\n");
  fprintf(stderr, "\n");
  exit(1);
}

/***********************************************************************
 * parse_args
 ***********************************************************************/

static void parse_args(int argc, char **argv)
{
  int opt;

  /* Save our name (for show_usage) */

  program_name = argv[0];

  /* At least three things must appear on the program line: the program name,
   * the BFD filname, and at least one option. */

  if (argc < 3)
    {
      fprintf(stderr, "ERROR: Missing required arguments\n\n");
      show_usage();
    }

  /* Get miscellaneous options from the command line. */

  while ((opt = getopt(argc, argv, "hrieLlxcbdav")) != -1)
    {
      switch (opt)
        {

        case 'h':              /* Dump the flat file header */
          dump_header++;
          break;

        case 'r':              /* Dump the flat file header */
          dump_relocs++;
          break;

        case 'i':              /* Dump the imported symbol table */
          dump_imports++;
          break;

        case 'c':              /* Disassembly text */
          if (print_insn_arm)
            {
              dump_text++;
            }
          else
            {
              printf("-c ignored: No disassembler available\n");
            }
          break;

        case 'b':              /* other-endian */
#ifdef ARCH_BIG_ENDIAN
          big_endian = 0;
#else
          big_endian++;
#endif
          break;

        case 'd':              /* Dump data */
          dump_data++;
          break;

        case 'a':              /* Dump everying */
          dump_header++;
          dump_relocs++;
          dump_imports++;
          dump_text++;
          dump_data++;
          break;

        case 'v':              /* Output verbose debug information */
          verbose++;
          break;

        default:
          fprintf(stderr, "%s Unknown option\n\n", argv[0]);
          show_usage();
          break;
        }
    }

  /* Get the name of the input BFD file. */

  nxflat_filename = argv[argc - 1];
}

/***********************************************************************
 * Public Functions
 ***********************************************************************/

/***********************************************************************
 * main
 ***********************************************************************/

int main(int argc, char **argv, char **envp)
{
  FILE *in_stream;
  struct nxflat_hdr_s header;
  int status;

  /* Get the input parameters */

  parse_args(argc, argv);

  /* Open the FLT file */

  in_stream = fopen(nxflat_filename, "rb");
  if (NULL == in_stream)
    {
      fprintf(stderr, "Cannot open file %s for reading\n", nxflat_filename);
      exit(1);
    }

  /* Read the FLT header */

  status = fread((void *)&header, sizeof(struct nxflat_hdr_s), 1, in_stream);
  if (status != 1)
    {
      if (ferror(in_stream))
        {
          fprintf(stderr, "ERROR: Read flat header: %s\n", strerror(errno));
        }
      else
        {
          fprintf(stderr, "ERROR: Read flat header: End-of-file\n");
        }
      exit(1);
    }

  printf("Dumping Flat Binary File: %s\n", nxflat_filename);

  /* Dump the contents of the FLT header */

  if (dump_header)
    {
      dump_hdr(&header);
    }

  /* Dump the relocation entries */

  if (dump_relocs)
    {
      dump_relocation_entries(in_stream, &header);
    }

  /* Dump all imported symbols */

  if (dump_imports)
    {
      dump_imported_symbols(in_stream, &header);
    }

   if (dump_text)
    {
      disassemble_text(in_stream, &header);
    }

  if (dump_data)
    {
      dump_hex_data(in_stream, &header);
    }

  fclose(in_stream);

  if (num_errors > 0)
    {
      fprintf(stderr, "Finished with %d errors\n", num_errors);
    }

  return 0;
}
