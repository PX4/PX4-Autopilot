/****************************************************************************
 * configs/pic32-starterkit/tools/mkpichex.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_LINE         1024

/* Line offsets */

#define START_OFFSET     0
#define LEN_OFFSET       1
#define ADDR_OFFSET      (LEN_OFFSET + 2)
#define TYPE_OFFSET      (ADDR_OFFSET + 4)
#define PAYLOAD_OFFSET   (TYPE_OFFSET + 2)
#define CHKSUM_OFFSET(n) (PAYLOAD_OFFSET+2*(n))

/* Record types:
 *
 * 00, data record, contains data and 16-bit address. The format described
 *     above.
 * 01, End Of File record, a file termination record. No data. Has to be
 *     the last line of the file, only one per file permitted. Usually
 *     ':00000001FF'. Originally the End Of File record could contain a
 *     start address for the program being loaded, e.g. :00AB2F0125
 *     would make a jump to address AB2F. This was convenient when programs
 *     were loaded from punched paper tape.
 * 02, Extended Segment Address Record, segment-base address. Used when 16
 *     bits are not enough, identical to 80x86 real mode addressing. The
 *     address specified by the 02 record is multiplied by 16 (shifted 4
 *     bits left) and added to the subsequent 00 record addresses. This
 *     allows addressing of up to a megabyte of address space. The address
 *     field of this record has to be 0000, the byte count is 02 (the segment
 *     is 16-bit). The least significant hex digit of the segment address is
 *     always 0.
 * 03, Start Segment Address Record. For 80x86 processors, it specifies the
 *     initial content of the CS:IP registers. The address field is 0000, the
 *     byte count is 04, the first two bytes are the CS value, the latter two
 *     are the IP value.
 * 04, Extended Linear Address Record, allowing for fully 32 bit addressing.
 *     The address field is 0000, the byte count is 02. The two data bytes
 *     represent the upper 16 bits of the 32 bit address, when combined with
 *     the address of the 00 type record.
 * 05, Start Linear Address Record. The address field is 0000, the byte
 *     count is 04. The 4 data bytes represent the 32-bit value loaded into
 *     the EIP register of the 80386 and higher CPU.
 */

#define TYPE_DATA     0
#define TYPE_EOF      1
#define TYPE_EXTSEG   2
#define TYPE_STARTSEG 3
#define TYPE_EXTLIN   4
#define TYPE_STARTLIN 5

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct hex_s
{
  unsigned char  len;      /* Length of the data payload */
  unsigned char  type;     /* Record type */
  unsigned short addr;     /* Lower 16-bit address */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char line[MAX_LINE+1];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline char *getfilepath(const char *path, const char *name, const char *extension)
{
  snprintf(line, MAX_LINE, "%s/%s.%s", path, name, extension);
  line[MAX_LINE] = '\0';
  return strdup(line);
}

static void show_usage(const char *progname)
{
  fprintf(stderr, "USAGE: %s <abs path to nuttx.hex>\n", progname);
  exit(1);
}

static unsigned char get4(char hex)
{
  if (hex >= '0' && hex <= '9')
    {
      return hex - '0';
    }
  else if (hex >= 'a' && hex <= 'f')
    {
      return hex - 'a' + 10;
    }
  else if (hex >= 'A' && hex <= 'F')
    {
      return hex - 'A' + 10;
    }

  fprintf(stderr, "Bad hex character code: %s\n", line);
  exit(2);
}

static unsigned char get8(const char *ptr)
{
  return get4(ptr[0]) << 4 | get4(ptr[1]);
}

static unsigned short get16(const char *ptr)
{
  return (unsigned short)get8(&ptr[0]) << 8 | (unsigned short)get8(&ptr[2]);
}

static int parse_line(struct hex_s *hexline)
{
  /* :LLAAAATT... */

  if (line[START_OFFSET] != ':')
    {
      fprintf(stderr, "Bad start code: %s\n", line);
      return 1;
    }

  hexline->len  = get8(&line[LEN_OFFSET]);
  hexline->addr = get16(&line[ADDR_OFFSET]);
  hexline->type = get8(&line[TYPE_OFFSET]);
  return 0;
}

#if 0
static unsigned char checksum(chksum_ndx)
{
  int chksum = 0;
  int ndx;

  for (ndx = 1; ndx < chksum_ndx; ndx += 2)
    {
      chksum += (int)get8(&line[ndx]);
    }
  return (unsigned char)((-chksum) & 0xff);
}
#endif

static void adjust_extlin(struct hex_s *hexline)
{
  unsigned short segment;
  int chksum;

  /* Make sure that the payload is exactly 2 bytes */

  if (hexline->len != 2)
    {
      fprintf(stderr, "Bad length on extended segment address record\n");
      fprintf(stderr, "  %s", line);
    }

  /* And the address field is supposed to be zero */

  if (hexline->addr != 0)
    {
      fprintf(stderr, "Bad address on extended segment address record\n");
      fprintf(stderr, "  %s", line);
    }

  /* Decode the 2 byte payload */

  segment = get16(&line[PAYLOAD_OFFSET]);

  /* Convert the address to a 29-bit physical address */

  segment &= 0x1fff;

  /* Recalculate the checksum and make sure that there is a null terminator
   * Since len=2, addr=0, type=4, the is a trivial calculation.
   */

  chksum = (-(segment + (segment >> 8) + 6)) & 0xff;

  /* Then create the new output record */

  snprintf(line, MAX_LINE-PAYLOAD_OFFSET, ":02000004%04X%02X\n", segment, chksum);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  struct hex_s hexline;
  char *srcfile;
  char *destfile;
  FILE *src;
  FILE *dest;

  if (argc != 2)
    {
      fprintf(stderr, "Unexpected number of arguments\n");
      show_usage(argv[0]);
    }

  srcfile = getfilepath(argv[1], "nuttx", "hex");
  if (!srcfile)
    {
      fprintf(stderr, "getfilepath failed\n");
      exit(2);
    }

  destfile = getfilepath(argv[1], "nuttx", "tmp");
  if (!destfile)
    {
      fprintf(stderr, "getfilepath failed\n");
      exit(2);
    }

  src = fopen(srcfile, "r");
  if (!src)
    {
      fprintf(stderr, "open %s failed: %s\n", srcfile, strerror(errno));
      exit(3);
    }

  dest = fopen(destfile, "w");
  if (!dest)
    {
      fprintf(stderr, "open %s failed: %s\n", destfile, strerror(errno));
      exit(3);
    }

  /* Read each line from the source file */

  while (fgets(line, MAX_LINE, src) != NULL)
    {
      if (parse_line(&hexline))
        {
          fprintf(stderr, "Failed to parse line\n");
          exit(1);
        }

      /* Adjust 'Extended Segment Address Records'. */

      if (hexline.type == TYPE_EXTLIN)
        {
          adjust_extlin(&hexline);
        }
      fputs(line, dest);
    }

  fclose(src);
  fclose(dest);

  /* Remove the original nuttx.hex file */

  if (remove(srcfile) != 0)
    {
      fprintf(stderr, "Failed to remove the old '%s'\n", srcfile);
    
    }

  /* Rename the new nuttx.tmp file to nuttx.hex */

  if (rename(destfile, srcfile) != 0)
    {
      fprintf(stderr, "Failed to rename '%s' to '%s'\n", destfile, srcfile);
    }

  return 0;
}
