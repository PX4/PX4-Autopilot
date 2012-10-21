/****************************************************************************
 * apps/system/free/free.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
#include <nuttx/progmem.h>

#include <stdio.h>
#include <stdlib.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* TODO Max block size only works on uniform prog mem */

static void free_getprogmeminfo(struct mallinfo * mem)
{
  uint16_t page = 0, stpage = 0xFFFF;
  uint16_t pagesize = 0;
  int status;

  mem->arena    = 0;
  mem->fordblks = 0;
  mem->uordblks = 0;
  mem->mxordblk = 0;

  for (status=0, page=0; status >= 0; page++)
    {
      status = up_progmem_ispageerased(page);
      pagesize = up_progmem_pagesize(page);

      mem->arena += pagesize;

      /* Is this beginning of new free space section */

      if (status == 0)
        {
          if (stpage == 0xFFFF) stpage = page;
          mem->fordblks += pagesize;
        }
      else if (status != 0)
        {
          mem->uordblks += pagesize;

          if (stpage != 0xFFFF && up_progmem_isuniform())
            {
              stpage = page - stpage;
              if (stpage > mem->mxordblk)
                {
                  mem->mxordblk = stpage;
                }
              stpage = 0xFFFF;
            }
        }
    }

  mem->mxordblk *= pagesize;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int free_main(int argc, char **argv)
{
  struct mallinfo data;
  struct mallinfo prog;

#ifdef CONFIG_CAN_PASS_STRUCTS
  data = mallinfo();
#else
  (void)mallinfo(&data);
#endif

  free_getprogmeminfo(&prog);

  printf("              total       used       free    largest\n");
  printf("Data:   %11d%11d%11d%11d\n",
         data.arena, data.uordblks, data.fordblks, data.mxordblk);
  printf("Prog:   %11d%11d%11d%11d\n",
         prog.arena, prog.uordblks, prog.fordblks, prog.mxordblk);

  return OK;
}
