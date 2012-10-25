/****************************************************************************
 * binfmt/libnxflat/libnxflat_init.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/stat.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <nxflat.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>
#include <nuttx/binfmt/nxflat.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG, CONFIG_DEBUG_VERBOSE, and CONFIG_DEBUG_BINFMT have to be
 * defined or CONFIG_NXFLAT_DUMPBUFFER does nothing.
 */

#if !defined(CONFIG_DEBUG_VERBOSE) || !defined (CONFIG_DEBUG_BINFMT)
#  undef CONFIG_NXFLAT_DUMPBUFFER
#endif

#ifdef CONFIG_NXFLAT_DUMPBUFFER
# define nxflat_dumpbuffer(m,b,n) bvdbgdumpbuffer(m,b,n)
#else
# define nxflat_dumpbuffer(m,b,n)
#endif

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxflat_init
 *
 * Description:
 *   This function is called to configure the library to process an NXFLAT
 *   program binary.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int nxflat_init(const char *filename, struct nxflat_loadinfo_s *loadinfo)
{
  uint32_t datastart;
  uint32_t dataend;
  uint32_t bssstart;
  uint32_t bssend;
  int      ret;

  bvdbg("filename: %s loadinfo: %p\n", filename, loadinfo);

  /* Clear the load info structure */

  memset(loadinfo, 0, sizeof(struct nxflat_loadinfo_s));

  /* Open the binary file */

  loadinfo->filfd = open(filename, O_RDONLY);
  if (loadinfo->filfd < 0)
    {
      int errval = errno;
      bdbg("Failed to open NXFLAT binary %s: %d\n", filename, errval);
      return -errval;      
    }

  /* Read the NXFLAT header from offset 0 */

  ret = nxflat_read(loadinfo, (char*)&loadinfo->header,
                    sizeof(struct nxflat_hdr_s), 0);
  if (ret < 0)
    {
      bdbg("Failed to read NXFLAT header: %d\n", ret);
      return ret;
    }
  nxflat_dumpbuffer("NXFLAT header", (FAR const uint8_t*)&loadinfo->header,
                    sizeof(struct nxflat_hdr_s));

  /* Verify the NXFLAT header */

  if (nxflat_verifyheader(&loadinfo->header) != 0)
    {
      /* This is not an error because we will be called to attempt loading
       * EVERY binary.  Returning -ENOEXEC simply informs the system that
       * the file is not an NXFLAT file.  Besides, if there is something worth
       * complaining about, nnxflat_verifyheader() has already
       * done so.
       */

      bdbg("Bad NXFLAT header\n");
      return -ENOEXEC;
    }

  /* Save all of the input values in the loadinfo structure 
   * and extract some additional information from the xflat
   * header.  Note that the information in the xflat header is in
   * network order.
   */

  datastart             = ntohl(loadinfo->header.h_datastart);
  dataend               = ntohl(loadinfo->header.h_dataend);
  bssstart              = dataend;
  bssend                = ntohl(loadinfo->header.h_bssend);

  /* And put this information into the loadinfo structure as well.
   *
   * Note that:
   *
   *   isize       = the address range from 0 up to datastart.
   *   datasize   = the address range from datastart up to dataend
   *   bsssize    = the address range from dataend up to bssend.
   */

  loadinfo->entryoffs   = ntohl(loadinfo->header.h_entry);
  loadinfo->isize       = datastart;

  loadinfo->datasize    = dataend - datastart;
  loadinfo->bsssize     = bssend - dataend;
  loadinfo->stacksize   = ntohl(loadinfo->header.h_stacksize);

  /* This is the initial dspace size.  We'll re-calculate this later
   * after the memory has been allocated.
   */

  loadinfo->dsize       = bssend - datastart;

  /* Get the offset to the start of the relocations (we'll relocate
   * this later).
   */

  loadinfo->relocstart  = ntohl(loadinfo->header.h_relocstart);
  loadinfo->reloccount  = ntohs(loadinfo->header.h_reloccount);

  return 0;
}

