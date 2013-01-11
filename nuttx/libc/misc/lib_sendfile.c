/************************************************************************
 * libc/misc/lib_streamsem.c
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <sys/sendfile.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#if CONFIG_NSOCKET_DESCRIPTORS > 0 || CONFIG_NFILE_DESCRIPTORS > 0

/************************************************************************
 * Private types
 ************************************************************************/

/************************************************************************
 * Private Variables
 ************************************************************************/

/************************************************************************
 * Public Variables
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: sendfile
 *
 * Description:
 *   sendfile() copies data between one file descriptor and another.
 *   sendfile() basically just wraps a sequence of reads() and writes()
 *   to perform a copy.  It serves a purpose in systems where there is
 *   a penalty for copies to between user and kernal space, but really
 *   nothing in NuttX but provide some Linux compatible (and adding
 *   another 'almost standard' interface). 
 *
 *   NOTE: This interface is *not* specified in POSIX.1-2001, or other
 *   standards.  The implementation here is very similar to the Linux
 *   sendfile interface.  Other UNIX systems implement sendfile() with
 *   different semantics and prototypes.  sendfile() should not be used
 *   in portable programs.
 *
 * Input Parmeters:
 *   infd   - A file (or socket) descriptor opened for reading
 *   outfd  - A descriptor opened for writing.
 *   offset - If 'offset' is not NULL, then it points to a variable
 *            holding the file offset from which sendfile() will start
 *            reading data from 'infd'.  When sendfile() returns, this
 *            variable will be set to the offset of the byte following
 *            the last byte that was read.  If 'offset' is not NULL,
 *            then sendfile() does not modify the current file offset of
 *            'infd'; otherwise the current file offset is adjusted to
 *            reflect the number of bytes read from 'infd.'
 *
 *            If 'offset' is NULL, then data will be read from 'infd'
 *            starting at the current file offset, and the file offset
 *            will be updated by the call.
 *   count -  The number of bytes to copy between the file descriptors.
 *
 * Returned Value:
 *   If the transfer was successful, the number of bytes written to outfd is
 *   returned.  On error, -1 is returned, and errno is set appropriately.
 *   There error values are those returned by read() or write() plus:
 *
 *   EINVAL - Bad input parameters.
 *   ENOMEM - Could not allocated an I/O buffer
 *
 ************************************************************************/

ssize_t sendfile(int outfd, int infd, off_t *offset, size_t count)
{
  FAR uint8_t *iobuffer;
  FAR uint8_t *wrbuffer;
  off_t startpos = 0;
  ssize_t nbytesread;
  ssize_t nbyteswritten;
  size_t  ntransferred;
  bool endxfr;

  /* Get the current file position. */

  if (offset)
    {
      /* Use lseek to get the current file position */

      startpos = lseek(infd, 0, SEEK_CUR);
      if (startpos == (off_t)-1)
        {
          return ERROR;
        }

      /* Use lseek again to set the new file position */

      if (lseek(infd, *offset, SEEK_SET) == (off_t)-1)
        {
          return ERROR;
        }
    }

  /* Allocate an I/O buffer */

  iobuffer = (FAR void *)malloc(CONFIG_LIB_SENDFILE_BUFSIZE);
  if (!iobuffer)
    {
      set_errno(ENOMEM);
      return ERROR;
    }

  /* Now transfer 'count' bytes from the infd to the outfd */

  for (ntransferred = 0, endxfr = false; ntransferred < count && !endxfr; )
    {
      /* Loop until the read side of the transfer comes to some conclusion */

      do
        {
          /* Read a buffer of data from the infd */

          nbytesread = read(infd, iobuffer, CONFIG_LIB_SENDFILE_BUFSIZE);

          /* Check for end of file */

          if (nbytesread == 0)
            {
              /* End of file.  Break out and return current number of bytes
               * transferred.
               */

              endxfr = true;
              break;
            }

          /* Check for a read ERROR.  EINTR is a special case.  This function
           * should break out and return an error if EINTR is returned and
           * no data has been transferred.  But what should it do if some
           * data has been transferred?  I suppose just continue?
           */

          else if (nbytesread < 0)
            {
              /* EINTR is not an error (but will still stop the copy) */

#ifndef CONFIG_DISABLE_SIGNALS
              if (errno != EINTR || ntransferred == 0)
#endif
                {
                  /* Read error.  Break out and return the error condition. */

                  ntransferred = ERROR;
                  endxfr       = true;
                  break;
                }
            }
        }
      while (nbytesread < 0);

      /* Was anything read? */

      if (!endxfr)
        {
          /* Yes.. Loop until the read side of the transfer comes to some
           * conclusion.
           */

          wrbuffer = iobuffer;
          do
            {
              /* Write the buffer of data to the outfd */

              nbyteswritten = write(outfd, wrbuffer, nbytesread);

              /* Check for a complete (or parial) write.  write() should not
               * return zero.
               */

              if (nbyteswritten >= 0)
                {
                  /* Advance the buffer pointer and decrement the number of bytes
                   * remaining in the iobuffer.  Typically, nbytesread will now
                   * be zero.
                   */

                  wrbuffer     += nbyteswritten;
                  nbytesread   -= nbyteswritten;

                  /* Increment the total number of bytes successfully transferred. */

                  ntransferred += nbyteswritten;
                }

              /* Otherwise an error occurred */

              else
                {
                  /* Check for a read ERROR.  EINTR is a special case.  This
                   * function should break out and return an error if EINTR
                   * is returned and no data has been transferred.  But what
                   * should it do if some data has been transferred?  I
                   * suppose just continue?
                   */

#ifndef CONFIG_DISABLE_SIGNALS
                  if (errno != EINTR || ntransferred == 0)
#endif
                    {
                      /* Write error.  Break out and return the error condition */

                      ntransferred = ERROR;
                      endxfr       = true;
                      break;
                    }
                }
            }
          while (nbytesread > 0);
        }
    }

  /* Release the I/O buffer */

  free(iobuffer);

  /* Return the current file position */

  if (offset)
    {
      /* Use lseek to get the current file position */

      off_t curpos = lseek(infd, 0, SEEK_CUR);
      if (curpos == (off_t)-1)
        {
          return ERROR;
        }

      /* Return the current file position */

      *offset = curpos;

      /* Use lseek again to restore the original file position */

      if (lseek(infd, startpos, SEEK_SET) == (off_t)-1)
        {
          return ERROR;
        }
    }

  /* Finally return the number of bytes actually transferred (or ERROR
   * if any failure occurred).
   */

  return ntransferred;
}

#endif /* CONFIG_NSOCKET_DESCRIPTORS > 0 || CONFIG_NFILE_DESCRIPTORS > 0 */
