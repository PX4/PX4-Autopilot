/****************************************************************************
 * libc/dirent/lib_readdirr.c
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#include <string.h>
#include <dirent.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: readdir_r
 *
 * Description:
 *   The readdir() function returns a pointer to a dirent
 *   structure representing the next directory entry in the
 *   directory stream pointed to by dir.  It returns NULL on
 *   reaching the end-of-file or if an error occurred.
 *
 * Inputs:
 *   dirp -- An instance of type DIR created by a previous
 *     call to opendir();
 *   entry -- The  storage  pointed to by entry must be large
 *     enough for a dirent with an array of char d_name
 *     members containing at least {NAME_MAX}+1 elements.
 *   result -- Upon successful return, the pointer returned
 *     at *result shall have the  same  value  as  the 
 *     argument entry. Upon reaching the end of the directory
 *     stream, this pointer shall have the value NULL.
 *
 * Return:
 *   If successful, the readdir_r() function return s zero;
 *   otherwise, an error number is returned to indicate the
 *   error.
 *
 *   EBADF   - Invalid directory stream descriptor dir
 *
 ****************************************************************************/

int readdir_r(FAR DIR *dirp, FAR struct dirent *entry,
              FAR struct dirent **result)
{
  struct dirent *tmp;

  /* NOTE: The following use or errno is *not* thread-safe */

  set_errno(0);
  tmp = readdir(dirp);
  if (!tmp)
    {
       int error = get_errno();
       if (!error)
          {
            if (result)
              {
                *result = NULL;
              }
            return 0;
          }
       else
          {
            return error;
          }
    }

  if (entry)
    {
      memcpy(entry, tmp, sizeof(struct dirent));
    }

  if (result)
    {
      *result = entry;
    }
  return 0;
}

