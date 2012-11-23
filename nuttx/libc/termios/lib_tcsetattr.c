/****************************************************************************
 * libc/termios/lib_tcsetattr.c
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

#include <sys/ioctl.h>

#include <termios.h>
#include <errno.h>

#include <nuttx/serial/tioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcsetattr
 *
 * Descripton:
 *   The tcsetattr() function sets the parameters associated with the
 *   terminal referred to by the open file descriptor 'fd' from the termios
 *   structure referenced by 'termiop' as follows:
 *
 *   If 'options' is TCSANOW, the change will occur immediately.
 *
 *   If 'options' is TCSADRAIN, the change will occur after all output
 *   written to 'fd' is transmitted. This function should be used when changing
 *   parameters that affect output.
 *
 *   If 'options' is TCSAFLUSH, the change will occur after all
 *   output written to 'fd' is transmitted, and all input so far received but
 *   not read will be discarded before the change is made.
 *
 *   The tcsetattr() function will return successfully if it was able to
 *   perform any of the requested actions, even if some of the requested
 *   actions could not be performed. It will set all the attributes that
 *   implementation supports as requested and leave all the attributes not
 *   supported by the implementation unchanged. If no part of the request
 *   can be honoured, it will return -1 and set errno to EINVAL.
 *
 *   The effect of tcsetattr() is undefined if the value of the termios
 *   structure pointed to by 'termiop' was not derived from the result of
 *   a call to tcgetattr() on 'fd'; an application should modify only fields
 *   and flags defined by this specification between the call to tcgetattr()
 *   and tcsetattr(), leaving all other fields and flags unmodified.
 *
 * Returned Value:
 *
 *   Upon successful completion, 0 is returned. Otherwise, -1 is returned
 *   and errno is set to indicate the error.  The following errors may be
 *   reported:
 *
 *   - EBADF: The 'fd' argument is not a valid file descriptor. 
 *   - EINTR:  A signal interrupted tcsetattr(). 
 *   - EINVAL: The 'options' argument is not a supported value, or
 *     an attempt was made to change an attribute represented in the
 *     termios structure to an unsupported value. 
 *   - ENOTTY: The file associated with 'fd' is not a terminal. 
 *
 ****************************************************************************/

int tcsetattr(int fd, int options, FAR const struct termios *termiosp)
{
  if (options == TCSANOW)
    {
      return ioctl(fd, TCSETS, (unsigned long)termiosp);
    }
  return -ENOSYS;
}
