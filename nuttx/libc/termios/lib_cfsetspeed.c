/****************************************************************************
 * libc/termios/lib_cfsetspeed.c
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

#include <sys/types.h>
#include <termios.h>
#include <assert.h>

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
 * Name: cfsetspeed
 *
 * Descripton:
 *   The cfsetspeed() function is a non-POSIX function that sets the baud
 *   stored in the structure pointed to by termiosp to speed.
 *
 *   There is no effect on the baud set in the hardware until a subsequent
 *   successful call to tcsetattr() on the same termios structure. 
 *
 *   NOTE 1: NuttX does not control input/output baud independently.  Both
 *   must be the same.  The POSIX standard interfaces, cfisetispeed() and
 *   cfisetospeed() are defined to be cfsetspeed() in termios.h.
 *
 *   NOTE 3:  A consequence of NOTE 1 is that you should never attempt to
 *   set the input and output baud to different values.
 *
 *   Also, the following POSIX requirement cannot be supported: "If the input
 *   baud rate stored in the termios structure pointed to by termios_p is 0,
 *   the input baud rate given to the hardware will be the same as the output
 *   baud rate stored in the termios structure."
 *
 *   NOTE 2.  In Nuttx, the speed_t is defined to be uint32_t and the baud
 *   encodings of termios.h are the actual baud values themselves.  Therefore,
 *   any baud value can be provided as the speed argument here.  However, if
 *   you do so, your code will *NOT* be portable to other environments where
 *   speed_t is smaller and where the termios.h baud values are encoded! To
 *   avoid portability issues, use the baud definitions in termios.h!
 *
 *   Linux, for example, would require this (also non-portable) sequence:
 *
 *     cfsetispeed(termiosp, BOTHER);
 *     termiosp->c_ispeed = baud;
 *
 *     cfsetospeed(termiosp, BOTHER);
 *     termiosp->c_ospeed = baud;
 *
 * Input Parameters:
 *   termiosp - The termiosp argument is a pointer to a termios structure.
 *   speed - The new input speed
 *
 * Returned Value:
 *   Baud is not checked... OK is always returned (this is non-standard
 *   behavior). 
 *
 ****************************************************************************/

int cfsetspeed(FAR struct termios *termiosp, speed_t speed)
{
  DEBUGASSERT(termiosp);
  termiosp->c_speed = speed;
  return OK;
}
