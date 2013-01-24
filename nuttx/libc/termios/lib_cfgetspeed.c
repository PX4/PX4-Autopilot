/****************************************************************************
 * libc/termios/lib_cfgetspeed.c
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
 * Name: cfgetspeed
 *
 * Descripton:
 *   The cfgetspeed() function is a non-POSIX function will extract the baud
 *   from the termios structure to which the termiosp argument points.
 *
 *   This function will return exactly the value in the termios data
 *   structure, without interpretation.
 *
 *   NOTE 1: NuttX does not control input/output baud independently.  Both
 *   must be the same.  The POSIX standard interfaces, cfisetispeed() and
 *   cfisetospeed() are defined to be cfgetspeed() in termios.h.
 *   NOTE 2.  In Nuttx, the speed_t is defined to be uint32_t and the baud
 *   encodings of termios.h are the actual baud values themselves.  Therefore,
 *   any baud value may be returned here... not just those enumerated in
 *   termios.h
 *
 * Input Parameters:
 *   termiosp - The termiosp argument is a pointer to a termios structure.
 *
 * Returned Value:
 *   Encoded baud value from the termios structure. 
 *
 ****************************************************************************/

speed_t cfgetspeed(FAR const struct termios *termiosp)
{
  DEBUGASSERT(termiosp);
  return termiosp->c_speed;
}
