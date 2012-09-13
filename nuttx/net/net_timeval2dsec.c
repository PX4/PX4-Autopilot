/****************************************************************************
 * net/net_timeval2dsec.c
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_SOCKOPTS) && !defined(CONFIG_DISABLE_CLOCK)

#include <sys/socket.h>
#include <stdint.h>
#include <errno.h>
#include <nuttx/clock.h>

#include "net_internal.h"

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Function: net_timeval2dsec
 *
 * Description:
 *   Convert a struct timeval to deciseconds.  Needed by setsockopt() to
 *   save new timeout values.
 *
 * Parameters:
 *   tv   The struct timeval to convert
 *
 * Returned Value:
 *   the converted value
 *
 * Assumptions:
 *
 ****************************************************************************/

socktimeo_t net_timeval2dsec(struct timeval *tv)
{
  return (uint16_t)(tv->tv_sec* DSEC_PER_SEC + tv->tv_usec / USEC_PER_DSEC);
}

#endif /* CONFIG_NET && CONFIG_NET_SOCKOPTS && !CONFIG_DISABLE_CLOCK */
