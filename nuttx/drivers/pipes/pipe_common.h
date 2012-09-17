/****************************************************************************
 * drivers/pipe/pipe_common.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __DRIVERS_PIPES_PIPE_COMMON_H
#define __DRIVERS_PIPES_PIPE_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <poll.h>

#ifndef CONFIG_DEV_PIPE_SIZE
#  define CONFIG_DEV_PIPE_SIZE 1024
#endif

#if CONFIG_DEV_PIPE_SIZE > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum number of threads than can be waiting for POLL events */

#ifndef CONFIG_DEV_PIPE_NPOLLWAITERS
#  define CONFIG_DEV_PIPE_NPOLLWAITERS 2
#endif

/* Maximum number of open's supported on pipe */

#define CONFIG_DEV_PIPE_MAXUSER 255

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Make the buffer index as small as possible for the configured pipe size */
 
#if CONFIG_DEV_PIPE_SIZE > 65535
typedef uint32_t pipe_ndx_t;  /* 32-bit index */
#elif CONFIG_DEV_PIPE_SIZE > 255
typedef uint16_t pipe_ndx_t;  /* 16-bit index */
#else
typedef uint8_t pipe_ndx_t;   /*  8-bit index */
#endif

/* This structure represents the state of one pipe.  A reference to this
 * structure is retained in the i_private field of the inode whenthe pipe/fifo
 * device is registered.
 */

struct pipe_dev_s
{
  sem_t      d_bfsem;       /* Used to serialize access to d_buffer and indices */
  sem_t      d_rdsem;       /* Empty buffer - Reader waits for data write */
  sem_t      d_wrsem;       /* Full buffer - Writer waits for data read */
  pipe_ndx_t d_wrndx;       /* Index in d_buffer to save next byte written */
  pipe_ndx_t d_rdndx;       /* Index in d_buffer to return the next byte read */
  uint8_t    d_refs;        /* References counts on pipe (limited to 255) */
  uint8_t    d_nwriters;    /* Number of reference counts for write access */
  uint8_t    d_pipeno;      /* Pipe minor number */
  uint8_t   *d_buffer;      /* Buffer allocated when device opened */

  /* The following is a list if poll structures of threads waiting for
   * driver events. The 'struct pollfd' reference for each open is also
   * retained in the f_priv field of the 'struct file'.
   */

#ifndef CONFIG_DISABLE_POLL
  struct pollfd *d_fds[CONFIG_DEV_PIPE_NPOLLWAITERS];
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C" {
#else
#  define EXTERN extern
#endif

EXTERN FAR struct pipe_dev_s *pipecommon_allocdev(void);
EXTERN void    pipecommon_freedev(FAR struct pipe_dev_s *dev);
EXTERN int     pipecommon_open(FAR struct file *filep);
EXTERN int     pipecommon_close(FAR struct file *filep);
EXTERN ssize_t pipecommon_read(FAR struct file *, FAR char *, size_t);
EXTERN ssize_t pipecommon_write(FAR struct file *, FAR const char *, size_t);
#ifndef CONFIG_DISABLE_POLL
EXTERN int     pipecommon_poll(FAR struct file *filep, FAR struct pollfd *fds,
                               bool setup);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_DEV_PIPE_SIZE > 0 */
#endif /* __DRIVERS_PIPES_PIPE_COMMON_H */
