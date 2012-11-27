/****************************************************************************
 * up_serial.c
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdbool.h>

#include <nuttx/fs/fs.h>

#include <arch/../src/up_internal.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
extern int hosttcp_init(int portno);
extern size_t hosttcp_read(int fd, char * buffer, size_t len);
extern size_t hosttcp_write(int fd, const char * buffer, size_t len);
extern void hosttcp_close(int fd);

#ifndef CONFIG_DISABLE_POLL
#define SERIAL_PORT_TCP_SIM(DEV) \
    static int fd_##DEV = 0; \
    void up_##DEV(const int port); \
    void down_##DEV(void); \
    static ssize_t read_##DEV(struct file *, char *, size_t); \
    static ssize_t write_##DEV(struct file *, const char *, size_t); \
    static int     poll_##DEV(FAR struct file *filep, FAR struct pollfd *fds, \
                                   bool setup); \
    static const struct file_operations fops_##DEV = \
    { \
      .read		= read_##DEV, \
      .write	= write_##DEV, \
      .poll     = poll_##DEV, \
    }; \
    static ssize_t read_##DEV(struct file *filp, char *buffer, size_t len) \
    { \
      return hosttcp_read(fd_##DEV,buffer, len); \
    } \
    static ssize_t write_##DEV(struct file *filp, const char *buffer, size_t len) \
    { \
      return hosttcp_write(fd_##DEV,buffer, len); \
    } \ 
    static int poll_##DEV(FAR struct file *filep, FAR struct pollfd *fds, \
                               bool setup) \
    { \
      return OK; \
    } \
    void up_##DEV(const int portno) \
    { \
        fd_##DEV = hosttcp_init(portno); \
        ASSERT(fd_##DEV != NULL); \
        register_driver("/dev/" #DEV, &fops_##DEV, 0666, NULL); \
    } \
    void down_##DEV(void) \
    { \
        hosttcp_close(fd_##DEV); \
    }
#else
#define SERIAL_PORT_TCP_SIM(DEV) \
    static int fd_##DEV = 0; \
    void up_##DEV(const int port); \
    void down_##DEV(void); \
    static ssize_t read_##DEV(struct file *, char *, size_t); \
    static ssize_t write_##DEV(struct file *, const char *, size_t); \
    static const struct file_operations fops_##DEV = \
    { \
      .read		= read_##DEV, \
      .write	= write_##DEV, \
    }; \
    static ssize_t read_##DEV(struct file *filp, char *buffer, size_t len) \
    { \
      return hosttcp_read(fd_##DEV,buffer, len); \
    } \
    static ssize_t write_##DEV(struct file *filp, const char *buffer, size_t len) \
    { \
      return hosttcp_write(fd_##DEV,buffer, len); \
    } \ 
    void \
    up_##DEV(const int portno) \
    { \
        fd_##DEV = hosttcp_init(portno); \
        ASSERT(fd_##DEV != NULL); \
        register_driver("/dev/" #DEV, &fops_##DEV, 0666, NULL); \
    } \
    void \
    down_##DEV(void) \
    { \
        hosttcp_close(fd_##DEV); \
    }
#endif

SERIAL_PORT_TCP_SIM(ttyS0)
SERIAL_PORT_TCP_SIM(ttyS1)
SERIAL_PORT_TCP_SIM(ttyS2)
