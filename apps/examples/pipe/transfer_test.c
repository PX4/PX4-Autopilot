/****************************************************************************
 * examples/pipe/transfer_test.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <unistd.h>
#include <errno.h>

#include "pipe.h"

/****************************************************************************
 * Pre-proecessor Definitions
 ****************************************************************************/

#define MAX_BYTE      13

#define WRITE_SIZE    MAX_BYTE
#define NWRITES       1400
#define NWRITE_BYTES (NWRITES * WRITE_SIZE)

#define READ_SIZE    (2*MAX_BYTE)
#define NREADS       (NWRITES / 2)
#define NREAD_BYTES   NWRITE_BYTES

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: transfer_reader
 ****************************************************************************/

static void *transfer_reader(pthread_addr_t pvarg)
{
  char buffer[READ_SIZE];
  int fd = (int)pvarg;
  int ret;
  int nbytes;
  int value;
  int ndx;

  printf("transfer_reader: started\n");
  for (nbytes = 0, value = 0; nbytes < NREAD_BYTES;)
    {
      ret = read(fd, buffer, READ_SIZE);
      if (ret < 0 )
        {
           fprintf(stderr, "transfer_reader: read failed, errno=%d\n", errno);
           return (void*)1;
        }
      else if (ret == 0)
        {
          if (nbytes < NREAD_BYTES)
            {
              fprintf(stderr, "transfer_reader: Too few bytes read -- aborting: %d\n", nbytes);
              return (void*)2;
            }
          break;
        }
      for (ndx = 0; ndx < ret; ndx++)
        {
          if (value >= WRITE_SIZE)
            {
              value = 0;
            }
          if (buffer[ndx] != value)
            {
              fprintf(stderr, "transfer_reader: Byte %d, expected %d, found %d\n",
                      nbytes + ndx, value, buffer[ndx]);
              return (void*)3;
            }
          value++;
        }
      nbytes += ret;
      if (nbytes > NREAD_BYTES)
        {
          fprintf(stderr, "transfer_reader: Too many bytes read -- aborting: %d\n", nbytes);
          return (void*)4;
        }
    }
  printf("transfer_reader: %d bytes read\n", nbytes);
  return (void*)0;
}

/****************************************************************************
 * Name: transfer_writer
 ****************************************************************************/

static void *transfer_writer(pthread_addr_t pvarg)
{
  char buffer[WRITE_SIZE];
  int fd = (int)pvarg;
  int ret;
  int i;

  printf("transfer_writer: started\n");
  for (i = 0; i < WRITE_SIZE; i++)
    {
      buffer[i] = i;
    }

  for (i = 0; i < NWRITES; i++)
    {
      ret = write(fd, buffer, WRITE_SIZE);
      if (ret < 0 )
        {
           fprintf(stderr, "transfer_writer: write failed, errno=%d\n", errno);
           return (void*)1;
        }
      else if (ret != WRITE_SIZE)
        {
           fprintf(stderr, "transfer_writer: Unexpected write size=%d\n", ret);
           return (void*)2;
        }
    }
  printf("transfer_writer: %d bytes written\n", NWRITE_BYTES);
  return (void*)0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: transfer_test
 ****************************************************************************/

int transfer_test(int fdin, int fdout)
{
  pthread_t readerid;
  pthread_t writerid;
  void *value;
  int tmp;
  int ret;

  /* Start transfer_reader thread */

  printf("transfer_test: Starting transfer_reader thread\n");
  ret = pthread_create(&readerid, NULL, transfer_reader, (pthread_addr_t)fdin);
  if (ret != 0)
    {
      fprintf(stderr, "transfer_test: Failed to create transfer_reader thread, error=%d\n", ret);
      return 1;
    }

  /* Start transfer_writer thread */

  printf("transfer_test: Starting transfer_writer thread\n");
  ret = pthread_create(&writerid, NULL, transfer_writer, (pthread_addr_t)fdout);
  if (ret != 0)
    {
      fprintf(stderr, "transfer_test: Failed to create transfer_writer thread, error=%d\n", ret);
      pthread_detach(readerid);
      ret = pthread_cancel(readerid);
      if (ret != 0)
        {
          fprintf(stderr, "transfer_test: Failed to cancel transfer_reader thread, error=%d\n", ret);
        }
      return 2;
    }

  /* Wait for transfer_writer thread to complete */

  printf("transfer_test: Waiting for transfer_writer thread\n");
  ret = pthread_join(writerid, &value);
  if (ret != 0)
    {
      fprintf(stderr, "transfer_test: pthread_join failed, error=%d\n", ret);
    }
  else
    {
      ret = (int)value;
      printf("transfer_test: transfer_writer returned %d\n", ret);
    }

  /* Wait for transfer_reader thread to complete */

  printf("transfer_test: Waiting for transfer_reader thread\n");
  tmp = pthread_join(readerid, &value);
  if (tmp != 0)
    {
      fprintf(stderr, "transfer_test: pthread_join failed, error=%d\n", ret);
    }
  else
    {
      tmp = (int)value;
      printf("transfer_test: transfer_reader returned %d\n", tmp);
    }

  if (ret == 0)
    {
      ret = tmp;
    }
  printf("transfer_test: returning %d\n", ret);
  return ret;
}

