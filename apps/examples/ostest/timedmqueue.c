/**************************************************************************
 * apps/examples/ostest/mqueue.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <fcntl.h>
#include <pthread.h>
#include <mqueue.h>
#include <sched.h>
#include <errno.h>

#include "ostest.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

#define TEST_MESSAGE        "This is a test and only a test"
#if defined(SDCC) || defined(__ZILOG__)
   /* Cannot use strlen in array size */

#  define TEST_MSGLEN         (31)
#else
   /* Message lenght is the size of the message plus the null terminator */

#  define TEST_MSGLEN         (strlen(TEST_MESSAGE)+1)
#endif

#define TEST_SEND_NMSGS     (10)
#define TEST_RECEIVE_NMSGS  (10)

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Public Functions
 **************************************************************************/

static void *sender_thread(void *arg)
{
  mqd_t mqfd;
  char msg_buffer[TEST_MSGLEN];
  struct mq_attr attr;
  int status = 0;
  int nerrors = 0;
  int i;

  printf("sender_thread: Starting\n");

  /* Fill in attributes for message queue */

  attr.mq_maxmsg  = TEST_SEND_NMSGS-1;
  attr.mq_msgsize = TEST_MSGLEN;
  attr.mq_flags   = 0;

  /* Set the flags for the open of the queue.
   * Make it a blocking open on the queue, meaning it will block if
   * this process tries to send to the queue and the queue is full.
   *
   *   O_CREAT - the queue will get created if it does not already exist.
   *   O_WRONLY - we are only planning to write to the queue.
   *
   * Open the queue, and create it if the receiving process hasn't
   * already created it.
   */

  mqfd = mq_open("testmq", O_WRONLY|O_CREAT, 0666, &attr);
  if (mqfd < 0)
    {
        printf("sender_thread: ERROR mq_open failed\n");
        pthread_exit((pthread_addr_t)1);
    }

  /* Fill in a test message buffer to send */

  memcpy(msg_buffer, TEST_MESSAGE, TEST_MSGLEN);

  /* Perform the send TEST_SEND_NMSGS times */

  for (i = 0; i < TEST_SEND_NMSGS; i++)
    {
       struct timespec ts;
       status = clock_gettime(CLOCK_REALTIME, &ts);
       if (status != 0)
         {
           printf("sender_thread: ERROR clock_gettime failed\n");
         }
       ts.tv_sec += 5;

      /* The first TEST_SEND_NMSGS-1 send should succeed.  The last
       * one should fail with errno == ETIMEDOUT
       */

      status = mq_timedsend(mqfd, msg_buffer, TEST_MSGLEN, 42, &ts);
      if (status < 0)
        {
          if (i == TEST_SEND_NMSGS-1 && errno == ETIMEDOUT)
            {
              printf("sender_thread: mq_timedsend %d timed out as expected\n", i);
            }
          else
            {
              printf("sender_thread: ERROR mq_timedsend failure=%d on msg %d\n", errno, i);
              nerrors++;
            }
        }
      else
        {
          if (i == TEST_SEND_NMSGS-1)
            {
              printf("sender_thread: ERROR mq_timedsend of msg %d succeeded\n", i);
              nerrors++;
            }
          else
            {
              printf("sender_thread: mq_timedsend succeeded on msg %d\n", i);
            }
        }
    }

  /* Close the queue and return success */

  if (mq_close(mqfd) < 0)
    {
      printf("sender_thread: ERROR mq_close failed\n");
    }

  printf("sender_thread: returning nerrors=%d\n", nerrors);
  FFLUSH();
  return (pthread_addr_t)nerrors;
}

static void *receiver_thread(void *arg)
{
  mqd_t mqfd;
  char msg_buffer[TEST_MSGLEN];
  struct mq_attr attr;
  int nbytes;
  int nerrors = 0;
  int i;

  printf("receiver_thread: Starting\n");

  /* Fill in attributes for message queue */

  attr.mq_maxmsg  = TEST_SEND_NMSGS-1;
  attr.mq_msgsize = TEST_MSGLEN;
  attr.mq_flags   = 0;

  /* Set the flags for the open of the queue.
   * Make it a blocking open on the queue, meaning it will block if
   * this process tries to* send to the queue and the queue is full.
   *
   *   O_CREAT - the queue will get created if it does not already exist.
   *   O_RDONLY - we are only planning to write to the queue.
   *
   * Open the queue, and create it if the sending process hasn't
   * already created it.
   */

   mqfd = mq_open("testmq", O_RDONLY|O_CREAT, 0666, &attr);
   if (mqfd < 0)
     {
       printf("receiver_thread: ERROR mq_open failed\n");
       pthread_exit((pthread_addr_t)1);
     }

   /* Perform the receive TEST_RECEIVE_NMSGS times */

   for (i = 0; i < TEST_RECEIVE_NMSGS; i++)
    {
       struct timespec ts;
       int status = clock_gettime(CLOCK_REALTIME, &ts);
       if (status != 0)
         {
           printf("sender_thread: ERROR clock_gettime failed\n");
         }
       ts.tv_sec += 5;

      /* The first TEST_SEND_NMSGS-1 send should succeed.  The last
       * one should fail with errno == ETIMEDOUT
       */

      memset(msg_buffer, 0xaa, TEST_MSGLEN);
      nbytes = mq_timedreceive(mqfd, msg_buffer, TEST_MSGLEN, 0, &ts);
      if (nbytes < 0)
        {
          if (i == TEST_SEND_NMSGS-1 && errno == ETIMEDOUT)
            {
              printf("receiver_thread: Receive %d timed out as expected\n", i);
            }
          else
            {
              printf("receiver_thread: ERROR mq_timedreceive failure=%d on msg %d\n", errno, i);
              nerrors++;
            }
        }
      else if (nbytes != TEST_MSGLEN)
        {
          printf("receiver_thread: mq_timedreceive return bad size %d on msg %d\n", nbytes, i);
          nerrors++;
        }
      else if (memcmp(TEST_MESSAGE, msg_buffer, nbytes) != 0)
        {
          int j;

          printf("receiver_thread: mq_timedreceive returned corrupt message on msg %d\n", i);
          printf("receiver_thread:                  i  Expected Received\n");

          for (j = 0; j < TEST_MSGLEN-1; j++)
            {
              if (isprint(msg_buffer[j]))
                {
                 printf("receiver_thread:                  %2d %02x (%c) %02x (%c)\n",
                         j, TEST_MESSAGE[j], TEST_MESSAGE[j], msg_buffer[j], msg_buffer[j]);
                }
              else
                {
                  printf("receiver_thread:                  %2d %02x (%c) %02x\n",
                         j, TEST_MESSAGE[j], TEST_MESSAGE[j], msg_buffer[j]);
                }
            }
          printf("receiver_thread:                  %2d 00      %02x\n",
                  j, msg_buffer[j]);
        }
      else if (i == TEST_SEND_NMSGS-1)
        {
          printf("receiver_thread: ERROR mq_timedreceive of msg %d succeeded\n", i);
          nerrors++;
        }
      else
        {
          printf("receiver_thread: mq_timedreceive succeeded on msg %d\n", i);
        }
    }

  /* Close the queue and return success */

  if (mq_close(mqfd) < 0)
    {
      printf("receiver_thread: ERROR mq_close failed\n");
      nerrors++;
    }

  /* Destroy the queue */

  if (mq_unlink("testmq") < 0)
    {
      printf("receiver_thread: ERROR mq_close failed\n");
      nerrors++;
    }

  printf("receiver_thread: returning nerrors=%d\n", nerrors);
  FFLUSH();
  pthread_exit((pthread_addr_t)nerrors);
  return (pthread_addr_t)nerrors;
}

void timedmqueue_test(void)
{
  pthread_t sender;
  pthread_t receiver;
  void *result;
  pthread_attr_t attr;
  int status;

  /* Start the sending thread at the default priority */

  printf("timedmqueue_test: Starting sender\n");
  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("timedmqueue_test: pthread_attr_init failed, status=%d\n", status);
    }

  status = pthread_attr_setstacksize(&attr, STACKSIZE);
  if (status != 0)
    {
      printf("timedmqueue_test: pthread_attr_setstacksize failed, status=%d\n", status);
    }

  status = pthread_create(&sender, &attr, sender_thread, NULL);
  if (status != 0)
    {
      printf("timedmqueue_test: pthread_create failed, status=%d\n", status);
    }

  /* Wait for the sending thread to complete */

  printf("timedmqueue_test: Waiting for sender to complete\n");
  pthread_join(sender, &result);
  if (result != (void*)0)
    {
      printf("timedmqueue_test: ERROR sender thread exited with %d errors\n", (int)result);
    }

  /* Start the receiving thread at the default priority */

  printf("timedmqueue_test: Starting receiver\n");
  status = pthread_attr_init(&attr);
  if (status != 0)
    {
      printf("timedmqueue_test: pthread_attr_init failed, status=%d\n", status);
    }

  status = pthread_attr_setstacksize(&attr, STACKSIZE);
  if (status != 0)
    {
      printf("timedmqueue_test: pthread_attr_setstacksize failed, status=%d\n", status);
    }

  status = pthread_create(&receiver, &attr, receiver_thread, NULL);
  if (status != 0)
    {
      printf("timedmqueue_test: pthread_create failed, status=%d\n", status);
    }

  /* Wait for the receiving thread to complete */

  printf("timedmqueue_test: Waiting for receiver to complete\n");
  pthread_join(receiver, &result);
  if (result != (void*)0)
    {
      printf("timedmqueue_test: ERROR receiver thread exited with %d errors\n", (int)result);
    }

  printf("timedmqueue_test: Test complete\n");
}


