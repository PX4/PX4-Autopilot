/****************************************************************************
 * include/nuttx/mqueue.h
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
 ****************************************************************************/

#ifndef ___INCLUDE_NUTTX_MQUEUE_H
#define ___INCLUDE_NUTTX_MQUEUE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <mqueue.h>
#include <queue.h>
#include <signal.h>

#if CONFIG_MQ_MAXMSGSIZE > 0

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Global Type Declarations
 ****************************************************************************/

/* This structure defines a message queue */

struct mq_des; /* forward reference */

struct msgq_s
{
  FAR struct msgq_s *flink;   /* Forward link to next message queue */
  sq_queue_t   msglist;       /* Prioritized message list */
  int16_t      maxmsgs;       /* Maximum number of messages in the queue */
  int16_t      nmsgs;         /* Number of message in the queue */
  int16_t      nconnect;      /* Number of connections to message queue */
  int16_t      nwaitnotfull;  /* Number tasks waiting for not full */
  int16_t      nwaitnotempty; /* Number tasks waiting for not empty */
  uint8_t      maxmsgsize;    /* Max size of message in message queue */
  bool         unlinked;      /* true if the msg queue has been unlinked */
#ifndef CONFIG_DISABLE_SIGNALS
  FAR struct mq_des *ntmqdes; /* Notification: Owning mqdes (NULL if none) */
  pid_t        ntpid;         /* Notification: Receiving Task's PID */
  int          ntsigno;       /* Notification: Signal number */
  union sigval ntvalue;       /* Notification: Signal value */
#endif
  char         name[1];       /* Start of the queue name */
};

typedef struct msgq_s msgq_t;

#define SIZEOF_MQ_HEADER ((int)(((msgq_t*)NULL)->name))

/* This describes the message queue descriptor that is held in the
 * task's TCB
 */

struct mq_des
{
  FAR struct mq_des *flink;   /* Forward link to next message descriptor */
  FAR msgq_t  *msgq;          /* Pointer to associated message queue */
  int          oflags;        /* Flags set when message queue was opened */
};

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MQ_MAXMSGSIZE > 0 */
#endif /* ___INCLUDE_NUTTX_MQUEUE_H */

