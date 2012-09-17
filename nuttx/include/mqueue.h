/********************************************************************************
 * include/mqueue.h
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
 ********************************************************************************/

#ifndef __INCLUDE_MQUEUE_H
#define __INCLUDE_MQUEUE_H

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <sys/types.h>
#include <signal.h>
#include "queue.h"

/********************************************************************************
 * Compilations Switches
 ********************************************************************************/

/********************************************************************************
 * Definitions
 ********************************************************************************/

#define MQ_NONBLOCK O_NONBLOCK

/********************************************************************************
 * Global Type Declarations
 ********************************************************************************/

/* Message queue attributes */

struct mq_attr
{
  size_t         mq_maxmsg;    /* Max number of messages in queue */
  size_t         mq_msgsize;   /* Max message size */
  unsigned       mq_flags;     /* Queue flags */
  size_t         mq_curmsgs;   /* Number of messages currently in queue */
};

/* Message queue descriptor */

typedef FAR struct mq_des *mqd_t;

/********************************************************************************
 * Global Variables
 ********************************************************************************/

/********************************************************************************
 * Global Function Prototypes
 ********************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN mqd_t   mq_open(const char *mq_name, int oflags, ...);
EXTERN int     mq_close(mqd_t mqdes );
EXTERN int     mq_unlink(const char *mq_name);
EXTERN int     mq_send(mqd_t mqdes, const void *msg, size_t msglen, int prio);
EXTERN int     mq_timedsend(mqd_t mqdes, const char *msg, size_t msglen, int prio,
                            const struct timespec *abstime);
EXTERN ssize_t mq_receive(mqd_t mqdes, void *msg, size_t msglen, int *prio);
EXTERN ssize_t mq_timedreceive(mqd_t mqdes, void *msg, size_t msglen,
                               int *prio, const struct timespec *abstime);
EXTERN int     mq_notify(mqd_t mqdes, const struct sigevent *notification);
EXTERN int     mq_setattr(mqd_t mqdes, const struct mq_attr *mq_stat,
                  struct mq_attr *oldstat);
EXTERN int     mq_getattr(mqd_t mqdes, struct mq_attr *mq_stat);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_MQUEUE_H */
