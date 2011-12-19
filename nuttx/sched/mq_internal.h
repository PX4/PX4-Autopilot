/****************************************************************************
 * sched/mq_internal.h
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __SCHED_MQ_INTERNAL_H
#define __SCHED_MQ_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <mqueue.h>
#include <sched.h>
#include <signal.h>

#include <nuttx/mqueue.h>

#if CONFIG_MQ_MAXMSGSIZE > 0

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define MQ_MAX_BYTES   CONFIG_MQ_MAXMSGSIZE
#define MQ_MAX_MSGS    16
#define MQ_PRIO_MAX    _POSIX_MQ_PRIO_MAX

/* This defines the number of messages descriptors to allocate
 * at each "gulp."
 */

#define NUM_MSG_DESCRIPTORS 24

/* This defines the number of messages to set aside for
 * exclusive use by interrupt handlers
 */

#define NUM_INTERRUPT_MSGS   8

/****************************************************************************
 * Global Type Declarations
 ****************************************************************************/

enum mqalloc_e
{
  MQ_ALLOC_FIXED = 0,  /* pre-allocated; never freed */
  MQ_ALLOC_DYN,        /* dynamically allocated; free when unused */
  MQ_ALLOC_IRQ         /* Preallocated, reserved for interrupt handling */
};
typedef enum mqalloc_e mqalloc_t;

/* This structure describes one buffered POSIX message. */

struct mqmsg
{
  FAR struct mqmsg  *next;    /* Forward link to next message */
  uint8_t      type;          /* (Used to manage allocations) */
  uint8_t      priority;      /* priority of message          */
#if MQ_MAX_BYTES < 256
  uint8_t      msglen;        /* Message data length          */
#else
  uint16_t     msglen;        /* Message data length          */
#endif
  uint8_t      mail[MQ_MAX_BYTES]; /* Message data            */
};
typedef struct mqmsg mqmsg_t;

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/* This is a list of all opened message queues */

extern sq_queue_t  g_msgqueues;

/* The g_msgfree is a list of messages that are available
 * for general use.  The number of messages in this list is a
 * system configuration item.
 */

extern sq_queue_t  g_msgfree;

/* The g_msgfreeInt is a list of messages that are reserved
 * for use by interrupt handlers.
 */

extern sq_queue_t  g_msgfreeirq;

/* The g_desfree data structure is a list of message
 * descriptors available to the operating system for general use.
 * The number of messages in the pool is a constant.
 */

extern sq_queue_t  g_desfree;

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* Functions defined in mq_initialize.c ************************************/

EXTERN void weak_function mq_initialize(void);
EXTERN void         mq_desblockalloc(void);

EXTERN mqd_t        mq_descreate(FAR _TCB* mtcb, FAR msgq_t* msgq, int oflags);
EXTERN FAR msgq_t  *mq_findnamed(const char *mq_name);
EXTERN void         mq_msgfree(FAR mqmsg_t *mqmsg);
EXTERN void         mq_msgqfree(FAR msgq_t *msgq);

/* mq_waitirq.c ************************************************************/

EXTERN void         mq_waitirq(FAR _TCB *wtcb, int errcode);

/* mq_rcvinternal.c ********************************************************/

EXTERN int          mq_verifyreceive(mqd_t mqdes, void *msg, size_t msglen);
EXTERN FAR mqmsg_t *mq_waitreceive(mqd_t mqdes);
EXTERN ssize_t      mq_doreceive(mqd_t mqdes, mqmsg_t *mqmsg, void *ubuffer,
                                 int *prio);

/* mq_sndinternal.c ********************************************************/

EXTERN int          mq_verifysend(mqd_t mqdes, const void *msg, size_t msglen,
                                  int prio);
EXTERN FAR mqmsg_t *mq_msgalloc(void);
EXTERN int          mq_waitsend(mqd_t mqdes);
EXTERN int          mq_dosend(mqd_t mqdes, FAR mqmsg_t *mqmsg, const void *msg,
                              size_t msglen, int prio);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_MQ_MAXMSGSIZE > 0 */
#endif /* __SCHED_MQ_INTERNAL_H */

