/****************************************************************************
 * examples/can/can_main.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/can.h>

#include "can.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_EXAMPLES_CAN_READONLY)
#  undef CONFIG_EXAMPLES_CAN_WRITEONLY
#  undef CONFIG_EXAMPLES_CAN_READWRITE
#  define CAN_OFLAGS O_RDONLY
#elif defined(CONFIG_EXAMPLES_CAN_WRITEONLY)
#  undef CONFIG_EXAMPLES_CAN_READWRITE
#  define CAN_OFLAGS O_WRONLY
#else
#  undef CONFIG_EXAMPLES_CAN_READWRITE
#  define CONFIG_EXAMPLES_CAN_READWRITE 1
#  define CAN_OFLAGS O_RDWR
#endif

#ifdef CONFIG_CAN_EXTID
#  define MAX_ID (1 << 29)
#else
#  define MAX_ID (1 << 11)
#endif

#ifdef CONFIG_NSH_BUILTIN_APPS
#  define MAIN_NAME   can_main
#  define MAIN_STRING "can_main: "
#else
#  define MAIN_NAME   user_start
#  define MAIN_STRING "user_start: "
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: user_start/can_main
 ****************************************************************************/

int MAIN_NAME(int argc, char *argv[])
{
#ifndef CONFIG_EXAMPLES_CAN_READONLY
  struct can_msg_s txmsg;
#ifdef CONFIG_CAN_EXTID
  uint32_t msgid;
#else
  uint16_t msgid;
#endif
  int msgdlc;
  uint8_t msgdata;
#endif

#ifndef CONFIG_EXAMPLES_CAN_WRITEONLY
  struct can_msg_s rxmsg;
#endif

  size_t msgsize;
  ssize_t nbytes;
#if defined(CONFIG_NSH_BUILTIN_APPS) || defined(CONFIG_EXAMPLES_CAN_NMSGS)
  long nmsgs;
#endif

  int fd;
  int errval = 0;
  int ret;
  int i;

  /* If this example is configured as an NX add-on, then limit the number of
   * samples that we collect before returning.  Otherwise, we never return
   */

#if defined(CONFIG_NSH_BUILTIN_APPS)
  nmsgs = CONFIG_EXAMPLES_CAN_NMSGS;
  if (argc > 1)
    {
      nmsgs = strtol(argv[1], NULL, 10);
    }
  message(MAIN_STRING "nmsgs: %d\n", nmsgs);
#elif defined(CONFIG_EXAMPLES_CAN_NMSGS)
  message(MAIN_STRING "nmsgs: %d\n", CONFIG_EXAMPLES_CAN_NMSGS);
#endif

  /* Initialization of the CAN hardware is performed by logic external to
   * this test.
   */

  message(MAIN_STRING "Initializing external CAN device\n");
  ret = can_devinit();
  if (ret != OK)
    {
      message(MAIN_STRING "can_devinit failed: %d\n", ret);
      errval = 1;
      goto errout;
    }

  /* Open the CAN device for reading */

  message(MAIN_STRING "Hardware initialized. Opening the CAN device\n");
  fd = open(CONFIG_EXAMPLES_CAN_DEVPATH, CAN_OFLAGS);
  if (fd < 0)
    {
      message(MAIN_STRING "open %s failed: %d\n",
              CONFIG_EXAMPLES_CAN_DEVPATH, errno);
      errval = 2;
      goto errout_with_dev;
    }

  /* Now loop the appropriate number of times, performing one loopback test
   * on each pass.
   */

#ifndef CONFIG_EXAMPLES_CAN_READONLY
  msgdlc  = 1;
  msgid   = 1;
  msgdata = 0;
#endif

#if defined(CONFIG_NSH_BUILTIN_APPS)
  for (; nmsgs > 0; nmsgs--)
#elif defined(CONFIG_EXAMPLES_CAN_NMSGS)
  for (nmsgs = 0; nmsgs < CONFIG_EXAMPLES_CAN_NMSGS; nmsgs++)
#else
  for (;;)
#endif
  {
    /* Flush any output before the loop entered or from the previous pass
     * through the loop.
     */

    msgflush();

    /* Construct the next TX message */

#ifndef CONFIG_EXAMPLES_CAN_READONLY
    txmsg.cm_hdr.ch_id    = msgid;
    txmsg.cm_hdr.ch_rtr   = false;
    txmsg.cm_hdr.ch_dlc   = msgdlc;
#ifdef CONFIG_CAN_EXTID
    txmsg.cm_hdr.ch_extid = true;
#endif

    for (i = 0; i < msgdlc; i++)
      {
        txmsg.cm_data[i] = msgdata + i;
      }

    /* Send the TX message */

    msgsize = CAN_MSGLEN(msgdlc);
    nbytes = write(fd, &txmsg, msgsize);
    if (nbytes != msgsize)
      {
        message("ERROR: write(%d) returned %d\n", msgsize, nbytes);
        errval = 3;
        goto errout_with_dev;
      }
#endif

#ifdef CONFIG_EXAMPLES_CAN_WRITEONLY
    message("  ID: %4d DLC: %d\n", msgid, msgdlc);
#endif

    /* Read the RX message */

#ifndef CONFIG_EXAMPLES_CAN_WRITEONLY
    msgsize = sizeof(struct can_msg_s);
    nbytes = read(fd, &rxmsg, msgsize);
    if (nbytes < CAN_MSGLEN(0) || nbytes > msgsize)
      {
        message("ERROR: read(%d) returned %d\n", msgsize, nbytes);
        errval = 4;
        goto errout_with_dev;
      }
#endif

#ifndef CONFIG_EXAMPLES_CAN_READONLY
    message("  ID: %4d DLC: %d\n", rxmsg.cm_hdr.id, rxmsg.cm_hdr.dlc);
#endif

    /* Verify that the received messages are the same */

#ifdef CONFIG_EXAMPLES_CAN_READWRITE
    if (memcmp(&txmsg.cm_hdr, &rxmsg.cm_hdr, sizeof(struct can_hdr_s)) != 0)
      {
        message("ERROR: Sent header does not match received header:\n");
        lib_dumpbuffer("Sent header", (FAR const uint8_t*)&txmsg.cm_hdr, 
                       sizeof(struct can_hdr_s));
        lib_dumpbuffer("Received header", (FAR const uint8_t*)&rxmsg.cm_hdr, 
                       sizeof(struct can_hdr_s));
        errval = 4;
        goto errout_with_dev;
      }

    if (memcmp(txmsg.cm_data, rxmsg.cm_data, msgdlc) != 0)
      {
        message("ERROR: Data does not match. DLC=%d\n", msgdlc);
        for (i = 0; i < msgdlc; i++)
          {
            message("  %d: TX %02x RX %02x\n", i, txmsg.cm_data[i], rxmsg.cm_data[i]);
            errval = 5;
            goto errout_with_dev;
          }
      }

    /* Report success */
  
    message("  ID: %4d DLC: %d -- OK\n", msgid, msgdlc);
#endif

    /* Set up for the next pass */

#ifndef CONFIG_EXAMPLES_CAN_READONLY
    msgdata += msgdlc;
 
    if (++msgid >= MAX_ID)
      {
        msgid = 1;
      }

    if (++msgdlc > CAN_MAXDATALEN)
      {
        msgdlc = 1;
      }
#endif
  }

errout_with_dev:
  close(fd);

errout:
  message("Terminating!\n");
  msgflush();
  return errval;
}
