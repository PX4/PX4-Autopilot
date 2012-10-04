/****************************************************************************
 * apps/examples/ftpd/ftpd.h
 * Interface for the Contiki ftpd.
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
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __APPS_EXAMPLES_FTPD_FTPD_H
#define __APPS_EXAMPLES_FTPD_FTPD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_EXAMPLES_FTPD_PRIO - Priority of the FTP daemon.
 *   Default: SCHED_PRIORITY_DEFAULT
 * CONFIG_EXAMPLES_FTPD_STACKSIZE - Stack size allocated for the
 *   FTP daemon. Default: 2048
 * CONFIG_EXAMPLES_FTPD_NONETINIT - Define to suppress configuration of the
 *   network by apps/examples/ftpd.  You would need to suppress network
 *   configuration if the network is configuration prior to running the
 *   example.
 *
 * If CONFIG_EXAMPLES_FTPD_NONETINIT is not defined, then the following may
 * be specified to customized the network configuration:
 *
 * CONFIG_EXAMPLES_FTPD_NOMAC - If the hardware has no MAC address of its
 *   own, define this =y to provide a bogus address for testing.
 * CONFIG_EXAMPLES_FTPD_IPADDR - The target IP address.  Default 10.0.0.2
 * CONFIG_EXAMPLES_FTPD_DRIPADDR - The default router address. Default
 *   10.0.0.1
 * CONFIG_EXAMPLES_FTPD_NETMASK - The network mask.  Default: 255.255.255.0
 */

#ifndef CONFIG_EXAMPLES_FTPD_PRIO
#  define CONFIG_EXAMPLES_FTPD_PRIO SCHED_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_EXAMPLES_FTPD_STACKSIZE
#  define CONFIG_EXAMPLES_FTPD_STACKSIZE 2048
#endif

#ifndef CONFIG_EXAMPLES_FTPD_CLIENTPRIO
#  define CONFIG_EXAMPLES_FTPD_CLIENTPRIO SCHED_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_EXAMPLES_FTPD_CLIENTSTACKSIZE
#  define CONFIG_EXAMPLES_FTPD_CLIENTSTACKSIZE 2048
#endif

/* NSH always initializes the network */

#if defined(CONFIG_NSH_BUILTIN_APPS) && !defined(CONFIG_EXAMPLES_FTPD_NONETINIT)
#  define CONFIG_EXAMPLES_FTPD_NONETINIT 1
#endif

#ifdef CONFIG_EXAMPLES_FTPD_NONETINIT
#  undef CONFIG_EXAMPLES_FTPD_IPADDR
#  undef CONFIG_EXAMPLES_FTPD_DRIPADDR
#  undef CONFIG_EXAMPLES_FTPD_NETMASK
#else
#  ifndef CONFIG_EXAMPLES_FTPD_IPADDR
#    define CONFIG_EXAMPLES_FTPD_IPADDR 0x0a000002
#  endif
#  ifndef CONFIG_EXAMPLES_FTPD_DRIPADDR
#    define CONFIG_EXAMPLES_FTPD_DRIPADDR 0x0a000001
#  endif
#  ifndef CONFIG_EXAMPLES_FTPD_NETMASK
#    define CONFIG_EXAMPLES_FTPD_NETMASK 0xffffff00
#  endif
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure describes one entry in a table of accounts */

struct fptd_account_s
{
  uint8_t         flags;
  FAR const char *user;
  FAR const char *password;
  FAR const char *home;
};

/* To minimize the probability of name collisitions, all FTPD example
 * global data is maintained in single structure.
 */

struct ftpd_globals_s
{
  bool          initialized; /* True: Networking is initialized.  The
                              * network must be initialized only once.
                              */
#ifdef CONFIG_NSH_BUILTIN_APPS
  volatile bool stop;        /* True: Request daemon to exit */
  volatile bool running;     /* True: The daemon is running */
#endif
  pid_t         pid;         /* Task ID of the FTPD daemon.  The value
                              * -1 is a redundant indication that the
                              * daemon is not running.
                              */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* To minimize the probability of name collisitions, all FTPD example
 * global data is maintained in a single instance of a structure.
 */

extern struct ftpd_globals_s g_ftpdglob;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __APPS_EXAMPLES_FTPD_FTPD_H */
